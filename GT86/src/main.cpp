#include <Arduino.h>
#include <TFT_eSPI.h>
#include <BLEDevice.h>
#include <BLEScan.h>
#include <BLEAdvertisedDevice.h>
#include <nvs_flash.h>

TFT_eSPI tft = TFT_eSPI();

// ─── UART inter-ESP ───────────────────────────────────────
#define UART_BAUD 115200
#define UART_RX   25
#define UART_TX   26

// ─── Protocole trame UART ─────────────────────────────────
#define FRAME_SYNC1    0xAA
#define FRAME_SYNC2    0x55
#define FRAME_TYPE_RB  0x01
#define FRAME_TYPE_OBD 0x02
#define FRAME_LEN_RB   12
#define OBD_FRAME_LEN  22

struct __attribute__((packed)) OBDFrameExt {
    uint8_t  sync1, sync2, type;
    uint16_t rpm;
    int16_t  coolant, intakeTemp;
    uint16_t batteryMV, afr;
    uint8_t  load, fuel;
    int8_t   timing;
    uint8_t  mil;
    int16_t  oilTemp;
    uint8_t  speed, crc;
};

// ─── BLE OBD : buffer + Stream ───────────────────────────
#define BLE_BUF_SIZE 512
static uint8_t bleBuf[BLE_BUF_SIZE];
static volatile int bleHead = 0, bleTail = 0;
static BLERemoteCharacteristic* bleNotifyChar = nullptr;
static BLERemoteCharacteristic* bleWriteChar  = nullptr;
static BLEClient* pClient = nullptr;

static void bleNotifyCB(BLERemoteCharacteristic*, uint8_t* data, size_t len, bool) {
    for (size_t i = 0; i < len; i++) {
        int next = (bleHead + 1) % BLE_BUF_SIZE;
        if (next != bleTail) { bleBuf[bleHead] = data[i]; bleHead = next; }
    }
}

class BLEOBDStream : public Stream {
public:
    int available() override { return (bleHead - bleTail + BLE_BUF_SIZE) % BLE_BUF_SIZE; }
    int read() override {
        if (bleHead == bleTail) return -1;
        uint8_t b = bleBuf[bleTail]; bleTail = (bleTail + 1) % BLE_BUF_SIZE; return b;
    }
    int peek() override { return bleHead == bleTail ? -1 : bleBuf[bleTail]; }
    size_t write(uint8_t b) override {
        if (!bleWriteChar) return 0;
        bleWriteChar->writeValue(&b, 1); return 1;
    }
    size_t write(const uint8_t* buf, size_t len) override {
        if (!bleWriteChar) return 0;
        bleWriteChar->writeValue((uint8_t*)buf, len); return len;
    }
};
BLEOBDStream bleStream;

// ─── Données OBD ──────────────────────────────────────────
uint16_t obdRpm=0; int16_t obdCoolant=0; int16_t obdIntakeTemp=0;
uint16_t obdBatteryMV=0; uint16_t obdAFR=0; uint8_t obdLoad=0;
uint8_t obdFuel=0; int8_t obdTiming=0; uint8_t obdMIL=0;
int16_t obdOilTemp=-999; uint8_t obdSpeed=0;
bool obdConnected=false; bool obdFresh=false;

// ─── PID round-robin ─────────────────────────────────────
struct PIDDef { const char* cmd; uint8_t pid; uint8_t dataBytes; };
const PIDDef pidList[] = {
    {"010C\r",0x0C,2},  // RPM
    {"0105\r",0x05,1},  // Coolant
    {"010C\r",0x0C,2},  // RPM (2x)
    {"0104\r",0x04,1},  // Load
    {"010D\r",0x0D,1},  // Speed
};
#define PID_COUNT 5
int currentPID=0; bool queryPending=false; uint32_t queryStart=0; String obdResp="";

// ─── RaceBox (reçu du S3) ────────────────────────────────
float rxSpeed=0; uint8_t rxFix=0; uint32_t lastRx=0;

// ─── Parser UART ─────────────────────────────────────────
uint8_t rxBufU[FRAME_LEN_RB]; uint8_t rxPosU=0;
void processUART() {
    while (Serial2.available()) {
        uint8_t b = Serial2.read();
        if (rxPosU==0) { if(b==FRAME_SYNC1) rxBufU[rxPosU++]=b; }
        else if (rxPosU==1) { if(b==FRAME_SYNC2) rxBufU[rxPosU++]=b; else rxPosU=0; }
        else {
            rxBufU[rxPosU++]=b;
            if (rxPosU==FRAME_LEN_RB) {
                rxPosU=0; uint8_t crc=0;
                for(int i=2;i<FRAME_LEN_RB-1;i++) crc^=rxBufU[i];
                if(crc==rxBufU[FRAME_LEN_RB-1]&&rxBufU[2]==FRAME_TYPE_RB) {
                    rxSpeed=((uint16_t)(rxBufU[3]|rxBufU[4]<<8))/10.0f;
                    rxFix=rxBufU[9]; lastRx=millis();
                }
            }
        }
    }
}

// ─── Envoi trame OBD ─────────────────────────────────────
void sendOBDFrame() {
    OBDFrameExt f={}; f.sync1=FRAME_SYNC1; f.sync2=FRAME_SYNC2; f.type=FRAME_TYPE_OBD;
    f.rpm=obdRpm; f.coolant=obdCoolant; f.intakeTemp=obdIntakeTemp;
    f.batteryMV=obdBatteryMV; f.afr=obdAFR; f.load=obdLoad;
    f.fuel=obdFuel; f.timing=obdTiming; f.mil=obdMIL;
    f.oilTemp=obdOilTemp; f.speed=obdSpeed;
    uint8_t crc=0; for(int i=2;i<OBD_FRAME_LEN-1;i++) crc^=((uint8_t*)&f)[i];
    f.crc=crc; Serial2.write((const uint8_t*)&f, OBD_FRAME_LEN);
}

// ─── Parser réponse OBD ──────────────────────────────────
bool parseOBDHex(const String& resp, uint8_t pid, uint8_t* data, int count) {
    char prefix[8]; snprintf(prefix,8,"41%02X",pid);
    String clean=resp; clean.replace(" ","");
    int idx=clean.indexOf(prefix); if(idx<0) return false;
    idx+=4;
    for(int i=0;i<count;i++) {
        if(idx+2>(int)clean.length()) return false;
        data[i]=strtol(clean.substring(idx,idx+2).c_str(),NULL,16); idx+=2;
    }
    return true;
}

void applyPIDValue() {
    uint8_t data[4]; uint8_t pid=pidList[currentPID].pid;
    if(!parseOBDHex(obdResp,pid,data,pidList[currentPID].dataBytes)) return;
    switch(pid) {
        case 0x0C: obdRpm=(data[0]*256+data[1])/4; break;
        case 0x05: obdCoolant=(int16_t)data[0]-40; break;
        case 0x04: obdLoad=data[0]*100/255; break;
        case 0x0D: obdSpeed=data[0]; break;
        case 0x0F: obdIntakeTemp=(int16_t)data[0]-40; break;
        case 0x42: obdBatteryMV=data[0]*256+data[1]; break;
        case 0x2F: obdFuel=data[0]*100/255; break;
        case 0x0E: obdTiming=(int8_t)(data[0]/2-64); break;
        case 0x24: obdAFR=(uint16_t)((data[0]*256+data[1])*1470L/32768); break;
        case 0x01: obdMIL=data[0]; break;
        case 0x5C: obdOilTemp=(int16_t)data[0]-40; break;
    }
    obdFresh=true;
}

void processOBD() {
    if(!obdConnected) return;
    while(bleStream.available()) {
        char c=bleStream.read();
        if(c=='>') {
            if(queryPending){applyPIDValue();queryPending=false;currentPID=(currentPID+1)%PID_COUNT;}
            obdResp="";
        } else if(c>=0x20&&c<=0x7E) obdResp+=c;
    }
    if(!queryPending&&millis()-queryStart>=50) {
        queryStart=millis(); bleHead=bleTail=0; obdResp="";
        bleWriteChar->writeValue((uint8_t*)pidList[currentPID].cmd, strlen(pidList[currentPID].cmd));
        queryPending=true;
    }
    if(queryPending&&millis()-queryStart>5000) {
        queryPending=false; obdResp=""; currentPID=(currentPID+1)%PID_COUNT;
    }
}

// ─── Affichage GC9A01 ────────────────────────────────────
int prevCool=-999; int prevRpm=-1; uint8_t prevFix=255; bool prevS3=false; bool prevObd=false;

void drawCoolant(int t) {
    tft.fillRect(20,68,200,80,TFT_BLACK);
    uint16_t c=t>100?TFT_RED:(t>90?TFT_ORANGE:TFT_GREEN);
    uint8_t sz=(t<100&&t>-10)?8:6;
    int d=(abs(t)<10)?1:(abs(t)<100)?2:3; if(t<0)d++;
    tft.setTextSize(sz); tft.setTextColor(c,TFT_BLACK);
    tft.setCursor((240-d*6*sz)/2,75); tft.print(t);
}
void drawRpm(int r) {
    tft.fillRect(20,160,200,26,TFT_BLACK);
    tft.setTextSize(2); tft.setTextColor(TFT_WHITE,TFT_BLACK);
    char b[16]; snprintf(b,16,"%d RPM",r);
    tft.setCursor((240-(int)strlen(b)*12)/2,162); tft.print(b);
}
void drawInd(bool s3,uint8_t fx,bool ob) {
    tft.fillRect(20,196,200,32,TFT_BLACK);
    tft.fillCircle(60,210,6,fx>=3?TFT_GREEN:TFT_RED);
    tft.setTextSize(1);
    tft.setTextColor(s3?TFT_GREEN:(uint16_t)0x4208,TFT_BLACK);
    tft.setCursor(80,206); tft.print(s3?"S3":"--");
    tft.setTextColor(ob?TFT_GREEN:(uint16_t)0x4208,TFT_BLACK);
    tft.setCursor(110,206); tft.print(ob?"OBD":"---");
}
void updateDisplay() {
    bool s3=(millis()-lastRx)<2000;
    if(obdCoolant!=prevCool){drawCoolant(obdCoolant);prevCool=obdCoolant;}
    if((int)obdRpm!=prevRpm){drawRpm(obdRpm);prevRpm=obdRpm;}
    if(rxFix!=prevFix||s3!=prevS3||obdConnected!=prevObd) {
        drawInd(s3,rxFix,obdConnected); prevFix=rxFix; prevS3=s3; prevObd=obdConnected;
    }
}

// ─── Scan BLE + connexion ────────────────────────────────
#define MAX_SCAN 8
String btNames[MAX_SCAN], btAddrs[MAX_SCAN]; int btCount=0;
void drawScan(const char* status) {
    tft.fillRect(10,55,220,165,TFT_BLACK);
    tft.setTextSize(2); tft.setTextColor(TFT_YELLOW,TFT_BLACK);
    tft.setCursor(54,58); tft.print("BLE SCAN");
    tft.setTextSize(1);
    for(int i=0;i<btCount&&i<6;i++) {
        bool t=btNames[i].indexOf("Vlink")>=0||btNames[i].indexOf("iCar")>=0||btNames[i].indexOf("OBD")>=0;
        tft.setTextColor(t?TFT_GREEN:TFT_WHITE,TFT_BLACK);
        String txt=btNames[i]; if(txt.length()>24)txt=txt.substring(0,24);
        tft.setCursor((240-(int)txt.length()*6)/2,82+i*14); tft.print(txt.c_str());
    }
    if(status){tft.setTextSize(1);tft.setTextColor(TFT_ORANGE,TFT_BLACK);
    tft.setCursor((240-(int)strlen(status)*6)/2,170);tft.print(status);}
}

bool scanAndConnect() {
    btCount=0;
    bleHead=bleTail=0; bleNotifyChar=nullptr; bleWriteChar=nullptr;
    if(pClient){pClient->disconnect(); delete pClient; pClient=nullptr;}

    drawScan("scanning...");
    BLEScan* pScan = BLEDevice::getScan();
    pScan->setActiveScan(true);
    pScan->setInterval(100);
    pScan->setWindow(99);
    BLEScanResults results = pScan->start(6, false);

    // Trouver l'OBD dans les résultats
    int targetIdx = -1;
    for(int i=0; i<results.getCount()&&btCount<MAX_SCAN; i++) {
        BLEAdvertisedDevice dev = results.getDevice(i);
        String name=String(dev.getName().c_str());
        if(name.length()==0) continue;
        btNames[btCount]=name;
        btAddrs[btCount]=String(dev.getAddress().toString().c_str());
        if(name.indexOf("Vlink")>=0||name.indexOf("iCar")>=0||name.indexOf("OBD")>=0||name.indexOf("ELM")>=0)
            targetIdx=btCount;
        btCount++;
    }

    char msg[32]; snprintf(msg,32,"%d appareils",btCount);
    drawScan(targetIdx>=0?btNames[targetIdx].c_str():msg);
    if(targetIdx<0){delay(500);pScan->clearResults();return false;}

    // ── Connexion directe depuis les résultats (avant clearResults) ──
    drawScan("1/4 connexion...");
    pClient = BLEDevice::createClient();
    BLEAdvertisedDevice targetDev = results.getDevice(targetIdx >= btCount ? 0 : targetIdx);
    // Retrouver le vrai index dans results (btCount peut différer car on skip les sans-nom)
    for(int i=0; i<results.getCount(); i++) {
        BLEAdvertisedDevice d = results.getDevice(i);
        if(String(d.getAddress().toString().c_str())==btAddrs[targetIdx]) {
            targetDev=d; break;
        }
    }
    pScan->clearResults();

    Serial.printf("[BLE] Connecting to %s\n", btAddrs[targetIdx].c_str());
    if(!pClient->connect(&targetDev)) {drawScan("ECHEC connexion");delay(2000);return false;}
    Serial.println("[BLE] Connected!");

    // ── Services ──
    drawScan("2/4 services...");
    auto* svcs = pClient->getServices();
    for(auto it=svcs->begin(); it!=svcs->end()&&!bleWriteChar; ++it) {
        auto* chars = it->second->getCharacteristics();
        BLERemoteCharacteristic *nCh=nullptr, *wCh=nullptr;
        for(auto cit=chars->begin(); cit!=chars->end(); ++cit) {
            auto* ch=cit->second;
            if(ch->canNotify()&&!nCh) nCh=ch;
            if(ch->canWrite()&&!wCh) wCh=ch;
        }
        if(nCh&&wCh){bleNotifyChar=nCh; bleWriteChar=wCh;}
    }
    if(!bleNotifyChar||!bleWriteChar){drawScan("ECHEC: no chars");delay(2000);pClient->disconnect();return false;}

    // ── Notify ──
    drawScan("3/4 notify...");
    bleNotifyChar->registerForNotify(bleNotifyCB);
    delay(500);

    // ── Init AT ──
    const char* cmds[]={"ATI\r","ATE0\r","ATL0\r","ATSP0\r"};
    const char* labels[]={"ATI","ATE0","ATL0","ATSP0"};
    String responses[4]; bool hasPrompt=false;
    for(int c=0;c<4;c++) {
        bleHead=bleTail=0;
        drawScan(labels[c]);
        bleWriteChar->writeValue((uint8_t*)cmds[c], strlen(cmds[c]));
        delay(2000);
        String r="";
        while(bleStream.available()&&r.length()<50){char ch=bleStream.read();r+=(ch>=0x20&&ch<=0x7E)?ch:'.';}
        responses[c]=r;
        if(r.indexOf('>')>=0) hasPrompt=true;
    }
    tft.fillRect(10,55,220,165,TFT_BLACK); tft.setTextSize(1);
    for(int c=0;c<4;c++){
        tft.setTextColor(TFT_YELLOW,TFT_BLACK); tft.setCursor(25,62+c*24); tft.print(labels[c]); tft.print(":");
        tft.setTextColor(TFT_WHITE,TFT_BLACK); tft.setCursor(25,72+c*24); tft.print(responses[c].substring(0,28).c_str());
    }
    tft.setTextColor(hasPrompt?TFT_GREEN:TFT_RED,TFT_BLACK);
    tft.setCursor(40,164); tft.print(hasPrompt?"ELM327 OK!":"pas de prompt >");
    delay(3000);
    if(!hasPrompt){pClient->disconnect();return false;}
    bleHead=bleTail=0;
    return true;
}

void switchToOBDDisplay() {
    tft.fillRect(10,55,220,170,TFT_BLACK);
    tft.setTextSize(2); tft.setTextColor(0x4208,TFT_BLACK);
    tft.setCursor(100,148); tft.print("\xF7""C");
    drawCoolant(0); drawRpm(0); drawInd(false,0,true);
}

// ─── Setup ────────────────────────────────────────────────
void setup() {
    Serial.begin(115200);
    Serial2.begin(UART_BAUD, SERIAL_8N1, UART_RX, UART_TX);
    tft.begin(); tft.setRotation(0); tft.fillScreen(TFT_BLACK);
    tft.drawCircle(120,120,118,TFT_ORANGE);
    tft.setTextSize(3); tft.setTextColor(TFT_WHITE,TFT_BLACK);
    tft.setCursor(74,24); tft.print("GT86");

    // Nettoyer le NVS BLE (bonds corrompus par les changements de stack)
    nvs_flash_erase();
    nvs_flash_init();

    BLEDevice::init("");
    obdConnected = scanAndConnect();
    if(obdConnected) switchToOBDDisplay();
}

// ─── Loop ─────────────────────────────────────────────────
uint32_t lastDraw=0, lastOBD=0, lastHB=0, lastRetry=0;
void loop() {
    processUART(); processOBD();
    if(millis()-lastDraw>=100){lastDraw=millis(); if(obdConnected) updateDisplay();}
    if(obdConnected&&millis()-lastOBD>=200){lastOBD=millis(); sendOBDFrame();}
    if(!obdConnected&&millis()-lastRetry>=8000){lastRetry=millis();
        obdConnected=scanAndConnect();
        if(obdConnected) switchToOBDDisplay();
    }
    if(millis()-lastHB>=5000){lastHB=millis();
        Serial.printf("[HB] obd=%d rpm=%d cool=%d bat=%d mil=%02X\n",
                      obdConnected,obdRpm,obdCoolant,obdBatteryMV,obdMIL);}
}
