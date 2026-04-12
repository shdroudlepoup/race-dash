#include <Arduino.h>
#include <Arduino_GFX_Library.h>
#include <NimBLEDevice.h>

// ─── Display Waveshare ESP32-S3-Touch-LCD-7 ───────────────
Arduino_ESP32RGBPanel *rgbpanel = new Arduino_ESP32RGBPanel(
     5,  3, 46,  7,
     1,  2, 42, 41, 40,
    39,  0, 45, 48, 47, 21,
    14, 38, 18, 17, 10,
    0, 8, 4, 8, 0, 8, 4, 8, 1, 14000000
);
Arduino_RGB_Display *gfx = new Arduino_RGB_Display(800, 480, rgbpanel, 0, true);

// ─── UART inter-ESP (switch Waveshare en UART2) ───────────
#define UART_BAUD  115200
#define INTER_TX   43
#define INTER_RX   44
HardwareSerial InterSerial(0);

// ─── Protocole trame UART (S3 → Wroom) ───────────────────
#define FRAME_SYNC1    0xAA
#define FRAME_SYNC2    0x55
#define FRAME_TYPE_RB  0x01
#define FRAME_LEN_RB   12

struct __attribute__((packed)) UARTFrame {
    uint8_t  sync1, sync2, type;
    uint16_t speedX10;
    int16_t  gxX1000, gyX1000;
    uint8_t  fix, svs, crc;
};

// ─── RaceBox BLE ──────────────────────────────────────────
#define RB_SN        "3242708879"
#define NUS_SVC_UUID "6E400001-B5A3-F393-E0A9-E50E24DCCA9E"
#define NUS_TX_UUID  "6E400003-B5A3-F393-E0A9-E50E24DCCA9E"

struct RBData {
    float speedKmh=0, gx=0, gy=0, voltage=0;
    uint8_t fix=0, svs=0;
};
RBData        rb;
volatile bool rbFresh = false;
NimBLEClient* rbClient = nullptr;
bool          rbConnected = false;

void onRBNotify(NimBLERemoteCharacteristic*, uint8_t* d, size_t len, bool) {
    if (len<88||d[0]!=0xB5||d[1]!=0x62||d[2]!=0xFF||d[3]!=0x01) return;
    uint8_t ca=0,cb=0;
    for(size_t i=2;i<len-2;i++){ca+=d[i];cb+=ca;}
    if(ca!=d[len-2]||cb!=d[len-1]) return;
    const uint8_t* p=d+6;
    rb.speedKmh=(int32_t)(p[48]|(uint32_t)p[49]<<8|(uint32_t)p[50]<<16|(uint32_t)p[51]<<24)*3.6f/1000.0f;
    rb.gx=(int16_t)(p[68]|(uint16_t)p[69]<<8)/1000.0f;
    rb.gy=(int16_t)(p[70]|(uint16_t)p[71]<<8)/1000.0f;
    rb.fix=p[20]; rb.svs=p[23]; rb.voltage=p[67]/10.0f;
    rbFresh=true;
}

// ─── OBD BLE (iCare Pro 2S = "iOS-Vlink") ────────────────
#define OBD_BLE_NAME "Vlink"

#define BLE_BUF_SIZE 512
static uint8_t bleBuf[BLE_BUF_SIZE];
static volatile int bleHead=0, bleTail=0;
static NimBLERemoteCharacteristic* obdNotifyChar=nullptr;
static NimBLERemoteCharacteristic* obdWriteChar=nullptr;
NimBLEClient* obdClient=nullptr;
bool obdConnected=false;

static void obdNotifyCB(NimBLERemoteCharacteristic*, uint8_t* data, size_t len, bool) {
    for(size_t i=0;i<len;i++){
        int next=(bleHead+1)%BLE_BUF_SIZE;
        if(next!=bleTail){bleBuf[bleHead]=data[i];bleHead=next;}
    }
}

int obdBufAvailable() { return (bleHead-bleTail+BLE_BUF_SIZE)%BLE_BUF_SIZE; }
int obdBufRead() { if(bleHead==bleTail)return -1; uint8_t b=bleBuf[bleTail];bleTail=(bleTail+1)%BLE_BUF_SIZE;return b; }

// OBD data
uint16_t obdRpm=0; int16_t obdCoolant=0;
bool obdFresh=false;

// PID round-robin
struct PIDDef{const char*cmd;uint8_t pid;uint8_t bytes;};
const PIDDef pidList[]={
    {"010C\r",0x0C,2},  // RPM
    {"0105\r",0x05,1},  // Coolant
    {"010C\r",0x0C,2},  // RPM (2x)
};
#define PID_COUNT 3
int currentPID=0; bool queryPending=false; uint32_t queryStart=0; String obdResp="";

bool parseOBDHex(const String& resp,uint8_t pid,uint8_t* data,int count){
    char prefix[8];snprintf(prefix,8,"41%02X",pid);
    String clean=resp;clean.replace(" ","");
    int idx=clean.indexOf(prefix);if(idx<0)return false;idx+=4;
    for(int i=0;i<count;i++){if(idx+2>(int)clean.length())return false;
    data[i]=strtol(clean.substring(idx,idx+2).c_str(),NULL,16);idx+=2;}
    return true;
}

void applyPIDValue(){
    uint8_t data[4];uint8_t pid=pidList[currentPID].pid;
    if(!parseOBDHex(obdResp,pid,data,pidList[currentPID].bytes))return;
    switch(pid){
        case 0x0C:obdRpm=(data[0]*256+data[1])/4;break;
        case 0x05:obdCoolant=(int16_t)data[0]-40;break;
    }
    obdFresh=true;
}

void processOBD(){
    if(!obdConnected||!obdWriteChar)return;
    while(obdBufAvailable()){char c=obdBufRead();
        if(c=='>'){if(queryPending){applyPIDValue();queryPending=false;currentPID=(currentPID+1)%PID_COUNT;}obdResp="";}
        else if(c>=0x20&&c<=0x7E)obdResp+=c;}
    if(!queryPending&&millis()-queryStart>=50){queryStart=millis();bleHead=bleTail=0;obdResp="";
        obdWriteChar->writeValue((const uint8_t*)pidList[currentPID].cmd,strlen(pidList[currentPID].cmd),true);
        queryPending=true;}
    if(queryPending&&millis()-queryStart>5000){queryPending=false;obdResp="";currentPID=(currentPID+1)%PID_COUNT;}
}

// ─── BLE Scan : cherche RaceBox ET iCare ─────────────────
NimBLEAdvertisedDevice* pendingRB = nullptr;
NimBLEAdvertisedDevice* pendingOBD = nullptr;
volatile bool doScan = true;

#define MAX_SCAN 7
String scanNames[MAX_SCAN];
int scanFoundCount=0;
bool scanDirty=false;

class ScanCB : public NimBLEAdvertisedDeviceCallbacks {
    void onResult(NimBLEAdvertisedDevice* dev) override {
        String name=String(dev->getName().c_str());
        if(name.length()==0)return;

        bool known=false;
        for(int i=0;i<scanFoundCount;i++) if(scanNames[i]==name){known=true;break;}
        if(!known&&scanFoundCount<MAX_SCAN){scanNames[scanFoundCount++]=name;scanDirty=true;}

        // RaceBox
        if(!rbConnected && !pendingRB && name.indexOf(RB_SN)>=0) {
            pendingRB = dev;
        }
        // OBD iCare
        if(!obdConnected && !pendingOBD && name.indexOf(OBD_BLE_NAME)>=0) {
            pendingOBD = dev;
        }
        // Arrêter le scan si les deux sont trouvés
        if((rbConnected||pendingRB) && (obdConnected||pendingOBD)) {
            NimBLEDevice::getScan()->stop();
        }
    }
};

void onScanEnded(NimBLEScanResults) {
    if(!rbConnected || !obdConnected) doScan=true;
}

bool connectRB() {
    if(!pendingRB) return false;
    rbClient = NimBLEDevice::createClient();
    if(!rbClient->connect(pendingRB)){pendingRB=nullptr;return false;}
    auto* svc=rbClient->getService(NUS_SVC_UUID);
    if(!svc){rbClient->disconnect();pendingRB=nullptr;return false;}
    auto* ch=svc->getCharacteristic(NUS_TX_UUID);
    if(!ch||!ch->canNotify()){rbClient->disconnect();pendingRB=nullptr;return false;}
    ch->subscribe(true,onRBNotify);
    rbConnected=true; pendingRB=nullptr;
    return true;
}

bool connectOBD() {
    if(!pendingOBD) return false;
    obdClient = NimBLEDevice::createClient();
    if(!obdClient->connect(pendingOBD)){pendingOBD=nullptr;return false;}

    // Auto-découverte services
    auto svcs=obdClient->getServices(true);
    if(!svcs){obdClient->disconnect();pendingOBD=nullptr;return false;}
    for(auto* svc:*svcs){
        auto chars=svc->getCharacteristics(true);
        if(!chars)continue;
        NimBLERemoteCharacteristic *nCh=nullptr,*wCh=nullptr;
        for(auto* ch:*chars){
            if((ch->canNotify()||ch->canIndicate())&&!nCh)nCh=ch;
            if((ch->canWrite()||ch->canWriteNoResponse())&&!wCh)wCh=ch;
        }
        if(nCh&&wCh){obdNotifyChar=nCh;obdWriteChar=wCh;break;}
    }
    if(!obdNotifyChar||!obdWriteChar){obdClient->disconnect();pendingOBD=nullptr;return false;}

    // Subscribe
    if(obdNotifyChar->canIndicate())
        obdNotifyChar->subscribe(false,obdNotifyCB,true);
    else
        obdNotifyChar->subscribe(true,obdNotifyCB);

    // Init AT (sans ATZ !)
    const char* cmds[]={"ATI\r","ATE0\r","ATL0\r","ATSP0\r"};
    for(int c=0;c<4;c++){
        bleHead=bleTail=0;
        obdWriteChar->writeValue((const uint8_t*)cmds[c],strlen(cmds[c]),true);
        delay(1500);
    }
    // Vérifier qu'on a un prompt
    bool hasPrompt=false;
    while(obdBufAvailable()){char c=obdBufRead();if(c=='>')hasPrompt=true;}
    if(!hasPrompt){obdClient->disconnect();pendingOBD=nullptr;return false;}

    obdConnected=true; pendingOBD=nullptr;
    bleHead=bleTail=0;
    return true;
}

// ─── Envoi UART RaceBox → Wroom ──────────────────────────
void sendUARTFrame() {
    UARTFrame f;
    f.sync1=FRAME_SYNC1;f.sync2=FRAME_SYNC2;f.type=FRAME_TYPE_RB;
    f.speedX10=(uint16_t)(rb.speedKmh*10.0f);
    f.gxX1000=(int16_t)(rb.gx*1000.0f);f.gyX1000=(int16_t)(rb.gy*1000.0f);
    f.fix=rb.fix;f.svs=rb.svs;
    uint8_t crc=0;const uint8_t*p=(const uint8_t*)&f;
    for(int i=2;i<FRAME_LEN_RB-1;i++)crc^=p[i];
    f.crc=crc;InterSerial.write((const uint8_t*)&f,FRAME_LEN_RB);
}

// ─── Layout display ──────────────────────────────────────
#define GCX 625
#define GCY 255
#define GCR 145
#define GSCALE 2.5f
static int prevDotX=GCX,prevDotY=GCY;

void drawBackground(){
    gfx->fillScreen(BLACK);
    gfx->fillRect(0,0,800,4,0xF800);gfx->fillRect(0,476,800,4,0xF800);
    gfx->drawFastVLine(455,42,430,0x2945);
    gfx->drawCircle(GCX,GCY,GCR,0x4208);
    gfx->drawCircle(GCX,GCY,(int)(GCR/GSCALE),0x2945);
    gfx->drawFastHLine(GCX-GCR,GCY,GCR*2,0x2945);
    gfx->drawFastVLine(GCX,GCY-GCR,GCR*2,0x2945);
    gfx->setTextColor(0x4208);gfx->setTextSize(1);
    gfx->setCursor(GCX+4,GCY-GCR-12);gfx->print("BRAKE");
    gfx->setCursor(GCX+4,GCY+GCR+4);gfx->print("ACCEL");
    gfx->fillCircle(GCX,GCY,13,0x07E0);
    gfx->setTextColor(0x4208);gfx->setTextSize(2);
    gfx->setCursor(170,400);gfx->print("km/h");
    gfx->drawRect(10,440,780,32,0x2945);
}

#define RPM_MAX 7500
uint16_t prevRpmBar=0;
int16_t prevCoolS3=-999;

void drawRpmBar(){
    if(obdRpm==prevRpmBar)return;prevRpmBar=obdRpm;
    int barW=(int)((float)obdRpm/RPM_MAX*776);if(barW>776)barW=776;
    uint16_t col=obdRpm>6500?0xF800:(obdRpm>5000?0xFD20:0x07E0);
    gfx->fillRect(12,442,barW,28,col);gfx->fillRect(12+barW,442,776-barW,28,BLACK);
    gfx->setTextSize(2);gfx->setTextColor(WHITE);
    char buf[10];snprintf(buf,10,"%d",obdRpm);int w=strlen(buf)*12;
    gfx->setCursor((800-w)/2,446);gfx->print(buf);
}

void drawOBDInfo(){
    if(obdCoolant==prevCoolS3)return;prevCoolS3=obdCoolant;
    gfx->fillRect(460,4,340,38,BLACK);gfx->setTextSize(2);
    uint16_t col=obdCoolant>100?0xF800:(obdCoolant>90?0xFD20:WHITE);
    gfx->setTextColor(col);
    char buf[16];snprintf(buf,16,"EAU %d%cC",obdCoolant,0xF7);
    gfx->setCursor(465,12);gfx->print(buf);
    // OBD indicator
    gfx->setTextColor(obdConnected?(uint16_t)0x07E0:(uint16_t)0xF800);
    gfx->setCursor(700,12);gfx->print(obdConnected?"OBD":"---");
}

bool prevBleOk=false; uint8_t prevFix=255; uint8_t prevSvs=255; float prevVoltage=-1; int prevSpd=-1;

void drawStatusBar(){
    bool changed=(rbConnected!=prevBleOk)||(rb.fix!=prevFix)||(rb.svs!=prevSvs)||(fabsf(rb.voltage-prevVoltage)>=0.05f);
    if(!changed)return;
    gfx->fillRect(0,4,456,38,BLACK);gfx->setTextSize(2);
    gfx->setCursor(8,12);gfx->setTextColor(rbConnected?(uint16_t)0x07E0:(uint16_t)0xF800);
    gfx->print(rbConnected?"RB OK":"RB...");
    gfx->setCursor(140,12);
    const char*fixStr[]={"NO FIX","DEAD-R","  2D  ","  3D  "};
    gfx->setTextColor(rb.fix>=3?(uint16_t)0x07E0:(uint16_t)0xF800);
    gfx->print(rb.fix<=3?fixStr[rb.fix]:"  ??  ");
    char buf[20];snprintf(buf,20," %2dSV %4.1fV",rb.svs,rb.voltage);
    gfx->setTextColor(WHITE);gfx->setCursor(300,12);gfx->print(buf);
    prevBleOk=rbConnected;prevFix=rb.fix;prevSvs=rb.svs;prevVoltage=rb.voltage;
}

void drawScanList(){
    gfx->fillRect(10,50,435,380,BLACK);
    gfx->setTextSize(2);gfx->setTextColor(0xFFE0);gfx->setCursor(10,58);gfx->print("SCAN BLE...");
    gfx->setTextSize(1);gfx->setTextColor(WHITE);
    for(int i=0;i<scanFoundCount;i++){
        gfx->setCursor(10,90+i*20);gfx->print("> ");gfx->print(scanNames[i].c_str());}
    if(scanFoundCount==0){gfx->setTextColor(0x4208);gfx->setCursor(10,90);gfx->print("recherche...");}
}

void drawSpeed(){
    int spd=(int)(rb.speedKmh+0.5f);if(spd<0)spd=0;if(spd==prevSpd)return;prevSpd=spd;
    gfx->fillRect(10,50,435,335,BLACK);
    uint8_t sz;int x;
    if(spd<10){sz=16;x=(455-6*16)/2;}else if(spd<100){sz=16;x=(455-12*16)/2;}else{sz=12;x=(455-18*12)/2;}
    gfx->setTextSize(sz);gfx->setTextColor(WHITE);gfx->setCursor(x>0?x:5,115);gfx->print(spd);
}

void drawGForce(){
    gfx->fillCircle(prevDotX,prevDotY,13,BLACK);
    gfx->drawFastHLine(GCX-GCR,GCY,GCR*2,0x2945);
    gfx->drawFastVLine(GCX,GCY-GCR,GCR*2,0x2945);
    float nx=(rb.gx/GSCALE)*GCR;float ny=(-rb.gy/GSCALE)*GCR;
    float dist=sqrtf(nx*nx+ny*ny);
    if(dist>GCR){float s=GCR/dist;nx*=s;ny*=s;dist=GCR;}
    prevDotX=GCX+(int)nx;prevDotY=GCY+(int)ny;
    float mag=dist/GCR;
    uint16_t col=mag>0.8f?0xF800:(mag>0.4f?0xFD20:0x07E0);
    gfx->fillCircle(prevDotX,prevDotY,13,col);
}

// ─── Setup & Loop ─────────────────────────────────────────
void setup(){
    delay(500);
    InterSerial.begin(UART_BAUD,SERIAL_8N1,INTER_RX,INTER_TX);
    if(!gfx->begin())return;
    drawBackground();drawStatusBar();drawScanList();

    NimBLEDevice::init("GT86-S3");
    auto*scan=NimBLEDevice::getScan();
    scan->setAdvertisedDeviceCallbacks(new ScanCB());
    scan->setInterval(100);scan->setWindow(99);
    scan->setActiveScan(true);scan->setDuplicateFilter(false);
}

uint32_t lastDraw=0,lastUART=0;

void loop(){
    // ── Connexions BLE ────────────────────────────────────
    if(pendingRB && !rbConnected){
        NimBLEDevice::getScan()->stop();
        connectRB();
        drawStatusBar();
        if(rbConnected) drawSpeed();
        if(!rbConnected && !obdConnected) doScan=true;
    }
    if(pendingOBD && !obdConnected){
        NimBLEDevice::getScan()->stop();
        if(connectOBD()){
            drawOBDInfo();
            InterSerial.println("[OBD] Connected!");
        }
        if(!rbConnected || !obdConnected) doScan=true;
    }

    // ── Scan ──────────────────────────────────────────────
    if(doScan){
        doScan=false;
        scanFoundCount=0;scanDirty=true;
        pendingRB=nullptr;pendingOBD=nullptr;
        NimBLEDevice::getScan()->clearResults();
        NimBLEDevice::getScan()->start(30,onScanEnded);
    }
    if(scanDirty && (!rbConnected || !obdConnected)){
        scanDirty=false;drawScanList();
    }

    // ── OBD PID queries ───────────────────────────────────
    processOBD();

    // ── Display ───────────────────────────────────────────
    if(rbFresh && millis()-lastDraw>=40){
        rbFresh=false;lastDraw=millis();
        drawSpeed();drawGForce();drawStatusBar();
    }
    if(obdFresh){
        obdFresh=false;drawRpmBar();drawOBDInfo();
    }

    // ── UART → Wroom ──────────────────────────────────────
    if(rbConnected && millis()-lastUART>=100){
        lastUART=millis();sendUARTFrame();
    }
}
