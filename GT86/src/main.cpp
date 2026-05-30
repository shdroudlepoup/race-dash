#include <Arduino.h>
#include <Arduino_GFX_Library.h>
#include <BLEDevice.h>
#include <BLEScan.h>
#include <BLEAdvertisedDevice.h>

// ─── Display GC9A01 (SW SPI C3 Super Mini) ───────────────
Arduino_DataBus *bus = new Arduino_SWSPI(5,7,4,6);
Arduino_GC9A01 *gfx = new Arduino_GC9A01(bus, 3);

// ─── UART (Serial1 sur GPIO20/21) ────────────────────────
#define UART_BAUD 115200
#define FRAME_SYNC1 0xAA
#define FRAME_SYNC2 0x55
#define FRAME_TYPE_RB  0x01
#define FRAME_TYPE_OBD 0x02
#define FRAME_LEN_RB  12
#define FRAME_LEN_OBD 10

struct __attribute__((packed)) OBDMiniFrame {
    uint8_t sync1,sync2,type; uint16_t rpm; int16_t coolant; uint8_t mil,pad,crc;
};

// ─── OBD BLE (Bluedroid) ─────────────────────────────────
#define OBD_BLE_NAME "Vlink"
#define BLE_BUF_SIZE 512
static uint8_t bleBuf[BLE_BUF_SIZE];
static volatile int bleHead=0,bleTail=0;
static BLERemoteCharacteristic*obdNotifyChar=nullptr,*obdWriteChar=nullptr;
static BLEClient*obdClient=nullptr;
bool obdConnected=false;

static void obdNotifyCB(BLERemoteCharacteristic*,uint8_t*data,size_t len,bool){
    for(size_t i=0;i<len;i++){int next=(bleHead+1)%BLE_BUF_SIZE;
    if(next!=bleTail){bleBuf[bleHead]=data[i];bleHead=next;}}}
int obdAvail(){return(bleHead-bleTail+BLE_BUF_SIZE)%BLE_BUF_SIZE;}
int obdRead(){if(bleHead==bleTail)return-1;uint8_t b=bleBuf[bleTail];bleTail=(bleTail+1)%BLE_BUF_SIZE;return b;}

uint16_t obdRpm=0; int16_t obdCoolant=0; uint8_t obdMIL=0;
bool obdFresh=false;

struct PIDDef{const char*cmd;uint8_t pid;uint8_t bytes;};
const PIDDef pidList[]={{"010C\r",0x0C,2},{"0105\r",0x05,1},{"010C\r",0x0C,2},{"0101\r",0x01,4}};
#define PID_COUNT 4
int currentPID=0;bool queryPending=false;uint32_t queryStart=0;String obdResp="";

bool parseOBDHex(const String&r,uint8_t pid,uint8_t*d,int c){
    char pf[8];snprintf(pf,8,"41%02X",pid);String cl=r;cl.replace(" ","");
    int idx=cl.indexOf(pf);if(idx<0)return false;idx+=4;
    for(int i=0;i<c;i++){if(idx+2>(int)cl.length())return false;
    d[i]=strtol(cl.substring(idx,idx+2).c_str(),NULL,16);idx+=2;}return true;}
void applyPIDValue(){uint8_t d[4];uint8_t pid=pidList[currentPID].pid;
    if(!parseOBDHex(obdResp,pid,d,pidList[currentPID].bytes))return;
    switch(pid){case 0x0C:obdRpm=(d[0]*256+d[1])/4;break;case 0x05:obdCoolant=(int16_t)d[0]-40;break;
    case 0x01:obdMIL=d[0];break;}obdFresh=true;}
void processOBD(){if(!obdConnected||!obdWriteChar)return;
    while(obdAvail()){char c=obdRead();
        if(c=='>'){if(queryPending){applyPIDValue();queryPending=false;currentPID=(currentPID+1)%PID_COUNT;}obdResp="";}
        else if(c>=0x20&&c<=0x7E)obdResp+=c;}
    if(!queryPending&&millis()-queryStart>=50){queryStart=millis();bleHead=bleTail=0;obdResp="";
        obdWriteChar->writeValue((uint8_t*)pidList[currentPID].cmd,strlen(pidList[currentPID].cmd));queryPending=true;}
    if(queryPending&&millis()-queryStart>5000){queryPending=false;obdResp="";currentPID=(currentPID+1)%PID_COUNT;}}

bool scanAndConnectOBD(){
    BLEDevice::deinit(false);delay(300);BLEDevice::init("");
    bleHead=bleTail=0;obdNotifyChar=nullptr;obdWriteChar=nullptr;
    if(obdClient){obdClient->disconnect();delete obdClient;obdClient=nullptr;}
    BLEScan*pScan=BLEDevice::getScan();pScan->setActiveScan(true);pScan->setInterval(100);pScan->setWindow(99);
    BLEScanResults results=pScan->start(6,false);
    BLEAdvertisedDevice*target=nullptr;
    for(int i=0;i<results.getCount();i++){BLEAdvertisedDevice dev=results.getDevice(i);
        if(String(dev.getName().c_str()).indexOf(OBD_BLE_NAME)>=0){target=new BLEAdvertisedDevice(dev);break;}}
    pScan->clearResults();if(!target)return false;
    obdClient=BLEDevice::createClient();
    if(!obdClient->connect(target)){delete target;return false;}delete target;
    auto*svcs=obdClient->getServices();
    for(auto it=svcs->begin();it!=svcs->end()&&!obdWriteChar;++it){
        auto*chars=it->second->getCharacteristics();BLERemoteCharacteristic*nCh=nullptr,*wCh=nullptr;
        for(auto cit=chars->begin();cit!=chars->end();++cit){auto*ch=cit->second;
            if(ch->canNotify()&&!nCh)nCh=ch;if(ch->canWrite()&&!wCh)wCh=ch;}
        if(nCh&&wCh){obdNotifyChar=nCh;obdWriteChar=wCh;}}
    if(!obdNotifyChar||!obdWriteChar){obdClient->disconnect();return false;}
    obdNotifyChar->registerForNotify(obdNotifyCB);delay(500);
    const char*cmds[]={"ATI\r","ATE0\r","ATL0\r","ATSP0\r"};bool hasPrompt=false;
    for(int c=0;c<4;c++){bleHead=bleTail=0;obdWriteChar->writeValue((uint8_t*)cmds[c],strlen(cmds[c]));delay(1500);
        while(obdAvail()){if(obdRead()=='>')hasPrompt=true;}}
    if(!hasPrompt){obdClient->disconnect();return false;}
    bleHead=bleTail=0;return true;}

// ─── UART RX (RaceBox du S3) — pas utilisé pour l'affichage ─
void processUARTRx(){while(Serial1.available()){Serial1.read();}}

// ─── Envoi OBD vers S3 ──────────────────────────────────
void sendOBDFrame(){OBDMiniFrame f;f.sync1=FRAME_SYNC1;f.sync2=FRAME_SYNC2;f.type=FRAME_TYPE_OBD;
    f.rpm=obdRpm;f.coolant=obdCoolant;f.mil=obdMIL;f.pad=0;
    uint8_t crc=0;for(int i=2;i<FRAME_LEN_OBD-1;i++)crc^=((uint8_t*)&f)[i];
    f.crc=crc;Serial1.write((const uint8_t*)&f,FRAME_LEN_OBD);}

// ─── Affichage manomètre GC9A01 240×240 ─────────────────
// Sur cet écran : 0xFFFF = noir, 0x0000 = blanc (inversé natif + BGR)
#define BG_COLOR 0xFFFF  // noir sur cet écran
#define ARC_OFF  0xE79C  // gris sombre (proche de 0xFFFF = noir)
#define C_WHITE  0x0000  // blanc sur cet écran

#define CX 120
#define CY 120
#define ARC_R    108   // rayon extérieur arc
#define ARC_W    14    // épaisseur arc
#define ARC_START 135  // début (bas-gauche, en degrés)
#define ARC_END   405  // fin (bas-droite = 45°)
#define TEMP_MIN  0
#define TEMP_MAX  130
#define SHIFT_RPM 7000  // GT86 FA20 redline = 7400

// Couleur en fonction de la température
// Couleurs BGR (R et B inversés sur ce GC9A01)
uint16_t tempColor(int temp) {
    if (temp < 60)  return 0xF800;  // bleu (affiché comme bleu en BGR)
    if (temp < 85)  return 0x07E0;  // vert
    if (temp < 95)  return 0x07FF;  // jaune en BGR
    if (temp < 105) return 0x053F;  // orange en BGR
    return 0x001F;                  // rouge en BGR
}

// Dessiner un point épais sur l'arc
void arcDot(int angle, uint16_t col, int r, int thick) {
    float rad = angle * PI / 180.0f;
    int x = CX + (int)(r * cosf(rad));
    int y = CY + (int)(r * sinf(rad));
    gfx->fillCircle(x, y, thick, col);
}

int prevTempAngle = -1;
bool gaugeDrawn = false;

void drawGaugeBackground() {
    // Arc de fond (gris sombre)
    for (int a = ARC_START; a <= ARC_END; a += 2) {
        arcDot(a, ARC_OFF, ARC_R - ARC_W/2, ARC_W/2);
    }
    // Graduations
    gfx->setTextSize(1); gfx->setTextColor(0xBDF7);
    int temps[] = {0, 30, 60, 90, 120};
    for (int i = 0; i < 5; i++) {
        int angle = ARC_START + (int)((float)temps[i] / TEMP_MAX * (ARC_END - ARC_START));
        float rad = angle * PI / 180.0f;
        int x1 = CX + (int)((ARC_R + 2) * cosf(rad));
        int y1 = CY + (int)((ARC_R + 2) * sinf(rad));
        int x2 = CX + (int)((ARC_R - ARC_W - 2) * cosf(rad));
        int y2 = CY + (int)((ARC_R - ARC_W - 2) * sinf(rad));
        gfx->drawLine(x1, y1, x2, y2, 0x4208);
    }
    gaugeDrawn = true;
}

void drawTemperature(int temp) {
    int tempClamped = constrain(temp, TEMP_MIN, TEMP_MAX);
    int targetAngle = ARC_START + (int)((float)tempClamped / TEMP_MAX * (ARC_END - ARC_START));

    if (targetAngle == prevTempAngle) return;

    // Redessiner l'arc coloré
    uint16_t col = tempColor(temp);
    for (int a = ARC_START; a <= ARC_END; a += 2) {
        uint16_t c = (a <= targetAngle) ? col : ARC_OFF;
        arcDot(a, c, ARC_R - ARC_W/2, ARC_W/2);
    }

    // Nombre au centre (gros)
    gfx->fillRect(55, 88, 130, 48, BG_COLOR);
    gfx->setTextSize(6);
    gfx->setTextColor(col);
    char buf[6]; snprintf(buf, 6, "%d", temp);
    int w = strlen(buf) * 36;
    gfx->setCursor((240 - w) / 2, 92);
    gfx->print(buf);

    prevTempAngle = targetAngle;
}

int prevDispRpm = -1;

void drawRPM(int rpm) {
    if (rpm == prevDispRpm) return;
    prevDispRpm = rpm;

    gfx->fillRect(60, 148, 120, 24, BG_COLOR);
    gfx->setTextSize(2);
    gfx->setTextColor(C_WHITE);
    char buf[10]; snprintf(buf, 10, "%d rpm", rpm);
    int w = strlen(buf) * 12;
    gfx->setCursor((240 - w) / 2, 150);
    gfx->print(buf);
}

void drawOBDStatus() {
    gfx->fillRect(60, 190, 120, 20, BG_COLOR);
    gfx->setTextSize(2);
    gfx->setTextColor(obdConnected ? 0x07E0 : 0x4208);  // vert / gris
    gfx->setCursor(75, 192);
    gfx->print(obdConnected ? "OBD OK" : "OBD...");
}

// ─── Shift alert (écran rouge clignotant) ────────────────
bool shiftActive = false;
bool shiftFlashState = false;
uint32_t lastShiftFlash = 0;

void handleShiftAlert() {
    bool shouldShift = obdRpm > SHIFT_RPM && obdConnected;

    if (shouldShift && !shiftActive) {
        shiftActive = true;
        lastShiftFlash = millis();
    }
    if (!shouldShift && shiftActive) {
        shiftActive = false;
        shiftFlashState = false;
        // Redessiner tout
        gfx->fillScreen(BG_COLOR);
        drawGaugeBackground();
        drawOBDStatus();
        prevTempAngle = -1;
        prevDispRpm = -1;
        drawTemperature(obdCoolant);
        drawRPM(obdRpm);
    }

    if (shiftActive && millis() - lastShiftFlash >= 150) {
        lastShiftFlash = millis();
        shiftFlashState = !shiftFlashState;
        gfx->fillScreen(shiftFlashState ? 0x001F : BG_COLOR);  // rouge en BGR
        if (shiftFlashState) {
            gfx->setTextSize(4); gfx->setTextColor(C_WHITE);
            gfx->setCursor(40, 90); gfx->print("SHIFT!");
            gfx->setTextSize(3);
            char buf[10]; snprintf(buf, 10, "%d", obdRpm);
            int w = strlen(buf) * 18;
            gfx->setCursor((240 - w) / 2, 140);
            gfx->print(buf);
        }
    }
}

// ─── Setup ────────────────────────────────────────────────
void setup() {
    Serial1.begin(UART_BAUD, SERIAL_8N1, 20, 21);
    delay(500);
    if (!gfx->begin()) return;
    gfx->fillScreen(BG_COLOR);
    drawGaugeBackground();
    drawTemperature(0);
    drawRPM(0);
    drawOBDStatus();
    BLEDevice::init("");
}

// ─── Loop ─────────────────────────────────────────────────
uint32_t lastOBDSend=0, lastOBDRetry=0;
bool prevObd=false;

void loop() {
    processUARTRx(); processOBD();

    if (obdFresh) {
        obdFresh = false;
        if (!shiftActive) {
            drawTemperature(obdCoolant);
            drawRPM(obdRpm);
        }
    }
    handleShiftAlert();

    if (obdConnected && millis()-lastOBDSend>=500) { lastOBDSend=millis(); sendOBDFrame(); }
    if (obdConnected!=prevObd) { prevObd=obdConnected; if(!shiftActive) drawOBDStatus(); }
    if (!obdConnected && millis()-lastOBDRetry>=15000) { lastOBDRetry=millis();
        obdConnected=scanAndConnectOBD(); drawOBDStatus(); }
}
