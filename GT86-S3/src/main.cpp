#include <Arduino.h>
#include <Arduino_GFX_Library.h>
#include <BLEDevice.h>
#include <BLEScan.h>
#include <BLEAdvertisedDevice.h>

// ─── Display ──────────────────────────────────────────────
Arduino_ESP32RGBPanel *rgbpanel = new Arduino_ESP32RGBPanel(
     5,  3, 46,  7,
     1,  2, 42, 41, 40,
    39,  0, 45, 48, 47, 21,
    14, 38, 18, 17, 10,
    0, 8, 4, 8, 0, 8, 4, 8, 1, 14000000
);
Arduino_RGB_Display *gfx = new Arduino_RGB_Display(800, 480, rgbpanel, 0, true);

// ─── UART ─────────────────────────────────────────────────
HardwareSerial InterSerial(0);
#define FRAME_SYNC1 0xAA
#define FRAME_SYNC2 0x55
#define FRAME_TYPE_RB 0x01
#define FRAME_TYPE_OBD 0x02
#define FRAME_TYPE_LAP 0x03
#define FRAME_LEN_RB 12
#define FRAME_LEN_OBD 10
#define FRAME_LEN_LAP 18
#define FRAME_MAX 18

struct __attribute__((packed)) OBDMiniFrame {
    uint8_t  sync1, sync2, type;
    uint16_t rpm;
    int16_t  coolant;
    uint8_t  mil;
    uint8_t  pad;
    uint8_t  crc;
};

// ─── RaceBox (du Wroom via UART) ─────────────────────────
float rbSpeed=0, rbGx=0, rbGy=0;
uint8_t rbFix=0, rbSvs=0;
bool rbFresh=false;
uint32_t lastRbRx=0;

// ─── Lap data (du Wroom via UART) ────────────────────────
uint32_t lapBestMs=0, lapLastMs=0;
uint8_t  lapNum=0, prevLapNum=255;
bool     lapGateSet=false;
bool     lapFresh=false;
uint32_t lapLocalStart=0;  // timer local S3 pour affichage fluide
bool     lapRunning=false;

uint8_t uartBuf[FRAME_MAX]; uint8_t uartPos=0; int uartExpected=0;

void processUART() {
    while(InterSerial.available()) {
        uint8_t b=InterSerial.read();
        if(uartPos==0){if(b==FRAME_SYNC1)uartBuf[uartPos++]=b;}
        else if(uartPos==1){if(b==FRAME_SYNC2)uartBuf[uartPos++]=b;else uartPos=0;}
        else if(uartPos==2){
            uartBuf[uartPos++]=b;
            if(b==FRAME_TYPE_RB) uartExpected=FRAME_LEN_RB;
            else if(b==FRAME_TYPE_LAP) uartExpected=FRAME_LEN_LAP;
            else uartPos=0;
        } else {
            uartBuf[uartPos++]=b;
            if(uartPos==uartExpected){
                uartPos=0;
                uint8_t crc=0;
                for(int i=2;i<uartExpected-1;i++)crc^=uartBuf[i];
                if(crc!=uartBuf[uartExpected-1]) continue;

                if(uartBuf[2]==FRAME_TYPE_RB){
                    rbSpeed=((uint16_t)(uartBuf[3]|uartBuf[4]<<8))/10.0f;
                    rbGx=(int16_t)(uartBuf[5]|uartBuf[6]<<8)/1000.0f;
                    rbGy=(int16_t)(uartBuf[7]|uartBuf[8]<<8)/1000.0f;
                    rbFix=uartBuf[9];rbSvs=uartBuf[10];
                    rbFresh=true;lastRbRx=millis();
                }
                else if(uartBuf[2]==FRAME_TYPE_LAP){
                    uint32_t rxCurMs=uartBuf[3]|(uint32_t)uartBuf[4]<<8|(uint32_t)uartBuf[5]<<16|(uint32_t)uartBuf[6]<<24;
                    lapBestMs=uartBuf[7]|(uint32_t)uartBuf[8]<<8|(uint32_t)uartBuf[9]<<16|(uint32_t)uartBuf[10]<<24;
                    lapLastMs=uartBuf[11]|(uint32_t)uartBuf[12]<<8|(uint32_t)uartBuf[13]<<16|(uint32_t)uartBuf[14]<<24;
                    lapNum=uartBuf[15];
                    lapGateSet=uartBuf[16]&1;
                    // Sync timer local
                    if (!lapGateSet) { lapRunning=false; }
                    else if (lapNum!=prevLapNum || !lapRunning) {
                        lapLocalStart=millis()-rxCurMs;  // sync avec le Wroom
                        lapRunning=true;
                    }
                    prevLapNum=lapNum;
                    lapFresh=true;
                }
            }
        }
    }
}

// ─── OBD BLE (Bluedroid) ─────────────────────────────────
#define OBD_BLE_NAME "Vlink"
#define BLE_BUF_SIZE 512
static uint8_t bleBuf[BLE_BUF_SIZE];
static volatile int bleHead=0, bleTail=0;
static BLERemoteCharacteristic* obdNotifyChar=nullptr;
static BLERemoteCharacteristic* obdWriteChar=nullptr;
static BLEClient* obdClient=nullptr;
bool obdConnected=false;

static void obdNotifyCB(BLERemoteCharacteristic*, uint8_t* data, size_t len, bool) {
    for(size_t i=0;i<len;i++){int next=(bleHead+1)%BLE_BUF_SIZE;
    if(next!=bleTail){bleBuf[bleHead]=data[i];bleHead=next;}}
}
int obdAvail(){return(bleHead-bleTail+BLE_BUF_SIZE)%BLE_BUF_SIZE;}
int obdRead(){if(bleHead==bleTail)return -1;uint8_t b=bleBuf[bleTail];bleTail=(bleTail+1)%BLE_BUF_SIZE;return b;}

// OBD data
uint16_t obdRpm=0; int16_t obdCoolant=0; uint8_t obdMIL=0;
bool obdFresh=false;

// PID round-robin
struct PIDDef{const char*cmd;uint8_t pid;uint8_t bytes;};
const PIDDef pidList[]={{"010C\r",0x0C,2},{"0105\r",0x05,1},{"010C\r",0x0C,2},{"0101\r",0x01,4}};
#define PID_COUNT 4
int currentPID=0;bool queryPending=false;uint32_t queryStart=0;String obdResp="";

bool parseOBDHex(const String&r,uint8_t pid,uint8_t*d,int c){
    char pf[8];snprintf(pf,8,"41%02X",pid);String cl=r;cl.replace(" ","");
    int idx=cl.indexOf(pf);if(idx<0)return false;idx+=4;
    for(int i=0;i<c;i++){if(idx+2>(int)cl.length())return false;
    d[i]=strtol(cl.substring(idx,idx+2).c_str(),NULL,16);idx+=2;}return true;
}
void applyPIDValue(){uint8_t d[4];uint8_t pid=pidList[currentPID].pid;
    if(!parseOBDHex(obdResp,pid,d,pidList[currentPID].bytes))return;
    switch(pid){case 0x0C:obdRpm=(d[0]*256+d[1])/4;break;case 0x05:obdCoolant=(int16_t)d[0]-40;break;case 0x01:obdMIL=d[0];break;}
    obdFresh=true;}
void processOBD(){if(!obdConnected||!obdWriteChar)return;
    while(obdAvail()){char c=obdRead();
        if(c=='>'){if(queryPending){applyPIDValue();queryPending=false;currentPID=(currentPID+1)%PID_COUNT;}obdResp="";}
        else if(c>=0x20&&c<=0x7E)obdResp+=c;}
    if(!queryPending&&millis()-queryStart>=50){queryStart=millis();bleHead=bleTail=0;obdResp="";
        obdWriteChar->writeValue((uint8_t*)pidList[currentPID].cmd,strlen(pidList[currentPID].cmd));queryPending=true;}
    if(queryPending&&millis()-queryStart>5000){queryPending=false;obdResp="";currentPID=(currentPID+1)%PID_COUNT;}
}

bool scanAndConnectOBD(){
    BLEDevice::deinit(false);delay(300);BLEDevice::init("");
    bleHead=bleTail=0;obdNotifyChar=nullptr;obdWriteChar=nullptr;
    if(obdClient){obdClient->disconnect();delete obdClient;obdClient=nullptr;}
    BLEScan*pScan=BLEDevice::getScan();pScan->setActiveScan(true);
    pScan->setInterval(100);pScan->setWindow(99);
    BLEScanResults results=pScan->start(6,false);
    BLEAdvertisedDevice*target=nullptr;
    for(int i=0;i<results.getCount();i++){BLEAdvertisedDevice dev=results.getDevice(i);
        if(String(dev.getName().c_str()).indexOf(OBD_BLE_NAME)>=0){target=new BLEAdvertisedDevice(dev);break;}}
    pScan->clearResults();if(!target)return false;
    obdClient=BLEDevice::createClient();
    if(!obdClient->connect(target)){delete target;return false;}delete target;
    auto*svcs=obdClient->getServices();
    for(auto it=svcs->begin();it!=svcs->end()&&!obdWriteChar;++it){
        auto*chars=it->second->getCharacteristics();
        BLERemoteCharacteristic*nCh=nullptr,*wCh=nullptr;
        for(auto cit=chars->begin();cit!=chars->end();++cit){auto*ch=cit->second;
            if(ch->canNotify()&&!nCh)nCh=ch;if(ch->canWrite()&&!wCh)wCh=ch;}
        if(nCh&&wCh){obdNotifyChar=nCh;obdWriteChar=wCh;}}
    if(!obdNotifyChar||!obdWriteChar){obdClient->disconnect();return false;}
    obdNotifyChar->registerForNotify(obdNotifyCB);delay(500);
    const char*cmds[]={"ATI\r","ATE0\r","ATL0\r","ATSP0\r"};bool hasPrompt=false;
    for(int c=0;c<4;c++){bleHead=bleTail=0;
        obdWriteChar->writeValue((uint8_t*)cmds[c],strlen(cmds[c]));delay(1500);
        while(obdAvail()){if(obdRead()=='>')hasPrompt=true;}}
    if(!hasPrompt){obdClient->disconnect();return false;}
    bleHead=bleTail=0;return true;
}

// ─── Gear estimation (GT86 FA20 6MT) ─────────────────────
const float gearRatios[]={3.626f,2.188f,1.541f,1.213f,1.000f,0.767f};
const float finalDrive=4.1f, tireCirc=1.943f;
int estimateGear(float spd,uint16_t rpm){
    if(rpm<500||spd<3)return 0;
    float factor=finalDrive*60.0f/(3.6f*tireCirc);
    int best=0;float bestD=99999;
    for(int g=0;g<6;g++){float exp=spd*gearRatios[g]*factor;float d=fabsf(exp-(float)rpm);
        if(d<bestD){bestD=d;best=g+1;}}
    if(bestD>rpm*0.20f)return 0;return best;
}
int currentGear=0,prevGear=-1;

// ─── Layout (positions nettoyées, sans petits labels) ────
#define RPM_MAX 7500
#define RPM_SEGS 68
#define RPM_SEGW 8
#define RPM_SEGH 24
#define RPM_GAP 1
#define RPM_X 8
#define RPM_Y 4

#define GCX 660
#define GCY 340
#define GCR 90
#define GSCALE 2.0f
static int prevDotX=GCX,prevDotY=GCY;

void drawBackground(){
    gfx->fillScreen(BLACK);

    // RPM LED bar (segments éteints)
    for(int i=0;i<RPM_SEGS;i++){
        int x=RPM_X+i*(RPM_SEGW+RPM_GAP);
        gfx->fillRect(x,RPM_Y,RPM_SEGW,RPM_SEGH,0x1082);
    }
    // Shift box
    int shiftX=RPM_X+RPM_SEGS*(RPM_SEGW+RPM_GAP)+4;
    gfx->drawRect(shiftX,RPM_Y,795-shiftX,RPM_SEGH,0x2945);

    // Séparateurs
    gfx->drawFastHLine(0,34,800,0x2945);
    gfx->drawFastVLine(210,34,446,0x2945);
    gfx->drawFastVLine(530,34,446,0x2945);
    gfx->drawFastHLine(0,370,210,0x2945);

    // Cadre gear
    gfx->drawRect(240,44,260,250,0x2945);

    // G-force circle
    gfx->drawCircle(GCX,GCY,GCR,0x4208);
    gfx->drawCircle(GCX,GCY,GCR/2,0x2945);
    gfx->drawFastHLine(GCX-GCR,GCY,GCR*2,0x2945);
    gfx->drawFastVLine(GCX,GCY-GCR,GCR*2,0x2945);
    gfx->fillCircle(GCX,GCY,8,0x07E0);
}

// ─── Affichage différentiel ──────────────────────────────
int prevSpd=-1; uint16_t prevRpmLed=0; int16_t prevCool=-999;

void drawRpmLedBar(){
    if(obdRpm==prevRpmLed)return;prevRpmLed=obdRpm;
    int lit=(int)((float)obdRpm/RPM_MAX*RPM_SEGS);
    for(int i=0;i<RPM_SEGS;i++){
        int x=RPM_X+i*(RPM_SEGW+RPM_GAP);
        uint16_t col;
        if(i<lit){float r=(float)i/RPM_SEGS;
            col=r>0.85f?0xF800:(r>0.65f?0xFD20:(r>0.45f?0xFFE0:0x07E0));}
        else col=0x1082;
        gfx->fillRect(x,RPM_Y,RPM_SEGW,RPM_SEGH,col);}
    bool shift=obdRpm>6800;
    int sx=RPM_X+RPM_SEGS*(RPM_SEGW+RPM_GAP)+4;
    gfx->fillRect(sx,RPM_Y,795-sx,RPM_SEGH,shift?0x001F:BLACK);
    gfx->drawRect(sx,RPM_Y,795-sx,RPM_SEGH,shift?0x001F:0x2945);
    gfx->setTextSize(2);gfx->setTextColor(shift?WHITE:0x2945);
    gfx->setCursor(sx+6,RPM_Y+4);gfx->print("Shift");
}

void drawGear(){
    currentGear=estimateGear(rbSpeed,obdRpm);
    if(currentGear==prevGear)return;prevGear=currentGear;
    gfx->fillRect(242,46,256,246,BLACK);
    gfx->setTextSize(18);
    const char*gs; uint16_t col=WHITE;
    if(currentGear==0){gs="N";col=0x4208;}
    else{static char gb[2];gb[0]='0'+currentGear;gb[1]=0;gs=gb;}
    gfx->setTextColor(col);
    gfx->setCursor(310,70);gfx->print(gs);
}

void drawSpeed(){
    int spd=(int)(rbSpeed+0.5f);if(spd<0)spd=0;
    if(spd==prevSpd)return;prevSpd=spd;
    gfx->fillRect(215,300,310,60,BLACK);
    uint8_t sz=5;int digits=spd<10?1:(spd<100?2:3);
    int numW=digits*6*sz;int totalW=numW+65;
    int startX=210+(320-totalW)/2;
    gfx->setTextSize(sz);gfx->setTextColor(WHITE);
    gfx->setCursor(startX,308);gfx->print(spd);
    gfx->setTextSize(2);gfx->setTextColor(0x4208);
    gfx->setCursor(startX+numW+5,318);gfx->print("km/h");
}

void drawCoolant(){
    if(obdCoolant==prevCool)return;prevCool=obdCoolant;
    gfx->fillRect(535,40,260,70,BLACK);
    uint16_t col=obdCoolant>100?0xF800:(obdCoolant>90?0xFD20:0x07E0);
    if(!obdConnected)col=0x4208;
    gfx->setTextSize(4);gfx->setTextColor(col);
    char buf[12];
    if(obdConnected) snprintf(buf,12,"%d\xF7""C",obdCoolant);
    else snprintf(buf,12,"--\xF7""C");
    gfx->setCursor(570,50);gfx->print(buf);
}

void drawGForce(){
    gfx->fillCircle(prevDotX,prevDotY,8,BLACK);
    gfx->drawFastHLine(GCX-GCR,GCY,GCR*2,0x2945);
    gfx->drawFastVLine(GCX,GCY-GCR,GCR*2,0x2945);
    float nx=(rbGx/GSCALE)*GCR;float ny=(-rbGy/GSCALE)*GCR;
    float dist=sqrtf(nx*nx+ny*ny);
    if(dist>GCR){float s=GCR/dist;nx*=s;ny*=s;dist=GCR;}
    prevDotX=GCX+(int)nx;prevDotY=GCY+(int)ny;
    float mag=dist/GCR;
    gfx->fillCircle(prevDotX,prevDotY,8,mag>0.8f?0xF800:(mag>0.4f?0xFD20:0x07E0));
    // G values
    gfx->fillRect(535,445,260,16,BLACK);
    gfx->setTextSize(1);gfx->setTextColor(WHITE);
    char buf[32];snprintf(buf,32,"G: %.1f LAT | %.1f LONG",fabsf(rbGx),fabsf(rbGy));
    gfx->setCursor(545,448);gfx->print(buf);
}

void drawStatus(){
    gfx->fillRect(215,375,310,30,BLACK);
    gfx->setTextSize(2);
    bool live=(millis()-lastRbRx)<2000;
    gfx->setTextColor(live?(uint16_t)0x07E0:(uint16_t)0xF800);
    gfx->setCursor(230,380);gfx->print(live?"RB":"--");
    gfx->setTextColor(rbFix>=3?(uint16_t)0x07E0:(uint16_t)0xF800);
    gfx->setCursor(290,380);gfx->print(rbFix>=3?"3D":"--");
    gfx->setTextColor(obdConnected?(uint16_t)0x07E0:(uint16_t)0xF800);
    gfx->setCursor(355,380);gfx->print(obdConnected?"OBD":"---");
    // MIL + Batterie
    bool milOn=obdMIL&0x80;
    gfx->setTextColor(milOn?0xF800:0x07E0);
    gfx->setCursor(420,380);
    gfx->print(milOn?"MIL!":"MIL");
}

// ─── Formatage temps mm:ss.cc ─────────────────────────────
void fmtTime(char* buf, int sz, uint32_t ms) {
    if (ms == 0) { snprintf(buf, sz, "--:--.--"); return; }
    int min = ms / 60000;
    int sec = (ms % 60000) / 1000;
    int cs  = (ms % 1000) / 10;
    snprintf(buf, sz, "%d:%02d.%02d", min, sec, cs);
}

uint32_t prevLapCur=0xFFFFFFFF, prevLapBest=0xFFFFFFFF, prevLapLast=0xFFFFFFFF;
int prevDelta=99999;

void drawChrono() {
    // Tour actuel (live, calculé localement)
    gfx->fillRect(5, 42, 200, 55, BLACK);
    gfx->setTextSize(1); gfx->setTextColor(0x4208);
    gfx->setCursor(10, 44); gfx->print(lapGateSet ? "TOUR EN COURS:" : "PAS DE LIGNE");
    char buf[16];
    uint32_t displayMs = lapRunning ? (millis() - lapLocalStart) : 0;
    fmtTime(buf, 16, displayMs);
    gfx->setTextSize(3); gfx->setTextColor(WHITE);
    gfx->setCursor(10, 58); gfx->print(buf);

    // Meilleur tour
    gfx->fillRect(5, 102, 200, 55, BLACK);
    gfx->setTextSize(1); gfx->setTextColor(0x4208);
    gfx->setCursor(10, 104); gfx->print("MEILLEUR:");
    fmtTime(buf, 16, lapBestMs);
    gfx->setTextSize(3); gfx->setTextColor(0x07E0);  // vert
    gfx->setCursor(10, 118); gfx->print(buf);

    // Dernier tour
    gfx->fillRect(5, 162, 200, 55, BLACK);
    gfx->setTextSize(1); gfx->setTextColor(0x4208);
    gfx->setCursor(10, 164); gfx->print("DERNIER:");
    fmtTime(buf, 16, lapLastMs);
    gfx->setTextSize(3); gfx->setTextColor(WHITE);
    gfx->setCursor(10, 178); gfx->print(buf);

    // Delta (dernier - meilleur)
    gfx->fillRect(5, 378, 200, 90, BLACK);
    if (lapLastMs > 0 && lapBestMs > 0) {
        int32_t delta = (int32_t)lapLastMs - (int32_t)lapBestMs;
        gfx->setTextSize(1); gfx->setTextColor(0x4208);
        gfx->setCursor(10, 380); gfx->print("DELTA:");
        gfx->setTextSize(4);
        gfx->setTextColor(delta <= 0 ? 0x07E0 : 0xF800);  // vert=plus rapide, rouge=plus lent
        snprintf(buf, 16, "%s%.2f", delta <= 0 ? "" : "+", delta / 1000.0f);
        gfx->setCursor(10, 400); gfx->print(buf);
    }

    // Tour N°
    gfx->fillRect(5, 230, 200, 30, BLACK);
    gfx->setTextSize(2); gfx->setTextColor(0x4208);
    gfx->setCursor(10, 235);
    gfx->print("TOUR: "); gfx->print(lapNum);
}

void drawChronoPlaceholders() {
    drawChrono();
    gfx->setTextSize(2); gfx->setTextColor(0x4208);
    gfx->setCursor(215, 445); gfx->print("FUEL: --L");
}

// ─── Setup & Loop ─────────────────────────────────────────
void setup(){
    delay(500);
    InterSerial.begin(115200,SERIAL_8N1,44,43);
    if(!gfx->begin())return;
    drawBackground();
    drawChronoPlaceholders();
    drawCoolant();
    drawStatus();
    BLEDevice::init("");
}

uint32_t lastDraw=0,lastStatus=0,lastOBDRetry=0,lastOBDSend=0;
bool prevLive=false,prevObdOk=false;

void sendOBDToWroom() {
    OBDMiniFrame f;
    f.sync1=FRAME_SYNC1; f.sync2=FRAME_SYNC2; f.type=FRAME_TYPE_OBD;
    f.rpm=obdRpm; f.coolant=obdCoolant; f.mil=obdMIL; f.pad=0;
    uint8_t crc=0; for(int i=2;i<FRAME_LEN_OBD-1;i++) crc^=((uint8_t*)&f)[i];
    f.crc=crc;
    InterSerial.write((const uint8_t*)&f, FRAME_LEN_OBD);
}

void loop(){
    processUART();processOBD();
    if(rbFresh&&millis()-lastDraw>=40){rbFresh=false;lastDraw=millis();
        drawGear();drawSpeed();drawGForce();}
    if(lapFresh){lapFresh=false;drawChrono();}
    // Chrono live à 10Hz (indépendant de l'UART)
    static uint32_t lastChronoRefresh=0;
    if(lapRunning&&millis()-lastChronoRefresh>=100){lastChronoRefresh=millis();drawChrono();}
    // Envoyer OBD au Wroom à 2Hz
    if(obdConnected&&millis()-lastOBDSend>=500){lastOBDSend=millis();sendOBDToWroom();}
    if(obdFresh){obdFresh=false;drawRpmLedBar();drawCoolant();
        // MIL check — alerte si voyant moteur allumé
        static bool milShown=false;
        bool milOn=obdMIL&0x80;
        if(milOn&&!milShown){
            gfx->fillRect(150,180,500,120,0xF800);
            gfx->drawRect(150,180,500,120,WHITE);
            gfx->setTextSize(4);gfx->setTextColor(WHITE);
            gfx->setCursor(180,200);gfx->print("CHECK ENGINE");
            gfx->setTextSize(2);
            char buf[20];snprintf(buf,20,"%d defaut(s)",obdMIL&0x7F);
            gfx->setCursor(280,260);gfx->print(buf);
            milShown=true;
        } else if(!milOn&&milShown){
            milShown=false;drawBackground();drawChronoPlaceholders();prevCool=-999;prevRpmLed=0;prevSpd=-1;prevGear=-1;
        }
    }
    if(millis()-lastStatus>=2000){lastStatus=millis();
        bool live=(millis()-lastRbRx)<2000;
        if(live!=prevLive||obdConnected!=prevObdOk){drawStatus();prevLive=live;prevObdOk=obdConnected;}}
    if(!obdConnected&&!lapRunning&&millis()-lastOBDRetry>=30000){lastOBDRetry=millis();
        obdConnected=scanAndConnectOBD();drawStatus();drawCoolant();}
}
