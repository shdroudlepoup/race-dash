#include <Arduino.h>
#include <Arduino_GFX_Library.h>
#include <NimBLEDevice.h>
#include <Preferences.h>

// ─── Display GC9A01 (SW SPI sur C3 Super Mini) ───────────
Arduino_DataBus *bus = new Arduino_SWSPI(5 /* DC */, 7 /* CS */, 4 /* SCK */, 6 /* MOSI */);
Arduino_GC9A01 *gfx = new Arduino_GC9A01(bus, 3 /* RST */);
Preferences prefs;

// ─── UART inter-ESP (Serial = UART0, GPIO20 RX / GPIO21 TX) ─
#define UART_BAUD 115200
#define FRAME_SYNC1    0xAA
#define FRAME_SYNC2    0x55
#define FRAME_TYPE_RB  0x01
#define FRAME_TYPE_OBD 0x02
#define FRAME_TYPE_LAP 0x03
#define FRAME_LEN_RB   12
#define FRAME_LEN_OBD  10
#define FRAME_LEN_LAP  18

struct __attribute__((packed)) UARTFrame {
    uint8_t sync1,sync2,type; uint16_t speedX10; int16_t gxX1000,gyX1000; uint8_t fix,svs,crc;
};
struct __attribute__((packed)) LapFrame {
    uint8_t sync1,sync2,type; uint32_t currentMs,bestMs,lastMs; uint8_t lapNum,flags,crc;
};

// ─── OBD (reçu du S3) ────────────────────────────────────
uint16_t rxRpm=0; int16_t rxCoolant=0; uint8_t rxMIL=0; bool rxOBDFresh=false;

uint8_t rxBufU[12]; uint8_t rxPosU=0; int rxExpected=0;
void processUARTRx() {
    while(Serial.available()){uint8_t b=Serial.read();
        if(rxPosU==0){if(b==FRAME_SYNC1)rxBufU[rxPosU++]=b;}
        else if(rxPosU==1){if(b==FRAME_SYNC2)rxBufU[rxPosU++]=b;else rxPosU=0;}
        else if(rxPosU==2){rxBufU[rxPosU++]=b;if(b==FRAME_TYPE_OBD)rxExpected=FRAME_LEN_OBD;else rxPosU=0;}
        else{rxBufU[rxPosU++]=b;if(rxPosU==rxExpected){rxPosU=0;uint8_t crc=0;
            for(int i=2;i<rxExpected-1;i++)crc^=rxBufU[i];
            if(crc==rxBufU[rxExpected-1]){rxRpm=rxBufU[3]|(uint16_t)rxBufU[4]<<8;
                rxCoolant=(int16_t)(rxBufU[5]|(uint16_t)rxBufU[6]<<8);rxMIL=rxBufU[7];rxOBDFresh=true;}}}}
}

// ─── Boutons ──────────────────────────────────────────────
#define BTN_SET   0
#define BTN_RESET 1

// ─── RaceBox BLE ──────────────────────────────────────────
#define RB_SN "3242708879"
#define NUS_SVC "6E400001-B5A3-F393-E0A9-E50E24DCCA9E"
#define NUS_TX  "6E400003-B5A3-F393-E0A9-E50E24DCCA9E"

struct RBData { float speedKmh=0,gx=0,gy=0,lat=0,lon=0,heading=0; uint8_t fix=0,svs=0; };
RBData rb; volatile bool rbFresh=false,bleConnected=false,doConnect=false,doScan=true;
NimBLEAdvertisedDevice* advDevice=nullptr;

void onRBNotify(NimBLERemoteCharacteristic*,uint8_t*d,size_t len,bool){
    if(len<88||d[0]!=0xB5||d[1]!=0x62||d[2]!=0xFF||d[3]!=0x01)return;
    uint8_t ca=0,cb=0;for(size_t i=2;i<len-2;i++){ca+=d[i];cb+=ca;}
    if(ca!=d[len-2]||cb!=d[len-1])return;
    const uint8_t*p=d+6;
    rb.speedKmh=(int32_t)(p[48]|(uint32_t)p[49]<<8|(uint32_t)p[50]<<16|(uint32_t)p[51]<<24)*3.6f/1000.0f;
    rb.gx=(int16_t)(p[68]|(uint16_t)p[69]<<8)/1000.0f;
    rb.gy=(int16_t)(p[70]|(uint16_t)p[71]<<8)/1000.0f;
    rb.fix=p[20];rb.svs=p[23];
    rb.lon=(int32_t)(p[24]|(uint32_t)p[25]<<8|(uint32_t)p[26]<<16|(uint32_t)p[27]<<24)/10000000.0f;
    rb.lat=(int32_t)(p[28]|(uint32_t)p[29]<<8|(uint32_t)p[30]<<16|(uint32_t)p[31]<<24)/10000000.0f;
    rb.heading=(int32_t)(p[52]|(uint32_t)p[53]<<8|(uint32_t)p[54]<<16|(uint32_t)p[55]<<24)/100000.0f;
    rbFresh=true;
}

class ClientCB:public NimBLEClientCallbacks{
    void onConnect(NimBLEClient*)override{bleConnected=true;}
    void onDisconnect(NimBLEClient*)override{bleConnected=false;doScan=true;}};
class ScanCB:public NimBLEAdvertisedDeviceCallbacks{
    void onResult(NimBLEAdvertisedDevice*dev)override{
        if(String(dev->getName().c_str()).indexOf(RB_SN)>=0){NimBLEDevice::getScan()->stop();advDevice=dev;doConnect=true;}}};
void onScanEnded(NimBLEScanResults){if(!bleConnected)doScan=true;}

bool connectRaceBox(){auto*c=NimBLEDevice::createClient();c->setClientCallbacks(new ClientCB(),false);
    if(!c->connect(advDevice))return false;auto*svc=c->getService(NUS_SVC);if(!svc){c->disconnect();return false;}
    auto*ch=svc->getCharacteristic(NUS_TX);if(!ch||!ch->canNotify()){c->disconnect();return false;}
    return ch->subscribe(true,onRBNotify);}

// ─── Envoi UART ───────────────────────────────────────────
void sendRBFrame(){UARTFrame f;f.sync1=FRAME_SYNC1;f.sync2=FRAME_SYNC2;f.type=FRAME_TYPE_RB;
    f.speedX10=(uint16_t)(rb.speedKmh*10.0f);f.gxX1000=(int16_t)(rb.gx*1000.0f);f.gyX1000=(int16_t)(rb.gy*1000.0f);
    f.fix=rb.fix;f.svs=rb.svs;uint8_t crc=0;for(int i=2;i<FRAME_LEN_RB-1;i++)crc^=((uint8_t*)&f)[i];
    f.crc=crc;Serial.write((const uint8_t*)&f,FRAME_LEN_RB);}

// ─── Chrono ───────────────────────────────────────────────
bool gateSet=false; float gateLat=0,gateLon=0,gateHeadRad=0; float prevDistG=0;
uint32_t lapStartMs=0,bestLapMs=0,lastLapMs=0; uint8_t lapNum=0; uint32_t lastCrossTime=0;

void saveBestToNVS(){prefs.begin("lap",false);prefs.putUInt("best",bestLapMs);prefs.end();}
void saveGateToNVS(){prefs.begin("lap",false);prefs.putFloat("glat",gateLat);prefs.putFloat("glon",gateLon);
    prefs.putFloat("ghrd",gateHeadRad);prefs.putBool("gset",true);prefs.end();}
void loadFromNVS(){prefs.begin("lap",true);bestLapMs=prefs.getUInt("best",0);gateLat=prefs.getFloat("glat",0);
    gateLon=prefs.getFloat("glon",0);gateHeadRad=prefs.getFloat("ghrd",0);gateSet=prefs.getBool("gset",false);prefs.end();}

void sendLapFrame();

void showBtnMsg(const char*msg,uint16_t col){
    gfx->fillRect(20,55,200,16,BLACK);gfx->setTextSize(1);gfx->setTextColor(col);
    int w=strlen(msg)*6;gfx->setCursor((240-w)/2,58);gfx->print(msg);}

void handleButtons(){
    static uint32_t lastBtn=0;static uint32_t btn1DownTime=0;static bool btn1WasDown=false;
    if(millis()-lastBtn<200)return;
    bool btn1Down=digitalRead(BTN_SET)==LOW;
    if(btn1Down&&!btn1WasDown)btn1DownTime=millis();
    if(btn1Down&&btn1WasDown&&(millis()-btn1DownTime)>2000){lastBtn=millis();gateSet=false;sendLapFrame();
        showBtnMsg("CHRONO STOP",RED);btn1DownTime=millis()+10000;}
    if(!btn1Down&&btn1WasDown){uint32_t held=millis()-btn1DownTime;
        if(held<2000&&held>50){lastBtn=millis();gateLat=rb.lat;gateLon=rb.lon;gateHeadRad=rb.heading*PI/180.0f;
            gateSet=true;lapStartMs=millis();lapNum=0;lastLapMs=0;prevDistG=0;lastCrossTime=millis();
            saveGateToNVS();sendLapFrame();
            if(rb.fix>=3)showBtnMsg("LIGNE SAUVEE!",GREEN);else showBtnMsg("LIGNE (pas GPS)",0xFD20);}}
    btn1WasDown=btn1Down;
    if(digitalRead(BTN_RESET)==LOW&&millis()-lastBtn>500){lastBtn=millis();bestLapMs=0;lastLapMs=0;lapNum=0;
        saveBestToNVS();showBtnMsg("BEST RESET!",YELLOW);}
}

#define GATE_RADIUS 8.0f
void checkLapCrossing(){if(!gateSet||rb.fix<3||rb.speedKmh<20)return;
    float dx=(rb.lon-gateLon)*cosf(gateLat*PI/180.0f)*111320.0f;float dy=(rb.lat-gateLat)*111320.0f;
    float dist=dx*sinf(gateHeadRad)+dy*cosf(gateHeadRad);
    float lateral=-dx*cosf(gateHeadRad)+dy*sinf(gateHeadRad);
    if(prevDistG<=0&&dist>0&&fabsf(lateral)<GATE_RADIUS&&(millis()-lastCrossTime)>10000){
        uint32_t now=millis();uint32_t lapTime=now-lapStartMs;
        if(lapTime>10000){lastLapMs=lapTime;lapNum++;
            if(bestLapMs==0||lapTime<bestLapMs){bestLapMs=lapTime;saveBestToNVS();}
            lapStartMs=now;lastCrossTime=now;}}
    prevDistG=dist;}

void sendLapFrame(){LapFrame f;f.sync1=FRAME_SYNC1;f.sync2=FRAME_SYNC2;f.type=FRAME_TYPE_LAP;
    f.currentMs=gateSet?(millis()-lapStartMs):0;f.bestMs=bestLapMs;f.lastMs=lastLapMs;f.lapNum=lapNum;
    f.flags=gateSet?1:0;uint8_t crc=0;for(int i=2;i<FRAME_LEN_LAP-1;i++)crc^=((uint8_t*)&f)[i];
    f.crc=crc;Serial.write((const uint8_t*)&f,FRAME_LEN_LAP);}

// ─── Affichage GC9A01 ────────────────────────────────────
#define GCX 120
#define GCY 115
#define GCR 55
#define GSCALE 2.0f
int prevDotX=GCX,prevDotY=GCY;int prevSpd=-1;bool prevBle=false;bool prevGateD=false;
int prevRxCool=-999;int prevRxRpm=-1;

void drawGForceGrid(){gfx->drawCircle(GCX,GCY,GCR,0x4208);gfx->drawCircle(GCX,GCY,GCR/2,0x2945);
    gfx->drawFastHLine(GCX-GCR,GCY,GCR*2,0x2945);gfx->drawFastVLine(GCX,GCY-GCR,GCR*2,0x2945);
    gfx->fillCircle(GCX,GCY,7,GREEN);}

void drawGForce(){gfx->fillCircle(prevDotX,prevDotY,7,BLACK);
    gfx->drawFastHLine(GCX-GCR,GCY,GCR*2,0x2945);gfx->drawFastVLine(GCX,GCY-GCR,GCR*2,0x2945);
    float nx=(rb.gx/GSCALE)*GCR;float ny=(-rb.gy/GSCALE)*GCR;float d=sqrtf(nx*nx+ny*ny);
    if(d>GCR){float s=GCR/d;nx*=s;ny*=s;d=GCR;}
    prevDotX=GCX+(int)nx;prevDotY=GCY+(int)ny;float mag=d/GCR;
    uint16_t col=mag>0.8f?RED:(mag>0.4f?0xFD20:GREEN);gfx->fillCircle(prevDotX,prevDotY,7,col);}

void drawOBDInfo(){
    if(rxCoolant!=prevRxCool){prevRxCool=rxCoolant;gfx->fillRect(30,170,80,18,BLACK);
        gfx->setTextSize(2);uint16_t col=rxCoolant>100?RED:(rxCoolant>90?0xFD20:GREEN);
        gfx->setTextColor(col);char buf[8];snprintf(buf,8,"%d\xF7""C",rxCoolant);gfx->setCursor(35,172);gfx->print(buf);}
    if((int)rxRpm!=prevRxRpm){prevRxRpm=rxRpm;gfx->fillRect(130,170,80,18,BLACK);
        gfx->setTextSize(2);gfx->setTextColor(WHITE);char buf[8];snprintf(buf,8,"%d",rxRpm);gfx->setCursor(135,172);gfx->print(buf);}
}

void drawOBDDefaults(){gfx->setTextSize(2);gfx->setTextColor(0x4208);
    gfx->setCursor(35,172);gfx->print("--\xF7""C");gfx->setCursor(135,172);gfx->print("----");}

void drawBottom(){gfx->fillRect(30,185,180,35,BLACK);int spd=(int)(rb.speedKmh+0.5f);
    gfx->setTextSize(2);gfx->setTextColor(WHITE);char buf[12];snprintf(buf,12,"%d",spd);
    gfx->setCursor(50,190);gfx->print(buf);gfx->setTextSize(1);gfx->setTextColor(0x4208);
    gfx->setCursor(50+strlen(buf)*12+2,196);gfx->print("km/h");
    gfx->setTextSize(2);gfx->setTextColor(bleConnected?GREEN:(uint16_t)0x4208);
    gfx->setCursor(140,190);gfx->print("RB");
    gfx->fillCircle(175,197,4,gateSet?GREEN:RED);}

void updateDisplay(){drawGForce();int spd=(int)(rb.speedKmh+0.5f);
    if(spd!=prevSpd||bleConnected!=prevBle||gateSet!=prevGateD){drawBottom();prevSpd=spd;prevBle=bleConnected;prevGateD=gateSet;}}

// ─── Setup ────────────────────────────────────────────────
void setup(){
    Serial.begin(UART_BAUD);
    pinMode(BTN_SET,INPUT_PULLUP);pinMode(BTN_RESET,INPUT_PULLUP);pinMode(10,INPUT_PULLUP);

    delay(500);
    if(!gfx->begin())return;
    gfx->fillScreen(BLACK);gfx->drawCircle(120,120,118,0xFD20);
    gfx->setTextSize(2);gfx->setTextColor(WHITE);gfx->setCursor(84,40);gfx->print("GT86");
    drawGForceGrid();drawOBDDefaults();drawBottom();

    loadFromNVS();
    if(gateSet)showBtnMsg("LIGNE CHARGEE",GREEN);

    NimBLEDevice::init("GT86");auto*scan=NimBLEDevice::getScan();
    scan->setAdvertisedDeviceCallbacks(new ScanCB());scan->setInterval(100);scan->setWindow(99);
    scan->setActiveScan(true);scan->setDuplicateFilter(false);
}

// ─── Loop ─────────────────────────────────────────────────
uint32_t lastDraw=0,lastUART=0,lastLapSend=0;
void loop(){
    if(doConnect){doConnect=false;if(!connectRaceBox())doScan=true;}
    if(doScan){doScan=false;NimBLEDevice::getScan()->clearResults();NimBLEDevice::getScan()->start(30,onScanEnded);}
    handleButtons();processUARTRx();
    if(rbFresh&&millis()-lastDraw>=40){rbFresh=false;lastDraw=millis();checkLapCrossing();updateDisplay();}
    if(rxOBDFresh){rxOBDFresh=false;drawOBDInfo();}
    if(bleConnected&&millis()-lastUART>=100){lastUART=millis();sendRBFrame();}
    if(gateSet&&millis()-lastLapSend>=500){lastLapSend=millis();sendLapFrame();}
    if(millis()-lastDraw>=2000){lastDraw=millis();drawBottom();}
}
