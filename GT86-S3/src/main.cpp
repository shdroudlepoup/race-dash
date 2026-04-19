#include <Arduino.h>
#include <Arduino_GFX_Library.h>
#include <NimBLEDevice.h>

// ─── Display Waveshare ESP32-S3-Touch-LCD-7 ───────────────
Arduino_ESP32RGBPanel *rgbpanel = new Arduino_ESP32RGBPanel(
     5,3,46,7, 1,2,42,41,40, 39,0,45,48,47,21, 14,38,18,17,10,
     0,8,4,8,0,8,4,8,1,14000000);
Arduino_RGB_Display *gfx = new Arduino_RGB_Display(800,480,rgbpanel,0,true);

// ─── UART inter-ESP (switch UART2) ───────────────────────
HardwareSerial InterSerial(0);
#define FRAME_SYNC1 0xAA
#define FRAME_SYNC2 0x55
#define FRAME_TYPE_RB  0x01
#define FRAME_TYPE_OBD 0x02
#define FRAME_LEN_RB  12
#define FRAME_LEN_OBD 10
#define FRAME_MAX 12

struct __attribute__((packed)) UARTFrame {
    uint8_t sync1,sync2,type; uint16_t speedX10; int16_t gxX1000,gyX1000; uint8_t fix,svs,crc;
};

// ─── RaceBox BLE (NimBLE) ────────────────────────────────
#define RB_SN "3242708879"
#define NUS_SVC "6E400001-B5A3-F393-E0A9-E50E24DCCA9E"
#define NUS_TX  "6E400003-B5A3-F393-E0A9-E50E24DCCA9E"

struct RBData{float speedKmh=0,gx=0,gy=0,lat=0,lon=0,heading=0,voltage=0;uint8_t fix=0,svs=0;};
RBData rb; volatile bool rbFresh=false,bleConnected=false,doConnect=false,doScan=true;
NimBLEAdvertisedDevice*advDevice=nullptr;

void onRBNotify(NimBLERemoteCharacteristic*,uint8_t*d,size_t len,bool){
    if(len<88||d[0]!=0xB5||d[1]!=0x62||d[2]!=0xFF||d[3]!=0x01)return;
    uint8_t ca=0,cb=0;for(size_t i=2;i<len-2;i++){ca+=d[i];cb+=ca;}
    if(ca!=d[len-2]||cb!=d[len-1])return; const uint8_t*p=d+6;
    rb.speedKmh=(int32_t)(p[48]|(uint32_t)p[49]<<8|(uint32_t)p[50]<<16|(uint32_t)p[51]<<24)*3.6f/1000.0f;
    rb.gx=(int16_t)(p[68]|(uint16_t)p[69]<<8)/1000.0f;
    rb.gy=(int16_t)(p[70]|(uint16_t)p[71]<<8)/1000.0f;
    rb.fix=p[20];rb.svs=p[23];rb.voltage=p[67]/10.0f;
    rb.lon=(int32_t)(p[24]|(uint32_t)p[25]<<8|(uint32_t)p[26]<<16|(uint32_t)p[27]<<24)/10000000.0f;
    rb.lat=(int32_t)(p[28]|(uint32_t)p[29]<<8|(uint32_t)p[30]<<16|(uint32_t)p[31]<<24)/10000000.0f;
    rb.heading=(int32_t)(p[52]|(uint32_t)p[53]<<8|(uint32_t)p[54]<<16|(uint32_t)p[55]<<24)/100000.0f;
    rbFresh=true;}

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

// ─── OBD (reçu du C3 via UART) ──────────────────────────
uint16_t obdRpm=0; int16_t obdCoolant=0; uint8_t obdMIL=0;
bool obdFresh=false; bool obdConnected=false; uint32_t lastObdRx=0;

uint8_t uartBuf[FRAME_MAX]; uint8_t uartPos=0; int uartExpected=0;
void processUART(){
    while(InterSerial.available()){uint8_t b=InterSerial.read();
        if(uartPos==0){if(b==FRAME_SYNC1)uartBuf[uartPos++]=b;}
        else if(uartPos==1){if(b==FRAME_SYNC2)uartBuf[uartPos++]=b;else uartPos=0;}
        else if(uartPos==2){uartBuf[uartPos++]=b;
            if(b==FRAME_TYPE_OBD)uartExpected=FRAME_LEN_OBD;else uartPos=0;}
        else{uartBuf[uartPos++]=b;if(uartPos==uartExpected){uartPos=0;uint8_t crc=0;
            for(int i=2;i<uartExpected-1;i++)crc^=uartBuf[i];
            if(crc==uartBuf[uartExpected-1]&&uartBuf[2]==FRAME_TYPE_OBD){
                obdRpm=uartBuf[3]|(uint16_t)uartBuf[4]<<8;
                obdCoolant=(int16_t)(uartBuf[5]|(uint16_t)uartBuf[6]<<8);
                obdMIL=uartBuf[7];obdFresh=true;obdConnected=true;lastObdRx=millis();}}}}}

// ─── Envoi RaceBox → C3 ─────────────────────────────────
void sendRBFrame(){UARTFrame f;f.sync1=FRAME_SYNC1;f.sync2=FRAME_SYNC2;f.type=FRAME_TYPE_RB;
    f.speedX10=(uint16_t)(rb.speedKmh*10.0f);f.gxX1000=(int16_t)(rb.gx*1000.0f);f.gyX1000=(int16_t)(rb.gy*1000.0f);
    f.fix=rb.fix;f.svs=rb.svs;uint8_t crc=0;for(int i=2;i<FRAME_LEN_RB-1;i++)crc^=((uint8_t*)&f)[i];
    f.crc=crc;InterSerial.write((const uint8_t*)&f,FRAME_LEN_RB);}

// ─── Gear estimation (GT86 FA20 6MT) ─────────────────────
const float gearRatios[]={3.626f,2.188f,1.541f,1.213f,1.000f,0.767f};
const float finalDrive=4.1f,tireCirc=1.943f;
int estimateGear(float spd,uint16_t rpm){if(rpm<500||spd<3)return 0;
    float factor=finalDrive*60.0f/(3.6f*tireCirc);int best=0;float bestD=99999;
    for(int g=0;g<6;g++){float exp=spd*gearRatios[g]*factor;float d=fabsf(exp-(float)rpm);
        if(d<bestD){bestD=d;best=g+1;}}if(bestD>rpm*0.20f)return 0;return best;}
int currentGear=0,prevGear=-1;

// ─── Layout display ──────────────────────────────────────
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

void drawBackground(){gfx->fillScreen(BLACK);
    for(int i=0;i<RPM_SEGS;i++){int x=RPM_X+i*(RPM_SEGW+RPM_GAP);gfx->fillRect(x,RPM_Y,RPM_SEGW,RPM_SEGH,0x1082);}
    int shiftX=RPM_X+RPM_SEGS*(RPM_SEGW+RPM_GAP)+4;gfx->drawRect(shiftX,RPM_Y,795-shiftX,RPM_SEGH,0x2945);
    gfx->drawFastHLine(0,34,800,0x2945);gfx->drawFastVLine(210,34,446,0x2945);gfx->drawFastVLine(530,34,446,0x2945);
    gfx->drawFastHLine(0,370,210,0x2945);gfx->drawRect(240,44,260,250,0x2945);
    gfx->drawCircle(GCX,GCY,GCR,0x4208);gfx->drawCircle(GCX,GCY,GCR/2,0x2945);
    gfx->drawFastHLine(GCX-GCR,GCY,GCR*2,0x2945);gfx->drawFastVLine(GCX,GCY-GCR,GCR*2,0x2945);
    gfx->fillCircle(GCX,GCY,8,0x07E0);}

int prevSpd=-1;uint16_t prevRpmLed=0;int16_t prevCool=-999;

void drawRpmLedBar(){if(obdRpm==prevRpmLed)return;prevRpmLed=obdRpm;
    int lit=(int)((float)obdRpm/RPM_MAX*RPM_SEGS);
    for(int i=0;i<RPM_SEGS;i++){int x=RPM_X+i*(RPM_SEGW+RPM_GAP);uint16_t col;
        if(i<lit){float r=(float)i/RPM_SEGS;col=r>0.85f?0xF800:(r>0.65f?0xFD20:(r>0.45f?0xFFE0:0x07E0));}else col=0x1082;
        gfx->fillRect(x,RPM_Y,RPM_SEGW,RPM_SEGH,col);}
    bool shift=obdRpm>6800;int sx=RPM_X+RPM_SEGS*(RPM_SEGW+RPM_GAP)+4;
    gfx->fillRect(sx,RPM_Y,795-sx,RPM_SEGH,shift?0x001F:BLACK);
    gfx->drawRect(sx,RPM_Y,795-sx,RPM_SEGH,shift?0x001F:0x2945);
    gfx->setTextSize(2);gfx->setTextColor(shift?WHITE:0x2945);gfx->setCursor(sx+6,RPM_Y+4);gfx->print("Shift");}

void drawGear(){currentGear=estimateGear(rb.speedKmh,obdRpm);
    if(currentGear==prevGear)return;prevGear=currentGear;
    gfx->fillRect(242,46,256,246,BLACK);gfx->setTextSize(18);
    const char*gs;uint16_t col=WHITE;
    if(currentGear==0){gs="N";col=0x4208;}else{static char gb[2];gb[0]='0'+currentGear;gb[1]=0;gs=gb;}
    gfx->setTextColor(col);gfx->setCursor(310,70);gfx->print(gs);}

void drawSpeed(){int spd=(int)(rb.speedKmh+0.5f);if(spd<0)spd=0;if(spd==prevSpd)return;prevSpd=spd;
    gfx->fillRect(215,300,310,60,BLACK);uint8_t sz=5;int digits=spd<10?1:(spd<100?2:3);
    int numW=digits*6*sz;int totalW=numW+65;int startX=210+(320-totalW)/2;
    gfx->setTextSize(sz);gfx->setTextColor(WHITE);gfx->setCursor(startX,308);gfx->print(spd);
    gfx->setTextSize(2);gfx->setTextColor(0x4208);gfx->setCursor(startX+numW+5,318);gfx->print("km/h");}

void drawCoolant(){if(obdCoolant==prevCool)return;prevCool=obdCoolant;
    gfx->fillRect(535,40,260,70,BLACK);uint16_t col=obdCoolant>100?0xF800:(obdCoolant>90?0xFD20:0x07E0);
    if(!obdConnected)col=0x4208;gfx->setTextSize(4);gfx->setTextColor(col);
    char buf[12];if(obdConnected)snprintf(buf,12,"%d\xF7""C",obdCoolant);else snprintf(buf,12,"--\xF7""C");
    gfx->setCursor(570,50);gfx->print(buf);}

void drawGForce(){gfx->fillCircle(prevDotX,prevDotY,8,BLACK);
    gfx->drawFastHLine(GCX-GCR,GCY,GCR*2,0x2945);gfx->drawFastVLine(GCX,GCY-GCR,GCR*2,0x2945);
    float nx=(rb.gx/GSCALE)*GCR;float ny=(-rb.gy/GSCALE)*GCR;float dist=sqrtf(nx*nx+ny*ny);
    if(dist>GCR){float s=GCR/dist;nx*=s;ny*=s;dist=GCR;}
    prevDotX=GCX+(int)nx;prevDotY=GCY+(int)ny;float mag=dist/GCR;
    gfx->fillCircle(prevDotX,prevDotY,8,mag>0.8f?0xF800:(mag>0.4f?0xFD20:0x07E0));
    gfx->fillRect(535,445,260,16,BLACK);gfx->setTextSize(1);gfx->setTextColor(WHITE);
    char buf[32];snprintf(buf,32,"G: %.1f LAT | %.1f LONG",fabsf(rb.gx),fabsf(rb.gy));
    gfx->setCursor(545,448);gfx->print(buf);}

void drawStatus(){gfx->fillRect(215,375,310,30,BLACK);gfx->setTextSize(2);
    gfx->setTextColor(bleConnected?(uint16_t)0x07E0:(uint16_t)0xF800);gfx->setCursor(230,380);gfx->print(bleConnected?"RB":"--");
    gfx->setTextColor(rb.fix>=3?(uint16_t)0x07E0:(uint16_t)0xF800);gfx->setCursor(290,380);gfx->print(rb.fix>=3?"3D":"--");
    gfx->setTextColor(obdConnected?(uint16_t)0x07E0:(uint16_t)0xF800);gfx->setCursor(355,380);gfx->print(obdConnected?"OBD":"---");
    bool milOn=obdMIL&0x80;gfx->setTextColor(milOn?0xF800:0x07E0);gfx->setCursor(420,380);gfx->print(milOn?"MIL!":"MIL");}

void drawPlaceholders(){gfx->setTextSize(2);gfx->setTextColor(WHITE);
    gfx->setCursor(15,50);gfx->print("--:--.--");gfx->setCursor(15,100);gfx->print("--:--.--");
    gfx->setTextSize(4);gfx->setTextColor(0x4208);gfx->setCursor(15,390);gfx->print("+0.00");}

// ─── Setup & Loop ─────────────────────────────────────────
void setup(){delay(500);InterSerial.begin(115200,SERIAL_8N1,44,43);
    if(!gfx->begin())return;drawBackground();drawPlaceholders();drawCoolant();drawStatus();
    NimBLEDevice::init("GT86-S3");auto*scan=NimBLEDevice::getScan();
    scan->setAdvertisedDeviceCallbacks(new ScanCB());scan->setInterval(100);scan->setWindow(99);
    scan->setActiveScan(true);scan->setDuplicateFilter(false);}

uint32_t lastDraw=0,lastUART=0,lastStatus=0;
bool prevBleOk=false,prevObdOk=false;

void loop(){
    if(doConnect){doConnect=false;if(!connectRaceBox())doScan=true;drawStatus();}
    if(doScan){doScan=false;NimBLEDevice::getScan()->clearResults();NimBLEDevice::getScan()->start(30,onScanEnded);}
    processUART();
    if(rbFresh&&millis()-lastDraw>=40){rbFresh=false;lastDraw=millis();drawGear();drawSpeed();drawGForce();}
    if(obdFresh){obdFresh=false;drawRpmLedBar();drawCoolant();}
    if(bleConnected&&millis()-lastUART>=100){lastUART=millis();sendRBFrame();}
    // OBD timeout
    if(obdConnected&&millis()-lastObdRx>5000){obdConnected=false;}
    if(millis()-lastStatus>=2000){lastStatus=millis();
        if(bleConnected!=prevBleOk||obdConnected!=prevObdOk){drawStatus();prevBleOk=bleConnected;prevObdOk=obdConnected;}}
}
