#include <Arduino.h>
#include <Arduino_GFX_Library.h>
#include <BLEDevice.h>
#include <BLEScan.h>
#include <BLEAdvertisedDevice.h>

// ─── Display GC9A01 (SW SPI C3 Super Mini) ───────────────
Arduino_DataBus *bus = new Arduino_SWSPI(5,7,4,6); // DC,CS,SCK,MOSI
Arduino_GC9A01 *gfx = new Arduino_GC9A01(bus, 3);  // RST

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

// ─── RaceBox (reçu du S3 via UART) ──────────────────────
float rbGx=0,rbGy=0;
uint8_t rxBufU[FRAME_LEN_RB]; uint8_t rxPosU=0;

void processUARTRx(){
    while(Serial1.available()){uint8_t b=Serial1.read();
        if(rxPosU==0){if(b==FRAME_SYNC1)rxBufU[rxPosU++]=b;}
        else if(rxPosU==1){if(b==FRAME_SYNC2)rxBufU[rxPosU++]=b;else rxPosU=0;}
        else{rxBufU[rxPosU++]=b;if(rxPosU==FRAME_LEN_RB){rxPosU=0;uint8_t crc=0;
            for(int i=2;i<FRAME_LEN_RB-1;i++)crc^=rxBufU[i];
            if(crc==rxBufU[FRAME_LEN_RB-1]&&rxBufU[2]==FRAME_TYPE_RB){
                rbGx=(int16_t)(rxBufU[5]|rxBufU[6]<<8)/1000.0f;
                rbGy=(int16_t)(rxBufU[7]|rxBufU[8]<<8)/1000.0f;}}}}}

// ─── Envoi OBD vers S3 ──────────────────────────────────
void sendOBDFrame(){OBDMiniFrame f;f.sync1=FRAME_SYNC1;f.sync2=FRAME_SYNC2;f.type=FRAME_TYPE_OBD;
    f.rpm=obdRpm;f.coolant=obdCoolant;f.mil=obdMIL;f.pad=0;
    uint8_t crc=0;for(int i=2;i<FRAME_LEN_OBD-1;i++)crc^=((uint8_t*)&f)[i];
    f.crc=crc;Serial1.write((const uint8_t*)&f,FRAME_LEN_OBD);}

// ─── Affichage GC9A01 ────────────────────────────────────
#define GCX 120
#define GCY 115
#define GCR 55
#define GSCALE 2.0f
int prevDotX=GCX,prevDotY=GCY;
int prevCool=-999,prevRpm=-1;

void drawGForceGrid(){gfx->drawCircle(GCX,GCY,GCR,0x4208);gfx->drawCircle(GCX,GCY,GCR/2,0x2945);
    gfx->drawFastHLine(GCX-GCR,GCY,GCR*2,0x2945);gfx->drawFastVLine(GCX,GCY-GCR,GCR*2,0x2945);
    gfx->fillCircle(GCX,GCY,7,GREEN);}

void drawGForce(){gfx->fillCircle(prevDotX,prevDotY,7,BLACK);
    gfx->drawFastHLine(GCX-GCR,GCY,GCR*2,0x2945);gfx->drawFastVLine(GCX,GCY-GCR,GCR*2,0x2945);
    float nx=(rbGx/GSCALE)*GCR;float ny=(-rbGy/GSCALE)*GCR;float d=sqrtf(nx*nx+ny*ny);
    if(d>GCR){float s=GCR/d;nx*=s;ny*=s;d=GCR;}
    prevDotX=GCX+(int)nx;prevDotY=GCY+(int)ny;
    gfx->fillCircle(prevDotX,prevDotY,7,d/GCR>0.8f?RED:(d/GCR>0.4f?0xFD20:GREEN));}

void drawOBDValues(){
    if(obdCoolant!=prevCool){prevCool=obdCoolant;gfx->fillRect(30,170,80,18,BLACK);gfx->setTextSize(2);
        gfx->setTextColor(obdCoolant>100?RED:(obdCoolant>90?0xFD20:GREEN));
        char buf[8];snprintf(buf,8,"%d\xF7""C",obdCoolant);gfx->setCursor(35,172);gfx->print(buf);}
    if((int)obdRpm!=prevRpm){prevRpm=obdRpm;gfx->fillRect(130,170,80,18,BLACK);gfx->setTextSize(2);
        gfx->setTextColor(WHITE);char buf[8];snprintf(buf,8,"%d",obdRpm);gfx->setCursor(135,172);gfx->print(buf);}
}

void drawStatus(){
    gfx->fillRect(30,190,180,25,BLACK);gfx->setTextSize(2);
    gfx->setTextColor(obdConnected?GREEN:(uint16_t)0x4208);
    gfx->setCursor(70,194);gfx->print(obdConnected?"OBD":"---");
    bool milOn=obdMIL&0x80;if(milOn){gfx->setTextColor(RED);gfx->setCursor(140,194);gfx->print("MIL");}
}

// ─── Setup ────────────────────────────────────────────────
void setup(){
    Serial1.begin(UART_BAUD,SERIAL_8N1,20,21);
    delay(500);
    if(!gfx->begin())return;
    gfx->fillScreen(BLACK);gfx->drawCircle(120,120,118,0xFD20);
    gfx->setTextSize(2);gfx->setTextColor(WHITE);gfx->setCursor(84,40);gfx->print("GT86");
    drawGForceGrid();
    gfx->setTextSize(2);gfx->setTextColor(0x4208);
    gfx->setCursor(35,172);gfx->print("--\xF7""C");gfx->setCursor(135,172);gfx->print("----");
    drawStatus();
    BLEDevice::init("");
}

// ─── Loop ─────────────────────────────────────────────────
uint32_t lastDraw=0,lastOBDSend=0,lastOBDRetry=0,lastStatus=0;
bool prevObd=false;

void loop(){
    processUARTRx();processOBD();
    // Accéléromètre (données du S3)
    if(millis()-lastDraw>=50){lastDraw=millis();drawGForce();}
    // OBD values
    if(obdFresh){obdFresh=false;drawOBDValues();}
    // Envoi OBD au S3 à 2Hz
    if(obdConnected&&millis()-lastOBDSend>=500){lastOBDSend=millis();sendOBDFrame();}
    // Status
    if(obdConnected!=prevObd){prevObd=obdConnected;drawStatus();}
    // Retry OBD
    if(!obdConnected&&millis()-lastOBDRetry>=15000){lastOBDRetry=millis();
        obdConnected=scanAndConnectOBD();drawStatus();}
}
