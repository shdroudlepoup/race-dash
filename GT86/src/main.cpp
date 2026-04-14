#include <Arduino.h>
#include <TFT_eSPI.h>
#include <NimBLEDevice.h>

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
#define FRAME_TYPE_LAP 0x03
#define FRAME_LEN_RB   12
#define FRAME_LEN_OBD  10
#define FRAME_LEN_LAP  18
#define FRAME_MAX_RX   12  // max frame we receive (OBD=10)

struct __attribute__((packed)) UARTFrame {
    uint8_t  sync1, sync2, type;
    uint16_t speedX10;
    int16_t  gxX1000, gyX1000;
    uint8_t  fix, svs, crc;
};

struct __attribute__((packed)) LapFrame {
    uint8_t  sync1, sync2, type;
    uint32_t currentMs;
    uint32_t bestMs;
    uint32_t lastMs;
    uint8_t  lapNum;
    uint8_t  flags;   // bit0=gateSet
    uint8_t  crc;
};

// ─── OBD data (reçu du S3 via UART) ──────────────────────
uint16_t rxRpm = 0;
int16_t  rxCoolant = 0;
uint8_t  rxMIL = 0;
bool     rxOBDFresh = false;

// ─── Réception UART du S3 ────────────────────────────────
uint8_t rxBufU[12]; uint8_t rxPosU=0; int rxExpected=0;

void processUARTRx() {
    while (Serial2.available()) {
        uint8_t b = Serial2.read();
        if (rxPosU==0) { if(b==FRAME_SYNC1) rxBufU[rxPosU++]=b; }
        else if (rxPosU==1) { if(b==FRAME_SYNC2) rxBufU[rxPosU++]=b; else rxPosU=0; }
        else if (rxPosU==2) {
            rxBufU[rxPosU++]=b;
            if (b==FRAME_TYPE_OBD) rxExpected=FRAME_LEN_OBD;
            else rxPosU=0;  // on ne reçoit que OBD du S3
        } else {
            rxBufU[rxPosU++]=b;
            if (rxPosU==rxExpected) {
                rxPosU=0;
                uint8_t crc=0;
                for(int i=2;i<rxExpected-1;i++) crc^=rxBufU[i];
                if(crc==rxBufU[rxExpected-1]) {
                    rxRpm = rxBufU[3]|(uint16_t)rxBufU[4]<<8;
                    rxCoolant = (int16_t)(rxBufU[5]|(uint16_t)rxBufU[6]<<8);
                    rxMIL = rxBufU[7];
                    rxOBDFresh = true;
                }
            }
        }
    }
}

// ─── Boutons ──────────────────────────────────────────────
#define BTN_SET   13   // Placer la ligne
#define BTN_RESET 14   // Reset meilleur tour

// ─── RaceBox BLE ──────────────────────────────────────────
#define RB_SN  "3242708879"
#define NUS_SVC "6E400001-B5A3-F393-E0A9-E50E24DCCA9E"
#define NUS_TX  "6E400003-B5A3-F393-E0A9-E50E24DCCA9E"

struct RBData {
    float speedKmh=0, gx=0, gy=0;
    float lat=0, lon=0, heading=0;
    uint8_t fix=0, svs=0;
};

RBData        rb;
volatile bool rbFresh      = false;
volatile bool bleConnected = false;
volatile bool doConnect    = false;
volatile bool doScan       = true;
NimBLEAdvertisedDevice* advDevice = nullptr;

void onRBNotify(NimBLERemoteCharacteristic*, uint8_t* d, size_t len, bool) {
    if (len < 88 || d[0] != 0xB5 || d[1] != 0x62) return;
    if (d[2] != 0xFF || d[3] != 0x01) return;
    uint8_t ca=0, cb=0;
    for (size_t i=2; i<len-2; i++) { ca+=d[i]; cb+=ca; }
    if (ca!=d[len-2] || cb!=d[len-1]) return;
    const uint8_t* p = d + 6;

    rb.speedKmh = (int32_t)(p[48]|(uint32_t)p[49]<<8|(uint32_t)p[50]<<16|(uint32_t)p[51]<<24) * 3.6f / 1000.0f;
    rb.gx  = (int16_t)(p[68]|(uint16_t)p[69]<<8) / 1000.0f;
    rb.gy  = (int16_t)(p[70]|(uint16_t)p[71]<<8) / 1000.0f;
    rb.fix = p[20]; rb.svs = p[23];

    // GPS position + heading pour le chrono
    rb.lon = (int32_t)(p[24]|(uint32_t)p[25]<<8|(uint32_t)p[26]<<16|(uint32_t)p[27]<<24) / 10000000.0f;
    rb.lat = (int32_t)(p[28]|(uint32_t)p[29]<<8|(uint32_t)p[30]<<16|(uint32_t)p[31]<<24) / 10000000.0f;
    rb.heading = (int32_t)(p[52]|(uint32_t)p[53]<<8|(uint32_t)p[54]<<16|(uint32_t)p[55]<<24) / 100000.0f;

    rbFresh = true;
}

// ─── BLE Callbacks ────────────────────────────────────────
class ClientCB : public NimBLEClientCallbacks {
    void onConnect(NimBLEClient*) override { bleConnected = true; }
    void onDisconnect(NimBLEClient*) override { bleConnected = false; doScan = true; }
};
class ScanCB : public NimBLEAdvertisedDeviceCallbacks {
    void onResult(NimBLEAdvertisedDevice* dev) override {
        if (String(dev->getName().c_str()).indexOf(RB_SN) >= 0) {
            NimBLEDevice::getScan()->stop(); advDevice = dev; doConnect = true;
        }
    }
};
void onScanEnded(NimBLEScanResults) { if (!bleConnected) doScan = true; }

bool connectRaceBox() {
    auto* c = NimBLEDevice::createClient();
    c->setClientCallbacks(new ClientCB(), false);
    if (!c->connect(advDevice)) return false;
    auto* svc = c->getService(NUS_SVC);
    if (!svc) { c->disconnect(); return false; }
    auto* ch = svc->getCharacteristic(NUS_TX);
    if (!ch || !ch->canNotify()) { c->disconnect(); return false; }
    return ch->subscribe(true, onRBNotify);
}

// ─── Envoi UART ───────────────────────────────────────────
void sendRBFrame() {
    UARTFrame f;
    f.sync1=FRAME_SYNC1; f.sync2=FRAME_SYNC2; f.type=FRAME_TYPE_RB;
    f.speedX10=(uint16_t)(rb.speedKmh*10.0f);
    f.gxX1000=(int16_t)(rb.gx*1000.0f); f.gyX1000=(int16_t)(rb.gy*1000.0f);
    f.fix=rb.fix; f.svs=rb.svs;
    uint8_t crc=0; for(int i=2;i<FRAME_LEN_RB-1;i++) crc^=((uint8_t*)&f)[i];
    f.crc=crc; Serial2.write((const uint8_t*)&f, FRAME_LEN_RB);
}

// ─── Chrono / Lap Timer ──────────────────────────────────
bool gateSet = false;
float gateLat=0, gateLon=0, gateHeadRad=0;
float prevDist = 0;
uint32_t lapStartMs = 0;
uint32_t bestLapMs = 0, lastLapMs = 0;
uint8_t  lapNum = 0;
uint32_t lastCrossTime = 0;

void showBtnMsg(const char* msg, uint16_t col) {
    tft.fillRect(20, 55, 200, 16, TFT_BLACK);
    tft.setTextSize(1); tft.setTextColor(col, TFT_BLACK);
    int w = strlen(msg) * 6;
    tft.setCursor((240 - w) / 2, 58);
    tft.print(msg);
}

void handleButtons() {
    static uint32_t lastBtn = 0;
    static uint32_t btn1DownTime = 0;
    static bool btn1WasDown = false;

    if (millis() - lastBtn < 200) return;

    bool btn1Down = digitalRead(BTN_SET) == LOW;

    // BTN1 : détection appui court / long
    if (btn1Down && !btn1WasDown) {
        btn1DownTime = millis();  // début appui
    }
    if (btn1Down && btn1WasDown && (millis() - btn1DownTime) > 2000) {
        // Appui long > 2s → arrêter le chrono
        lastBtn = millis();
        gateSet = false;
        showBtnMsg("CHRONO STOP", TFT_RED);
        Serial.println("[LAP] Chrono stopped");
        btn1DownTime = millis() + 10000;  // empêcher re-trigger
    }
    if (!btn1Down && btn1WasDown) {
        uint32_t held = millis() - btn1DownTime;
        if (held < 2000 && held > 50) {
            // Appui court → placer la ligne
            lastBtn = millis();
            gateLat = rb.lat;
            gateLon = rb.lon;
            gateHeadRad = rb.heading * PI / 180.0f;
            gateSet = true;
            lapStartMs = millis();
            lapNum = 0;
            lastLapMs = 0;
            prevDist = 0;
            lastCrossTime = millis();
            if (rb.fix >= 3) showBtnMsg("LIGNE PLACEE!", TFT_GREEN);
            else showBtnMsg("LIGNE (pas GPS)", TFT_ORANGE);
            Serial.printf("[LAP] Gate set! fix=%d\n", rb.fix);
        }
    }
    btn1WasDown = btn1Down;

    // BTN2 : reset best
    if (digitalRead(BTN_RESET) == LOW && millis() - lastBtn > 500) {
        lastBtn = millis();
        bestLapMs = 0;
        lastLapMs = 0;
        lapNum = 0;
        showBtnMsg("BEST RESET!", TFT_YELLOW);
        Serial.println("[LAP] Best reset!");
    }
}

void checkLapCrossing() {
    if (!gateSet || rb.fix < 3 || rb.speedKmh < 20) return;

    // Distance signée le long du heading (perpendiculaire à la ligne)
    float dx = (rb.lon - gateLon) * cosf(gateLat * PI / 180.0f) * 111320.0f;
    float dy = (rb.lat - gateLat) * 111320.0f;
    float dist = dx * sinf(gateHeadRad) + dy * cosf(gateHeadRad);

    // Passage : le signe change (direction positive seulement)
    if (prevDist <= 0 && dist > 0 && (millis() - lastCrossTime) > 10000) {
        uint32_t now = millis();
        uint32_t lapTime = now - lapStartMs;

        if (lapTime > 10000) {  // minimum 10s
            lastLapMs = lapTime;
            lapNum++;
            if (bestLapMs == 0 || lapTime < bestLapMs) bestLapMs = lapTime;
            lapStartMs = now;
            lastCrossTime = now;
            Serial.printf("[LAP] Tour %d: %d.%03ds (best: %d.%03ds)\n",
                          lapNum, lapTime/1000, lapTime%1000, bestLapMs/1000, bestLapMs%1000);
        }
    }
    prevDist = dist;
}

void sendLapFrame() {
    LapFrame f;
    f.sync1=FRAME_SYNC1; f.sync2=FRAME_SYNC2; f.type=FRAME_TYPE_LAP;
    f.currentMs = gateSet ? (millis() - lapStartMs) : 0;
    f.bestMs = bestLapMs;
    f.lastMs = lastLapMs;
    f.lapNum = lapNum;
    f.flags = gateSet ? 1 : 0;
    uint8_t crc=0; for(int i=2;i<FRAME_LEN_LAP-1;i++) crc^=((uint8_t*)&f)[i];
    f.crc=crc; Serial2.write((const uint8_t*)&f, FRAME_LEN_LAP);
}

// ─── Affichage GC9A01 ────────────────────────────────────
#define GCX 120
#define GCY 115
#define GCR 55
#define GSCALE 2.0f
int prevDotX=GCX, prevDotY=GCY;
int prevSpd=-1; bool prevBle=false; bool prevGate=false;

void drawGForceGrid() {
    tft.drawCircle(GCX,GCY,GCR,0x4208);
    tft.drawCircle(GCX,GCY,GCR/2,0x2945);
    tft.drawFastHLine(GCX-GCR,GCY,GCR*2,0x2945);
    tft.drawFastVLine(GCX,GCY-GCR,GCR*2,0x2945);
    tft.fillCircle(GCX,GCY,7,TFT_GREEN);
}

void drawGForce() {
    tft.fillCircle(prevDotX,prevDotY,7,TFT_BLACK);
    tft.drawFastHLine(GCX-GCR,GCY,GCR*2,0x2945);
    tft.drawFastVLine(GCX,GCY-GCR,GCR*2,0x2945);
    float nx=(rb.gx/GSCALE)*GCR; float ny=(-rb.gy/GSCALE)*GCR;
    float dist=sqrtf(nx*nx+ny*ny);
    if(dist>GCR){float s=GCR/dist;nx*=s;ny*=s;dist=GCR;}
    prevDotX=GCX+(int)nx; prevDotY=GCY+(int)ny;
    float mag=dist/GCR;
    tft.fillCircle(prevDotX,prevDotY,7,mag>0.8f?TFT_RED:(mag>0.4f?TFT_ORANGE:TFT_GREEN));
}

void drawButtonDebug() {
    tft.fillRect(30, 30, 180, 20, TFT_BLACK);
    tft.setTextSize(1); tft.setTextColor(TFT_WHITE, TFT_BLACK);
    tft.setCursor(35, 34);
    tft.printf("B1:%d B2:%d B3:%d", digitalRead(BTN_SET), digitalRead(BTN_RESET), digitalRead(27));
}

int prevRxCool = -999;
int prevRxRpm = -1;

bool obdReceived = false;

void drawOBDInfo() {
    obdReceived = true;
    // Temp eau en bas à gauche (dans le cercle)
    if (rxCoolant != prevRxCool) {
        prevRxCool = rxCoolant;
        tft.fillRect(30, 170, 80, 18, TFT_BLACK);
        tft.setTextSize(2);
        uint16_t col = rxCoolant > 100 ? TFT_RED : (rxCoolant > 90 ? TFT_ORANGE : TFT_GREEN);
        tft.setTextColor(col, TFT_BLACK);
        char buf[8]; snprintf(buf, 8, "%d\xF7""C", rxCoolant);
        tft.setCursor(35, 172);
        tft.print(buf);
    }
    // RPM en bas à droite
    if ((int)rxRpm != prevRxRpm) {
        prevRxRpm = rxRpm;
        tft.fillRect(130, 170, 80, 18, TFT_BLACK);
        tft.setTextSize(2);
        tft.setTextColor(TFT_WHITE, TFT_BLACK);
        char buf[8]; snprintf(buf, 8, "%d", rxRpm);
        tft.setCursor(135, 172);
        tft.print(buf);
    }
}

void drawOBDDefaults() {
    tft.setTextSize(2);
    tft.setTextColor(0x4208, TFT_BLACK);
    tft.setCursor(35, 172); tft.print("--\xF7""C");
    tft.setCursor(135, 172); tft.print("----");
}

void drawBottom() {
    tft.fillRect(30,185,180,35,TFT_BLACK);
    int spd=(int)(rb.speedKmh+0.5f);
    tft.setTextSize(2); tft.setTextColor(TFT_WHITE,TFT_BLACK);
    char buf[12]; snprintf(buf,12,"%d",spd);
    tft.setCursor(50,190); tft.print(buf);
    tft.setTextSize(1); tft.setTextColor(0x4208,TFT_BLACK);
    tft.setCursor(50+strlen(buf)*12+2,196); tft.print("km/h");

    tft.setTextSize(2);
    tft.setTextColor(bleConnected?TFT_GREEN:(uint16_t)0x4208,TFT_BLACK);
    tft.setCursor(140,190); tft.print("RB");

    // Gate indicator
    tft.fillCircle(175,197,4,gateSet?TFT_GREEN:TFT_RED);
}

void updateDisplay() {
    drawGForce();
    int spd=(int)(rb.speedKmh+0.5f);
    if(spd!=prevSpd||bleConnected!=prevBle||gateSet!=prevGate) {
        drawBottom(); prevSpd=spd; prevBle=bleConnected; prevGate=gateSet;
    }
}

// ─── Setup ────────────────────────────────────────────────
void setup() {
    Serial.begin(115200);
    Serial2.begin(UART_BAUD, SERIAL_8N1, UART_RX, UART_TX);

    pinMode(BTN_SET, INPUT_PULLUP);
    pinMode(BTN_RESET, INPUT_PULLUP);
    pinMode(27, INPUT_PULLUP);

    tft.begin(); tft.setRotation(0); tft.fillScreen(TFT_BLACK);
    tft.drawCircle(120,120,118,TFT_ORANGE);
    tft.setTextSize(2); tft.setTextColor(TFT_WHITE,TFT_BLACK);
    tft.setCursor(84,40); tft.print("GT86");
    drawGForceGrid();
    drawOBDDefaults();
    drawBottom();

    NimBLEDevice::init("GT86");
    auto* scan = NimBLEDevice::getScan();
    scan->setAdvertisedDeviceCallbacks(new ScanCB());
    scan->setInterval(100); scan->setWindow(99);
    scan->setActiveScan(true); scan->setDuplicateFilter(false);
}

// ─── Loop ─────────────────────────────────────────────────
uint32_t lastDraw=0, lastUART=0, lastLapSend=0;

void loop() {
    if (doConnect) { doConnect=false; if(!connectRaceBox()) doScan=true; }
    if (doScan) { doScan=false; NimBLEDevice::getScan()->clearResults();
        NimBLEDevice::getScan()->start(30, onScanEnded); }

    handleButtons();
    processUARTRx();

    if (rbFresh && millis()-lastDraw>=40) {
        rbFresh=false; lastDraw=millis();
        checkLapCrossing();
        updateDisplay();
    }
    if (rxOBDFresh) { rxOBDFresh=false; drawOBDInfo(); }
    // RaceBox → S3 à 10Hz
    if (bleConnected && millis()-lastUART>=100) {
        lastUART=millis(); sendRBFrame();
    }
    // Lap → S3 à 2Hz
    if (gateSet && millis()-lastLapSend>=500) {
        lastLapSend=millis(); sendLapFrame();
    }
    // Refresh indicateurs + debug boutons
    if (millis()-lastDraw>=2000) { lastDraw=millis(); drawBottom(); }
    static uint32_t lastBtnDbg=0;
    if (millis()-lastBtnDbg>=200) { lastBtnDbg=millis(); drawButtonDebug(); }
}
