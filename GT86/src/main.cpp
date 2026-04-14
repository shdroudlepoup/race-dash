#include <Arduino.h>
#include <TFT_eSPI.h>
#include <NimBLEDevice.h>

TFT_eSPI tft = TFT_eSPI();

// ─── UART inter-ESP ───────────────────────────────────────
#define UART_BAUD 115200
#define UART_RX   25
#define UART_TX   26

// ─── Protocole trame UART (RaceBox → S3) ─────────────────
#define FRAME_SYNC1   0xAA
#define FRAME_SYNC2   0x55
#define FRAME_TYPE_RB 0x01
#define FRAME_LEN_RB  12

struct __attribute__((packed)) UARTFrame {
    uint8_t  sync1, sync2, type;
    uint16_t speedX10;
    int16_t  gxX1000, gyX1000;
    uint8_t  fix, svs, crc;
};

// ─── RaceBox BLE ──────────────────────────────────────────
#define RB_SN  "3242708879"
#define NUS_SVC "6E400001-B5A3-F393-E0A9-E50E24DCCA9E"
#define NUS_TX  "6E400003-B5A3-F393-E0A9-E50E24DCCA9E"

struct RBData {
    float speedKmh = 0, gx = 0, gy = 0;
    uint8_t fix = 0, svs = 0;
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
    uint8_t ca = 0, cb = 0;
    for (size_t i = 2; i < len - 2; i++) { ca += d[i]; cb += ca; }
    if (ca != d[len-2] || cb != d[len-1]) return;
    const uint8_t* p = d + 6;
    rb.speedKmh = (int32_t)(p[48]|(uint32_t)p[49]<<8|(uint32_t)p[50]<<16|(uint32_t)p[51]<<24) * 3.6f / 1000.0f;
    rb.gx  = (int16_t)(p[68]|(uint16_t)p[69]<<8) / 1000.0f;
    rb.gy  = (int16_t)(p[70]|(uint16_t)p[71]<<8) / 1000.0f;
    rb.fix = p[20]; rb.svs = p[23];
    rbFresh = true;
}

// ─── BLE Callbacks ────────────────────────────────────────
class ClientCB : public NimBLEClientCallbacks {
    void onConnect(NimBLEClient*) override { bleConnected = true; }
    void onDisconnect(NimBLEClient*) override { bleConnected = false; doScan = true; }
};

class ScanCB : public NimBLEAdvertisedDeviceCallbacks {
    void onResult(NimBLEAdvertisedDevice* dev) override {
        String name = String(dev->getName().c_str());
        if (name.indexOf(RB_SN) >= 0) {
            NimBLEDevice::getScan()->stop();
            advDevice = dev;
            doConnect = true;
        }
    }
};

void onScanEnded(NimBLEScanResults) {
    if (!bleConnected) doScan = true;
}

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

// ─── Envoi UART RaceBox → S3 ─────────────────────────────
void sendUARTFrame() {
    UARTFrame f;
    f.sync1 = FRAME_SYNC1; f.sync2 = FRAME_SYNC2; f.type = FRAME_TYPE_RB;
    f.speedX10 = (uint16_t)(rb.speedKmh * 10.0f);
    f.gxX1000  = (int16_t)(rb.gx * 1000.0f);
    f.gyX1000  = (int16_t)(rb.gy * 1000.0f);
    f.fix = rb.fix; f.svs = rb.svs;
    uint8_t crc = 0;
    const uint8_t* p = (const uint8_t*)&f;
    for (int i = 2; i < FRAME_LEN_RB - 1; i++) crc ^= p[i];
    f.crc = crc;
    Serial2.write((const uint8_t*)&f, FRAME_LEN_RB);
}

// ─── Affichage GC9A01 240×240 ─────────────────────────────
// G-force au centre, vitesse en bas

#define GCX    120    // centre cercle G
#define GCY    115    // légèrement au-dessus du centre géo
#define GCR    55     // rayon cercle G
#define GSCALE 2.0f   // G max = bord

int prevDotX = GCX, prevDotY = GCY;
int prevSpd = -1;
bool prevBle = false;

void drawGForceGrid() {
    // Cercle extérieur + croix
    tft.drawCircle(GCX, GCY, GCR, 0x4208);
    tft.drawCircle(GCX, GCY, GCR / 2, 0x2945);  // anneau 1G
    tft.drawFastHLine(GCX - GCR, GCY, GCR * 2, 0x2945);
    tft.drawFastVLine(GCX, GCY - GCR, GCR * 2, 0x2945);
    // Point initial au centre
    tft.fillCircle(GCX, GCY, 7, TFT_GREEN);
}

void drawGForce() {
    // Effacer ancien point
    tft.fillCircle(prevDotX, prevDotY, 7, TFT_BLACK);
    // Restaurer grille si le point l'a recouverte
    tft.drawFastHLine(GCX - GCR, GCY, GCR * 2, 0x2945);
    tft.drawFastVLine(GCX, GCY - GCR, GCR * 2, 0x2945);

    float nx = (rb.gx / GSCALE) * GCR;
    float ny = (-rb.gy / GSCALE) * GCR;
    float dist = sqrtf(nx * nx + ny * ny);
    if (dist > GCR) { float s = GCR / dist; nx *= s; ny *= s; dist = GCR; }

    prevDotX = GCX + (int)nx;
    prevDotY = GCY + (int)ny;

    float mag = dist / GCR;
    uint16_t col = mag > 0.8f ? TFT_RED : (mag > 0.4f ? TFT_ORANGE : TFT_GREEN);
    tft.fillCircle(prevDotX, prevDotY, 7, col);
}

void drawBottom() {
    tft.fillRect(30, 185, 180, 35, TFT_BLACK);

    // Vitesse à gauche
    int spd = (int)(rb.speedKmh + 0.5f);
    tft.setTextSize(2);
    tft.setTextColor(TFT_WHITE, TFT_BLACK);
    char buf[12];
    snprintf(buf, 12, "%d", spd);
    tft.setCursor(50, 190);
    tft.print(buf);

    tft.setTextSize(1);
    tft.setTextColor(0x4208, TFT_BLACK);
    tft.setCursor(50 + strlen(buf) * 12 + 2, 196);
    tft.print("km/h");

    // RB à droite
    tft.setTextSize(2);
    tft.setTextColor(bleConnected ? TFT_GREEN : (uint16_t)0x4208, TFT_BLACK);
    tft.setCursor(155, 190);
    tft.print("RB");

    // GPS fix
    tft.fillCircle(145, 197, 4, rb.fix >= 3 ? TFT_GREEN : TFT_RED);
}

void updateDisplay() {
    drawGForce();

    int spd = (int)(rb.speedKmh + 0.5f);
    if (spd != prevSpd || bleConnected != prevBle) {
        drawBottom();
        prevSpd = spd;
        prevBle = bleConnected;
    }
}

// ─── Setup ────────────────────────────────────────────────
void setup() {
    Serial.begin(115200);
    Serial2.begin(UART_BAUD, SERIAL_8N1, UART_RX, UART_TX);

    tft.begin();
    tft.setRotation(0);
    tft.fillScreen(TFT_BLACK);
    tft.drawCircle(120, 120, 118, TFT_ORANGE);

    tft.setTextSize(2);
    tft.setTextColor(TFT_WHITE, TFT_BLACK);
    tft.setCursor(84, 40);
    tft.print("GT86");

    drawGForceGrid();
    drawBottom();

    NimBLEDevice::init("GT86");
    auto* scan = NimBLEDevice::getScan();
    scan->setAdvertisedDeviceCallbacks(new ScanCB());
    scan->setInterval(100);
    scan->setWindow(99);
    scan->setActiveScan(true);
    scan->setDuplicateFilter(false);
}

// ─── Loop ─────────────────────────────────────────────────
uint32_t lastDraw = 0, lastUART = 0;

void loop() {
    if (doConnect) {
        doConnect = false;
        if (!connectRaceBox()) doScan = true;
    }
    if (doScan) {
        doScan = false;
        NimBLEDevice::getScan()->clearResults();
        NimBLEDevice::getScan()->start(30, onScanEnded);
    }
    if (rbFresh && millis() - lastDraw >= 40) {
        rbFresh = false;
        lastDraw = millis();
        updateDisplay();
    }
    if (bleConnected && millis() - lastUART >= 100) {
        lastUART = millis();
        sendUARTFrame();
    }
    // Rafraîchir indicateur BLE même sans données
    if (millis() - lastDraw >= 2000) {
        lastDraw = millis();
        drawBottom();
    }
}
