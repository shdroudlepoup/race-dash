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

// ─── Protocole trame inter-ESP ────────────────────────────
#define FRAME_SYNC1    0xAA
#define FRAME_SYNC2    0x55
#define FRAME_TYPE_RB  0x01
#define FRAME_TYPE_OBD 0x02
#define FRAME_LEN_RB   12
#define FRAME_LEN_OBD  22
#define FRAME_MAX_LEN  22

struct __attribute__((packed)) UARTFrame {
    uint8_t  sync1, sync2, type;
    uint16_t speedX10;
    int16_t  gxX1000;
    int16_t  gyX1000;
    uint8_t  fix, svs, crc;
};

struct __attribute__((packed)) OBDFrameExt {
    uint8_t  sync1, sync2, type;
    uint16_t rpm;
    int16_t  coolant;
    int16_t  intakeTemp;
    uint16_t batteryMV;
    uint16_t afr;
    uint8_t  load;
    uint8_t  fuel;
    int8_t   timing;
    uint8_t  mil;
    int16_t  oilTemp;
    uint8_t  speed;
    uint8_t  crc;
};

// ─── RaceBox BLE ──────────────────────────────────────────
#define RB_NAME  "Vroomvroom"
#define SVC_UUID "6E400001-B5A3-F393-E0A9-E50E24DCCA9E"
#define TX_UUID  "6E400003-B5A3-F393-E0A9-E50E24DCCA9E"

// ─── Données RaceBox ──────────────────────────────────────
struct RBData {
    float   speedKmh = 0;
    float   gx = 0, gy = 0;
    uint8_t fix = 0, svs = 0;
    float   voltage = 0;
};

RBData        rb;
volatile bool rbFresh      = false;
volatile bool bleConnected = false;
volatile bool doConnect    = false;
volatile bool doScan       = true;

NimBLEAdvertisedDevice* advDevice = nullptr;

// ─── Données OBD (reçues du GT86 via UART) ───────────────
struct OBDData {
    uint16_t rpm       = 0;
    int16_t  coolant   = 0;
    int16_t  intakeTemp = 0;
    uint16_t batteryMV = 0;
    uint16_t afr       = 0;   // ×100
    uint8_t  load      = 0;
    uint8_t  fuel      = 0;
    int8_t   timing    = 0;
    uint8_t  mil       = 0;   // bit7=MIL, 0-6=DTC
    int16_t  oilTemp   = -999;
    uint8_t  speed     = 0;
};
OBDData       obd;
volatile bool obdFresh = false;

// ─── Alertes popup ───────────────────────────────────────
uint32_t alertStart  = 0;
uint8_t  alertActive = 0;  // 0=rien, 1=MIL, 2=temp eau, 3=huile, 4=batterie

void showAlert(const char* line1, const char* line2, uint16_t bgColor) {
    gfx->fillRect(100, 150, 600, 180, bgColor);
    gfx->drawRect(100, 150, 600, 180, WHITE);
    gfx->setTextColor(WHITE);
    gfx->setTextSize(4);
    int w1 = strlen(line1) * 24;
    gfx->setCursor((800 - w1) / 2, 180);
    gfx->print(line1);
    gfx->setTextSize(2);
    int w2 = strlen(line2) * 12;
    gfx->setCursor((800 - w2) / 2, 250);
    gfx->print(line2);
    alertStart = millis();
}

void checkAlerts() {
    // Effacer alerte après 5 secondes (le loop fera le redraw)
    if (alertActive && millis() - alertStart > 5000) {
        alertActive = 0;
        return;  // le loop détectera alertActive=0 et redessinera
    }

    // MIL (check engine)
    if ((obd.mil & 0x80) && alertActive != 1) {
        uint8_t dtcCount = obd.mil & 0x7F;
        char buf[24];
        snprintf(buf, sizeof(buf), "%d defaut(s)", dtcCount);
        showAlert("CHECK ENGINE", buf, 0xF800);
        alertActive = 1;
    }
    // Temp eau > 105°C
    else if (obd.coolant > 105 && alertActive != 2) {
        char buf[24];
        snprintf(buf, sizeof(buf), "EAU: %d C", obd.coolant);
        showAlert("TEMP HAUTE!", buf, 0xF800);
        alertActive = 2;
    }
    // Huile > 130°C
    else if (obd.oilTemp > 130 && obd.oilTemp != -999 && alertActive != 3) {
        char buf[24];
        snprintf(buf, sizeof(buf), "HUILE: %d C", obd.oilTemp);
        showAlert("HUILE CHAUDE!", buf, 0xFD20);
        alertActive = 3;
    }
    // Batterie < 12.0V
    else if (obd.batteryMV > 0 && obd.batteryMV < 12000 && alertActive != 4) {
        char buf[24];
        snprintf(buf, sizeof(buf), "%.1fV", obd.batteryMV / 1000.0f);
        showAlert("TENSION BASSE", buf, 0xFD20);
        alertActive = 4;
    }
}

// ─── Parser UART multi-trame ─────────────────────────────
uint8_t uartBuf[FRAME_MAX_LEN];
uint8_t uartPos = 0;
int     uartExpected = 0;

void processUART() {
    while (InterSerial.available()) {
        uint8_t b = InterSerial.read();
        if (uartPos == 0) {
            if (b == FRAME_SYNC1) uartBuf[uartPos++] = b;
        } else if (uartPos == 1) {
            if (b == FRAME_SYNC2) uartBuf[uartPos++] = b;
            else uartPos = 0;
        } else if (uartPos == 2) {
            uartBuf[uartPos++] = b;
            if (b == FRAME_TYPE_RB)       uartExpected = FRAME_LEN_RB;
            else if (b == FRAME_TYPE_OBD) uartExpected = FRAME_LEN_OBD;
            else uartPos = 0;
        } else {
            uartBuf[uartPos++] = b;
            if (uartPos == uartExpected) {
                uartPos = 0;
                uint8_t crc = 0;
                for (int i = 2; i < uartExpected - 1; i++) crc ^= uartBuf[i];
                if (crc != uartBuf[uartExpected - 1]) continue;

                if (uartBuf[2] == FRAME_TYPE_OBD) {
                    OBDFrameExt* f = (OBDFrameExt*)uartBuf;
                    obd.rpm       = f->rpm;
                    obd.coolant   = f->coolant;
                    obd.intakeTemp = f->intakeTemp;
                    obd.batteryMV = f->batteryMV;
                    obd.afr       = f->afr;
                    obd.load      = f->load;
                    obd.fuel      = f->fuel;
                    obd.timing    = f->timing;
                    obd.mil       = f->mil;
                    obd.oilTemp   = f->oilTemp;
                    obd.speed     = f->speed;
                    obdFresh = true;
                }
            }
        }
    }
}

// ─── Envoi trame UART ─────────────────────────────────────
void sendUARTFrame() {
    UARTFrame f;
    f.sync1    = FRAME_SYNC1;
    f.sync2    = FRAME_SYNC2;
    f.type     = FRAME_TYPE_RB;
    f.speedX10 = (uint16_t)(rb.speedKmh * 10.0f);
    f.gxX1000  = (int16_t)(rb.gx * 1000.0f);
    f.gyX1000  = (int16_t)(rb.gy * 1000.0f);
    f.fix      = rb.fix;
    f.svs      = rb.svs;
    uint8_t crc = 0;
    const uint8_t* p = (const uint8_t*)&f;
    for (int i = 2; i < FRAME_LEN_RB - 1; i++) crc ^= p[i];
    f.crc = crc;
    InterSerial.write((const uint8_t*)&f, FRAME_LEN_RB);
}

// ─── Scan debug ───────────────────────────────────────────
#define MAX_SCAN 7
String scanNames[MAX_SCAN];
int    scanFoundCount  = 0;
bool   scanDirty       = false;

// ─── Parser paquet RaceBox 88 octets ──────────────────────
void onNotify(NimBLERemoteCharacteristic*, uint8_t* d, size_t len, bool) {
    if (len < 88) return;
    if (d[0] != 0xB5 || d[1] != 0x62) return;
    if (d[2] != 0xFF || d[3] != 0x01) return;
    uint8_t ca = 0, cb = 0;
    for (size_t i = 2; i < len - 2; i++) { ca += d[i]; cb += ca; }
    if (ca != d[len-2] || cb != d[len-1]) return;

    const uint8_t* p = d + 6;
    rb.speedKmh = (int32_t)(p[48] | (uint32_t)p[49]<<8 | (uint32_t)p[50]<<16 | (uint32_t)p[51]<<24) * 3.6f / 1000.0f;
    rb.gx       = (int16_t)(p[68] | (uint16_t)p[69]<<8) / 1000.0f;
    rb.gy       = (int16_t)(p[70] | (uint16_t)p[71]<<8) / 1000.0f;
    rb.fix      = p[20];
    rb.svs      = p[23];
    rb.voltage  = p[67] / 10.0f;
    rbFresh     = true;
}

// ─── BLE Callbacks ────────────────────────────────────────
class ClientCB : public NimBLEClientCallbacks {
    void onConnect(NimBLEClient*) override {
        bleConnected = true;
    }
    void onDisconnect(NimBLEClient*) override {
        bleConnected = false;
        scanFoundCount = 0;
        doScan = true;
    }
};

class ScanCB : public NimBLEAdvertisedDeviceCallbacks {
    void onResult(NimBLEAdvertisedDevice* dev) override {
        String name = String(dev->getName().c_str());

        // Ajouter à la liste debug (dédupliqué par nom)
        bool known = false;
        for (int i = 0; i < scanFoundCount; i++) {
            if (scanNames[i] == name) { known = true; break; }
        }
        if (!known && scanFoundCount < MAX_SCAN) {
            scanNames[scanFoundCount++] = name.isEmpty() ? String("(no name) ") + dev->getAddress().toString().c_str() : name;
            scanDirty = true;
        }

        // Match sur le numéro de série (le nom BLE est "RaceBox Micro XXXXXXXXX")
        if (name.indexOf("3242708879") >= 0) {
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
    auto* svc = c->getService(SVC_UUID);
    if (!svc) { c->disconnect(); return false; }
    auto* ch = svc->getCharacteristic(TX_UUID);
    if (!ch || !ch->canNotify()) { c->disconnect(); return false; }
    return ch->subscribe(true, onNotify);
}

// ─── Layout display ───────────────────────────────────────
#define GCX   625
#define GCY   255
#define GCR   145
#define GSCALE 2.5f

static int prevDotX = GCX, prevDotY = GCY;

void drawBackground() {
    gfx->fillScreen(BLACK);
    gfx->fillRect(0,   0, 800,  4, 0xF800);
    gfx->fillRect(0, 476, 800,  4, 0xF800);
    gfx->drawFastVLine(455, 42, 430, 0x2945);

    gfx->drawCircle(GCX, GCY, GCR, 0x4208);
    gfx->drawCircle(GCX, GCY, (int)(GCR / GSCALE), 0x2945);
    gfx->drawFastHLine(GCX - GCR, GCY, GCR * 2, 0x2945);
    gfx->drawFastVLine(GCX, GCY - GCR, GCR * 2, 0x2945);
    gfx->setTextColor(0x4208); gfx->setTextSize(1);
    gfx->setCursor(GCX + 4, GCY - GCR - 12); gfx->print("BRAKE");
    gfx->setCursor(GCX + 4, GCY + GCR +  4); gfx->print("ACCEL");

    // Point G-force au centre (état initial)
    gfx->fillCircle(GCX, GCY, 13, 0x07E0);

    gfx->setTextColor(0x4208); gfx->setTextSize(2);
    gfx->setCursor(170, 400); gfx->print("km/h");

    // Fond barre RPM (y=440 à y=472)
    gfx->drawRect(10, 440, 780, 32, 0x2945);
}

// ─── Barre RPM + température (données OBD) ───────────────
#define RPM_MAX 7500
uint16_t prevRpmBar = 0;
int16_t  prevCoolS3 = -999;

void drawRpmBar() {
    if (obd.rpm == prevRpmBar) return;
    prevRpmBar = obd.rpm;

    int barW = (int)((float)obd.rpm / RPM_MAX * 776);
    if (barW > 776) barW = 776;

    // Couleur par zone RPM
    uint16_t col;
    if (obd.rpm > 6500)      col = 0xF800;  // rouge
    else if (obd.rpm > 5000) col = 0xFD20;  // orange
    else                     col = 0x07E0;  // vert

    gfx->fillRect(12, 442, barW, 28, col);
    gfx->fillRect(12 + barW, 442, 776 - barW, 28, BLACK);

    // Texte RPM au centre de la barre
    gfx->setTextSize(2);
    gfx->setTextColor(WHITE);
    char buf[10];
    snprintf(buf, sizeof(buf), "%d", obd.rpm);
    int w = strlen(buf) * 12;
    gfx->setCursor((800 - w) / 2, 446);
    gfx->print(buf);
}

void drawOBDInfo() {
    // Zone info OBD : panneau droit, au-dessus du cercle G-force
    gfx->fillRect(460, 4, 340, 38, BLACK);
    gfx->setTextSize(2);

    // Temp eau
    uint16_t col = obd.coolant > 100 ? 0xF800 : (obd.coolant > 90 ? 0xFD20 : WHITE);
    gfx->setTextColor(col);
    char buf[40];
    snprintf(buf, sizeof(buf), "%d%cC", obd.coolant, 0xF7);
    gfx->setCursor(465, 12);
    gfx->print(buf);

    // Batterie
    if (obd.batteryMV > 0) {
        uint16_t bCol = obd.batteryMV < 12000 ? 0xF800 : WHITE;
        gfx->setTextColor(bCol);
        snprintf(buf, sizeof(buf), "%.1fV", obd.batteryMV / 1000.0f);
        gfx->setCursor(560, 12);
        gfx->print(buf);
    }

    // Fuel
    if (obd.fuel > 0) {
        gfx->setTextColor(obd.fuel < 15 ? 0xFD20 : WHITE);
        snprintf(buf, sizeof(buf), "%d%%", obd.fuel);
        gfx->setCursor(680, 12);
        gfx->print(buf);
    }

    // MIL indicator
    if (obd.mil & 0x80) {
        gfx->setTextColor(0xF800);
        gfx->setCursor(740, 12);
        gfx->print("MIL");
    }
}

// Valeurs précédentes pour affichage différentiel
bool    prevBleOk  = false;
uint8_t prevFix    = 255;
uint8_t prevSvs    = 255;
float   prevVoltage = -1;
int     prevSpd     = -1;

void drawStatusBar() {
    bool bleOk = bleConnected;
    bool changed = (bleOk != prevBleOk) || (rb.fix != prevFix)
                || (rb.svs != prevSvs)  || (fabsf(rb.voltage - prevVoltage) >= 0.05f);
    if (!changed) return;

    gfx->fillRect(0, 4, 456, 38, BLACK);
    gfx->setTextSize(2);

    gfx->setCursor(8, 12);
    gfx->setTextColor(bleOk ? (uint16_t)0x07E0 : (uint16_t)0xF800);
    gfx->print(bleOk ? "BLE OK" : "BLE...");

    gfx->setCursor(168, 12);
    const char* fixStr[] = {"NO FIX", "DEAD-R", "  2D  ", "  3D  "};
    gfx->setTextColor(rb.fix >= 3 ? (uint16_t)0x07E0 : (uint16_t)0xF800);
    gfx->print(rb.fix <= 3 ? fixStr[rb.fix] : "  ??  ");

    char buf[20];
    snprintf(buf, sizeof(buf), " %2dSV %4.1fV", rb.svs, rb.voltage);
    gfx->setTextColor(WHITE);
    gfx->setCursor(312, 12);
    gfx->print(buf);

    prevBleOk  = bleOk;
    prevFix    = rb.fix;
    prevSvs    = rb.svs;
    prevVoltage = rb.voltage;
}

// Panneau gauche : écran d'attente pendant le scan BLE
void drawScanList() {
    gfx->fillRect(10, 50, 435, 380, BLACK);
    gfx->setTextSize(2);
    gfx->setTextColor(0xFFE0);
    gfx->setCursor(10, 58);
    gfx->print("SCAN BLE...");
    gfx->setTextSize(1);
    gfx->setTextColor(WHITE);
    for (int i = 0; i < scanFoundCount; i++) {
        gfx->setCursor(10, 90 + i * 20);
        gfx->print("> ");
        gfx->print(scanNames[i].c_str());
    }
    if (scanFoundCount == 0) {
        gfx->setTextColor(0x4208);
        gfx->setCursor(10, 90);
        gfx->print("recherche en cours...");
    }
}

void drawSpeed() {
    int spd = (int)(rb.speedKmh + 0.5f);
    if (spd < 0) spd = 0;
    if (spd == prevSpd) return;
    prevSpd = spd;

    gfx->fillRect(10, 50, 435, 335, BLACK);
    uint8_t sz;
    int x;
    if (spd < 10)       { sz = 16; x = (455 - 1 * 6 * 16) / 2; }
    else if (spd < 100) { sz = 16; x = (455 - 2 * 6 * 16) / 2; }
    else                { sz = 12; x = (455 - 3 * 6 * 12) / 2; }

    gfx->setTextSize(sz);
    gfx->setTextColor(WHITE);
    gfx->setCursor(x > 0 ? x : 5, 115);
    gfx->print(spd);
}

void drawGForce() {
    gfx->fillCircle(prevDotX, prevDotY, 13, BLACK);
    gfx->drawFastHLine(GCX - GCR, GCY, GCR * 2, 0x2945);
    gfx->drawFastVLine(GCX, GCY - GCR, GCR * 2, 0x2945);

    float nx   = ( rb.gx / GSCALE) * GCR;
    float ny   = (-rb.gy / GSCALE) * GCR;
    float dist = sqrtf(nx * nx + ny * ny);
    if (dist > GCR) { float s = GCR / dist; nx *= s; ny *= s; dist = GCR; }

    prevDotX = GCX + (int)nx;
    prevDotY = GCY + (int)ny;

    float mag = dist / GCR;
    uint16_t col = mag > 0.8f ? 0xF800 : (mag > 0.4f ? 0xFD20 : 0x07E0);
    gfx->fillCircle(prevDotX, prevDotY, 13, col);
}

// ─── Setup & Loop ─────────────────────────────────────────
void setup() {
    delay(500);
    InterSerial.begin(UART_BAUD, SERIAL_8N1, INTER_RX, INTER_TX);

    if (!gfx->begin()) return;
    drawBackground();
    drawStatusBar();
    drawScanList();

    NimBLEDevice::init("GT86-S3");
    InterSerial.println("[BLE] init done");
    auto* scan = NimBLEDevice::getScan();
    scan->setAdvertisedDeviceCallbacks(new ScanCB());
    scan->setInterval(100);      // 62.5 ms
    scan->setWindow(99);         // ~100% duty cycle
    scan->setActiveScan(true);
    scan->setDuplicateFilter(false);
}

uint32_t lastDraw = 0;
uint32_t lastUART = 0;

void loop() {
    if (doConnect) {
        doConnect = false;
        if (!connectRaceBox()) doScan = true;
        drawStatusBar();
        if (bleConnected) drawSpeed();
    }
    if (doScan) {
        doScan = false;
        scanFoundCount = 0;
        scanDirty = true;
        NimBLEDevice::getScan()->clearResults();
        NimBLEDevice::getScan()->start(30, onScanEnded);  // 30s non-bloquant, relancé par callback
        InterSerial.println("[BLE] scan started");
        drawStatusBar();
    }
    if (scanDirty && !bleConnected) {
        scanDirty = false;
        drawScanList();
    }
    processUART();  // reçoit trames OBD du GT86

    if (rbFresh && millis() - lastDraw >= 40) {
        rbFresh  = false;
        lastDraw = millis();
        drawSpeed();
        drawGForce();
        drawStatusBar();
    }
    if (obdFresh) {
        obdFresh = false;
        if (!alertActive) {
            drawRpmBar();
            drawOBDInfo();
        }
        checkAlerts();
    }
    if (bleConnected && millis() - lastUART >= 100) {  // 10 Hz vers GT86
        lastUART = millis();
        sendUARTFrame();
    }
}
