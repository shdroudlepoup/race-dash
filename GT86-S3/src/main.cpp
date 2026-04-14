#include <Arduino.h>
#include <Arduino_GFX_Library.h>
#include <BLEDevice.h>
#include <BLEScan.h>
#include <BLEAdvertisedDevice.h>

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

// ─── Protocole trame UART ─────────────────────────────────
#define FRAME_SYNC1    0xAA
#define FRAME_SYNC2    0x55
#define FRAME_TYPE_RB  0x01
#define FRAME_LEN_RB   12

// ─── Données RaceBox (reçues du Wroom via UART) ──────────
float   rbSpeed = 0;
float   rbGx = 0, rbGy = 0;
uint8_t rbFix = 0, rbSvs = 0;
bool    rbFresh = false;
uint32_t lastRbRx = 0;

// ─── Parser UART ─────────────────────────────────────────
uint8_t uartBuf[FRAME_LEN_RB];
uint8_t uartPos = 0;

void processUART() {
    while (InterSerial.available()) {
        uint8_t b = InterSerial.read();
        if (uartPos == 0) {
            if (b == FRAME_SYNC1) uartBuf[uartPos++] = b;
        } else if (uartPos == 1) {
            if (b == FRAME_SYNC2) uartBuf[uartPos++] = b;
            else uartPos = 0;
        } else {
            uartBuf[uartPos++] = b;
            if (uartPos == FRAME_LEN_RB) {
                uartPos = 0;
                uint8_t crc = 0;
                for (int i = 2; i < FRAME_LEN_RB - 1; i++) crc ^= uartBuf[i];
                if (crc != uartBuf[FRAME_LEN_RB - 1]) continue;
                if (uartBuf[2] == FRAME_TYPE_RB) {
                    rbSpeed = ((uint16_t)(uartBuf[3] | uartBuf[4] << 8)) / 10.0f;
                    rbGx = (int16_t)(uartBuf[5] | uartBuf[6] << 8) / 1000.0f;
                    rbGy = (int16_t)(uartBuf[7] | uartBuf[8] << 8) / 1000.0f;
                    rbFix = uartBuf[9];
                    rbSvs = uartBuf[10];
                    rbFresh = true;
                    lastRbRx = millis();
                }
            }
        }
    }
}

// ─── OBD BLE (Bluedroid) ─────────────────────────────────
#define OBD_BLE_NAME "Vlink"

#define BLE_BUF_SIZE 512
static uint8_t bleBuf[BLE_BUF_SIZE];
static volatile int bleHead = 0, bleTail = 0;
static BLERemoteCharacteristic* obdNotifyChar = nullptr;
static BLERemoteCharacteristic* obdWriteChar  = nullptr;
static BLEClient* obdClient = nullptr;
bool obdConnected = false;

static void obdNotifyCB(BLERemoteCharacteristic*, uint8_t* data, size_t len, bool) {
    for (size_t i = 0; i < len; i++) {
        int next = (bleHead + 1) % BLE_BUF_SIZE;
        if (next != bleTail) { bleBuf[bleHead] = data[i]; bleHead = next; }
    }
}

int obdAvail() { return (bleHead - bleTail + BLE_BUF_SIZE) % BLE_BUF_SIZE; }
int obdRead()  { if (bleHead == bleTail) return -1; uint8_t b = bleBuf[bleTail]; bleTail = (bleTail+1) % BLE_BUF_SIZE; return b; }

// OBD data
uint16_t obdRpm = 0;
int16_t  obdCoolant = 0;
bool     obdFresh = false;

// PID round-robin
struct PIDDef { const char* cmd; uint8_t pid; uint8_t bytes; };
const PIDDef pidList[] = {
    {"010C\r", 0x0C, 2},  // RPM
    {"0105\r", 0x05, 1},  // Coolant
    {"010C\r", 0x0C, 2},  // RPM (2x)
};
#define PID_COUNT 3
int currentPID = 0;
bool queryPending = false;
uint32_t queryStart = 0;
String obdResp = "";

bool parseOBDHex(const String& resp, uint8_t pid, uint8_t* data, int count) {
    char prefix[8]; snprintf(prefix, 8, "41%02X", pid);
    String clean = resp; clean.replace(" ", "");
    int idx = clean.indexOf(prefix); if (idx < 0) return false; idx += 4;
    for (int i = 0; i < count; i++) {
        if (idx + 2 > (int)clean.length()) return false;
        data[i] = strtol(clean.substring(idx, idx+2).c_str(), NULL, 16); idx += 2;
    }
    return true;
}

void applyPIDValue() {
    uint8_t data[4]; uint8_t pid = pidList[currentPID].pid;
    if (!parseOBDHex(obdResp, pid, data, pidList[currentPID].bytes)) return;
    switch (pid) {
        case 0x0C: obdRpm = (data[0]*256 + data[1]) / 4; break;
        case 0x05: obdCoolant = (int16_t)data[0] - 40; break;
    }
    obdFresh = true;
}

void processOBD() {
    if (!obdConnected || !obdWriteChar) return;
    while (obdAvail()) {
        char c = obdRead();
        if (c == '>') {
            if (queryPending) { applyPIDValue(); queryPending = false; currentPID = (currentPID+1) % PID_COUNT; }
            obdResp = "";
        } else if (c >= 0x20 && c <= 0x7E) obdResp += c;
    }
    if (!queryPending && millis() - queryStart >= 50) {
        queryStart = millis(); bleHead = bleTail = 0; obdResp = "";
        obdWriteChar->writeValue((uint8_t*)pidList[currentPID].cmd, strlen(pidList[currentPID].cmd));
        queryPending = true;
    }
    if (queryPending && millis() - queryStart > 5000) {
        queryPending = false; obdResp = ""; currentPID = (currentPID+1) % PID_COUNT;
    }
}

// ─── OBD scan + connexion (Bluedroid) ────────────────────
bool scanAndConnectOBD() {
    BLEDevice::deinit(false);
    delay(300);
    BLEDevice::init("");

    bleHead = bleTail = 0;
    obdNotifyChar = nullptr; obdWriteChar = nullptr;
    if (obdClient) { obdClient->disconnect(); delete obdClient; obdClient = nullptr; }

    BLEScan* pScan = BLEDevice::getScan();
    pScan->setActiveScan(true);
    pScan->setInterval(100);
    pScan->setWindow(99);
    BLEScanResults results = pScan->start(6, false);

    // Trouver iCare
    BLEAdvertisedDevice* target = nullptr;
    for (int i = 0; i < results.getCount(); i++) {
        BLEAdvertisedDevice dev = results.getDevice(i);
        String name = String(dev.getName().c_str());
        if (name.indexOf(OBD_BLE_NAME) >= 0) {
            target = new BLEAdvertisedDevice(dev);
            break;
        }
    }
    pScan->clearResults();
    if (!target) return false;

    // Connexion
    obdClient = BLEDevice::createClient();
    if (!obdClient->connect(target)) { delete target; return false; }
    delete target;

    // Découverte services
    auto* svcs = obdClient->getServices();
    for (auto it = svcs->begin(); it != svcs->end() && !obdWriteChar; ++it) {
        auto* chars = it->second->getCharacteristics();
        BLERemoteCharacteristic *nCh = nullptr, *wCh = nullptr;
        for (auto cit = chars->begin(); cit != chars->end(); ++cit) {
            auto* ch = cit->second;
            if (ch->canNotify() && !nCh) nCh = ch;
            if (ch->canWrite() && !wCh) wCh = ch;
        }
        if (nCh && wCh) { obdNotifyChar = nCh; obdWriteChar = wCh; }
    }
    if (!obdNotifyChar || !obdWriteChar) { obdClient->disconnect(); return false; }

    obdNotifyChar->registerForNotify(obdNotifyCB);
    delay(500);

    // Init AT (sans ATZ)
    const char* cmds[] = {"ATI\r", "ATE0\r", "ATL0\r", "ATSP0\r"};
    bool hasPrompt = false;
    for (int c = 0; c < 4; c++) {
        bleHead = bleTail = 0;
        obdWriteChar->writeValue((uint8_t*)cmds[c], strlen(cmds[c]));
        delay(1500);
        while (obdAvail()) { if (obdRead() == '>') hasPrompt = true; }
    }
    if (!hasPrompt) { obdClient->disconnect(); return false; }

    bleHead = bleTail = 0;
    return true;
}

// ─── Layout display 800×480 ──────────────────────────────
#define GCX   625
#define GCY   255
#define GCR   145
#define GSCALE 2.5f
static int prevDotX = GCX, prevDotY = GCY;

void drawBackground() {
    gfx->fillScreen(BLACK);
    gfx->fillRect(0, 0, 800, 4, 0xF800);
    gfx->fillRect(0, 476, 800, 4, 0xF800);
    gfx->drawFastVLine(455, 42, 430, 0x2945);

    // G-force circle
    gfx->drawCircle(GCX, GCY, GCR, 0x4208);
    gfx->drawCircle(GCX, GCY, (int)(GCR / GSCALE), 0x2945);
    gfx->drawFastHLine(GCX-GCR, GCY, GCR*2, 0x2945);
    gfx->drawFastVLine(GCX, GCY-GCR, GCR*2, 0x2945);
    gfx->setTextColor(0x4208); gfx->setTextSize(1);
    gfx->setCursor(GCX+4, GCY-GCR-12); gfx->print("BRAKE");
    gfx->setCursor(GCX+4, GCY+GCR+4);  gfx->print("ACCEL");
    gfx->fillCircle(GCX, GCY, 13, 0x07E0);

    gfx->setTextColor(0x4208); gfx->setTextSize(2);
    gfx->setCursor(170, 400); gfx->print("km/h");

    // RPM bar frame
    gfx->drawRect(10, 440, 780, 32, 0x2945);
}

// ─── Affichage différentiel ──────────────────────────────
int prevSpd = -1;
uint16_t prevRpmBar = 0;
int16_t prevCoolS3 = -999;
bool prevWroomLive = false;
bool prevObdOk = false;

void drawStatusBar() {
    gfx->fillRect(0, 4, 800, 38, BLACK);
    gfx->setTextSize(2);

    // Wroom link
    bool wroomLive = (millis() - lastRbRx) < 2000;
    gfx->setCursor(8, 12);
    gfx->setTextColor(wroomLive ? (uint16_t)0x07E0 : (uint16_t)0xF800);
    gfx->print(wroomLive ? "RB OK" : "RB...");

    // GPS fix
    gfx->setCursor(140, 12);
    gfx->setTextColor(rbFix >= 3 ? (uint16_t)0x07E0 : (uint16_t)0xF800);
    const char* fixStr[] = {"NO FIX", "DEAD-R", "  2D  ", "  3D  "};
    gfx->print(rbFix <= 3 ? fixStr[rbFix] : "  ??  ");

    // SVs
    gfx->setTextColor(WHITE);
    gfx->setCursor(280, 12);
    gfx->print(rbSvs); gfx->print("SV");

    // OBD status
    gfx->setCursor(370, 12);
    gfx->setTextColor(obdConnected ? (uint16_t)0x07E0 : (uint16_t)0xF800);
    gfx->print(obdConnected ? "OBD" : "---");

    // Coolant
    if (obdConnected) {
        uint16_t col = obdCoolant > 100 ? 0xF800 : (obdCoolant > 90 ? 0xFD20 : WHITE);
        gfx->setTextColor(col);
        char buf[16]; snprintf(buf, 16, "EAU %d%cC", obdCoolant, 0xF7);
        gfx->setCursor(465, 12); gfx->print(buf);
    }

    prevWroomLive = wroomLive;
    prevObdOk = obdConnected;
}

void drawSpeed() {
    int spd = (int)(rbSpeed + 0.5f);
    if (spd < 0) spd = 0;
    if (spd == prevSpd) return;
    prevSpd = spd;

    gfx->fillRect(10, 50, 435, 335, BLACK);
    uint8_t sz; int x;
    if (spd < 10)       { sz = 16; x = (455 - 1*6*16) / 2; }
    else if (spd < 100) { sz = 16; x = (455 - 2*6*16) / 2; }
    else                { sz = 12; x = (455 - 3*6*12) / 2; }
    gfx->setTextSize(sz); gfx->setTextColor(WHITE);
    gfx->setCursor(x > 0 ? x : 5, 115);
    gfx->print(spd);
}

void drawGForce() {
    gfx->fillCircle(prevDotX, prevDotY, 13, BLACK);
    gfx->drawFastHLine(GCX-GCR, GCY, GCR*2, 0x2945);
    gfx->drawFastVLine(GCX, GCY-GCR, GCR*2, 0x2945);

    float nx = (rbGx / GSCALE) * GCR;
    float ny = (-rbGy / GSCALE) * GCR;
    float dist = sqrtf(nx*nx + ny*ny);
    if (dist > GCR) { float s = GCR/dist; nx *= s; ny *= s; dist = GCR; }
    prevDotX = GCX + (int)nx;
    prevDotY = GCY + (int)ny;
    float mag = dist / GCR;
    uint16_t col = mag > 0.8f ? 0xF800 : (mag > 0.4f ? 0xFD20 : 0x07E0);
    gfx->fillCircle(prevDotX, prevDotY, 13, col);
}

#define RPM_MAX 7500
void drawRpmBar() {
    if (obdRpm == prevRpmBar) return;
    prevRpmBar = obdRpm;
    int barW = (int)((float)obdRpm / RPM_MAX * 776);
    if (barW > 776) barW = 776;
    uint16_t col = obdRpm > 6500 ? 0xF800 : (obdRpm > 5000 ? 0xFD20 : 0x07E0);
    gfx->fillRect(12, 442, barW, 28, col);
    gfx->fillRect(12+barW, 442, 776-barW, 28, BLACK);
    gfx->setTextSize(2); gfx->setTextColor(WHITE);
    char buf[10]; snprintf(buf, 10, "%d", obdRpm);
    int w = strlen(buf) * 12;
    gfx->setCursor((800-w)/2, 446); gfx->print(buf);
}

// ─── Setup ────────────────────────────────────────────────
void setup() {
    delay(500);
    InterSerial.begin(UART_BAUD, SERIAL_8N1, INTER_RX, INTER_TX);

    if (!gfx->begin()) return;
    drawBackground();
    drawStatusBar();

    // OBD BLE init (Bluedroid) — échouera sans iCare, c'est OK
    BLEDevice::init("");
}

// ─── Loop ─────────────────────────────────────────────────
uint32_t lastDraw = 0, lastStatus = 0, lastOBDRetry = 0;

void loop() {
    processUART();
    processOBD();

    // Affichage RaceBox (données du Wroom via UART)
    if (rbFresh && millis() - lastDraw >= 40) {
        rbFresh = false;
        lastDraw = millis();
        drawSpeed();
        drawGForce();
    }

    // Affichage OBD
    if (obdFresh) {
        obdFresh = false;
        drawRpmBar();
    }

    // Status bar toutes les 2s
    if (millis() - lastStatus >= 2000) {
        lastStatus = millis();
        bool wroomLive = (millis() - lastRbRx) < 2000;
        if (wroomLive != prevWroomLive || obdConnected != prevObdOk) {
            drawStatusBar();
        }
    }

    // Retry OBD toutes les 15s si pas connecté
    if (!obdConnected && millis() - lastOBDRetry >= 15000) {
        lastOBDRetry = millis();
        obdConnected = scanAndConnectOBD();
        drawStatusBar();
    }
}
