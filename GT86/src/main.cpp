#include <Arduino.h>
#include <TFT_eSPI.h>

TFT_eSPI tft = TFT_eSPI();

// ─── UART inter-ESP : GPIO25=RX, GPIO26=TX (Serial2) ──────
#define UART_BAUD 115200
#define UART_RX   25
#define UART_TX   26

// ─── Protocole trame inter-ESP ────────────────────────────
#define FRAME_SYNC1   0xAA
#define FRAME_SYNC2   0x55
#define FRAME_TYPE_RB 0x01
#define FRAME_LEN     12

struct __attribute__((packed)) UARTFrame {
    uint8_t  sync1, sync2, type;
    uint16_t speedX10;
    int16_t  gxX1000;
    int16_t  gyX1000;
    uint8_t  fix, svs, crc;
};

// ─── Données reçues ───────────────────────────────────────
float   rxSpeed = 0;
uint8_t rxFix   = 0, rxSvs = 0;
bool    rxFresh = false;
uint32_t lastRx = 0;

// ─── Parser UART (machine à états) ────────────────────────
uint8_t rxBuf[FRAME_LEN];
uint8_t rxPos = 0;

void processUART() {
    while (Serial2.available()) {
        uint8_t b = Serial2.read();
        if (rxPos == 0) {
            if (b == FRAME_SYNC1) rxBuf[rxPos++] = b;
        } else if (rxPos == 1) {
            if (b == FRAME_SYNC2) rxBuf[rxPos++] = b;
            else rxPos = 0;
        } else {
            rxBuf[rxPos++] = b;
            if (rxPos == FRAME_LEN) {
                rxPos = 0;
                uint8_t crc = 0;
                for (int i = 2; i < FRAME_LEN - 1; i++) crc ^= rxBuf[i];
                if (crc != rxBuf[FRAME_LEN - 1]) { Serial.println("[ERR] CRC"); return; }
                UARTFrame* f = (UARTFrame*)rxBuf;
                if (f->type == FRAME_TYPE_RB) {
                    rxSpeed = f->speedX10 / 10.0f;
                    rxFix   = f->fix;
                    rxSvs   = f->svs;
                    rxFresh = true;
                    lastRx  = millis();
                }
            }
        }
    }
}

// ─── Affichage différentiel GC9A01 240×240 ───────────────
int     prevSpd  = -1;
uint8_t prevFix  = 255;
bool    prevLive = false;

void drawSpeed(int spd) {
    tft.fillRect(10, 72, 220, 108, TFT_BLACK);
    uint8_t sz;
    int x;
    if      (spd < 10)  { sz = 9; x = (240 - 1*6*9) / 2; }
    else if (spd < 100) { sz = 9; x = (240 - 2*6*9) / 2; }
    else                { sz = 7; x = (240 - 3*6*7) / 2; }
    tft.setTextSize(sz);
    tft.setTextColor(TFT_WHITE, TFT_BLACK);
    tft.setCursor(x > 0 ? x : 5, 80);
    tft.print(spd);
}

void drawFix(uint8_t fix) {
    tft.fillCircle(120, 208, 7, fix >= 3 ? TFT_GREEN : TFT_RED);
}

void drawLive(bool live) {
    tft.fillRect(90, 220, 60, 14, TFT_BLACK);
    tft.setTextSize(1);
    tft.setTextColor(live ? TFT_GREEN : (uint16_t)0x4208, TFT_BLACK);
    tft.setCursor(95, 222);
    tft.print(live ? " S3 OK " : " S3 -- ");
}

void updateDisplay() {
    int  spd  = (int)(rxSpeed + 0.5f);
    bool live = (millis() - lastRx) < 2000;

    if (spd != prevSpd) {
        drawSpeed(spd);
        prevSpd = spd;
    }
    if (rxFix != prevFix) {
        drawFix(rxFix);
        prevFix = rxFix;
    }
    if (live != prevLive) {
        drawLive(live);
        prevLive = live;
    }
}

void setup() {
    Serial.begin(115200);
    Serial2.begin(UART_BAUD, SERIAL_8N1, UART_RX, UART_TX);

    tft.begin();
    tft.setRotation(0);
    tft.fillScreen(TFT_BLACK);
    tft.drawCircle(120, 120, 118, TFT_ORANGE);

    tft.setTextSize(3);
    tft.setTextColor(TFT_WHITE, TFT_BLACK);
    tft.setCursor(74, 42);
    tft.print("GT86");

    tft.setTextSize(2);
    tft.setTextColor(0x4208, TFT_BLACK);
    tft.setCursor(84, 178);
    tft.print("km/h");

    // Dessin initial (force les valeurs par défaut)
    drawSpeed(0);
    drawFix(0);
    drawLive(false);
}

uint32_t lastHB = 0;

void loop() {
    processUART();

    if (rxFresh) {
        rxFresh = false;
        updateDisplay();
    }
    // Timeout connexion S3 : rafraîchit l'indicateur live si état change
    updateDisplay();

    if (millis() - lastHB >= 5000) {
        lastHB = millis();
        Serial.printf("[HB] speed=%.1f fix=%d lastRx=%lums ago\n",
                      rxSpeed, rxFix, millis() - lastRx);
    }
}
