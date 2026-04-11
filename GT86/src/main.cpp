#include <Arduino.h>
#include <TFT_eSPI.h>

TFT_eSPI tft = TFT_eSPI();

// UART inter-ESP : GPIO25=RX2, GPIO26=TX2 (Serial2)
#define UART_BAUD 115200
#define UART_RX   25
#define UART_TX   26

uint32_t pingRx = 0;  // PINGs reçus de S3
uint32_t pongTx = 0;  // PONGs renvoyés

void drawStatus() {
    tft.fillRect(20, 88, 200, 90, TFT_BLACK);

    tft.setTextSize(3);

    tft.setTextColor(TFT_ORANGE, TFT_BLACK);
    tft.setCursor(28, 100);
    tft.print("RX:");
    tft.print(pingRx);

    tft.setTextColor(TFT_GREEN, TFT_BLACK);
    tft.setCursor(28, 148);
    tft.print("TX:");
    tft.print(pongTx);
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
    tft.setCursor(74, 52);
    tft.print("GT86");

    drawStatus();
}

uint32_t lastHB = 0;

void loop() {
    if (millis() - lastHB >= 2000) {
        lastHB = millis();
        Serial.println("[HB] pingRx=" + String(pingRx)
                       + " pongTx=" + String(pongTx)
                       + " avail=" + String(Serial2.available()));
    }

    if (Serial2.available()) {
        String msg = Serial2.readStringUntil('\n');
        msg.trim();
        Serial.println("[RX] '" + msg + "'");
        if (msg == "PING") {
            pingRx++;
            Serial2.println("PONG");
            pongTx++;
            drawStatus();
        }
    }
}
