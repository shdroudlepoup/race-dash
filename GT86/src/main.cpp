#include <Arduino.h>
#include <TFT_eSPI.h>

TFT_eSPI tft = TFT_eSPI();

// UART inter-ESP : GPIO16=RX2, GPIO17=TX2 (Serial2)
#define UART_BAUD 115200
#define UART_RX   25
#define UART_TX   26

uint32_t pongCount = 0;

void setup() {
  Serial.begin(115200);
  Serial2.begin(UART_BAUD, SERIAL_8N1, UART_RX, UART_TX);

  tft.begin();
  tft.setRotation(0);
  tft.fillScreen(TFT_BLACK);
  tft.drawCircle(120, 120, 118, TFT_ORANGE);
  tft.setTextColor(TFT_WHITE, TFT_BLACK);
  tft.setTextDatum(MC_DATUM);
  tft.drawString("GT86", 120, 100, 4);
  tft.drawString("READY", 120, 130, 4);
}

void loop() {
  if (Serial2.available()) {
    String msg = Serial2.readStringUntil('\n');
    msg.trim();
    Serial.println("Recu: [" + msg + "]");
    if (msg == "PING") {
      Serial2.println("PONG");
      pongCount++;
      Serial.println("PONG #" + String(pongCount));
      tft.fillRect(60, 160, 120, 30, TFT_BLACK);
      tft.setTextDatum(MC_DATUM);
      tft.setTextColor(TFT_GREEN, TFT_BLACK);
      tft.drawString(String(pongCount), 120, 175, 4);
    }
  }
}