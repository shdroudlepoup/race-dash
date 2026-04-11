#include <Arduino.h>
#include <Arduino_GFX_Library.h>

// --- Pinout Waveshare ESP32-S3-Touch-LCD-7 (800x480 RGB) ---
// Source : https://github.com/paulhamsh/Waveshare-ESP32-S3-LCD-7-LVGL
#define GFX_BL 2  // backlight (vérifier si nécessaire)

Arduino_ESP32RGBPanel *rgbpanel = new Arduino_ESP32RGBPanel(
     5 /* DE */,  3 /* VSYNC */, 46 /* HSYNC */,  7 /* PCLK */,
     1 /* R0 */,  2 /* R1 */, 42 /* R2 */, 41 /* R3 */, 40 /* R4 */,
    39 /* G0 */,  0 /* G1 */, 45 /* G2 */, 48 /* G3 */, 47 /* G4 */, 21 /* G5 */,
    14 /* B0 */, 38 /* B1 */, 18 /* B2 */, 17 /* B3 */, 10 /* B4 */,
    0 /* hsync_polarity */,  8 /* hsync_front_porch */,  4 /* hsync_pulse_width */,  8 /* hsync_back_porch */,
    0 /* vsync_polarity */,  8 /* vsync_front_porch */,  4 /* vsync_pulse_width */,  8 /* vsync_back_porch */,
    1 /* pclk_active_neg */, 14000000 /* prefer_speed */
);

Arduino_RGB_Display *gfx = new Arduino_RGB_Display(
    800 /* width */, 480 /* height */, rgbpanel, 0 /* rotation */, true /* auto_flush */
);

// UART inter-ESP : GPIO43=TX, GPIO44=RX (UART0 exclusif)
#define UART_BAUD 115200
#define INTER_TX  43
#define INTER_RX  44
HardwareSerial InterSerial(0);  // UART0 — seul utilisateur de GPIO43/44

uint32_t lastPing    = 0;
uint32_t pingCount   = 0;
uint32_t pongCount   = 0;

void drawStatus() {
    gfx->fillRect(0, 80, 800, 320, BLACK);
    gfx->setTextSize(3);

    gfx->setTextColor(0xFD20);
    gfx->setCursor(60, 120);
    gfx->print("PING envoyes : ");
    gfx->print(pingCount);

    gfx->setTextColor(pongCount == pingCount ? 0x07E0 : 0xF800);  // vert si OK, rouge si timeout
    gfx->setCursor(60, 180);
    gfx->print("PONG recus   : ");
    gfx->print(pongCount);

    gfx->setTextColor(WHITE);
    gfx->setCursor(60, 260);
    gfx->setTextSize(2);
    if (pongCount == pingCount && pingCount > 0) {
        gfx->print("UART OK - GT86 repond");
    } else if (pingCount > 0) {
        gfx->print("En attente de GT86...");
    }
}

void setup() {
    delay(500);
    InterSerial.begin(UART_BAUD, SERIAL_8N1, INTER_RX, INTER_TX);

    if (!gfx->begin()) return;

    gfx->fillScreen(BLACK);
    gfx->fillRect(0, 0, 800, 6, 0xF800);
    gfx->fillRect(0, 474, 800, 6, 0xF800);

    gfx->setTextColor(WHITE);
    gfx->setTextSize(4);
    gfx->setCursor(220, 30);
    gfx->print("UART TEST");

    drawStatus();
}

void loop() {
    // Envoie PING toutes les secondes
    if (millis() - lastPing >= 1000) {
        lastPing = millis();
        pingCount++;
        InterSerial.println("PING");
        drawStatus();
    }

    // Lit la réponse
    if (InterSerial.available()) {
        String resp = InterSerial.readStringUntil('\n');
        resp.trim();
        if (resp == "PONG") {
            pongCount++;
            drawStatus();
        }
    }
}