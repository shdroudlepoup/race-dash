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

void setup() {
    delay(500);

    if (!gfx->begin()) return;

    gfx->fillScreen(BLACK);

    // Barre rouge en haut (style GT3)
    gfx->fillRect(0, 0, 800, 6, 0xF800);

    // Titre
    gfx->setTextColor(WHITE);
    gfx->setTextSize(5);
    gfx->setCursor(220, 180);
    gfx->print("GT86 READY");

    // Sous-titre
    gfx->setTextColor(0xFD20);  // orange
    gfx->setTextSize(2);
    gfx->setCursor(290, 270);
    gfx->print("Race Dash  S3  800x480");

    // Barre rouge en bas
    gfx->fillRect(0, 474, 800, 6, 0xF800);
}

void loop() {
    delay(1000);
}