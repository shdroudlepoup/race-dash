#include <Arduino.h>
#include <TFT_eSPI.h>

// Plus besoin de déclarer les pins ici, PlatformIO s'en charge !
TFT_eSPI tft = TFT_eSPI(); 

void setup() {
  Serial.begin(115200);
  Serial.println("Demarrage TFT_eSPI...");

  tft.begin();
  tft.setRotation(0); 

  // Fond noir
  tft.fillScreen(TFT_BLACK);
  
  // Petit cercle de test
  tft.drawCircle(120, 120, 118, TFT_ORANGE);
  
  // Texte au centre
  tft.setTextColor(TFT_WHITE, TFT_BLACK);
  tft.setTextDatum(MC_DATUM); // Aligne le texte au Milieu-Centre
  tft.drawString("GT86 READY", 120, 120, 4); // Le '4' est la taille de la police
}

void loop() {
  // Un petit point qui clignote pour voir que l'ESP tourne
  tft.fillCircle(120, 160, 5, TFT_RED);
  delay(500);
  tft.fillCircle(120, 160, 5, TFT_BLACK);
  delay(500);
}