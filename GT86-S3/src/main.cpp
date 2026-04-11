#include <Arduino.h>

void setup() {
    // Crucial pour le S3 : attendre que l'USB soit prêt
    delay(2000); 
    Serial.begin(115200);
}

void loop() {
    Serial.println(">>> LA CARTE NE REBOOT PLUS ! <<<");
    delay(1000);
}