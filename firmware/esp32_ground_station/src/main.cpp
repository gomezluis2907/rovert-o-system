#include <Arduino.h>

// GPIO 2 is the standard built-in LED on most ESP32 boards.
// Update this if your specific ESP32-S3 DevKit uses a different pin.
#define LED_PIN 2

void setup() {
    pinMode(LED_PIN, OUTPUT);
}

void loop() {
    digitalWrite(LED_PIN, HIGH);
    delay(500);
    digitalWrite(LED_PIN, LOW);
    delay(500);
}