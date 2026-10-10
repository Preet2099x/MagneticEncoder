#include <Arduino.h>

// Teensy 4.0 on-board LED (pin 13). 500 ms on, 500 ms off.
void setup() {
  pinMode(LED_BUILTIN, OUTPUT);
}

void loop() {
  digitalWrite(LED_BUILTIN, HIGH);
  delay(500);
  digitalWrite(LED_BUILTIN, LOW);
  delay(500);
}
