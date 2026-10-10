#include <Arduino.h>

const int DIR_PIN = 4;
const int PWM_PIN = 6;

bool running = false;
unsigned long stopAt = 0;   // 0 = run until stopped

void setup() {
  pinMode(DIR_PIN, OUTPUT);
  pinMode(PWM_PIN, OUTPUT);
  analogWriteFrequency(PWM_PIN, 20000);
  analogWrite(PWM_PIN, 0);

  Serial.begin(115200);       // ignored on Teensy USB, but harmless
  while (!Serial && millis() < 3000) {}

  Serial.println("Commands:");
  Serial.println("  f <speed%> [ms]   extend   e.g. f 70   or  f 70 2000");
  Serial.println("  r <speed%> [ms]   retract  e.g. r 50 1500");
  Serial.println("  s                 stop");
}

void drive(bool forward, int percent, unsigned long durationMs) {
  percent = constrain(percent, 0, 100);
  digitalWrite(DIR_PIN, forward ? HIGH : LOW);
  analogWrite(PWM_PIN, map(percent, 0, 100, 0, 255));
  running = true;
  stopAt = durationMs ? millis() + durationMs : 0;

  Serial.print(forward ? "Extending" : "Retracting");
  Serial.print(" at ");
  Serial.print(percent);
  Serial.print("%");
  if (durationMs) { Serial.print(" for "); Serial.print(durationMs); Serial.print(" ms"); }
  Serial.println();
}

void stopActuator() {
  analogWrite(PWM_PIN, 0);
  running = false;
  stopAt = 0;
  Serial.println("Stopped");
}

void handleCommand(String line) {
  line.trim();
  line.toLowerCase();
  if (line.length() == 0) return;

  char cmd = line.charAt(0);

  if (cmd == 's') {
    stopActuator();
    return;
  }

  if (cmd == 'f' || cmd == 'r') {
    int speed = 0;
    long dur = 0;
    int n = sscanf(line.c_str() + 1, "%d %ld", &speed, &dur);
    if (n < 1) {
      Serial.println("Usage: f|r <speed 0-100> [ms]");
      return;
    }
    drive(cmd == 'f', speed, dur > 0 ? dur : 0);
    return;
  }

  Serial.println("Unknown command");
}

void loop() {
  if (Serial.available()) {
    String line = Serial.readStringUntil('\n');
    handleCommand(line);
  }

  if (running && stopAt && (long)(millis() - stopAt) >= 0) {
    stopActuator();
  }
}