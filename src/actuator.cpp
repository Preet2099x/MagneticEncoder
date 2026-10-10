#include <Arduino.h>

const int DIR_PIN = 4;
const int PWM_PIN = 6;

// PWM: 20 kHz, 12-bit (4096 steps) for smooth duty changes
const int   PWM_FREQ_HZ = 20000;
const int   PWM_BITS    = 12;
const int   PWM_MAX     = (1 << PWM_BITS) - 1;

// Slew limit: how fast the duty may change. 100 %/s means 0 -> 100 % in 1 s.
// No position feedback exists on this board, so ramping (not PID) is what
// removes the jerk on start, stop and reversal.
float rampPctPerSec = 150.0f;

const unsigned long TICK_MS       = 5;     // control loop period
const unsigned long REVERSE_PAUSE = 100;   // standstill before DIR flips (ms)

float curDuty = 0;          // duty actually applied (%)
bool  curDir  = true;       // direction actually applied (true = extend)
float tgtDuty = 0;          // requested duty (%)
bool  tgtDir  = true;       // requested direction

unsigned long holdMs     = 0;   // 0 = hold until stopped
unsigned long holdUntil  = 0;   // set once the target speed is reached
bool          holdArmed  = false;
bool          active     = false;   // a move is in progress (for DONE message)
unsigned long reversedAt = 0;
unsigned long lastTick   = 0;

void applyOutput() {
  digitalWrite(DIR_PIN, curDir ? HIGH : LOW);
  analogWrite(PWM_PIN, (int)(curDuty * PWM_MAX / 100.0f + 0.5f));
}

void setup() {
  pinMode(DIR_PIN, OUTPUT);
  pinMode(PWM_PIN, OUTPUT);
  analogWriteResolution(PWM_BITS);
  analogWriteFrequency(PWM_PIN, PWM_FREQ_HZ);
  applyOutput();

  Serial.begin(115200);       // ignored on Teensy USB, but harmless
  while (!Serial && millis() < 3000) {}

  Serial.println("Commands (speeds ramp smoothly, direction changes stop first):");
  Serial.println("  f <speed%> [ms]   extend   e.g. f 70   or  f 70 2000");
  Serial.println("  r <speed%> [ms]   retract  e.g. r 50 1500");
  Serial.println("                    ms = time held at full speed, ramps excluded");
  Serial.println("  s                 smooth stop");
  Serial.println("  x                 emergency stop (immediate)");
  Serial.println("  a <%/s>           set ramp rate, e.g. a 150");
  Serial.println("  p <Hz>            set PWM frequency, e.g. p 1000 (default 20000)");
}

void drive(bool forward, int percent, unsigned long durationMs) {
  percent = constrain(percent, 0, 100);
  tgtDir  = forward;
  tgtDuty = percent;
  holdMs  = durationMs;
  holdArmed = false;
  active  = true;

  Serial.print(forward ? "Extending" : "Retracting");
  Serial.print(" at ");
  Serial.print(percent);
  Serial.print("%");
  if (durationMs) { Serial.print(" hold "); Serial.print(durationMs); Serial.print(" ms"); }
  Serial.println();
}

void requestStop() {
  tgtDuty = 0;
  holdMs = 0;
  holdArmed = false;
}

void emergencyStop() {
  tgtDuty = 0;
  curDuty = 0;
  holdMs = 0;
  holdArmed = false;
  applyOutput();
  if (active) { active = false; Serial.println("DONE"); }
  Serial.println("Stopped (emergency)");
}

void handleCommand(String line) {
  line.trim();
  line.toLowerCase();
  if (line.length() == 0) return;

  char cmd = line.charAt(0);

  if (cmd == 's') {
    requestStop();
    Serial.println("Stopping");
    return;
  }

  if (cmd == 'x') {
    emergencyStop();
    return;
  }

  if (cmd == 'a') {
    float rate = 0;
    if (sscanf(line.c_str() + 1, "%f", &rate) == 1 && rate >= 10 && rate <= 1000) {
      rampPctPerSec = rate;
      Serial.print("Ramp rate "); Serial.print(rate); Serial.println(" %/s");
    } else {
      Serial.println("Usage: a <10-1000 %/s>");
    }
    return;
  }

  if (cmd == 'p') {
    long hz = 0;
    if (sscanf(line.c_str() + 1, "%ld", &hz) == 1 && hz >= 100 && hz <= 40000) {
      analogWriteFrequency(PWM_PIN, hz);
      applyOutput();
      Serial.print("PWM frequency "); Serial.print(hz); Serial.println(" Hz");
    } else {
      Serial.println("Usage: p <100-40000 Hz>");
    }
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

void controlTick(unsigned long now, float dt) {
  // Timed hold: starts counting only once the target speed has been reached.
  if (holdMs && tgtDuty > 0) {
    if (!holdArmed && curDuty == tgtDuty && curDir == tgtDir) {
      holdArmed = true;
      holdUntil = now + holdMs;
    }
    if (holdArmed && (long)(now - holdUntil) >= 0) {
      requestStop();
    }
  }

  // Wanted duty this tick: ramp to zero first if the direction must change.
  bool needFlip = (tgtDuty > 0) && (tgtDir != curDir);
  float wanted  = needFlip ? 0 : tgtDuty;

  if (curDuty == 0 && needFlip) {
    if (reversedAt == 0) reversedAt = now;
    if (now - reversedAt >= REVERSE_PAUSE) {
      curDir = tgtDir;
      reversedAt = 0;
    }
    applyOutput();
    return;
  }
  reversedAt = 0;

  float step = rampPctPerSec * dt;
  if (curDuty < wanted)       curDuty = min(curDuty + step, wanted);
  else if (curDuty > wanted)  curDuty = max(curDuty - step, wanted);

  applyOutput();

  if (active && curDuty == 0 && tgtDuty == 0) {
    active = false;
    Serial.println("DONE");
  }
}

void loop() {
  if (Serial.available()) {
    String line = Serial.readStringUntil('\n');
    handleCommand(line);
  }

  unsigned long now = millis();
  if (now - lastTick >= TICK_MS) {
    float dt = (now - lastTick) / 1000.0f;
    lastTick = now;
    controlTick(now, dt);
  }
}
