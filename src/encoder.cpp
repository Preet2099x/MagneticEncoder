#include <Arduino.h>

// ==========================================================
// iC-TW39 + Teensy 4.0 -- ABZ (quadrature) only
//
// A = pin 18, B = pin 19. Z is not wired.
// Decoded x4 (every edge of A and B) in an interrupt.
//
// Serial commands (115200):
//   z          zero the count
//   c <n>      set counts per revolution, e.g. c 4096
//   c          set counts per revolution from the current |count|
//              (zero with 'z', turn exactly one full revolution, send 'c')
// ==========================================================

#define ABZ_A 18
#define ABZ_B 19

volatile int32_t  abzCount  = 0;
volatile uint32_t abzErrors = 0;   // skipped states (edges too fast / noise)
volatile uint8_t  abzState  = 0;

// Index = (previous AB << 2) | new AB, AB = (A << 1) | B.
// Forward sequence 00 -> 01 -> 11 -> 10 counts up.
static const int8_t QDEC[16] =
{
   0, +1, -1,  0,
  -1,  0,  0, +1,
  +1,  0,  0, -1,
   0, -1, +1,  0
};

void abzISR()
{
  uint8_t s = (digitalReadFast(ABZ_A) << 1) | digitalReadFast(ABZ_B);
  int8_t  d = QDEC[(abzState << 2) | s];

  if (d == 0 && s != abzState) abzErrors++;

  abzCount += d;
  abzState = s;
}


// ----------------------------------------------------------
// State
// ----------------------------------------------------------

// iC-TW39 datasheet Rev B6, p.19: ABZ_RES is in edges per revolution, factory
// default 4096 (= 1024 AB cycles). Decoded x4 here, so counts/rev = ABZ_RES.
// Change with 'c <n>' if the chip's EEPROM was programmed differently.
int32_t countsPerRev = 4096;       // 0 = unknown, angle/RPM shown as n/a

const unsigned long PRINT_MS = 50;
unsigned long lastPrint = 0;
int32_t       lastCount = 0;

String cmdLine;


int32_t readCount()
{
  noInterrupts();
  int32_t c = abzCount;
  interrupts();
  return c;
}

void zeroCount()
{
  noInterrupts();
  abzCount  = 0;
  abzErrors = 0;
  interrupts();
  lastCount = 0;
}


// ----------------------------------------------------------
// Commands
// ----------------------------------------------------------

void handleCommand(String line)
{
  line.trim();
  line.toLowerCase();
  if (line.length() == 0) return;

  char cmd = line.charAt(0);

  if (cmd == 'z')
  {
    zeroCount();
    Serial.println("Zeroed");
    return;
  }

  if (cmd == 'c')
  {
    long n = 0;

    if (sscanf(line.c_str() + 1, "%ld", &n) != 1 || n <= 0)
    {
      n = abs(readCount());
      if (n == 0)
      {
        Serial.println("Usage: c <counts per rev>  (or 'z', turn one rev, 'c')");
        return;
      }
    }

    countsPerRev = (int32_t)n;
    Serial.print("Counts per revolution = ");
    Serial.println(countsPerRev);
    return;
  }

  Serial.println("Commands: z = zero, c <n> = counts/rev, c = counts/rev from one turn");
}


// ----------------------------------------------------------
// SETUP
// ----------------------------------------------------------

void setup()
{
  Serial.begin(115200);
  delay(1500);

  pinMode(ABZ_A, INPUT);
  pinMode(ABZ_B, INPUT);

  abzState = (digitalReadFast(ABZ_A) << 1) | digitalReadFast(ABZ_B);
  attachInterrupt(digitalPinToInterrupt(ABZ_A), abzISR, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ABZ_B), abzISR, CHANGE);

  Serial.println();
  Serial.println("======================================");
  Serial.println(" iC-TW39 ABZ  (A=18, B=19, x4 decode)");
  Serial.println("======================================");
  Serial.println("  z        zero the count");
  Serial.println("  c <n>    set counts per rev, e.g. c 4096");
  Serial.println("  c        counts per rev from one full turn since 'z'");
  Serial.println();

  lastPrint = millis();
}


// ----------------------------------------------------------
// MAIN LOOP
// ----------------------------------------------------------

void loop()
{
  while (Serial.available())
  {
    char ch = (char)Serial.read();
    if (ch == '\n' || ch == '\r')
    {
      handleCommand(cmdLine);
      cmdLine = "";
    }
    else if (cmdLine.length() < 32)
    {
      cmdLine += ch;
    }
  }

  unsigned long now = millis();
  if (now - lastPrint < PRINT_MS) return;

  float dt = (now - lastPrint) / 1000.0f;
  lastPrint = now;

  noInterrupts();
  int32_t  count = abzCount;
  uint32_t err   = abzErrors;
  interrupts();

  int32_t delta = count - lastCount;
  lastCount = count;

  float cps = delta / dt;                       // counts per second
  int   dir = (delta > 0) - (delta < 0);        // +1 / -1 / 0 (stopped)

  Serial.print("COUNT = ");
  Serial.print(count);

  if (countsPerRev > 0)
  {
    int32_t rev = count / countsPerRev;
    int32_t pos = count % countsPerRev;
    if (pos < 0) { pos += countsPerRev; rev--; }

    Serial.print("   REV = ");
    Serial.print(rev);
    Serial.print("   POS = ");
    Serial.print(pos);
    Serial.print("/");
    Serial.print(countsPerRev);
    Serial.print("   DEG = ");
    Serial.print(pos * 360.0f / countsPerRev, 2);
  }
  else
  {
    Serial.print("   REV = n/a   POS = n/a   DEG = n/a");
  }

  Serial.print("   DIR = ");
  Serial.print(dir);

  Serial.print("   CPS = ");
  Serial.print(cps, 0);

  Serial.print("   RPM = ");
  if (countsPerRev > 0) Serial.print(cps * 60.0f / countsPerRev, 2);
  else                  Serial.print("n/a");

  Serial.print("   ERR = ");
  Serial.print(err);

  Serial.print("   A=");
  Serial.print(digitalReadFast(ABZ_A));
  Serial.print(" B=");
  Serial.println(digitalReadFast(ABZ_B));
}
