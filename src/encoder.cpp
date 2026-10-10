#include <SPI.h>

// ==========================================================
// iC-TW39 + Teensy 4.0
// Correct continuous SPI Position Read
// Datasheet Rev B6
// ==========================================================

#define TW39_CS     10
#define TW39_NERR    2
#define TW39_NPRE    3

// TW39 supports Mode 0.
// Start conservatively at 500 kHz.
SPISettings TW39_SPI(500000, MSBFIRST, SPI_MODE0);


// ----------------------------------------------------------
// Exactly one 64-bit SPI transaction
// ----------------------------------------------------------
void transfer64(const uint8_t tx[8], uint8_t rx[8])
{
  SPI.beginTransaction(TW39_SPI);

  digitalWrite(TW39_CS, LOW);
  delayMicroseconds(2);

  for (int i = 0; i < 8; i++)
    rx[i] = SPI.transfer(tx[i]);

  delayMicroseconds(2);

  digitalWrite(TW39_CS, HIGH);

  SPI.endTransaction();

  // Datasheet minimum NCS-high time is 200 ns.
  delayMicroseconds(2);
}


// ----------------------------------------------------------
// Request Position
// ----------------------------------------------------------
bool readTW39Position(uint32_t &angleRaw,
                      float &angleDeg,
                      uint32_t &revolution,
                      uint8_t &status)
{
  // ========================================================
  // CONTROL WORD
  //
  // anc = 0
  // clr = 0
  // reg = 0
  // rm  = 4  -> Position Read
  // wm  = 0  -> Null Write
  //
  // Control word:
  //
  // bit 15 = anc
  // bit 14 = clr
  // bit 13 = reg
  // bits 12:10 = rm
  // bits 9:8   = wm
  // bits 7:0   = 0
  //
  // rm = 4 = 100b
  //
  // 100 << 10 = 0x1000
  // ========================================================

  uint8_t positionCommand[8] =
  {
    0x10, 0x00,     // Control Word = 0x1000
    0x00, 0x00,
    0x00, 0x00,
    0x00, 0x00
  };

  // NULL command used to clock out response
  uint8_t nullCommand[8] =
  {
    0x00, 0x00,
    0x00, 0x00,
    0x00, 0x00,
    0x00, 0x00
  };

  uint8_t ignored[8];
  uint8_t rx[8];

  // --------------------------------------------------------
  // Transaction #1
  // Send Position Read command.
  // Returned data belongs to PREVIOUS command -> ignore.
  // --------------------------------------------------------

  transfer64(positionCommand, ignored);


  // --------------------------------------------------------
  // Transaction #2
  // NULL command clocks out Position Read response.
  // --------------------------------------------------------

  transfer64(nullCommand, rx);


  // --------------------------------------------------------
  // Construct 64-bit response
  // --------------------------------------------------------

  uint64_t packet = 0;

  for (int i = 0; i < 8; i++)
  {
    packet <<= 8;
    packet |= rx[i];
  }


  // --------------------------------------------------------
  // Decode Position Read response
  //
  // [63:32] Revolution Count
  // [31:6]  Angle
  // [5:0]   Status
  // --------------------------------------------------------

  revolution = (uint32_t)(packet >> 32);

  angleRaw =
      (uint32_t)((packet >> 6) & 0x03FFFFFFULL);

  status =
      (uint8_t)(packet & 0x3F);


  // 26-bit position
  angleDeg =
      ((double)angleRaw * 360.0) /
      67108864.0;


  return true;
}


// ----------------------------------------------------------
// Sin/Cos ADC Read (rm = 7)
//
// Response: [47:32] corrected sin, [31:16] corrected cos
// (signed 14-bit, sign-extended to 16).
// Nominal vector amplitude sqrt(sin^2 + cos^2) = 2400;
// the chip flags 'scamp' outside 50 %..120 % (1200..2880).
// ----------------------------------------------------------
void readTW39SinCos(int16_t &sinVal, int16_t &cosVal)
{
  // rm = 7 = 111b -> 111 << 10 = 0x1C00
  uint8_t sinCosCommand[8] = { 0x1C, 0x00, 0, 0, 0, 0, 0, 0 };
  uint8_t nullCommand[8]   = { 0 };
  uint8_t ignored[8];
  uint8_t rx[8];

  transfer64(sinCosCommand, ignored);
  transfer64(nullCommand, rx);

  sinVal = (int16_t)((rx[2] << 8) | rx[3]);
  cosVal = (int16_t)((rx[4] << 8) | rx[5]);
}


// ----------------------------------------------------------
// Print 6-bit status
// ----------------------------------------------------------

void printStatus(uint8_t status)
{
  for (int i = 5; i >= 0; i--)
  {
    Serial.print((status >> i) & 1);
  }
}


// ----------------------------------------------------------
// SETUP
// ----------------------------------------------------------

void setup()
{
  Serial.begin(115200);

  delay(1500);

  Serial.println();
  Serial.println("======================================");
  Serial.println(" iC-TW39 SPI POSITION TEST");
  Serial.println("======================================");


  // CS
  pinMode(TW39_CS, OUTPUT);
  digitalWrite(TW39_CS, HIGH);


  // NERR
  pinMode(TW39_NERR, INPUT_PULLUP);


  // NPRE
  //
  // Your hardware currently works with NPRE HIGH.
  //
  pinMode(TW39_NPRE, OUTPUT);
  digitalWrite(TW39_NPRE, HIGH);


  SPI.begin();

  delay(100);


  Serial.println("SPI Mode     : MODE 0");
  Serial.println("SPI Clock    : 500 kHz");
  Serial.println("Frame        : 64 bit");
  Serial.println("Position RM  : 4");
  Serial.println();
  Serial.println("Rotate magnet...");
  Serial.println();
}


// ----------------------------------------------------------
// MAIN LOOP
// ----------------------------------------------------------

void loop()
{
  uint32_t raw;
  uint32_t revolution;
  uint8_t status;
  float angle;


  readTW39Position(
      raw,
      angle,
      revolution,
      status
  );


  Serial.print("RAW = ");
  Serial.print(raw);


  Serial.print("   ANGLE = ");
  Serial.print(angle, 4);
  Serial.print(" deg");


  Serial.print("   REV = ");
  Serial.print(revolution);


  Serial.print("   STATUS = 0b");
  printStatus(status);


  Serial.print("   NERR=");
  Serial.print(digitalRead(TW39_NERR));


  Serial.print("   NPRE=");
  Serial.print(digitalRead(TW39_NPRE));


  int16_t s, c;
  readTW39SinCos(s, c);

  Serial.print("   SIN=");
  Serial.print(s);
  Serial.print("   COS=");
  Serial.print(c);
  Serial.print("   AMP=");
  Serial.println(sqrtf((float)s * s + (float)c * c), 0);


  delay(20);
}