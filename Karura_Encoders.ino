#include <SPI.h>

#define baudRate 115200
#define timoutLimit 100

#define nop 0x00
#define rd_pos 0x10
#define set_zero_point 0x70

// --- Configure your encoder CS pins here ---
const int NUM_ENCODERS = 2;  // Change to however many you have (max 8)
const int CS_PINS[2] = {10, 9};

uint16_t positions[2];  // Stores the latest position for each encoder

void setup() {
  Serial.begin(baudRate);

  // Initialize all CS pins HIGH (deselected)
  for (int i = 0; i < NUM_ENCODERS; i++) {
    pinMode(CS_PINS[i], OUTPUT);
    digitalWrite(CS_PINS[i], HIGH);
  }

  SPI.begin();
  SPI.beginTransaction(SPISettings(500000, MSBFIRST, SPI_MODE0));

  delay(500);  // Wait for all encoders to initialize
}

void loop() {
  // Read each encoder in sequence
  for (int i = 0; i < NUM_ENCODERS; i++) {
    int16_t pos = readEncoder(i);
    if (pos >= 0) {
      positions[i] = (uint16_t)pos;
      float deg = 360.0 * (float(positions[i]) / 4096.0);

      Serial.print("Enc ");
      Serial.print(i);
      Serial.print(": ");
      Serial.print(deg, 2);
      Serial.print("°  (");
      Serial.print(positions[i], DEC);
      Serial.print(")\t");
    } else {
      Serial.print("Enc ");
      Serial.print(i);
      Serial.print(": ERROR\t");
    }
  }
  Serial.println();

  delay(250);
}

// Returns 12-bit position (0–4095) on success, -1 on timeout
int16_t readEncoder(int encoderIndex) {
  uint8_t data;
  uint8_t timeoutCounter = 0;
  uint16_t currentPosition;
  int csPin = CS_PINS[encoderIndex];

  // Send rd_pos command
  data = SPIWriteTo(csPin, rd_pos);

  // Wait for rd_pos echo
  while (data != rd_pos && timeoutCounter++ < timoutLimit) {
    data = SPIWriteTo(csPin, nop);
  }

  if (timeoutCounter < timoutLimit) {
    currentPosition = (SPIWriteTo(csPin, nop) & 0x0F) << 8;
    currentPosition |= SPIWriteTo(csPin, nop);
    return (int16_t)currentPosition;
  }

  return -1;  // Timeout
}

// SPI transfer targeting a specific CS pin
uint8_t SPIWriteTo(int csPin, uint8_t sendByte) {
  uint8_t data;

  digitalWrite(csPin, LOW);
  data = SPI.transfer(sendByte);
  digitalWrite(csPin, HIGH);

  delayMicroseconds(20);

  return data;
}
