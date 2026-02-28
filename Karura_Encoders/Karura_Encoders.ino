#include <SPI.h>

#define baudRate       115200
#define timeoutLimit   100

#define nop            0x00
#define rd_pos         0x10
#define set_zero_point 0x70

#define PIN_SCK   D8
#define PIN_MISO  D9
#define PIN_MOSI  D10

// One CS pin per encoder
const uint8_t CS_PINS[] = { D3, D2, D4};  // Add/remove as needed
const uint8_t NUM_ENCODERS = sizeof(CS_PINS) / sizeof(CS_PINS[0]);

void setup() {
  Serial.begin(baudRate);
  delay(2000);

  // All CS pins high before SPI starts
  for (uint8_t i = 0; i < NUM_ENCODERS; i++) {
    pinMode(CS_PINS[i], OUTPUT);
    digitalWrite(CS_PINS[i], HIGH);
  }

  SPI.setRX(PIN_MISO);
  SPI.setTX(PIN_MOSI);
  SPI.setSCK(PIN_SCK);
  SPI.begin();
  SPI.beginTransaction(SPISettings(250000, MSBFIRST, SPI_MODE1));

  delay(500);

  Serial.println("AMT203S-V Multi-Encoder Test - XIAO RP2350");
  Serial.println();
}

void loop() {
  for (uint8_t i = 0; i < NUM_ENCODERS; i++) {
    int16_t pos = readEncoder(i);

    Serial.print("Encoder ");
    Serial.print(i + 1);
    Serial.print(": ");

    if (pos >= 0) {
      float deg = 360.0f * ((float)pos / 4096.0f);
      Serial.print(deg, 1);
      Serial.print(" deg  (raw: ");
      Serial.print(pos);
      Serial.print(")");
    } else {
      Serial.print("ERROR");
    }

    Serial.print("  |  ");
  }

  Serial.println();
  delay(100);
}

int16_t readEncoder(uint8_t encoderIndex) {
  uint8_t data;
  uint8_t timeoutCounter = 0;

  data = SPIWriteTo(encoderIndex, rd_pos);

  while (data != rd_pos && timeoutCounter++ < timeoutLimit) {
    data = SPIWriteTo(encoderIndex, nop);
  }

  if (timeoutCounter >= timeoutLimit) {
    return -1;
  }

  uint16_t pos  = (SPIWriteTo(encoderIndex, nop) & 0x0F) << 8;
           pos |=  SPIWriteTo(encoderIndex, nop);

  return (int16_t)pos;
}

uint8_t SPIWriteTo(uint8_t encoderIndex, uint8_t sendByte) {
  uint8_t csPin = CS_PINS[encoderIndex];

  digitalWrite(csPin, LOW);
  delayMicroseconds(5);
  uint8_t received = SPI.transfer(sendByte);
  digitalWrite(csPin, HIGH);
  delayMicroseconds(25);

  return received;
}