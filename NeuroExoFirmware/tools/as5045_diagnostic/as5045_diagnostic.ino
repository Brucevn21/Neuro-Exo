#include <AS5045.h>

const uint8_t AS5045_CLK_PIN = 1;
const uint8_t AS5045_CS_PIN = 2;
const uint8_t AS5045_DATA_PIN = 0;

AS5045 encoder(AS5045_CS_PIN, AS5045_CLK_PIN, AS5045_DATA_PIN, 0xFF, 3);

void printBits(uint8_t value, uint8_t width) {
  for (int8_t bit = width - 1; bit >= 0; --bit) {
    Serial.print((value >> bit) & 1);
  }
}

void setup() {
  Serial.begin(115200);
  delay(500);

  pinMode(AS5045_CLK_PIN, OUTPUT);
  pinMode(AS5045_CS_PIN, OUTPUT);
  pinMode(AS5045_DATA_PIN, INPUT);
  digitalWrite(AS5045_CLK_PIN, HIGH);
  digitalWrite(AS5045_CS_PIN, HIGH);

  Serial.println("AS5045 raw diagnostic");
  Serial.println("Teensy 4.1 pins: CS=2 CLK=1 DATA=0");
  Serial.println("Rotate the shaft slowly; no calibration or angle conversion is applied.");
  Serial.println();
}

void loop() {
  const unsigned int raw = encoder.read();
  const uint8_t status = encoder.status();

  Serial.print("raw=");
  Serial.print(raw);
  Serial.print(" raw_hex=0x");
  if (raw < 0x1000) {
    Serial.print('0');
  }
  Serial.print(raw, HEX);
  Serial.print(" status=");
  printBits(status, 5);
  Serial.print(" status_hex=0x");
  Serial.print(status, HEX);
  Serial.print(" valid=");
  Serial.print(encoder.valid() ? "YES" : "NO");
  Serial.print(" cs=");
  Serial.print(digitalRead(AS5045_CS_PIN));
  Serial.print(" clk=");
  Serial.print(digitalRead(AS5045_CLK_PIN));
  Serial.print(" data_idle=");
  Serial.println(digitalRead(AS5045_DATA_PIN));

  delay(100);
}