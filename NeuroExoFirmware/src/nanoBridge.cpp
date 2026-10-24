#include <ArduinoBLE.h>
#include <Wire.h>
#include "commProtocol.h"

using namespace neuroexo;
BLEService motorService(SERVICE_UUID);
// Larger than a control frame so oversized writes cannot be truncated to a
// valid nine-byte prefix. ArduinoBLE caps longer writes at 20 (also invalid).
BLECharacteristic commandChar(COMMAND_UUID, BLEWrite, 20);
BLECharacteristic telemetryChar(TELEMETRY_UUID, BLENotify, TELEMETRY_SIZE, true);
uint32_t lastTelemetryMs = 0;

void onDisconnect(BLEDevice central) {
  (void)central;
  BLE.advertise();
}

void setup() {
  Serial.begin(115200);
  Wire.begin();
  Wire.setClock(100000);
  if (!BLE.begin()) {
    if (Serial) Serial.println("BLE initialization failed");
    while (true) delay(1000);
  }
  BLE.setLocalName("NanoNeuroExo");
  BLE.setAdvertisedService(motorService);
  motorService.addCharacteristic(commandChar);
  motorService.addCharacteristic(telemetryChar);
  BLE.addService(motorService);
  BLE.setEventHandler(BLEDisconnected, onDisconnect);
  BLE.advertise();
}

void loop() {
  BLE.poll();
  // Validation and all I2C transactions run here, outside callbacks.
  if (commandChar.written()) {
    Control command;
    if (BLE.connected() && decodeControl(commandChar.value(), commandChar.valueLength(), command)) {
      Wire.beginTransmission(TEENSY_ADDRESS);
      const size_t written = Wire.write(commandChar.value(), CONTROL_SIZE);
      const uint8_t result = Wire.endTransmission();
      if ((written != CONTROL_SIZE || result != 0) && Serial)
        Serial.println("Control I2C write failed");
    } else if (Serial) {
      Serial.println("Discarded invalid BLE control frame");
    }
  }
  const uint32_t now = millis();
  if (uint32_t(now - lastTelemetryMs) < TELEMETRY_PERIOD_MS) return;
  lastTelemetryMs = now;
  uint8_t raw[TELEMETRY_SIZE];
  const size_t received = Wire.requestFrom(TEENSY_ADDRESS, TELEMETRY_SIZE);
  size_t count = 0;
  bool oversized = false;
  while (Wire.available()) {
    const int byte = Wire.read();
    if (count < sizeof(raw)) raw[count++] = uint8_t(byte);
    else oversized = true;
  }
  Telemetry telemetry;
  if (received == TELEMETRY_SIZE && !oversized &&
      decodeTelemetry(raw, count, telemetry) && BLE.connected() && telemetryChar.subscribed())
    telemetryChar.writeValue(raw, sizeof(raw));
  // Never fabricate telemetry or replay commands. The Teensy owns the watchdog.
}
