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
uint32_t lastDiagnosticsMs = 0;
bool serialWasOpen = false;

// Power management from ProtocolDemo. D5 drives the relay module; A3 senses
// the rail through the fitted 101k/9.9k divider. LOW disconnects the load.
constexpr int PMS_ANALOG_PIN = A3, PMS_RELAY_PIN = 5;
constexpr float PMS_R1 = 101000.0f, PMS_R2 = 9900.0f;
constexpr float PMS_THRESHOLD_VOLTAGE = 22.0f, PMS_OVERVOLTAGE_CUTOFF = 30.0f;
constexpr float PMS_HYSTERESIS = 0.5f;
constexpr uint32_t PMS_CHECK_INTERVAL_MS = 500, DIAGNOSTIC_INTERVAL_MS = 1000;
uint32_t lastPMSCheckMs = 0;

void checkPowerSupply() {
  const uint32_t now = millis();
  if (uint32_t(now - lastPMSCheckMs) < PMS_CHECK_INTERVAL_MS) return;
  lastPMSCheckMs = now;
  const float railVoltage = analogRead(PMS_ANALOG_PIN) * (3.3f / 1023.0f) *
                            ((PMS_R1 + PMS_R2) / PMS_R2);
  if (railVoltage < PMS_THRESHOLD_VOLTAGE || railVoltage > PMS_OVERVOLTAGE_CUTOFF)
    digitalWrite(PMS_RELAY_PIN, LOW);
  else if (railVoltage > PMS_THRESHOLD_VOLTAGE + PMS_HYSTERESIS &&
           railVoltage < PMS_OVERVOLTAGE_CUTOFF - PMS_HYSTERESIS)
    digitalWrite(PMS_RELAY_PIN, HIGH);
}

void printDiagnostics(bool validTelemetry, const Telemetry& telemetry) {
  const bool serialOpen = bool(Serial);
  if (serialOpen && !serialWasOpen)
    Serial.println("[Nano] Ready: NanoNeuroExo BLE bridge, Teensy I2C address 0x08");
  const bool newlyOpened = serialOpen && !serialWasOpen;
  serialWasOpen = serialOpen;
  const uint32_t now = millis();
  if (!serialOpen || (!newlyOpened &&
      uint32_t(now - lastDiagnosticsMs) < DIAGNOSTIC_INTERVAL_MS)) return;
  lastDiagnosticsMs = now;
  Serial.print("[Nano] BLE=");
  Serial.print(BLE.connected() ? "connected" : "advertising");
  Serial.print(" subscribed=");
  Serial.print(telemetryChar.subscribed() ? 1 : 0);
  if (!validTelemetry) {
    Serial.println(" | Waiting for valid Teensy telemetry (check I2C/firmware)");
    return;
  }
  Serial.print(" | angle=");
  Serial.print(telemetry.currentAngleDeg);
  Serial.print(" deg current=");
  Serial.print(telemetry.currentMilliAmps);
  Serial.print(" mA status=0x");
  Serial.print(telemetry.status, HEX);
  Serial.print(" MotionActive=");
  Serial.print(telemetry.has(MotionActive) ? 1 : 0);
  Serial.print(" CommandTimeout=");
  Serial.print(telemetry.has(CommandTimeout) ? 1 : 0);
  Serial.print(" InvalidCommand=");
  Serial.print(telemetry.has(InvalidCommand) ? 1 : 0);
  Serial.print(" SafetyFault=");
  Serial.println(telemetry.has(SafetyFault) ? 1 : 0);
}

void onDisconnect(BLEDevice central) {
  (void)central;
  BLE.advertise();
}

void setup() {
  pinMode(PMS_RELAY_PIN, OUTPUT);
  digitalWrite(PMS_RELAY_PIN, LOW);
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
  checkPowerSupply();
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
      else if (Serial) {
        Serial.print("[Nano] Control relayed: mode=");
        Serial.print(uint8_t(command.mode));
        Serial.print(" speed=");
        Serial.print(uint8_t(command.speed));
        Serial.print(" target=");
        Serial.println(command.targetAngleDeg);
      }
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
  Telemetry telemetry{};
  const bool validTelemetry = received == TELEMETRY_SIZE && !oversized &&
                              decodeTelemetry(raw, count, telemetry);
  if (validTelemetry && BLE.connected() && telemetryChar.subscribed())
    telemetryChar.writeValue(raw, sizeof(raw));
  printDiagnostics(validTelemetry, telemetry);
  // Never fabricate telemetry or replay commands. The Teensy owns the watchdog.
}
