// Execute the real Nano sketch with host BLE/I2C substitutes.
#include "../../NeuroExoFirmware/src/nanoBridge.cpp"
#include <cassert>
#include <iostream>
int main() {
  setup();
  assert(relayState == LOW);
  uint8_t command[CONTROL_SIZE];
  assert(encodeControl({Mode::Assistive, Speed::High, -123}, command));
  commandChar.inject({command, command + CONTROL_SIZE});
  loop();
  assert(Wire.writes == 1 && Wire.sent == std::vector<uint8_t>(command, command + CONTROL_SIZE));
  // No command replay on future loops, including a full watchdog interval.
  for (int i = 0; i < 300; ++i) { ++fakeMillis; loop(); }
  assert(Wire.writes == 1);
  assert(telemetryChar.notifications == 0); // Empty I2C reads never fabricate telemetry.
  assert(Serial.output.find("Waiting for valid Teensy telemetry") != std::string::npos);
  std::vector<std::vector<uint8_t>> invalid;
  for (size_t i = 0; i < CONTROL_SIZE; ++i) invalid.emplace_back(command, command + i);
  auto oversized = std::vector<uint8_t>(command, command + CONTROL_SIZE);
  oversized.resize(30, 0);
  invalid.push_back(oversized);
  auto corrupted = std::vector<uint8_t>(command, command + CONTROL_SIZE);
  corrupted[7] ^= 1; invalid.push_back(corrupted);
  for (const auto& raw : invalid) { commandChar.inject(raw); loop(); }
  assert(Wire.writes == 1);
  Wire.response.resize(TELEMETRY_SIZE);
  encodeTelemetry({45, -1200, uint8_t(MotionActive | SafetyFault)}, Wire.response.data());
  fakeMillis += 40; loop();
  assert(telemetryChar.notifications == 1 && telemetryChar.raw == Wire.response);
  Wire.response[8] ^= 1;
  fakeMillis += 40; loop();
  assert(telemetryChar.notifications == 1);
  Wire.response.pop_back();
  fakeMillis += 40; loop();
  assert(telemetryChar.notifications == 1);
  Wire.response.resize(TELEMETRY_SIZE);
  encodeTelemetry({-123, 1200, CommandTimeout}, Wire.response.data());
  BLE.online = false;
  commandChar.inject({command, command + CONTROL_SIZE});
  fakeMillis += 40; loop();
  assert(Wire.writes == 1 && telemetryChar.notifications == 1);
  BLE.online = true;
  fakeMillis += 40; loop();
  assert(Wire.writes == 1 && telemetryChar.notifications == 2);
  // USB diagnostics work even without a BLE central and after opening Serial
  // long after boot. They must not publish stale/corrupt I2C data over BLE.
  BLE.online = false;
  Serial.open = false;
  fakeMillis += 1000; loop();
  Serial.output.clear();
  Serial.open = true;
  fakeMillis += 40; loop();
  assert(Serial.output.find("[Nano] Ready") != std::string::npos);
  assert(Serial.output.find("angle=-123 deg current=1200 mA") != std::string::npos);
  assert(Serial.output.find("CommandTimeout=1") != std::string::npos);
  assert(telemetryChar.notifications == 2);
  Wire.response[8] ^= 1;
  Serial.output.clear();
  fakeMillis += 1000; loop();
  assert(Serial.output.find("Waiting for valid Teensy telemetry") != std::string::npos);
  assert(Serial.output.find("angle=") == std::string::npos);
  // PMS still samples with BLE disconnected. ADC values represent rail volts.
  const auto sampleRail = [](float volts) {
    fakeAnalog = int(std::lround(volts * 1023.0f / 3.3f * PMS_R2 / (PMS_R1 + PMS_R2)));
    fakeMillis += 500; loop();
  };
  sampleRail(24); assert(relayState == HIGH);
  sampleRail(22.2f); assert(relayState == HIGH); // Lower hysteresis band.
  sampleRail(21); assert(relayState == LOW);
  sampleRail(22.2f); assert(relayState == LOW);
  sampleRail(24); assert(relayState == HIGH);
  sampleRail(31); assert(relayState == LOW);
  sampleRail(29.7f); assert(relayState == LOW); // Upper hysteresis band.
  sampleRail(24); assert(relayState == HIGH);
  std::cout << "Nano bridge: exact I2C relay, invalid frame discard, no replay/fabrication, disconnect PASS\n";
}
