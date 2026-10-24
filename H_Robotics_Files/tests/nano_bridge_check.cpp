// Execute the real Nano sketch with host BLE/I2C substitutes.
#include "../../NeuroExoFirmware/src/nanoBridge.cpp"
#include <cassert>
#include <iostream>
int main() {
  setup();
  uint8_t command[CONTROL_SIZE];
  assert(encodeControl({Mode::Assistive, Speed::High, -123}, command));
  commandChar.inject({command, command + CONTROL_SIZE});
  loop();
  assert(Wire.writes == 1 && Wire.sent == std::vector<uint8_t>(command, command + CONTROL_SIZE));
  // No command replay on future loops, including a full watchdog interval.
  for (int i = 0; i < 300; ++i) { ++fakeMillis; loop(); }
  assert(Wire.writes == 1);
  assert(telemetryChar.notifications == 0); // Empty I2C reads never fabricate telemetry.
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
  std::cout << "Nano bridge: exact I2C relay, invalid frame discard, no replay/fabrication, disconnect PASS\n";
}
