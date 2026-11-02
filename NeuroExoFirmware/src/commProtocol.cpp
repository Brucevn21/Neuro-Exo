#include "commProtocol.h"

namespace neuroexo {
uint8_t crc8(const uint8_t* data, size_t length) {
  uint8_t crc = 0;
  for (size_t i = 0; i < length; ++i) {
    crc ^= data[i];
    for (uint8_t bit = 0; bit < 8; ++bit)
      crc = uint8_t((crc & 0x80) ? (crc << 1) ^ 0x07 : crc << 1);
  }
  return crc;
}
namespace {
int16_t read16(const uint8_t* p) {
  const uint16_t bits = (uint16_t(p[0]) << 8) | p[1];
  return int16_t(bits < 0x8000 ? int32_t(bits) : int32_t(bits) - 65536);
}
void write16(uint8_t* p, int16_t value) {
  const uint16_t bits = uint16_t(value);
  p[0] = uint8_t(bits >> 8); p[1] = uint8_t(bits);
}
bool valid(const uint8_t* p, size_t length, uint8_t type, size_t expected) {
  return p && length == expected && p[0] == STX && p[1] == type &&
         p[2] == expected - 5 && p[expected - 1] == ETX &&
         p[expected - 2] == crc8(p + 1, expected - 3);
}
void finish(uint8_t* p, size_t length) {
  p[length - 2] = crc8(p + 1, length - 3); p[length - 1] = ETX;
}
}
bool encodeControl(const Control& command, uint8_t* p) {
  if (!p || uint8_t(command.mode) > 2 || uint8_t(command.speed) > 2) return false;
  p[0] = STX; p[1] = CONTROL_TYPE; p[2] = 4;
  p[3] = uint8_t(command.mode); p[4] = uint8_t(command.speed);
  write16(p + 5, command.targetAngleDeg); finish(p, CONTROL_SIZE);
  return true;
}
bool decodeControl(const uint8_t* p, size_t length, Control& command) {
  if (!valid(p, length, CONTROL_TYPE, CONTROL_SIZE) || p[3] > 2 || p[4] > 2) return false;
  command = {Mode(p[3]), Speed(p[4]), read16(p + 5)};
  return true;
}
void encodeTelemetry(const Telemetry& telemetry, uint8_t* p) {
  p[0] = STX; p[1] = TELEMETRY_TYPE; p[2] = 5;
  write16(p + 3, telemetry.currentAngleDeg); write16(p + 5, telemetry.currentMilliAmps);
  p[7] = telemetry.status; finish(p, TELEMETRY_SIZE);
}
bool decodeTelemetry(const uint8_t* p, size_t length, Telemetry& telemetry) {
  if (!valid(p, length, TELEMETRY_TYPE, TELEMETRY_SIZE)) return false;
  // Preserve reserved status bits so they remain visible to the host.
  telemetry = {read16(p + 3), read16(p + 5), p[7]};
  return true;
}
}  // namespace neuroexo
