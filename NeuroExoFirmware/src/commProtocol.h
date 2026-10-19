#pragma once
#include <stddef.h>
#include <stdint.h>

// Shared by the Nano bridge and BeagleBone client. No Arduino dependency.
namespace neuroexo {
constexpr uint8_t STX = 0x02, ETX = 0x03;
constexpr uint8_t CONTROL_TYPE = 0x10, TELEMETRY_TYPE = 0x11;
constexpr size_t CONTROL_SIZE = 9, TELEMETRY_SIZE = 10;
constexpr uint8_t TEENSY_ADDRESS = 0x08;
constexpr uint32_t COMMAND_TIMEOUT_MS = 250, TELEMETRY_PERIOD_MS = 40;
constexpr const char* SERVICE_UUID = "b7e20000-6a2b-4f10-9c31-8b674045a901";
constexpr const char* COMMAND_UUID = "b7e20001-6a2b-4f10-9c31-8b674045a901";
constexpr const char* TELEMETRY_UUID = "b7e20002-6a2b-4f10-9c31-8b674045a901";
enum class Mode : uint8_t { Resistive = 0, Assistive = 1, Neutral = 2 };
enum class Speed : uint8_t { Slow = 0, Medium = 1, High = 2 };
enum Status : uint8_t {
  MotionActive = 1 << 0, CommandTimeout = 1 << 1,
  InvalidCommand = 1 << 2, SafetyFault = 1 << 3
};
struct Control { Mode mode; Speed speed; int16_t targetAngleDeg; };
struct Telemetry {
  int16_t currentAngleDeg;
  int16_t currentMilliAmps;
  uint8_t status;
  bool has(Status bit) const { return (status & bit) != 0; }
};
// CRC-8/SMBUS: poly 0x07, initial 0, no reflection, final XOR 0.
uint8_t crc8(const uint8_t* data, size_t length);
bool encodeControl(const Control& command, uint8_t* output);
bool decodeControl(const uint8_t* data, size_t length, Control& command);
void encodeTelemetry(const Telemetry& telemetry, uint8_t* output);
bool decodeTelemetry(const uint8_t* data, size_t length, Telemetry& telemetry);
}  // namespace neuroexo
