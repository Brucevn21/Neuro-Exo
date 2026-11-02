#pragma once
#include "commProtocol.h"

// Separate diagnostic service: never interpret a test packet as motor control.
namespace packetdemo {
constexpr const char* SERVICE = "b7e21000-6a2b-4f10-9c31-8b674045a901";
constexpr const char* TX = "b7e21001-6a2b-4f10-9c31-8b674045a901";
constexpr const char* RX = "b7e21002-6a2b-4f10-9c31-8b674045a901";
constexpr const char* CONTROL = "b7e21003-6a2b-4f10-9c31-8b674045a901";
constexpr const char* STATS = "b7e21004-6a2b-4f10-9c31-8b674045a901";
constexpr uint8_t START = 0x30, DATA = 0x31, SNAPSHOT = 0x32;
constexpr uint8_t ECHO = 0x33, REPLY = 0x34, STATISTICS = 0x35;
constexpr size_t PACKET_SIZE = 9, ECHO_SIZE = 10, REPLY_SIZE = 25;
constexpr size_t STATS_WORDS = 15, STATS_SIZE = 5 + 4 * STATS_WORDS;
enum Field {
  Session, Token, NanoReceived, NanoDuplicates, NanoInvalid, QueueDrops,
  I2CErrors, TeensyReceived, TeensyDuplicates, TeensyInvalid,
  EchoAttempts, EchoFailures, Queued, TeensyLastSequence, Ready
};
inline uint32_t read32(const uint8_t* p) {
  return (uint32_t(p[0]) << 24) | (uint32_t(p[1]) << 16) |
         (uint32_t(p[2]) << 8) | p[3];
}
inline void write32(uint8_t* p, uint32_t value) {
  p[0] = uint8_t(value >> 24); p[1] = uint8_t(value >> 16);
  p[2] = uint8_t(value >> 8); p[3] = uint8_t(value);
}
inline void finish(uint8_t* p, uint8_t kind, size_t size) {
  p[0] = neuroexo::STX; p[1] = kind; p[2] = uint8_t(size - 5);
  p[size - 2] = neuroexo::crc8(p + 1, size - 3);
  p[size - 1] = neuroexo::ETX;
}
inline bool valid(const uint8_t* p, size_t size, uint8_t kind, size_t expected) {
  return p && size == expected && p[0] == neuroexo::STX && p[1] == kind &&
         p[2] == size - 5 && p[size - 1] == neuroexo::ETX &&
         p[size - 2] == neuroexo::crc8(p + 1, size - 3);
}
inline void packet(uint8_t* p, uint8_t kind, uint32_t value) {
  write32(p + 3, value); finish(p, kind, PACKET_SIZE);
}
// Deliberately bounded queue. Overflow is counted, never silently overwritten.
template <size_t Capacity> struct Queue {
  uint32_t values[Capacity] = {};
  size_t head = 0, tail = 0, count = 0;
  void clear() { head = tail = count = 0; }
  bool push(uint32_t value) {
    if (count == Capacity) return false;
    values[tail] = value; tail = (tail + 1) % Capacity; ++count; return true;
  }
  bool pop(uint32_t& value) {
    if (!count) return false;
    value = values[head]; head = (head + 1) % Capacity; --count; return true;
  }
};
}  // namespace packetdemo
