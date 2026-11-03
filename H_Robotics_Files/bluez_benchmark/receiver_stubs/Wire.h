#pragma once
#include <cstdint>
struct WireStub {
  unsigned begins = 0, writes = 0, requests = 0;
  void begin() { ++begins; }
  void beginTransmission(int) {}
  void write(const uint8_t*, uint8_t) { ++writes; }
  uint8_t endTransmission() { return 0; }
  int requestFrom(int, int) { ++requests; return 0; }
  int read() { return 0; }
};
using byte = uint8_t;
inline WireStub Wire;
