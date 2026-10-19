#pragma once
#include <cassert>
#include <cstddef>
#include <cstdint>
#include <vector>
struct WireStub {
  std::vector<uint8_t> sent, response;
  unsigned writes = 0, requests = 0;
  size_t position = 0;
  uint8_t result = 0;
  void begin() {}
  void setClock(unsigned) {}
  void beginTransmission(uint8_t address) { assert(address == 0x08); sent.clear(); }
  size_t write(const uint8_t* p, size_t size) { sent.assign(p, p + size); return size; }
  uint8_t endTransmission() { ++writes; return result; }
  size_t requestFrom(uint8_t address, size_t count) {
    assert(address == 0x08 && count == 10);
    ++requests; position = 0; return response.size();
  }
  int available() { return int(response.size() - position); }
  int read() { return response[position++]; }
};
inline WireStub Wire;
