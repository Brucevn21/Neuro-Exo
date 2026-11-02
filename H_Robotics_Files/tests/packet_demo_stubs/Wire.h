#pragma once
#include "Arduino.h"
#include <algorithm>
#include <cassert>
#include <vector>
struct WireStub {
  std::vector<uint8_t> outgoing, incoming, response;
  size_t position = 0;
  bool replying = false, failNext = false, corruptNextReply = false;
  void (*receive)(int) = nullptr;
  void (*request)() = nullptr;
  void begin() {}
  void begin(int) {}
  void setClock(int) {}
  void onReceive(void (*callback)(int)) { receive = callback; }
  void onRequest(void (*callback)()) { request = callback; }
  void beginTransmission(int address) { assert(address == 8); outgoing.clear(); }
  size_t write(const uint8_t* bytes, size_t size) {
    (replying ? response : outgoing).assign(bytes, bytes + size);
    return size;
  }
  int available() { return int(incoming.size() - position); }
  int read() { return incoming[position++]; }
  uint8_t endTransmission() {
    if (failNext) { failNext = false; return 2; }
    incoming = outgoing; position = 0;
    if (receive) receive(int(incoming.size()));
    return 0;
  }
  size_t requestFrom(int address, size_t size) {
    assert(address == 8);
    response.clear(); replying = true;
    if (request) request();
    replying = false;
    if (corruptNextReply && !response.empty()) { response[3] ^= 1; corruptNextReply = false; }
    incoming.assign(response.begin(), response.begin() + std::min(size, response.size()));
    position = 0; return incoming.size();
  }
};
inline WireStub Wire;
