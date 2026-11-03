#pragma once
#include "Arduino.h"
#include <functional>
#include <vector>
#include <algorithm>
constexpr int BLEWrite = 1, BLEWriteWithoutResponse = 2, BLERead = 4, BLENotify = 8, BLEWritten = 9;
inline unsigned pollCount = 0, characteristics = 0;
inline std::function<void()> receiveOnPoll;
struct BLEDevice {
  operator bool() const { return true; }
  const char* address() { return "AA:BB:CC:DD:EE:FF"; }
  bool connected() {
    if (pollCount++) return false;
    if (receiveOnPoll) receiveOnPoll();
    return true;
  }
};
class BLECharacteristic {
 public:
  BLECharacteristic(const char*, int, int) {}
  void setEventHandler(int, void (*handler)(BLEDevice, BLECharacteristic)) { handler_ = handler; }
  int readValue(uint8_t* dest, int size) {
    const int length = std::min(size, int(raw.size()));
    std::copy(raw.begin(), raw.begin() + length, dest);
    return length;
  }
  int writeValue(const uint8_t*, int) { ++notifications; return 1; }
  void inject(const uint8_t* data, unsigned size) {
    raw.assign(data, data + size);
    if (handler_) handler_(BLEDevice{}, *this);
  }
  unsigned notifications = 0;
 private:
  std::vector<uint8_t> raw;
  void (*handler_)(BLEDevice, BLECharacteristic) = nullptr;
};
struct BLEService {
  explicit BLEService(const char*) {}
  void addCharacteristic(BLECharacteristic&) { ++characteristics; }
};
struct BLEStub {
  bool begin() { return true; }
  BLEDevice central() { return {}; }
  void setLocalName(const char*) {}
  void setAdvertisedService(BLEService&) {}
  void setConnectionInterval(int, int) {}
  void addService(BLEService&) {}
  void advertise() {}
};
inline BLEStub BLE;
