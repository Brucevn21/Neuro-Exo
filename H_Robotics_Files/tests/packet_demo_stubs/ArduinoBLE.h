#pragma once
#include "Arduino.h"
#include <vector>
constexpr int BLEWrite = 1, BLENotify = 2, BLERead = 4, BLEWriteWithoutResponse = 8;
constexpr int BLEDisconnected = 1, BLEWritten = 2;
struct BLEDevice {};
class BLECharacteristic {
 public:
  using Callback = void (*)(BLEDevice, BLECharacteristic);
  BLECharacteristic(const char*, int properties, int capacity, bool = false)
      : properties(properties), capacity(capacity) {}
  void setEventHandler(int, Callback handler) { callback = handler; }
  const uint8_t* value() const { return raw.data(); }
  int valueLength() const { return int(raw.size()); }
  bool subscribed() const { return subscribed_; }
  void inject(std::vector<uint8_t> bytes) {
    if (bytes.size() > size_t(capacity)) bytes.resize(capacity);
    raw = bytes;
    if (callback) callback({}, *this);
  }
  int writeValue(const uint8_t* bytes, int size) {
    raw.assign(bytes, bytes + size);
    if (properties & BLENotify) {
      if (failNotify) return 0;
      notifications.push_back(raw);
    }
    return 1;
  }
  std::vector<uint8_t> raw;
  std::vector<std::vector<uint8_t>> notifications;
  bool subscribed_ = true, failNotify = false;
 private:
  int properties, capacity;
  Callback callback = nullptr;
};
struct BLEService {
  explicit BLEService(const char*) {}
  void addCharacteristic(BLECharacteristic&) {}
};
struct BLEStub {
  bool begin() { return true; }
  void poll() {}
  void advertise() {}
  bool connected() { return true; }
  void setLocalName(const char*) {}
  void setConnectionInterval(int, int) {}
  void setAdvertisedService(BLEService&) {}
  void addService(BLEService&) {}
  void setEventHandler(int, void (*)(BLEDevice)) {}
};
inline BLEStub BLE;
