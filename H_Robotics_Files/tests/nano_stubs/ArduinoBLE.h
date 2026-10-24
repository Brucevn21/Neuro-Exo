#pragma once
#include <algorithm>
#include <cstdint>
#include <vector>
inline uint32_t fakeMillis = 0;
inline uint32_t millis() { return fakeMillis; }
inline void delay(unsigned ms) { fakeMillis += ms; }
struct SerialStub {
  void begin(int) {}
  operator bool() const { return true; }
  void println(const char*) {}
};
inline SerialStub Serial;
constexpr int BLEWrite = 1, BLENotify = 2, BLEDisconnected = 3;
struct BLEDevice {};
class BLECharacteristic {
 public:
  BLECharacteristic(const char*, int, int capacity, bool = false) : capacity_(capacity) {}
  void inject(std::vector<uint8_t> value) {
    if (value.size() > size_t(capacity_)) value.resize(capacity_);
    raw = value; written_ = true;
  }
  bool written() { bool value = written_; written_ = false; return value; }
  const uint8_t* value() { return raw.data(); }
  int valueLength() { return int(raw.size()); }
  bool subscribed() { return subscribed_; }
  int writeValue(const uint8_t* p, int size) {
    raw.assign(p, p + size); ++notifications; return 1;
  }
  std::vector<uint8_t> raw;
  unsigned notifications = 0;
  bool subscribed_ = true;
 private:
  int capacity_;
  bool written_ = false;
};
struct BLEService {
  explicit BLEService(const char*) {}
  void addCharacteristic(BLECharacteristic&) {}
};
struct BLEStub {
  bool online = true;
  bool begin() { return true; }
  void poll() {}
  void advertise() {}
  bool connected() { return online; }
  void setLocalName(const char*) {}
  void setAdvertisedService(BLEService&) {}
  void addService(BLEService&) {}
  void setEventHandler(int, void (*)(BLEDevice)) {}
};
inline BLEStub BLE;
