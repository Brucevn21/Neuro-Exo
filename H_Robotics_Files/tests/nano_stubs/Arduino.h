#pragma once
#include <cmath>
#include <cstdint>
#include <sstream>
#include <string>

inline uint32_t fakeMillis = 0;
inline uint32_t millis() { return fakeMillis; }
inline unsigned long micros() { return fakeMillis * 1000UL; }
inline void delay(unsigned ms) { fakeMillis += ms; }
constexpr int LOW = 0, HIGH = 1, OUTPUT = 1, A3 = 3, HEX = 16;
inline int fakeAnalog = 0, relayState = LOW;
inline int analogRead(int) { return fakeAnalog; }
inline void pinMode(int, int) {}
inline void digitalWrite(int pin, int state) { if (pin == 5) relayState = state; }
inline void noInterrupts() {}
inline void interrupts() {}

struct SerialStub {
  bool open = true;
  std::string output;
  void begin(int) {}
  operator bool() const { return open; }
  template <typename T> void print(const T& value) {
    std::ostringstream stream;
    stream << value;
    output += stream.str();
  }
  void print(uint8_t value) { print(unsigned(value)); }
  template <typename T> void print(const T& value, int) { print(value); }
  void println() { output += '\n'; }
  template <typename T> void println(const T& value) { print(value); println(); }
  template <typename T> void println(const T& value, int n) { print(value, n); println(); }
};
inline SerialStub Serial;
