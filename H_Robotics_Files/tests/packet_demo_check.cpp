// Both real demo firmware branches connected through an in-memory I2C bus.
#include <Arduino.h>
#include <ArduinoBLE.h>
#include <Wire.h>
#include "../../NeuroExoFirmware/src/packetDemoProtocol.h"
#include "pinMap4.1.h"
#include <cassert>
#include <iomanip>
#include <iostream>

namespace teensy {
#define PACKET_DEMO_TEENSY
#include "../../NeuroExoFirmware/src/packetDemo.cpp"
#undef PACKET_DEMO_TEENSY
}
namespace nano {
#define PACKET_DEMO_NANO
#include "../../NeuroExoFirmware/src/packetDemo.cpp"
#undef PACKET_DEMO_NANO
}
namespace loopback {
#define PACKET_DEMO_NANO
#define PACKET_DEMO_LOOPBACK
#include "../../NeuroExoFirmware/src/packetDemo.cpp"
#undef PACKET_DEMO_LOOPBACK
#undef PACKET_DEMO_NANO
}
using namespace packetdemo;
void inject(BLECharacteristic& characteristic, uint8_t kind, uint32_t value) {
  uint8_t raw[PACKET_SIZE]; packet(raw, kind, value);
  characteristic.inject({raw, raw + sizeof(raw)});
}
void start(uint32_t session) {
  inject(nano::controlChar, START, session); nano::loop();
  assert(nano::counters[Ready] == 1 && teensy::received == 0);
}
void snapshot() {
  inject(nano::controlChar, SNAPSHOT, 1234);
  fakeMillis += 101; nano::loop();
  assert(valid(nano::statsChar.raw.data(), nano::statsChar.raw.size(), STATISTICS, STATS_SIZE));
  assert(nano::counters[Token] == 1234 && nano::counters[Queued] == 0);
}
void hex(const uint8_t* raw, size_t size) {
  for (size_t i = 0; i < size; ++i) std::cout << std::hex << std::setw(2) << std::setfill('0') << unsigned(raw[i]);
  std::cout << '\n';
}
int main(int argc, char**) {
  if (argc > 1) {
    uint8_t raw[STATS_SIZE] = {};
    packet(raw, DATA, 0x01020304); hex(raw, PACKET_SIZE);
    write32(raw + 3, 0x01020304); raw[7] = 1; finish(raw, ECHO, ECHO_SIZE); hex(raw, ECHO_SIZE);
    for (size_t i = 0; i < STATS_WORDS; ++i) write32(raw + 3 + i * 4, uint32_t(i + 100));
    finish(raw, STATISTICS, STATS_SIZE); hex(raw, STATS_SIZE);
    return 0;
  }
  teensy::setup(); nano::setup();
  assert(relayState == LOW);
  start(1);
  for (uint32_t sequence = 1; sequence <= 1000; ++sequence) {
    fakeMillis += 4;
    inject(nano::txChar, DATA, sequence); nano::loop();
    assert(teensy::received == sequence);
    assert(read32(nano::rxChar.raw.data() + 3) == sequence);
  }
  snapshot();
  assert(nano::counters[NanoReceived] == 1000 && nano::counters[TeensyReceived] == 1000);
  assert(nano::counters[EchoAttempts] == 1000 && nano::counters[I2CErrors] == 0);
  // Duplicate and corrupt frames must not increment valid counts or echo.
  inject(nano::txChar, DATA, 1000); nano::loop();
  assert(nano::counters[NanoDuplicates] == 1);
  nano::txChar.inject({2, DATA, 4, 0, 0, 0, 1, 0, 3}); nano::loop();
  assert(nano::counters[NanoInvalid] == 1 && teensy::received == 1000);
  // More writes in one BLE poll than queue capacity: all arrivals accounted for.
  start(2);
  for (uint32_t sequence = 1; sequence <= 100; ++sequence) inject(nano::txChar, DATA, sequence);
  for (unsigned i = 0; i < 64; ++i) nano::loop();
  snapshot();
  assert(nano::counters[NanoReceived] == 100 && nano::counters[QueueDrops] == 36);
  assert(nano::counters[TeensyReceived] == 64 && nano::counters[EchoAttempts] == 64);
  // Failed forward transaction versus failed return transaction are distinguishable.
  start(3);
  Wire.failNext = true; inject(nano::txChar, DATA, 1); nano::loop();
  Wire.corruptNextReply = true; inject(nano::txChar, DATA, 2); nano::loop();
  nano::rxChar.failNotify = true; inject(nano::txChar, DATA, 3); nano::loop();
  nano::rxChar.failNotify = false; inject(nano::txChar, DATA, 4); nano::loop();
  snapshot();
  assert(nano::counters[NanoReceived] == 4 && nano::counters[TeensyReceived] == 3);
  assert(nano::counters[I2CErrors] == 2 && nano::counters[EchoAttempts] == 2);
  assert(nano::counters[EchoFailures] == 1);
  // Starting against an absent Teensy cannot produce a ready session.
  Wire.failNext = true; inject(nano::controlChar, START, 4); nano::loop();
  assert(nano::counters[Ready] == 0);
  // Nano-only image must work with no I2C peer and must identify its route.
  Wire.receive = nullptr; Wire.request = nullptr; Wire.failNext = true;
  loopback::setup();
  inject(loopback::controlChar, START, 42); loopback::loop();
  assert(loopback::counters[Ready] == 2 && relayState == LOW);
  for (uint32_t sequence = 1; sequence <= 2500; ++sequence) {
    fakeMillis += 4;
    inject(loopback::txChar, DATA, sequence); loopback::loop();
    assert(valid(loopback::rxChar.raw.data(), loopback::rxChar.raw.size(), ECHO, ECHO_SIZE));
    assert(read32(loopback::rxChar.raw.data() + 3) == sequence);
  }
  inject(loopback::controlChar, SNAPSHOT, 43); fakeMillis += 101; loopback::loop();
  assert(loopback::counters[NanoReceived] == 2500 && loopback::counters[EchoAttempts] == 2500);
  assert(loopback::counters[TeensyReceived] == 0 && loopback::counters[I2CErrors] == 0);
  assert(Wire.failNext); // No I2C transaction consumed the sentinel failure.
  inject(loopback::txChar, DATA, 2500); loopback::loop();
  assert(loopback::counters[NanoDuplicates] == 1);
  loopback::rxChar.failNotify = true;
  inject(loopback::txChar, DATA, 2501); loopback::loop();
  assert(loopback::counters[EchoFailures] == 1);
  inject(loopback::controlChar, START, 44); loopback::loop();
  for (uint32_t sequence = 1; sequence <= 100; ++sequence) inject(loopback::txChar, DATA, sequence);
  assert(loopback::counters[QueueDrops] == 36);
  std::cout << "Packet demo: full route, queue overflow, corruption, duplicate, I2C/notify failures PASS\n";
}
