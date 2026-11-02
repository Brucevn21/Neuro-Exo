// Communication-only demo, built by the packet_demo / packet_loopback environments.
#include <Arduino.h>
#include <Wire.h>
#include "packetDemoProtocol.h"

#if defined(PACKET_DEMO_NANO)
#include <ArduinoBLE.h>
using namespace packetdemo;
BLEService demoService(SERVICE);
BLECharacteristic txChar(TX, BLEWrite | BLEWriteWithoutResponse, 20);
BLECharacteristic rxChar(RX, BLENotify, ECHO_SIZE, true);
BLECharacteristic controlChar(CONTROL, BLEWrite, 20);
BLECharacteristic statsChar(STATS, BLERead, STATS_SIZE, true);
Queue<64> pending;
uint32_t counters[STATS_WORDS] = {};
uint32_t lastSequence = 0, lastDataMs = 0, controlValue = 0;
uint8_t controlKind = 0;

void onData(BLEDevice, BLECharacteristic characteristic) {
  // ArduinoBLE invokes this for every write, including several in one poll().
  // No I2C, printing, or recursive BLE calls inside this callback.
  const uint8_t* p = characteristic.value();
  if (!counters[Session] ||
      !valid(p, characteristic.valueLength(), DATA, PACKET_SIZE) || !read32(p + 3)) {
    ++counters[NanoInvalid]; return;
  }
  lastDataMs = millis();
  const uint32_t sequence = read32(p + 3);
  if (sequence <= lastSequence) { ++counters[NanoDuplicates]; return; }
  lastSequence = sequence;
  ++counters[NanoReceived];
  if (!pending.push(sequence)) ++counters[QueueDrops];
}

void onControl(BLEDevice, BLECharacteristic characteristic) {
  const uint8_t* p = characteristic.value();
  const size_t size = characteristic.valueLength();
  if ((!valid(p, size, START, PACKET_SIZE) && !valid(p, size, SNAPSHOT, PACKET_SIZE)) ||
      !read32(p + 3) || controlKind) return;
  controlKind = p[1]; controlValue = read32(p + 3);
  // A quiet interval after the request allows already queued traffic to drain.
  lastDataMs = millis();
}

bool sendI2C(uint8_t kind, uint32_t value) {
  uint8_t raw[PACKET_SIZE]; packet(raw, kind, value);
  Wire.beginTransmission(neuroexo::TEENSY_ADDRESS);
  const size_t size = Wire.write(raw, sizeof(raw));
  const uint8_t error = Wire.endTransmission();
  if (size == sizeof(raw) && error == 0) return true;
  ++counters[I2CErrors]; return false;
}

bool readTeensy() {
  uint8_t raw[REPLY_SIZE];
  const size_t received = Wire.requestFrom(neuroexo::TEENSY_ADDRESS, REPLY_SIZE);
  size_t count = 0;
  while (Wire.available()) {
    const int byte = Wire.read();
    if (count < sizeof(raw)) raw[count] = uint8_t(byte);
    ++count;
  }
  if (received != sizeof(raw) || !valid(raw, count, REPLY, sizeof(raw)) ||
      read32(raw + 3) != counters[Session]) {
    ++counters[I2CErrors]; return false;
  }
  counters[TeensyReceived] = read32(raw + 7);
  counters[TeensyDuplicates] = read32(raw + 11);
  counters[TeensyInvalid] = read32(raw + 15);
  counters[TeensyLastSequence] = read32(raw + 19);
  return true;
}

void publishStats(uint32_t token, bool ready) {
  counters[Token] = token; counters[Queued] = pending.count;
  // 2 identifies Nano-only loopback; 1 is the original Teensy route.
#if defined(PACKET_DEMO_LOOPBACK)
  counters[Ready] = ready ? 2 : 0;
#else
  counters[Ready] = ready ? 1 : 0;
#endif
  uint8_t raw[STATS_SIZE];
  for (size_t i = 0; i < STATS_WORDS; ++i) write32(raw + 3 + i * 4, counters[i]);
  finish(raw, STATISTICS, sizeof(raw));
  // Read-only immutable snapshot: safe even when BLE reads it in MTU fragments.
  statsChar.writeValue(raw, sizeof(raw));
}

void disconnected(BLEDevice) { BLE.advertise(); }

void setup() {
  // Demo boards are USB-powered; the motor supply relay stays disconnected.
  pinMode(5, OUTPUT); digitalWrite(5, LOW);
#if !defined(PACKET_DEMO_LOOPBACK)
  Wire.begin(); Wire.setClock(100000); // Same bus rate as the production bridge.
#endif
  Serial.begin(115200);
  if (!BLE.begin()) { while (true) delay(1000); }
#if defined(PACKET_DEMO_LOOPBACK)
  BLE.setLocalName("NeuroExoLoopback");
#else
  BLE.setLocalName("NeuroExoPacketDemo");
#endif
  BLE.setConnectionInterval(6, 6); // Request 7.5 ms; the central decides.
  BLE.setAdvertisedService(demoService);
  demoService.addCharacteristic(txChar); demoService.addCharacteristic(rxChar);
  demoService.addCharacteristic(controlChar); demoService.addCharacteristic(statsChar);
  txChar.setEventHandler(BLEWritten, onData);
  controlChar.setEventHandler(BLEWritten, onControl);
  BLE.addService(demoService);
  BLE.setEventHandler(BLEDisconnected, disconnected);
  publishStats(0, false);
  BLE.advertise();
}

void loop() {
  BLE.poll();
  if (controlKind == START) {
    pending.clear();
    for (auto& value : counters) value = 0;
    counters[Session] = controlValue; lastSequence = 0;
#if defined(PACKET_DEMO_LOOPBACK)
    const bool ready = true;
#else
    const bool ready = sendI2C(START, controlValue) && readTeensy();
#endif
    publishStats(controlValue, ready);
    controlKind = 0;
  }
  uint32_t sequence;
  if (pending.pop(sequence)) {
#if defined(PACKET_DEMO_LOOPBACK)
    uint8_t raw[ECHO_SIZE]; write32(raw + 3, sequence); raw[7] = 1;
    finish(raw, ECHO, sizeof(raw));
    ++counters[EchoAttempts];
    if (!BLE.connected() || !rxChar.subscribed() || !rxChar.writeValue(raw, sizeof(raw)))
      ++counters[EchoFailures];
#else
    if (sendI2C(DATA, sequence) && readTeensy()) {
      // An echo requires an actual valid Teensy response for this sequence.
      if (counters[TeensyLastSequence] == sequence) {
        uint8_t raw[ECHO_SIZE]; write32(raw + 3, sequence); raw[7] = 1;
        finish(raw, ECHO, sizeof(raw));
        ++counters[EchoAttempts];
        if (!BLE.connected() || !rxChar.subscribed() || !rxChar.writeValue(raw, sizeof(raw)))
          ++counters[EchoFailures];
      } else ++counters[I2CErrors];
    }
#endif
  }
  if (controlKind == SNAPSHOT && !pending.count && uint32_t(millis() - lastDataMs) >= 100) {
#if defined(PACKET_DEMO_LOOPBACK)
    const bool ready = true;
#else
    const bool ready = readTeensy();
#endif
    publishStats(controlValue, ready);
    controlKind = 0;
  }
}

#elif defined(PACKET_DEMO_TEENSY)
#include "pinMap4.1.h"
using namespace packetdemo;
uint32_t session = 0, received = 0, duplicates = 0, invalid = 0, lastSequence = 0;

void onReceive(int reported) {
  uint8_t raw[32]; size_t count = 0;
  while (Wire.available()) {
    const int byte = Wire.read();
    if (count < sizeof(raw)) raw[count] = uint8_t(byte);
    ++count;
  }
  if (reported != int(count)) { ++invalid; return; }
  if (valid(raw, count, START, PACKET_SIZE) && read32(raw + 3)) {
    session = read32(raw + 3); received = duplicates = invalid = lastSequence = 0;
  } else if (session && valid(raw, count, DATA, PACKET_SIZE) && read32(raw + 3)) {
    const uint32_t sequence = read32(raw + 3);
    if (sequence <= lastSequence) ++duplicates;
    else { lastSequence = sequence; ++received; }
  } else ++invalid;
  // Bounded diagnostic validation only. No motor actions, waiting, or logging.
}

void onRequest() {
  uint8_t raw[REPLY_SIZE];
  write32(raw + 3, session); write32(raw + 7, received);
  write32(raw + 11, duplicates); write32(raw + 15, invalid);
  write32(raw + 19, lastSequence); finish(raw, REPLY, sizeof(raw));
  Wire.write(raw, sizeof(raw));
}

void setup() {
  pinMode(ENABLE_PIN, OUTPUT); digitalWrite(ENABLE_PIN, LOW);
  pinMode(PWM_PIN, OUTPUT); digitalWrite(PWM_PIN, LOW);
  Wire.begin(neuroexo::TEENSY_ADDRESS);
  Wire.onReceive(onReceive); Wire.onRequest(onRequest);
}
void loop() {}
#else
#error "Build with nano33ble_packet_demo or teensy41_packet_demo"
#endif
