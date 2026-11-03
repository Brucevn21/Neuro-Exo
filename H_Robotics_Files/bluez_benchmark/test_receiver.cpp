// Execute the actual sketch with isolated hardware stubs, not a copy of its logic.
#define BLE_PACKET_TEST 1
#define BLE_PACKET_TEST_RX_ONLY 1
#include "../../NeuroExoFirmware/src/Nano33BLEFirmware.ino"
#include <cassert>
#include <iostream>

int main() {
  fakeAnalog = 675;  // A healthy rail would enable the relay in normal mode.
  setup();
  assert(characteristics == 1 && Wire.begins == 0 && relayState == LOW);
  assert(Serial.output.find("RX-ONLY BENCHMARK") != std::string::npos);
  fakeMillis = 600;
  receiveOnPoll = [] {
    uint8_t bytes[MAX_FRAME_SIZE];
    for (int id : {0, 1, 3, 3}) {
      ControlPacket packet;
      packet.mode = Mode::Neutral;
      packet.setting = 0;
      packet.targetAngleDeg = id;
      commandChar.inject(bytes, encodeControlFrame(packet, bytes));
    }
  };
  loop();
  assert(Wire.requests == 0 && Wire.writes == 0 && telemetryChar.notifications == 0);
  assert(relayState == LOW);
  assert(testStats.bleWrites == 4 && testStats.uniqueReceived == 3 && testStats.duplicates == 1);
  assert(expectedPackets() == 4 && lostPackets() == 1);
  assert(Serial.output.find("final (disconnected)") != std::string::npos);
  std::cout << "RX-only firmware: sequence counters, no telemetry/I2C, relay OFF PASS\n";
}
