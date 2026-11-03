#include "Trial.hpp"
#include <cassert>
#include <iostream>
using namespace trial;
int main() {
  TelemetryPacket packet;
  packet.currentAngleDeg = -12; packet.currentMilliAmps = -123;
  packet.patientEffortCentiNm = -456;
  packet.status = uint8_t(TelemetryStatus::MotionActive);
  uint8_t raw[MAX_FRAME_SIZE]; const auto size = encodeTelemetryFrame(packet, raw);
  TelemetryPacket decoded;
  assert(size == 12 && decode(raw, size, decoded));
  assert(decoded.patientEffortCentiNm == -456 && decoded.currentAngleDeg == -12);
  assert(!decode(raw, 10, decoded));
  for (size_t i = 0; i < size; ++i) {
    raw[i] ^= 1; assert(!decode(raw, size, decoded)); raw[i] ^= 1;
  }
  Receiver receiver; const auto start = Clock::now();
  receiver.notification(raw, size, start); assert(receiver.get().seen);
  receiver.begin(start, 1);
  for (unsigned i = 0; i < 50; ++i)
    receiver.notification(raw, size, start + std::chrono::milliseconds(20*i));
  receiver.notification(raw, size, start + std::chrono::seconds(2));
  auto counts = receiver.finish(start + std::chrono::seconds(1));
  assert(counts.received == 50 && counts.motion == 50 && counts.maxGapMs == 20);
  // Identical telemetry is counted as notifications, not called duplicate/lost packets.
  assert(rateMet(250, 250, 1.001, 250, true));
  assert(!rateMet(250, 249, 1, 250, true));
  assert(!rateMet(250, 250, 1.1, 250, true));
  assert(!rateMet(250, 250, 1, 250, false));
  for (auto bytes : {stopFrame(), controlFrame(20, Speed::Slow)}) {
    FrameParser parser; Frame frame;
    for (auto value : bytes) parser.push(value);
    assert(parser.takeFrame(frame));
    if (bytes.size() == 5) assert(isStopFrame(frame));
    else {
      ControlPacket control;
      assert(bytes.size() == 9 && decodeControlFrame(frame, control));
      assert(control.mode == Mode::Neutral && control.targetAngleDeg == 20 && control.setting == 0);
    }
  }
  for (uint32_t id : {0u, 1u, 20u, 32767u, 32768u, 65535u}) {
    FrameParser parser; Frame frame; ControlPacket control;
    for (auto value : sequenceFrame(id)) parser.push(value);
    assert(parser.takeFrame(frame) && decodeControlFrame(frame, control));
    assert(uint16_t(control.targetAngleDeg) == id);
    assert(control.mode == Mode::Neutral && control.setting == 0);
  }
  bool rejected = false;
  try { sequenceFrame(65536); } catch (const std::runtime_error&) { rejected = true; }
  assert(rejected);
  receiver.begin(start, 1);
  raw[size - 2] ^= 1;
  receiver.notification(raw, size, start);
  counts = receiver.finish(start + std::chrono::seconds(1));
  assert(counts.invalid == 1 && counts.received == 0 && counts.maxGapMs == 1000);
  std::cout << "ProtocolDemo codec, notification accounting and TX criteria PASS\n";
}
