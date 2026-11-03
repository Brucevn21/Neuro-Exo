#pragma once
#include "commProtocol.h"
#include <algorithm>
#include <chrono>
#include <mutex>
#include <stdexcept>
#include <vector>

namespace trial {
using Clock = std::chrono::steady_clock;
using namespace NeuroExoProtocol;
inline bool decode(const uint8_t* raw, size_t size, TelemetryPacket& packet) {
  if (!raw || size != FRAME_OVERHEAD + TELEMETRY_PAYLOAD_SIZE || raw[0] != START_BYTE ||
      raw[1] != MESSAGE_TELEMETRY || raw[2] != TELEMETRY_PAYLOAD_SIZE || raw[size - 1] != STOP_BYTE)
    return false;
  FrameParser parser;
  for (size_t i = 0; i < size; ++i) parser.push(raw[i]);
  Frame frame;
  return parser.takeFrame(frame) && decodeTelemetryFrame(frame, packet);
}
struct Counts {
  uint32_t received = 0, invalid = 0, motion = 0;
  bool seen = false, fault = false;
  double maxGapMs = 0;
  TelemetryPacket latest;
  Clock::time_point last{};
};
class Receiver {
 public:
  void begin(Clock::time_point start, double duration) {
    std::lock_guard<std::mutex> lock(mutex_);
    counts_.received = counts_.invalid = counts_.motion = 0;
    counts_.fault = false; counts_.maxGapMs = 0;
    start_ = start; previous_ = start;
    end_ = start + std::chrono::duration_cast<Clock::duration>(std::chrono::duration<double>(duration));
    measuring_ = true;
  }
  void notification(const uint8_t* raw, size_t size, Clock::time_point now = Clock::now()) {
    std::lock_guard<std::mutex> lock(mutex_);
    const bool inWindow = measuring_ && now >= start_ && now < end_;
    TelemetryPacket value;
    if (!decode(raw, size, value)) { if (inWindow) ++counts_.invalid; return; }
    counts_.latest = value; counts_.last = now; counts_.seen = true;
    if (inWindow) {
      ++counts_.received;
      if (value.status & uint8_t(TelemetryStatus::MotionActive)) ++counts_.motion;
      if (value.status & (uint8_t(TelemetryStatus::SafetyFault) | uint8_t(TelemetryStatus::InvalidCommand))) counts_.fault = true;
      counts_.maxGapMs = std::max(counts_.maxGapMs, std::chrono::duration<double, std::milli>(now - previous_).count());
      previous_ = now;
    }
  }
  Counts get() const { std::lock_guard<std::mutex> lock(mutex_); return counts_; }
  Counts finish(Clock::time_point now) {
    std::lock_guard<std::mutex> lock(mutex_);
    if (measuring_) counts_.maxGapMs = std::max(counts_.maxGapMs,
        std::chrono::duration<double, std::milli>(std::min(now, end_) - previous_).count());
    measuring_ = false;
    return counts_;
  }
 private:
  mutable std::mutex mutex_;
  Counts counts_;
  Clock::time_point start_{}, end_{}, previous_{};
  bool measuring_ = false;
};
inline bool rateMet(uint32_t planned, uint32_t sent, double elapsed, unsigned rate, bool complete) {
  return complete && sent == planned && elapsed > 0 && sent / elapsed >= rate * 0.99;
}
inline std::vector<uint8_t> stopFrame() {
  std::vector<uint8_t> result(FRAME_OVERHEAD);
  encodeStopFrame(result.data()); return result;
}
inline std::vector<uint8_t> controlFrame(int16_t angle, Speed speed) {
  ControlPacket packet; packet.mode = Mode::Neutral;
  packet.setting = uint8_t(speed); packet.targetAngleDeg = angle;
  std::vector<uint8_t> result(FRAME_OVERHEAD + CONTROL_PAYLOAD_SIZE);
  encodeControlFrame(packet, result.data()); return result;
}
// IDs advance per attempted write, never per scheduled time slot. Skipped host
// slots must not appear as missing IDs in the Nano's receiver-loss calculation.
inline std::vector<uint8_t> sequenceFrame(uint32_t id) {
  if (id > 65535) throw std::runtime_error("sequence test is limited to 65536 packets per connection");
  const int32_t signedId = id < 32768 ? int32_t(id) : int32_t(id) - 65536;
  return controlFrame(int16_t(signedId), Speed::Slow);
}
}
