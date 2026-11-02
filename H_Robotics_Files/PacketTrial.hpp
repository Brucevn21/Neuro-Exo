#pragma once
#include "../NeuroExoFirmware/src/packetDemoProtocol.h"
#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <mutex>
#include <optional>
#include <stdexcept>
#include <vector>

namespace neuroexo {
using TrialClock = std::chrono::steady_clock;
using PacketStats = std::array<uint32_t, packetdemo::STATS_WORDS>;
inline std::vector<uint8_t> demoPacket(uint8_t kind, uint32_t value) {
  std::vector<uint8_t> raw(packetdemo::PACKET_SIZE);
  packetdemo::packet(raw.data(), kind, value);
  return raw;
}
inline PacketStats decodePacketStats(const std::vector<uint8_t>& raw) {
  if (!packetdemo::valid(raw.data(), raw.size(), packetdemo::STATISTICS, packetdemo::STATS_SIZE))
    throw std::runtime_error("invalid statistics frame/CRC");
  PacketStats stats{};
  for (size_t i = 0; i < stats.size(); ++i) stats[i] = packetdemo::read32(raw.data() + 3 + 4*i);
  return stats;
}
struct TrialResult {
  uint32_t planned = 0, attempted = 0, submitted = 0, received = 0, activeReceived = 0;
  uint32_t duplicates = 0, invalid = 0, unexpected = 0, maxConsecutiveMissing = 0;
  uint64_t scheduleLatenessSamples = 0, writeCallSamples = 0;
  uint64_t roundTripSamples = 0, interArrivalSamples = 0;
  double txHz = 0, rxHz = 0, scheduleLatenessMeanMs = 0, scheduleLatenessMaxMs = 0;
  double writeCallMeanMs = 0, writeCallMaxMs = 0;
  double roundTripMeanMs = 0, roundTripMaxMs = 0;
  double interArrivalMeanMs = 0, interArrivalMaxMs = 0, interArrivalJitterMs = 0;
  std::vector<uint32_t> missing;
  bool consistent = false, pass = false;
};
struct TimingMoments {
  uint64_t count = 0;
  double mean = 0, squaredDeviation = 0, maximum = 0;
  void add(double value) {
    ++count;
    const double delta = value - mean;
    mean += delta / count;
    squaredDeviation += delta * (value - mean);
    maximum = std::max(maximum, value);
  }
  double standardDeviation() const {
    return count > 1 ? std::sqrt(squaredDeviation / count) : 0;
  }
};
// No latest-value mailbox: each echo updates the corresponding numbered slot.
class PacketAccounting {
 public:
  PacketAccounting(unsigned rate, double duration)
      : rate_(rate), duration_(duration), slots_(size_t(std::ceil(rate * duration)) + 1) {}
  void start(TrialClock::time_point now) {
    std::lock_guard<std::mutex> lock(mutex_);
    started_ = now; active_ = true;
  }
  void attempted(uint32_t sequence, TrialClock::time_point scheduledAt,
                 TrialClock::time_point attemptedAt) {
    std::lock_guard<std::mutex> lock(mutex_);
    auto& slot = slots_.at(sequence);
    if (slot.sent) throw std::logic_error("packet sequence attempted more than once");
    slot.sent = true;
    slot.attemptedAt = attemptedAt;
    scheduleLateness_.add(std::max(0.0, std::chrono::duration<double, std::milli>(
        attemptedAt - scheduledAt).count()));
  }
  void submitted(uint32_t sequence, TrialClock::time_point writeStartedAt,
                 TrialClock::time_point submittedAt) {
    std::lock_guard<std::mutex> lock(mutex_);
    auto& slot = slots_.at(sequence);
    if (!slot.sent || slot.submitted) throw std::logic_error("invalid packet submission record");
    slot.submitted = true;
    writeCall_.add(std::chrono::duration<double, std::milli>(
        submittedAt - writeStartedAt).count());
  }
  void notification(const uint8_t* raw, size_t size, TrialClock::time_point now = TrialClock::now()) {
    std::lock_guard<std::mutex> lock(mutex_);
    if (!active_) return;
    if (!packetdemo::valid(raw, size, packetdemo::ECHO, packetdemo::ECHO_SIZE) || raw[7] != 1) {
      ++invalid_; return;
    }
    const auto sequence = packetdemo::read32(raw + 3);
    if (sequence == 0 || sequence >= slots_.size() || !slots_[sequence].sent) {
      ++unexpected_; return;
    }
    auto& slot = slots_[sequence];
    if (slot.echo) { ++duplicates_; return; }
    slot.echo = true;
    slot.inWindow = std::chrono::duration<double>(now - started_).count() < duration_;
    roundTrip_.add(std::chrono::duration<double, std::milli>(now - slot.attemptedAt).count());
    if (slot.inWindow) {
      if (previousArrival_)
        interArrival_.add(std::chrono::duration<double, std::milli>(now - *previousArrival_).count());
      previousArrival_ = now;
    }
    ++received_;
  }
  uint32_t received() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return received_;
  }
  TrialResult finish(uint32_t submitted, double elapsed, const PacketStats& stats, bool complete) {
    std::lock_guard<std::mutex> lock(mutex_);
    active_ = false;
    TrialResult r;
    r.planned = uint32_t(slots_.size() - 1); r.submitted = submitted;
    r.duplicates = duplicates_; r.invalid = invalid_; r.unexpected = unexpected_;
    uint32_t missingRun = 0;
    for (uint32_t i = 1; i <= r.planned; ++i) {
      if (slots_[i].sent) {
        ++r.attempted;
        if (!slots_[i].echo) {
          r.missing.push_back(i);
          r.maxConsecutiveMissing = std::max(r.maxConsecutiveMissing, ++missingRun);
        } else {
          missingRun = 0;
        }
      } else {
        missingRun = 0;
      }
      if (slots_[i].echo) ++r.received;
      if (slots_[i].inWindow) ++r.activeReceived;
    }
    r.scheduleLatenessMeanMs = scheduleLateness_.mean;
    r.scheduleLatenessMaxMs = scheduleLateness_.maximum;
    r.writeCallMeanMs = writeCall_.mean;
    r.writeCallMaxMs = writeCall_.maximum;
    r.roundTripMeanMs = roundTrip_.mean;
    r.roundTripMaxMs = roundTrip_.maximum;
    r.interArrivalMeanMs = interArrival_.mean;
    r.interArrivalMaxMs = interArrival_.maximum;
    r.interArrivalJitterMs = interArrival_.standardDeviation();
    r.scheduleLatenessSamples = scheduleLateness_.count;
    r.writeCallSamples = writeCall_.count;
    r.roundTripSamples = roundTrip_.count;
    r.interArrivalSamples = interArrival_.count;
    using namespace packetdemo;
    r.txHz = submitted / std::max(elapsed, 1e-9);
    r.rxHz = r.activeReceived / duration_;
    r.consistent = complete && stats[Ready] == 2 && stats[Queued] == 0 &&
        r.submitted == r.attempted && writeCall_.count == submitted &&
        roundTrip_.count == r.received && r.received <= stats[EchoAttempts] &&
        stats[EchoFailures] <= stats[EchoAttempts] &&
        stats[EchoAttempts] <= stats[NanoReceived] && stats[NanoReceived] <= r.attempted;
    bool clean = r.missing.empty() && !duplicates_ && !invalid_ && !unexpected_;
    for (auto field : {NanoDuplicates, NanoInvalid, QueueDrops, I2CErrors,
                       TeensyReceived, TeensyDuplicates, TeensyInvalid, EchoFailures})
      clean = clean && stats[field] == 0;
    r.pass = r.consistent && clean && submitted == r.planned &&
        r.txHz >= rate_ * 0.99 && r.rxHz >= rate_ * 0.99;
    return r;
  }
 private:
  struct Slot {
    bool sent = false, submitted = false, echo = false, inWindow = false;
    TrialClock::time_point attemptedAt{};
  };
  unsigned rate_;
  double duration_;
  std::vector<Slot> slots_;
  TrialClock::time_point started_{};
  std::optional<TrialClock::time_point> previousArrival_;
  TimingMoments scheduleLateness_, writeCall_, roundTrip_, interArrival_;
  bool active_ = false;
  uint32_t received_ = 0, duplicates_ = 0, invalid_ = 0, unexpected_ = 0;
  mutable std::mutex mutex_;
};
}
