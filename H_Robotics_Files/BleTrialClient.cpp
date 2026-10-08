#include "BleTrialClient.hpp"
#include <algorithm>
#include <cmath>
#include <stdexcept>
#include <thread>
#include <utility>

namespace neuroexo {
BleTrialClient::BleTrialClient(std::unique_ptr<GattTransport> transport)
    : transport_(std::move(transport)) {
  if (!transport_) throw std::invalid_argument("a GATT transport is required");
}
BleTrialClient::~BleTrialClient() { disconnect(); }
void BleTrialClient::setTimeout(int ms) {
  if (ms < 1 || ms > 10000) throw std::invalid_argument("timeout must be 1..10000 ms");
  timeoutMs_ = ms;
}
int32_t BleTrialClient::scaled(double value) {
  if (!std::isfinite(value) || value < 0 || value > 360)
    throw std::invalid_argument("position/velocity must be finite and within 0..360");
  const double scaledValue = value * SCALE;
  if (std::abs(scaledValue - std::round(scaledValue)) > 1e-7)
    throw std::invalid_argument("position/velocity resolution is 0.001");
  return int32_t(std::llround(scaledValue));
}
void BleTrialClient::connect(const std::string& address, const std::string& adapter) {
  disconnect();
  try {
    transport_->connect(address, adapter);
    const Packet bytes = transport_->info();
    Frame f;
    if (!decode(bytes.data(), bytes.size(), f) || f.kind != INFO || f.sequence ||
        f.trial || f.a != SCALE || f.b != int32_t(WATCHDOG_MS) || f.c != BOOT)
      throw std::runtime_error("Nano protocol/version/units/state mismatch");
    simulated_ = (f.flags & SIMULATED) != 0;
    sequence_ = activeTrial_ = 0;
    connected_ = true;
  } catch (...) {
    transport_->disconnect();
    throw;
  }
}
void BleTrialClient::disconnect() noexcept {
  if (connected_ && activeTrial_) {
    try { end(activeTrial_); } catch (...) {}
  }
  connected_ = false;
  activeTrial_ = 0;
  transport_->disconnect();
}
Frame BleTrialClient::command(uint8_t kind, uint16_t trial, int32_t a, int32_t b, int32_t c) {
  if (!connected_) throw std::runtime_error("BLE is not connected");
  sequence_ = sequence_ == 65535 ? 1 : uint16_t(sequence_ + 1);
  const Frame request = {kind, 0, sequence_, trial, a, b, c};
  Packet raw{};
  encode(request, raw.data());
  if (trace_) trace_(true, request);
  Packet bytes;
  try {
    bytes = transport_->exchange(raw, timeoutMs_);
  } catch (...) {
    // A timed-out write has an unknown outcome. Do not automatically replay Start.
    connected_ = false;
    activeTrial_ = 0;
    transport_->disconnect();
    throw;
  }
  Frame reply;
  if (!decode(bytes.data(), bytes.size(), reply) || reply.sequence != sequence_ ||
      reply.trial != trial || bool(reply.flags & SIMULATED) != simulated_) {
    connected_ = false;
    activeTrial_ = 0;
    transport_->disconnect();
    throw std::runtime_error("invalid BLE reply identity or format");
  }
  if (trace_) trace_(false, reply);
  if (reply.kind == ACK && reply.b != OK)
    throw std::runtime_error("Nano rejected command " + std::to_string(kind) +
                             " with status " + std::to_string(reply.b));
  const uint8_t expected = kind == GET_POSITION ? POSITION : ACK;
  if (reply.kind != expected || (expected == ACK && reply.a != kind) ||
      reply.c < BOOT || reply.c > FAULT)
    throw std::runtime_error("unexpected BLE response");
  if (reply.c == FAULT) throw std::runtime_error("Nano reported FAULT");
  return reply;
}
void BleTrialClient::calibrate() { command(CALIBRATE); activeTrial_ = 0; }
void BleTrialClient::setMaximumPosition(double degrees) {
  const int32_t maximum = scaled(degrees);
  if (!maximum) throw std::invalid_argument("maximum position must be positive");
  command(SET_MAX, 0, maximum);
}
void BleTrialClient::configure(const TrialSettings& settings) {
  if (!settings.id) throw std::invalid_argument("trial ID must be nonzero");
  const int32_t velocity = scaled(settings.assistanceDegreesPerSecond);
  if (!velocity) throw std::invalid_argument("assistance velocity must be positive");
  command(CONFIGURE, settings.id, scaled(settings.targetDegrees),
          scaled(settings.startDegrees), velocity);
  activeTrial_ = settings.id;
}
void BleTrialClient::start(uint16_t trial) {
  const Frame reply = command(START, trial);
  if (reply.c != RUNNING) throw std::runtime_error("Start was not accepted as RUNNING");
}
ArmPosition BleTrialClient::position(uint16_t trial) {
  const Frame reply = command(GET_POSITION, trial);
  return {double(reply.a) / SCALE, uint32_t(reply.b), State(reply.c), simulated_};
}
void BleTrialClient::end(uint16_t trial) {
  const Frame reply = command(END, trial);
  if (reply.c != ENDED) throw std::runtime_error("End was not confirmed");
  activeTrial_ = 0;
}
TrialTiming BleTrialClient::runTrial(const TrialSettings& settings,
                                   std::chrono::milliseconds duration,
                                   std::chrono::milliseconds period,
                                   std::function<void(const ArmPosition&)> onPosition,
                                   std::function<bool()> shouldStop) {
  if (duration.count() <= 0 || duration > std::chrono::hours(1) ||
      period.count() < 20 || period.count() > 1000)
    throw std::invalid_argument("duration must be (0,1h], period 20..1000 ms");
  TrialTiming timing;
  configure(settings);
  try {
    const ArmPosition initial = position(settings.id);
    if (initial.state != READY || std::abs(initial.degrees - settings.startDegrees) > 0.001)
      throw std::runtime_error("arm is not READY at the configured starting position");
    if (onPosition) onPosition(initial);
    if (shouldStop && shouldStop()) { end(settings.id); return timing; }
    start(settings.id);
    using Clock = std::chrono::steady_clock;
    const auto started = Clock::now();
    const auto finish = started + duration;
    auto due = started + period;
    auto previous = Clock::time_point{};
    while (due <= finish) {
      std::this_thread::sleep_until(due);
      if (shouldStop && shouldStop()) break;
      const auto sent = Clock::now();
      if (previous != Clock::time_point{})
        timing.maxRequestGapMs = std::max(timing.maxRequestGapMs,
          std::chrono::duration<double, std::milli>(sent - previous).count());
      previous = sent;
      const ArmPosition received = position(settings.id);
      ++timing.positionReplies;
      timing.maxRoundTripMs = std::max(timing.maxRoundTripMs,
        std::chrono::duration<double, std::milli>(Clock::now() - sent).count());
      if (received.state != RUNNING) throw std::runtime_error("arm left RUNNING");
      if (onPosition) onPosition(received);
      due += period;
      const auto now = Clock::now();
      if (due < now) {
        const auto skipped = (now - due) / period + 1;
        timing.skippedPeriods += unsigned(skipped);
        due += period * skipped;
      }
    }
    // Check cancellation before waiting out a short remainder.
    if (!(shouldStop && shouldStop())) std::this_thread::sleep_until(finish);
    end(settings.id);
  } catch (...) {
    try { if (connected_) end(settings.id); } catch (...) {}
    throw;
  }
  return timing;
}
}  // namespace neuroexo
