#include "BleJointClient.hpp"
#include <algorithm>
#include <stdexcept>
#include <thread>
#include <utility>

namespace neuroexo {
namespace {
using Clock = std::chrono::steady_clock;
constexpr size_t MAX_QUEUED_TELEMETRY = 1024;  // ~20 s at 50 Hz; oldest frames are dropped
double millisecondsBetween(Clock::time_point from, Clock::time_point to) {
  return std::chrono::duration<double, std::milli>(to - from).count();
}
}  // namespace

bool encodeCommand(const JointCommand& command, uint8_t (&frame)[CONTROL_FRAME_SIZE]) {
  if (uint8_t(command.mode) > uint8_t(proto::Mode::Neutral) ||
      uint8_t(command.speed) > uint8_t(proto::Speed::High))
    return false;
  proto::ControlPacket packet;
  packet.mode = command.mode;
  packet.speed = command.speed;
  packet.targetAngleDeg = command.targetAngleDeg;
  return proto::encodeControlFrame(packet, frame) == CONTROL_FRAME_SIZE;
}

bool decodeTelemetry(const uint8_t* data, size_t length, JointTelemetry& out) {
  if (!data || length != TELEMETRY_FRAME_SIZE) return false;
  proto::FrameParser parser;
  for (size_t i = 0; i < length; ++i) parser.push(data[i]);
  proto::Frame frame;
  proto::TelemetryPacket packet;
  if (!parser.takeFrame(frame) || !proto::decodeTelemetryFrame(frame, packet)) return false;
  out.angleDeg = packet.currentAngleDeg;
  out.milliAmps = packet.currentMilliAmps;
  out.status = packet.status;
  return true;
}

BleJointClient::BleJointClient(std::unique_ptr<GattTransport> transport)
    : transport_(std::move(transport)) {
  if (!transport_) throw std::invalid_argument("a GATT transport is required");
}
BleJointClient::~BleJointClient() { disconnect(); }

void BleJointClient::setTimeout(int ms) {
  if (ms < 1 || ms > 10000) throw std::invalid_argument("timeout must be 1..10000 ms");
  timeoutMs_ = ms;
}

void BleJointClient::connect(const std::string& address, const std::string& adapter) {
  disconnect();
  {
    std::lock_guard<std::mutex> guard(mutex_);
    queue_.clear();
    haveLatest_ = false;
  }
  rejected_ = 0;
  try {
    transport_->connect(address, adapter,
                        [this](const uint8_t* data, size_t length) { notification(data, length); });
    connected_ = true;
  } catch (...) {
    transport_->disconnect();
    throw;
  }
}

void BleJointClient::disconnect() noexcept {
  connected_ = false;
  transport_->disconnect();  // joins the event thread; no notifications after this
}

bool BleJointClient::connected() const { return connected_ && transport_->failure().empty(); }

void BleJointClient::requireConnected() const {
  if (!connected_) throw std::runtime_error("BLE is not connected");
  const std::string failure = transport_->failure();
  if (!failure.empty()) throw std::runtime_error("BLE link failed: " + failure);
}

void BleJointClient::notification(const uint8_t* data, size_t length) {
  JointTelemetry telemetry;
  if (!decodeTelemetry(data, length, telemetry)) {
    ++rejected_;  // the Nano only publishes validated frames; count anything else
    return;
  }
  telemetry.received = Clock::now();
  std::lock_guard<std::mutex> guard(mutex_);
  if (queue_.size() >= MAX_QUEUED_TELEMETRY) queue_.pop_front();
  queue_.push_back(telemetry);
  latest_ = telemetry;
  haveLatest_ = true;
}

std::vector<JointTelemetry> BleJointClient::takeTelemetry() {
  std::lock_guard<std::mutex> guard(mutex_);
  std::vector<JointTelemetry> frames(queue_.begin(), queue_.end());
  queue_.clear();
  return frames;
}

bool BleJointClient::latestTelemetry(JointTelemetry& out) const {
  std::lock_guard<std::mutex> guard(mutex_);
  if (haveLatest_) out = latest_;
  return haveLatest_;
}

JointTelemetry BleJointClient::waitForTelemetry(std::chrono::milliseconds timeout) {
  const auto started = Clock::now();
  const auto deadline = started + timeout;
  while (true) {
    requireConnected();
    JointTelemetry latest;
    if (latestTelemetry(latest) && latest.received >= started) return latest;
    if (Clock::now() >= deadline)
      throw std::runtime_error("no telemetry from the Nano; check the Teensy I2C link and power");
    std::this_thread::sleep_for(std::chrono::milliseconds(5));
  }
}

void BleJointClient::send(const JointCommand& command) {
  requireConnected();
  uint8_t frame[CONTROL_FRAME_SIZE];
  if (!encodeCommand(command, frame)) throw std::invalid_argument("mode or speed out of range");
  if (trace_) trace_(true, frame, sizeof(frame));
  try {
    transport_->write(frame, sizeof(frame), withResponse_, timeoutMs_);
  } catch (...) {
    // Stop using a link whose write outcome is unknown; silence stops the Teensy.
    connected_ = false;
    transport_->disconnect();
    throw;
  }
}

std::vector<JointTelemetry> BleJointClient::deliver(
    std::function<void(const JointTelemetry&)>& onTelemetry, TrialResult& result,
    Clock::time_point& lastTelemetry, bool stopping) {
  std::vector<JointTelemetry> frames = takeTelemetry();
  for (const JointTelemetry& telemetry : frames) {
    if (trace_) {
      uint8_t frame[TELEMETRY_FRAME_SIZE];
      proto::TelemetryPacket packet;
      packet.currentAngleDeg = telemetry.angleDeg;
      packet.currentMilliAmps = telemetry.milliAmps;
      packet.status = telemetry.status;
      proto::encodeTelemetryFrame(packet, frame);
      trace_(false, frame, sizeof(frame));
    }
    if (result.telemetryFrames)
      result.maxTelemetryGapMs = std::max(result.maxTelemetryGapMs,
                                          millisecondsBetween(lastTelemetry, telemetry.received));
    ++result.telemetryFrames;
    lastTelemetry = telemetry.received;
    if (onTelemetry) onTelemetry(telemetry);
    if (!stopping && telemetry.safetyFault())
      throw std::runtime_error("Teensy safety fault (e-stop latched); commands stopped");
  }
  return frames;
}

TrialResult BleJointClient::runTrial(const JointCommand& command, std::chrono::milliseconds timeLimit,
                                     std::chrono::milliseconds period,
                                     std::function<void(const JointTelemetry&)> onTelemetry,
                                     std::function<bool()> shouldStop) {
  // Keep every keepalive well inside the Teensy's 250 ms command timeout.
  if (timeLimit.count() <= 0 || timeLimit > std::chrono::hours(1) ||
      period.count() < 20 || period.count() > 150)
    throw std::invalid_argument("time limit must be (0,1h], period 20..150 ms");
  uint8_t check[CONTROL_FRAME_SIZE];
  if (!encodeCommand(command, check)) throw std::invalid_argument("mode or speed out of range");

  const JointTelemetry initial = waitForTelemetry(std::chrono::milliseconds(TELEMETRY_STALE_MS * 2));
  if (initial.safetyFault())
    throw std::runtime_error("Teensy safety fault is latched; it clears after 250 ms without commands");
  takeTelemetry();  // start the trial's telemetry from now

  TrialResult result;
  auto lastTelemetry = initial.received;
  const auto stale = std::chrono::milliseconds(TELEMETRY_STALE_MS);
  const auto timeoutWindow = std::chrono::milliseconds(proto::COMMAND_TIMEOUT_MS + 150);
  const auto started = Clock::now();
  const auto deadline = started + timeLimit;
  auto due = started;
  Clock::time_point lastSend{};
  // Writes happen only on this thread: any exception that leaves this function
  // also ends the keepalives, so the Teensy disables motion within 250 ms.
  while (true) {
    std::this_thread::sleep_until(due);
    for (const JointTelemetry& telemetry : deliver(onTelemetry, result, lastTelemetry, false)) {
      // A TargetReached left over from an earlier move arrives with
      // CommandTimeout set; only a frame from this active command counts.
      if (result.commandsSent && telemetry.targetReached() && telemetry.motionActive() &&
          !telemetry.commandTimeout())
        result.targetReached = true;
    }
    if (result.targetReached) break;
    if (shouldStop && shouldStop()) { result.stopped = true; break; }
    if (Clock::now() >= deadline) break;
    if (result.commandsSent && Clock::now() - lastTelemetry > stale)
      throw std::runtime_error("telemetry stopped during the trial; commands stopped");
    // After a full timeout window of keepalives, CommandTimeout means the
    // Teensy is not accepting them (I2C fault, rejected frames, or late writes).
    JointTelemetry latest;
    if (lastTelemetry - started > timeoutWindow && latestTelemetry(latest) && latest.commandTimeout())
      throw std::runtime_error("Teensy reports CommandTimeout while commands are being sent");
    const auto sent = Clock::now();
    send(command);
    lastSend = Clock::now();
    ++result.commandsSent;
    result.maxWriteMs = std::max(result.maxWriteMs, millisecondsBetween(sent, lastSend));
    due += period;
    const auto now = Clock::now();
    if (due < now) {  // never send a catch-up burst
      const auto skipped = (now - due) / period + 1;
      result.skippedPeriods += unsigned(skipped);
      due += period * skipped;
    }
  }
  result.elapsedMs = millisecondsBetween(started, Clock::now());

  // Keepalives have ceased. Wait for telemetry showing the Teensy went idle.
  const auto stopDeadline = Clock::now() + std::chrono::milliseconds(STOP_CONFIRM_MS);
  while (Clock::now() < stopDeadline && !result.stopConfirmed) {
    requireConnected();
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
    for (const JointTelemetry& telemetry : deliver(onTelemetry, result, lastTelemetry, true)) {
      if (telemetry.received > lastSend + std::chrono::milliseconds(proto::COMMAND_TIMEOUT_MS) &&
          telemetry.commandTimeout() && !telemetry.motionActive())
        result.stopConfirmed = true;
    }
    if (Clock::now() - lastTelemetry > stale) break;
  }
  if (!result.stopConfirmed)
    throw std::runtime_error("stop not confirmed: no CommandTimeout telemetry after keepalives ceased");
  if (!result.targetReached && !result.stopped)
    throw std::runtime_error("target not reached within the time limit; commands stopped");
  return result;
}
}  // namespace neuroexo
