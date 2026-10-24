#include "BleMotorClient.hpp"
#include <iomanip>
#include <sstream>
#include <stdexcept>
#include <utility>

namespace neuroexo {
BleMotorClient::BleMotorClient(std::unique_ptr<GattTransport> transport)
    : transport_(std::move(transport)) {
  if (!transport_) throw std::invalid_argument("a GATT transport is required");
}
BleMotorClient::~BleMotorClient() { disconnect(); }
void BleMotorClient::connect(const std::string& address, const std::string& adapter) {
  disconnect();
  try { transport_->connect(address, adapter); }
  catch (...) { transport_->disconnect(); throw; }
}
void BleMotorClient::disconnect() noexcept { transport_->disconnect(); }
bool BleMotorClient::connected() const { return transport_->connected(); }
void BleMotorClient::sendControl(const Control& command) {
  ControlPacket raw{};
  if (!encodeControl(command, raw.data())) throw std::invalid_argument("invalid mode or speed");
  if (!connected()) throw std::runtime_error("Nano is disconnected");
  // ATT success only confirms delivery to Nano, not application by Teensy.
  transport_->writeControl(raw, 100);
}
void BleMotorClient::stopMotor() { sendControl({Mode::Neutral, Speed::Slow, 0}); }
std::optional<TelemetryUpdate> BleMotorClient::receiveTelemetry(int timeoutMs) {
  if (timeoutMs < 0 || timeoutMs > 10000) throw std::invalid_argument("timeout must be 0..10000 ms");
  return transport_->receiveTelemetry(timeoutMs);
}
TelemetryUpdate BleMotorClient::latestTelemetry(int maxAgeMs) const {
  if (maxAgeMs <= 0) throw std::invalid_argument("telemetry maximum age must be positive");
  if (!connected()) throw std::runtime_error("Nano is disconnected");
  const auto update = transport_->latestTelemetry();
  if (!update || std::chrono::steady_clock::now() - update->receivedAt >
                 std::chrono::milliseconds(maxAgeMs))
    throw std::runtime_error("Teensy telemetry is missing or stale");
  return *update;
}
std::string describe(const Telemetry& t) {
  std::ostringstream out;
  out << "angle=" << t.currentAngleDeg << " deg current=" << t.currentMilliAmps
      << " mA status=0x" << std::hex << std::setw(2) << std::setfill('0') << unsigned(t.status)
      << std::dec << " MotionActive=" << t.has(MotionActive)
      << " CommandTimeout=" << t.has(CommandTimeout)
      << " InvalidCommand=" << t.has(InvalidCommand)
      << " SafetyFault=" << t.has(SafetyFault);
  return out.str();
}
}  // namespace neuroexo
