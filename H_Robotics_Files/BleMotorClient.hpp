#pragma once
#include "../NeuroExoFirmware/src/commProtocol.h"
#include <array>
#include <chrono>
#include <memory>
#include <optional>
#include <string>

namespace neuroexo {
using ControlPacket = std::array<uint8_t, CONTROL_SIZE>;
struct TelemetryUpdate {
  Telemetry value;
  std::chrono::steady_clock::time_point receivedAt;
};
// TX and RX are independent. Receiving never writes a command.
// Connection lifecycle has one owner; a separate consumer may receive telemetry.
class GattTransport {
 public:
  virtual ~GattTransport() = default;
  virtual void connect(const std::string& address, const std::string& adapter) = 0;
  virtual bool connected() const = 0;
  virtual void writeControl(const ControlPacket& request, int timeoutMs) = 0;
  virtual std::optional<TelemetryUpdate> receiveTelemetry(int timeoutMs) = 0;
  virtual std::optional<TelemetryUpdate> latestTelemetry() const = 0;
  virtual void disconnect() noexcept = 0;
};
class BleMotorClient {
 public:
  explicit BleMotorClient(std::unique_ptr<GattTransport> transport);
  ~BleMotorClient();
  BleMotorClient(const BleMotorClient&) = delete;
  BleMotorClient& operator=(const BleMotorClient&) = delete;
  void connect(const std::string& address, const std::string& adapter = "hci0");
  void disconnect() noexcept;
  bool connected() const;
  void sendControl(const Control& command);
  void stopMotor();
  std::optional<TelemetryUpdate> receiveTelemetry(int timeoutMs = 100);
  // Throws on missing/stale telemetry or lost connection; never returns a fake zero.
  TelemetryUpdate latestTelemetry(int maxAgeMs = 250) const;
 private:
  std::unique_ptr<GattTransport> transport_;
};
std::string describe(const Telemetry& telemetry);
std::unique_ptr<GattTransport> makeBluezGatt();
}  // namespace neuroexo
