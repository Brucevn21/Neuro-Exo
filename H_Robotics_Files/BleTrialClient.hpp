#pragma once
#include "NanoNeuroExoProtocol/trial_protocol.h"
#include <array>
#include <chrono>
#include <cstdint>
#include <functional>
#include <memory>
#include <string>

namespace neuroexo {
using Packet = std::array<uint8_t, 20>;

// A testable transport boundary. BluezGatt implements it with BlueZ's D-Bus API.
class GattTransport {
 public:
  virtual ~GattTransport() = default;
  virtual void connect(const std::string& address, const std::string& adapter) = 0;
  virtual Packet info() = 0;
  virtual Packet exchange(const Packet& request, int timeoutMs) = 0;
  virtual void disconnect() noexcept = 0;
};

struct ArmPosition {
  double degrees;
  uint32_t deviceMilliseconds;
  State state;
  bool simulated;
};
struct TrialSettings {
  uint16_t id;
  double targetDegrees;
  double startDegrees;
  double assistanceDegreesPerSecond;
};
struct TrialTiming {
  unsigned positionReplies = 0;
  unsigned skippedPeriods = 0;
  double maxRoundTripMs = 0;
  double maxRequestGapMs = 0;
};

class BleTrialClient {
 public:
  explicit BleTrialClient(std::unique_ptr<GattTransport> transport);
  ~BleTrialClient();
  BleTrialClient(const BleTrialClient&) = delete;
  BleTrialClient& operator=(const BleTrialClient&) = delete;
  void connect(const std::string& address, const std::string& adapter = "hci0");
  void disconnect() noexcept;
  bool connected() const { return connected_; }
  bool simulated() const { return simulated_; }
  void setTimeout(int milliseconds);
  void setTrace(std::function<void(bool, const Frame&)> callback) { trace_ = callback; }
  void calibrate();
  void setMaximumPosition(double degrees);
  void configure(const TrialSettings& settings);
  void start(uint16_t trial);
  ArmPosition position(uint16_t trial);
  void end(uint16_t trial);
  TrialTiming runTrial(const TrialSettings& settings, std::chrono::milliseconds duration,
                      std::chrono::milliseconds period = std::chrono::milliseconds(40),
                      std::function<void(const ArmPosition&)> onPosition = {},
                      std::function<bool()> shouldStop = {});
 private:
  Frame command(uint8_t kind, uint16_t trial = 0, int32_t a = 0, int32_t b = 0, int32_t c = 0);
  static int32_t scaled(double value);
  std::unique_ptr<GattTransport> transport_;
  bool connected_ = false, simulated_ = false;
  uint16_t sequence_ = 0, activeTrial_ = 0;
  int timeoutMs_ = 1000;
  std::function<void(bool, const Frame&)> trace_;
};
// BlueZ runs on the system bus on the BBB; this factory has no TCP backend.
std::unique_ptr<GattTransport> makeBluezGatt();
}  // namespace neuroexo
