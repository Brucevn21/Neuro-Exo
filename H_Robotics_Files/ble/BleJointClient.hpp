#pragma once
#include "../firmware/commProtocol/commProtocol.h"
#include <atomic>
#include <chrono>
#include <cstddef>
#include <cstdint>
#include <deque>
#include <functional>
#include <memory>
#include <mutex>
#include <string>
#include <vector>

namespace neuroexo {
namespace proto = NeuroExoProtocol;

// GATT layout advertised by Nano33BLEFirmware.ino as "Nano33BLE_Master".
constexpr const char* NANO_NAME = "Nano33BLE_Master";
constexpr const char* SERVICE_UUID = "0000180c-0000-1000-8000-00805f9b34fb";
constexpr const char* COMMAND_UUID = "00002a56-0000-1000-8000-00805f9b34fb";
constexpr const char* TELEMETRY_UUID = "00002a57-0000-1000-8000-00805f9b34fb";
constexpr size_t CONTROL_FRAME_SIZE = proto::FRAME_OVERHEAD + proto::CONTROL_PAYLOAD_SIZE;
constexpr size_t TELEMETRY_FRAME_SIZE = proto::FRAME_OVERHEAD + proto::TELEMETRY_PAYLOAD_SIZE;
// Status bit 4: the Teensy's controller holds the measured angle at the target.
// Not yet in the vendored commProtocol.h; switch to TelemetryStatus::TargetReached
// once the firmware defines it and the copy is refreshed.
constexpr uint8_t STATUS_TARGET_REACHED = 1 << 4;

// A testable transport boundary. BluezGatt implements it with BlueZ's D-Bus API.
class GattTransport {
 public:
  // Called on the transport's event thread with one notification value.
  using Notification = std::function<void(const uint8_t*, size_t)>;
  virtual ~GattTransport() = default;
  virtual void connect(const std::string& address, const std::string& adapter,
                       Notification onTelemetry) = 0;
  virtual void write(const uint8_t* data, size_t length, bool withResponse, int timeoutMs) = 0;
  // Empty while the link is healthy; otherwise why it failed.
  virtual std::string failure() const = 0;
  virtual void disconnect() noexcept = 0;
};

struct JointCommand {
  proto::Mode mode = proto::Mode::Neutral;
  proto::Speed speed = proto::Speed::Medium;
  int16_t targetAngleDeg = 0;
};

struct JointTelemetry {
  int16_t angleDeg = 0;
  int16_t milliAmps = 0;
  uint8_t status = 0;
  std::chrono::steady_clock::time_point received{};
  bool has(proto::TelemetryStatus bit) const { return status & uint8_t(bit); }
  bool motionActive() const { return has(proto::TelemetryStatus::MotionActive); }
  bool commandTimeout() const { return has(proto::TelemetryStatus::CommandTimeout); }
  bool invalidCommand() const { return has(proto::TelemetryStatus::InvalidCommand); }
  bool safetyFault() const { return has(proto::TelemetryStatus::SafetyFault); }
  bool targetReached() const { return status & STATUS_TARGET_REACHED; }
};

struct TrialResult {
  unsigned commandsSent = 0;
  unsigned telemetryFrames = 0;
  unsigned skippedPeriods = 0;
  double maxWriteMs = 0;
  double maxTelemetryGapMs = 0;
  double elapsedMs = 0;        // first command to TargetReached (or to the stop)
  bool targetReached = false;
  bool stopped = false;        // shouldStop() ended the trial early
  bool stopConfirmed = false;  // telemetry showed CommandTimeout after keepalives ceased
};

class BleJointClient {
 public:
  static constexpr int TELEMETRY_STALE_MS = 500;   // 50 Hz nominal
  static constexpr int STOP_CONFIRM_MS = 1000;

  explicit BleJointClient(std::unique_ptr<GattTransport> transport);
  ~BleJointClient();
  BleJointClient(const BleJointClient&) = delete;
  BleJointClient& operator=(const BleJointClient&) = delete;
  void connect(const std::string& address, const std::string& adapter = "hci0");
  void disconnect() noexcept;
  bool connected() const;
  void setTimeout(int milliseconds);
  void setWriteWithResponse(bool enabled) { withResponse_ = enabled; }
  // Receives every valid frame sent (tx=true) or received (tx=false).
  void setTrace(std::function<void(bool tx, const uint8_t*, size_t)> callback) { trace_ = callback; }

  // Writes one control frame. The Teensy disables motion unless a valid frame
  // arrives at least every 250 ms; runTrial() repeats the command for you.
  void send(const JointCommand& command);
  // Valid telemetry received since the last call, oldest first.
  std::vector<JointTelemetry> takeTelemetry();
  bool latestTelemetry(JointTelemetry& out) const;
  unsigned rejectedNotifications() const { return rejected_.load(); }
  // Blocks until the first valid telemetry frame or the timeout.
  JointTelemetry waitForTelemetry(std::chrono::milliseconds timeout);

  // Sends `command` (the start signal) and repeats it every `period` until the
  // Teensy reports TargetReached, then stops writing so its 250 ms watchdog
  // returns it to idle (motor disabled). Telemetry goes to onTelemetry on this
  // thread. Throws if `timeLimit` passes first, or on SafetyFault, stale
  // telemetry, CommandTimeout while sending, or link loss; writes stop before
  // any exception leaves.
  TrialResult runTrial(const JointCommand& command, std::chrono::milliseconds timeLimit,
                       std::chrono::milliseconds period = std::chrono::milliseconds(100),
                       std::function<void(const JointTelemetry&)> onTelemetry = {},
                       std::function<bool()> shouldStop = {});

 private:
  void notification(const uint8_t* data, size_t length);
  std::vector<JointTelemetry> deliver(std::function<void(const JointTelemetry&)>& onTelemetry,
                                      TrialResult& result,
                                      std::chrono::steady_clock::time_point& lastTelemetry,
                                      bool stopping);
  void requireConnected() const;
  std::unique_ptr<GattTransport> transport_;
  bool connected_ = false, withResponse_ = true;
  int timeoutMs_ = 1000;
  std::function<void(bool, const uint8_t*, size_t)> trace_;
  mutable std::mutex mutex_;
  std::deque<JointTelemetry> queue_;
  JointTelemetry latest_;
  bool haveLatest_ = false;
  std::atomic<unsigned> rejected_{0};
};

bool encodeCommand(const JointCommand& command, uint8_t (&frame)[CONTROL_FRAME_SIZE]);
// Accepts exactly one valid telemetry frame and nothing else.
bool decodeTelemetry(const uint8_t* data, size_t length, JointTelemetry& out);

// BlueZ runs on the system bus on the BBB.
std::unique_ptr<GattTransport> makeBluezGatt();
}  // namespace neuroexo
