#include "../ble/BleJointClient.hpp"
#include <cmath>
#include <csignal>
#include <iomanip>
#include <iostream>
#include <map>
#include <stdexcept>
#include <string>
#include <thread>

namespace {
namespace proto = NeuroExoProtocol;
volatile std::sig_atomic_t interrupted = 0;
void stop(int) { interrupted = 1; }
double number(const std::string& s) {
  size_t used = 0;
  const double value = std::stod(s, &used);
  if (used != s.size() || !std::isfinite(value)) throw std::invalid_argument("invalid numeric argument");
  return value;
}
proto::Mode mode(const std::string& name) {
  if (name == "resistive") return proto::Mode::Resistive;
  if (name == "assistive") return proto::Mode::Assistive;
  if (name == "neutral") return proto::Mode::Neutral;
  throw std::invalid_argument("--mode must be resistive, assistive, or neutral");
}
proto::Speed speed(const std::string& name) {
  if (name == "slow") return proto::Speed::Slow;
  if (name == "medium") return proto::Speed::Medium;
  if (name == "high") return proto::Speed::High;
  throw std::invalid_argument("--speed must be slow, medium, or high");
}
void hex(const uint8_t* data, size_t length) {
  std::cout << std::hex << std::setfill('0');
  for (size_t i = 0; i < length; ++i) std::cout << (i ? " " : "") << std::setw(2) << int(data[i]);
  std::cout << std::dec << std::setfill(' ');
}
void print(const neuroexo::JointTelemetry& t) {
  std::cout << "angle=" << t.angleDeg << " deg current=" << t.milliAmps << " mA status="
            << (t.motionActive() ? " MotionActive" : "") << (t.commandTimeout() ? " CommandTimeout" : "")
            << (t.invalidCommand() ? " InvalidCommand" : "") << (t.safetyFault() ? " SafetyFault" : "")
            << (t.targetReached() ? " TargetReached" : "")
            << (t.status ? "" : " none") << '\n';
}
}  // namespace

int main(int argc, char** argv) {
  try {
    std::map<std::string, std::string> args = {
      {"--address", ""}, {"--adapter", "hci0"}, {"--mode", "assistive"}, {"--speed", "medium"},
      {"--target", "45"}, {"--time-limit", "150"}, {"--duration", "5"}, {"--period-ms", "100"}, {"--trials", "1"},
      {"--rest", "1"}
    };
    bool monitor = false, trace = false, withoutResponse = false;
    for (int i = 1; i < argc; ++i) {
      const std::string key = argv[i];
      if (key == "--help") {
        std::cout << "neuroexo_ble_trial --address AA:BB:CC:DD:EE:FF [--adapter hci0]\n"
                     "  [--mode resistive|assistive|neutral] [--speed slow|medium|high]\n"
                     "  [--target 45] [--time-limit 150] [--period-ms 100] [--trials 1] [--rest 1]\n"
                     "  [--trace] [--write-without-response]\n"
                     "neuroexo_ble_trial --address AA:BB:CC:DD:EE:FF --monitor [--duration 5]\n"
                     "Talks to Nano33BLEFirmware (service 180C). Without --monitor this\n"
                     "COMMANDS REAL MOTOR MOTION: it repeats the command until the Teensy\n"
                     "reports TargetReached (status bit 4), then stops and lets it idle.\n";
        return 0;
      }
      if (key == "--monitor") { monitor = true; continue; }
      if (key == "--trace") { trace = true; continue; }
      if (key == "--write-without-response") { withoutResponse = true; continue; }
      if (!args.count(key) || ++i >= argc) throw std::invalid_argument("unknown or incomplete option: " + key);
      args[key] = argv[i];
    }
    if (args["--address"].empty()) throw std::invalid_argument("--address is required");
    const double target = number(args["--target"]), duration = number(args["--duration"]);
    const double timeLimit = number(args["--time-limit"]);
    const double period = number(args["--period-ms"]), trials = number(args["--trials"]);
    const double rest = number(args["--rest"]);
    if (target < -32768 || target > 32767 || std::floor(target) != target ||
        duration < .001 || duration > 3600 || timeLimit < .001 || timeLimit > 3600 ||
        period < 20 || period > 150 ||
        std::floor(period) != period || trials < 1 || trials > 1000 ||
        std::floor(trials) != trials || rest < 0.3 || rest > 3600)
      throw std::invalid_argument("invalid target (int16 degrees), duration, time limit, period (20..150 ms), "
                                  "trial count, or rest (>= 0.3 s)");
    const neuroexo::JointCommand command{mode(args["--mode"]), speed(args["--speed"]),
                                         int16_t(target)};

    neuroexo::BleJointClient client(neuroexo::makeBluezGatt());
    client.setWriteWithResponse(!withoutResponse);
    if (trace) client.setTrace([](bool tx, const uint8_t* frame, size_t length) {
      std::cout << (tx ? "TX " : "RX ");
      hex(frame, length);
      std::cout << '\n';
    });
    std::signal(SIGINT, stop);
    std::signal(SIGTERM, stop);
    client.connect(args["--address"], args["--adapter"]);
    std::cout << "Connected to " << neuroexo::NANO_NAME << ".\n";

    if (monitor) {
      const auto finish = std::chrono::steady_clock::now() +
                          std::chrono::milliseconds(std::llround(duration * 1000));
      client.waitForTelemetry(std::chrono::milliseconds(1000));
      while (std::chrono::steady_clock::now() < finish && !interrupted) {
        for (const auto& telemetry : client.takeTelemetry()) print(telemetry);
        if (!client.connected()) throw std::runtime_error("BLE link lost");
        std::this_thread::sleep_for(std::chrono::milliseconds(20));
      }
      std::cout << "Rejected notifications: " << client.rejectedNotifications() << '\n';
      client.disconnect();
      return interrupted ? 130 : 0;
    }

    unsigned skipped = 0;
    for (unsigned trial = 1; trial <= unsigned(trials) && !interrupted; ++trial) {
      if (trial > 1) {
        // Silence for longer than 250 ms also clears a latched SafetyFault.
        std::this_thread::sleep_for(std::chrono::milliseconds(std::llround(rest * 1000)));
        client.takeTelemetry();
      }
      const auto result = client.runTrial(command, std::chrono::milliseconds(std::llround(timeLimit * 1000)),
                                          std::chrono::milliseconds(int(period)),
                                          print, [] { return interrupted != 0; });
      skipped += result.skippedPeriods;
      std::cout << "Trial " << trial << (result.targetReached ? ": target reached in " : ": stopped after ")
                << result.elapsedMs / 1000 << " s, commands=" << result.commandsSent
                << " telemetry=" << result.telemetryFrames
                << " skipped periods=" << result.skippedPeriods
                << " max write=" << result.maxWriteMs << " ms"
                << " max telemetry gap=" << result.maxTelemetryGapMs << " ms"
                << (result.stopConfirmed ? " stop confirmed" : "") << '\n';
    }
    client.disconnect();
    return interrupted ? 130 : skipped ? 2 : 0;
  } catch (const std::exception& error) {
    std::cerr << "BLE trial failed: " << error.what() << '\n';
    return 1;
  }
}
