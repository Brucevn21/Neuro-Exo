#include "BleMotorClient.hpp"
#include <csignal>
#include <algorithm>
#include <iostream>
#include <map>
#include <stdexcept>
#include <string>

namespace {
volatile std::sig_atomic_t interrupted = 0;
void stop(int) { interrupted = 1; }
int integer(const std::string& value) {
  size_t used = 0;
  const int result = std::stoi(value, &used);
  if (used != value.size()) throw std::invalid_argument("expected an integer: " + value);
  return result;
}
}
int main(int argc, char** argv) {
  neuroexo::BleMotorClient client(neuroexo::makeBluezGatt());
  bool controlling = false;
  bool writeWithoutResponse = false;
  bool controlStarted = false;
  bool fault = false;
  auto cleanup = [&] {
    if (controlStarted && client.connected() && !fault) {
      try { client.stopMotor(); }
      catch (const std::exception& e) { std::cerr << "Neutral delivery unconfirmed: " << e.what() << '\n'; }
    }
    client.disconnect();
  };
  try {
    std::map<std::string, std::string> args = {
      {"--address", ""}, {"--adapter", "hci0"}, {"--mode", ""}, {"--speed", ""},
      {"--angle", ""}, {"--duration", "0"}, {"--period-ms", "100"}
    };
    for (int i = 1; i < argc; ++i) {
      const std::string key = argv[i];
      if (key == "--help") {
        std::cout << "neuroexo_motor --address AA:BB:CC:DD:EE:FF [--adapter hci0] [--duration seconds]\n"
                     "Default: receive and print real Teensy telemetry without sending commands.\n"
                     "Control: --mode resistive|assistive|neutral --speed slow|medium|high\n"
                     "         --angle signed_integer_degrees --duration seconds [--period-ms 100]\n"
                     "         [--write-without-response] uses unacknowledged periodic BLE writes;\n"
                     "         the final Neutral stop still requests an ATT response.\n";
        return 0;
      }
      if (key == "--write-without-response") {
        writeWithoutResponse = true;
        continue;
      }
      if (!args.count(key) || ++i >= argc) throw std::invalid_argument("unknown/incomplete option: " + key);
      args[key] = argv[i];
    }
    if (args["--address"].empty()) throw std::invalid_argument("--address is required");
    const int duration = integer(args["--duration"]), period = integer(args["--period-ms"]);
    controlling = !args["--mode"].empty() || !args["--speed"].empty() || !args["--angle"].empty();
    if (duration < 0 || duration > 3600 || period < 20 || period > 200 || (controlling && !duration))
      throw std::invalid_argument("duration must be 0..3600 (positive for control), period 20..200 ms");
    if (writeWithoutResponse && !controlling)
      throw std::invalid_argument("--write-without-response requires control mode");
    neuroexo::Control command{neuroexo::Mode::Neutral, neuroexo::Speed::Slow, 0};
    if (controlling) {
      const std::map<std::string, neuroexo::Mode> modes = {
        {"resistive", neuroexo::Mode::Resistive}, {"assistive", neuroexo::Mode::Assistive},
        {"neutral", neuroexo::Mode::Neutral}};
      const std::map<std::string, neuroexo::Speed> speeds = {
        {"slow", neuroexo::Speed::Slow}, {"medium", neuroexo::Speed::Medium}, {"high", neuroexo::Speed::High}};
      const int angle = integer(args["--angle"]);
      if (!modes.count(args["--mode"]) || !speeds.count(args["--speed"]) || angle < -32768 || angle > 32767)
        throw std::invalid_argument("invalid mode, speed, or signed 16-bit angle");
      command = {modes.at(args["--mode"]), speeds.at(args["--speed"]), int16_t(angle)};
    }
    std::signal(SIGINT, stop);
    std::signal(SIGTERM, stop);
    client.connect(args["--address"], args["--adapter"]);
    // Require valid feedback before the first command; monitoring sends nothing.
    const auto first = client.receiveTelemetry(500);
    if (!first) throw std::runtime_error("Teensy telemetry missing");
    std::cout << "RX " << neuroexo::describe(first->value) << std::endl;
    fault = first->value.has(neuroexo::SafetyFault);
    if (controlling && (fault || first->value.has(neuroexo::InvalidCommand)))
      throw std::runtime_error("Teensy fault; control transmission stopped");
    using Clock = std::chrono::steady_clock;
    const auto started = Clock::now();
    auto due = started;
    const auto periodDuration = std::chrono::milliseconds(period);
    while (!interrupted && (!duration || Clock::now() - started < std::chrono::seconds(duration))) {
      const auto latest = client.latestTelemetry(500);
      fault = latest.value.has(neuroexo::SafetyFault);
      if (controlling && (fault || latest.value.has(neuroexo::InvalidCommand)))
        throw std::runtime_error("Teensy fault; control transmission stopped");
      if (controlling && Clock::now() >= due) {
        controlStarted = true;
        client.sendControl(command, !writeWithoutResponse);
        due += periodDuration;
        const auto afterWrite = Clock::now();
        if (due <= afterWrite) due = afterWrite + periodDuration;
      }
      // Keep the receive wait inside the next absolute command deadline.
      int receiveWaitMs = 10;
      if (controlling) {
        const auto remaining = due - Clock::now();
        if (remaining <= Clock::duration::zero()) {
          receiveWaitMs = 0;
        } else {
          const auto untilDue = std::chrono::ceil<std::chrono::milliseconds>(remaining).count();
          receiveWaitMs = std::min(receiveWaitMs, int(untilDue));
        }
      }
      const auto update = client.receiveTelemetry(receiveWaitMs);
      if (update) {
        std::cout << "RX " << neuroexo::describe(update->value) << std::endl;
        fault = update->value.has(neuroexo::SafetyFault);
        if (controlling && (fault || update->value.has(neuroexo::InvalidCommand)))
          throw std::runtime_error("Teensy fault; control transmission stopped");
      }
    }
    cleanup();
    return interrupted ? 130 : 0;
  } catch (const std::exception& e) {
    cleanup();
    std::cerr << "Motor interface failed: " << e.what() << '\n';
    return 1;
  }
}
