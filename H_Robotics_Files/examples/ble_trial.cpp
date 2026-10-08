#include "../BleTrialClient.hpp"
#include <cmath>
#include <csignal>
#include <iomanip>
#include <iostream>
#include <map>
#include <stdexcept>
#include <string>

namespace {
volatile std::sig_atomic_t interrupted = 0;
void stop(int) { interrupted = 1; }
const char* kindName(int kind) {
  switch (kind) {
    case neuroexo::CALIBRATE: return "CALIBRATE";
    case neuroexo::SET_MAX: return "SET_MAX";
    case neuroexo::CONFIGURE: return "CONFIGURE";
    case neuroexo::GET_POSITION: return "GET_POSITION";
    case neuroexo::START: return "START";
    case neuroexo::END: return "END";
    case neuroexo::ACK: return "ACK";
    case neuroexo::POSITION: return "POSITION";
    default: return "UNKNOWN";
  }
}
double number(const std::string& s) {
  size_t used = 0;
  const double value = std::stod(s, &used);
  if (used != s.size() || !std::isfinite(value)) throw std::invalid_argument("invalid numeric argument");
  return value;
}
}
int main(int argc, char** argv) {
  try {
    std::map<std::string, std::string> args = {
      {"--address", ""}, {"--adapter", "hci0"}, {"--trials", "1"}, {"--duration", "2"},
      {"--start", "0"}, {"--target", "60"}, {"--assistance", "30"}, {"--max-position", "90"},
      {"--period-ms", "40"}
    };
    for (int i = 1; i < argc; ++i) {
      const std::string key = argv[i];
      if (key == "--help") {
        std::cout << "neuroexo_ble_trial --address AA:BB:CC:DD:EE:FF [--adapter hci0]\n"
                     "  [--trials 20] [--duration 2] [--period-ms 40]\n"
                     "  [--start 0] [--target 60] [--assistance 30] [--max-position 90]\n"
                     "Requires NanoNeuroExoProtocol; supplied Nano sketch simulates position.\n";
        return 0;
      }
      if (!args.count(key) || ++i >= argc) throw std::invalid_argument("unknown or incomplete option: " + key);
      args[key] = argv[i];
    }
    if (args["--address"].empty()) throw std::invalid_argument("--address is required");
    const double trialsValue = number(args["--trials"]);
    const double duration = number(args["--duration"]);
    const double periodValue = number(args["--period-ms"]);
    const double maximum = number(args["--max-position"]);
    const double start = number(args["--start"]), target = number(args["--target"]);
    const double assistance = number(args["--assistance"]);
    if (trialsValue < 1 || trialsValue > 1000 || std::floor(trialsValue) != trialsValue ||
        duration < .001 || duration > 3600 || periodValue < 20 || periodValue > 1000 ||
        std::floor(periodValue) != periodValue || maximum <= 0 || maximum > 360 ||
        start < 0 || target < 0 || start > maximum || target > maximum ||
        assistance <= 0 || assistance > 360)
      throw std::invalid_argument("invalid trial count, timing, position, or velocity");
    neuroexo::BleTrialClient client(neuroexo::makeBluezGatt());
    client.setTrace([](bool tx, const neuroexo::Frame& frame) {
      std::cout << (tx ? "TX " : "RX ") << kindName(frame.kind)
                << " seq=" << frame.sequence << " trial=" << frame.trial
                << " a=" << frame.a << " b=" << frame.b << " c=" << frame.c
                << ((frame.flags & neuroexo::SIMULATED) ? " [SIMULATED]" : "") << '\n';
    });
    client.connect(args["--address"], args["--adapter"]);
    std::cout << (client.simulated() ? "SIMULATED arm receiver.\n" : "Hardware arm receiver.\n");
    std::signal(SIGINT, stop);
    std::signal(SIGTERM, stop);
    client.calibrate();
    client.setMaximumPosition(maximum);
    unsigned skipped = 0;
    for (unsigned trial = 1; trial <= unsigned(trialsValue) && !interrupted; ++trial) {
      const auto timing = client.runTrial(
        {uint16_t(trial), target, start, assistance},
        std::chrono::milliseconds(std::llround(duration * 1000)),
        std::chrono::milliseconds(int(periodValue)),
        [](const neuroexo::ArmPosition& position) {
          std::cout << "Arm position=" << std::fixed << std::setprecision(3)
                    << position.degrees << " deg; device_ms=" << position.deviceMilliseconds << '\n';
        }, [] { return interrupted != 0; });
      skipped += timing.skippedPeriods;
      std::cout << "Trial " << trial << ": position replies=" << timing.positionReplies
                << " skipped periods=" << timing.skippedPeriods
                << " max round trip=" << timing.maxRoundTripMs << " ms\n";
    }
    client.disconnect();
    return interrupted ? 130 : skipped ? 2 : 0;
  } catch (const std::exception& error) {
    std::cerr << "BLE trial failed: " << error.what() << '\n';
    return 1;
  }
}
