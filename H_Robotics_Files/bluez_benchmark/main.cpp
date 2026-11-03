#include "BluezLink.hpp"
#include "Trial.hpp"
#include <cmath>
#include <csignal>
#include <iomanip>
#include <iostream>
#include <map>

namespace {
volatile std::sig_atomic_t interrupted = 0;
void interrupt(int) { interrupted = 1; }
double number(const std::string& text) {
  size_t count = 0; const double value = std::stod(text, &count);
  if (count != text.size() || !std::isfinite(value)) throw std::runtime_error("invalid number: " + text);
  return value;
}
void help() {
  std::cout << "ProtocolDemo BlueZ benchmark v3 (C++ transmit-only supported, no Python)\n"
      "Sequence: neuroexo_bluez_benchmark --address MAC --sequence-test [--rate 250] [--duration 10]\n"
      "          Requires Nano BLE_PACKET_TEST=1; verify the serial startup banner first.\n"
      "TX-only: add --tx-only to --sequence-test; no telemetry subscription or Teensy required.\n"
      "         Requires stationary Nano nano33ble_rx_only firmware; no --write-response.\n"
      "Monitor: neuroexo_bluez_benchmark --address MAC --monitor [--duration 10]\n"
      "Control: neuroexo_bluez_benchmark --address MAC --angle 20 [--speed slow|medium|high]\n"
      "         [--rate 250] [--duration 10] [--write-response] [--adapter hci0]\n"
      "Stop-only: add --packet stop instead of --angle (5-byte packets).\n"
      "Control packets are 9 bytes, Neutral mode, and can move the joint.\n"
      "Normal firmware ignores controls during an active trial and sends nominal 50 Hz telemetry.\n"
      "TX rate measures successful BlueZ writes; sequence receipts must be read from Nano serial.\n";
}
}
int main(int argc, char** argv) {
  using namespace trial;
  try {
    std::string address, adapter = "hci0", packet = "control", speedName = "slow";
    double rateValue = 250, duration = 10, angle = 0;
    bool monitor = false, response = false, angleSet = false, speedSet = false, sequenceTest = false, txOnly = false;
    for (int i = 1; i < argc; ++i) {
      const std::string key = argv[i];
      if (key == "--help" || key == "-h") { help(); return 0; }
      if (key == "--monitor") { monitor = true; continue; }
      if (key == "--sequence-test") { sequenceTest = true; continue; }
      if (key == "--tx-only") { txOnly = true; continue; }
      if (key == "--write-response") { response = true; continue; }
      if (++i >= argc) throw std::runtime_error("missing value for " + key);
      const std::string value = argv[i];
      if (key == "--address") address = value;
      else if (key == "--adapter") adapter = value;
      else if (key == "--packet") packet = value;
      else if (key == "--speed") { speedName = value; speedSet = true; }
      else if (key == "--rate") rateValue = number(value);
      else if (key == "--duration") duration = number(value);
      else if (key == "--angle") { angle = number(value); angleSet = true; }
      else throw std::runtime_error("unknown option: " + key);
    }
    if (address.empty()) { help(); return 1; }
    if (rateValue < 1 || rateValue > 1000 || std::floor(rateValue) != rateValue || duration < 1 || duration > 600)
      throw std::runtime_error("rate must be an integer 1..1000; duration 1..600 seconds");
    if (packet != "control" && packet != "stop") throw std::runtime_error("packet must be control or stop");
    if (txOnly && (!sequenceTest || response))
      throw std::runtime_error("tx-only requires --sequence-test and disallows --write-response");
    if (sequenceTest && (monitor || angleSet || speedSet || packet != "control"))
      throw std::runtime_error("sequence-test cannot include monitor, angle, speed or stop options");
    if (sequenceTest && std::ceil(rateValue * duration) > 65536)
      throw std::runtime_error("sequence-test allows at most 65536 planned packets per connection");
    if (monitor && (angleSet || speedSet || packet != "control")) throw std::runtime_error("monitor cannot include motion/packet options");
    if (packet == "stop" && (angleSet || speedSet)) throw std::runtime_error("stop packets have no angle/speed");
    if (!monitor && !sequenceTest && packet == "control" && !angleSet) throw std::runtime_error("control test requires --angle; it can move the joint");
    if (angle < 0 || angle > 70 || angle != std::floor(angle))
      throw std::runtime_error("this bench trial accepts integer target angles 0..70 degrees");
    const std::map<std::string, Speed> speeds{{"slow", Speed::Slow}, {"medium", Speed::Medium}, {"high", Speed::High}};
    if (!speeds.count(speedName)) throw std::runtime_error("speed must be slow, medium, or high");
    const unsigned rate = unsigned(rateValue);
    const auto raw = packet == "stop" ? stopFrame() : controlFrame(int16_t(angle), speeds.at(speedName));
    std::signal(SIGINT, interrupt); std::signal(SIGTERM, interrupt);
    Receiver receiver;
    BluezLink link([&](const uint8_t* bytes, size_t size) { receiver.notification(bytes, size); });
    std::cout << "Connecting to ProtocolDemo Nano via BlueZ..." << std::endl;
    link.connect(address, adapter, response, !txOnly);
    if (!txOnly) {
      const auto deadline = Clock::now() + std::chrono::seconds(2);
      while (!receiver.get().seen && Clock::now() < deadline && !interrupted) {
        link.check(); std::this_thread::sleep_for(std::chrono::milliseconds(10));
      }
      auto initial = receiver.get();
      if (!initial.seen) throw std::runtime_error("no valid 12-byte telemetry; check Teensy power, I2C and matching ProtocolDemo firmware");
      if (!monitor && packet == "control" && initial.latest.status &
          (uint8_t(TelemetryStatus::MotionActive) | uint8_t(TelemetryStatus::SafetyFault) | uint8_t(TelemetryStatus::InvalidCommand)))
        throw std::runtime_error("existing motion/fault/invalid-command status; resolve with the firmware operator before starting this trial");
      std::cout << "Initial angle=" << initial.latest.currentAngleDeg << " status=" << unsigned(initial.latest.status) << '\n';
    }
    if (!monitor) std::cout << "Sending " << raw.size() << "-byte " << packet << " packets at " << rate
        << "/s for " << duration << "s, " << (response ? "with" : "without") << " ATT response.\n";
    if (sequenceTest) std::cout << "Sequence IDs start at 0 and advance per attempted write. Verify BLE_PACKET_TEST=1.\n"
        << "Stationary test only: normal firmware interprets IDs as movement targets.\n";
    if (!monitor && !sequenceTest && packet == "control") std::cout << "Neutral mode, target=" << angle
        << " degrees, speed=" << speedName << ". Repeated controls during active motion are ignored by Teensy.\n";
    if (txOnly) std::cout << "TX-ONLY: telemetry subscription disabled; receiver counts are on Nano USB serial.\n"
                         << "No remote motion/fault feedback: use stationary nano33ble_rx_only firmware.\n";
    else std::cout << "Firmware telemetry target: 50/s. Receiver-side command loss: unavailable.\n";
    std::cout << std::flush;
    const auto start = Clock::now();
    const auto end = start + std::chrono::duration_cast<Clock::duration>(std::chrono::duration<double>(duration));
    if (!txOnly) receiver.begin(start, duration);
    auto nextReport = start + std::chrono::seconds(1);
    const uint32_t planned = monitor ? 0 : uint32_t(std::ceil(rate * duration));
    uint32_t slot = 0, attempted = 0, submitted = 0, errors = 0;
    double maxWriteMs = 0, totalWriteMs = 0;
    bool unexpectedMotion = false;
    std::string failure;
    try {
      while (!interrupted && Clock::now() < end) {
        if (monitor || slot >= planned) std::this_thread::sleep_for(std::chrono::milliseconds(2));
        else std::this_thread::sleep_until(start + std::chrono::duration_cast<Clock::duration>(
            std::chrono::duration<double>(double(slot) / rate)));
        const auto now = Clock::now();
        if (interrupted || now >= end) break;
        link.check();
        const auto current = receiver.get();
        if (!txOnly && now - current.last > std::chrono::seconds(1)) throw std::runtime_error("telemetry stale for over one second");
        if (!txOnly && !monitor && packet == "control" && current.fault) throw std::runtime_error("firmware reported SafetyFault or InvalidCommand");
        if (!txOnly && sequenceTest && current.motion) {
          unexpectedMotion = true;
          throw std::runtime_error("motion observed during stationary sequence test; verify test firmware");
        }
        if (!monitor && slot < planned) {
          slot = std::max(slot, uint32_t(std::chrono::duration<double>(now - start).count() * rate));
          if (slot >= planned) break;
          const auto frame = sequenceTest ? sequenceFrame(attempted) : raw;
          ++attempted;
          const auto before = Clock::now();
          try { link.write(frame, response); ++submitted; }
          catch (...) { ++errors; throw; }
          const double writeMs = std::chrono::duration<double, std::milli>(Clock::now() - before).count();
          maxWriteMs = std::max(maxWriteMs, writeMs);
          totalWriteMs += writeMs;
          ++slot;
        }
        if (Clock::now() >= nextReport) {
          const double elapsed = std::chrono::duration<double>(Clock::now() - start).count();
          std::cout << std::fixed << std::setprecision(1) << elapsed << "s TX=" << submitted
              << " TX/s=" << submitted / elapsed;
          if (!txOnly) std::cout << " RX=" << current.received << " RX/s=" << current.received / elapsed
              << " angle=" << current.latest.currentAngleDeg << " status=" << unsigned(current.latest.status);
          std::cout << std::endl;
          nextReport = Clock::now() + std::chrono::seconds(1);
        }
      }
      link.check();
    } catch (const std::exception& error) { failure = error.what(); }
    const auto finished = Clock::now();
    const auto counts = receiver.finish(finished);
    const double elapsed = std::chrono::duration<double>(finished - start).count();
    // Native stop packet, never the old client's Neutral-to-zero substitute.
    // It is outside the measured data stream. A failed write is never retried.
    if (!monitor && attempted && (!sequenceTest || unexpectedMotion)) {
      try { link.write(stopFrame(), true); std::cout << "Final stop write acknowledged by Nano (Teensy execution not confirmed).\n"; }
      catch (const std::exception& error) { failure += std::string(" Final stop unconfirmed: ") + error.what(); }
    }
    if (sequenceTest && attempted && failure.empty() && !interrupted) {
      // Allow queued writes to reach the Nano before disconnecting, without
      // adding these seconds to TX rate. No stop frame: test mode rejects it.
      std::cout << "Waiting up to 2 seconds for pending writes; read Nano serial summary after disconnect.\n" << std::flush;
      const auto drainEnd = Clock::now() + std::chrono::seconds(2);
      try {
        while (!interrupted && Clock::now() < drainEnd) {
          link.check();
          const auto current = receiver.get();
          if (!txOnly && current.latest.status & uint8_t(TelemetryStatus::MotionActive)) {
            link.write(stopFrame(), true);
            throw std::runtime_error("motion observed during drain; stop attempted, verify test firmware");
          }
          if (!txOnly && (Clock::now() - current.last > std::chrono::seconds(1) ||
              current.latest.status & (uint8_t(TelemetryStatus::SafetyFault) | uint8_t(TelemetryStatus::InvalidCommand))))
            throw std::runtime_error("telemetry stale or fault during drain");
          std::this_thread::sleep_for(std::chrono::milliseconds(10));
        }
      } catch (const std::exception& error) { failure = error.what(); }
    }
    link.disconnect();
    const bool complete = failure.empty() && !interrupted && elapsed >= duration;
    const bool met = rateMet(planned, submitted, elapsed, rate, complete);
    std::cout << std::fixed << std::setprecision(2)
        << "\n" << (!complete ? "INCOMPLETE" : monitor ? "MONITOR COMPLETE" : met ? "TX RATE MET" : "TX RATE NOT MET")
        << " | TX=" << submitted / std::max(elapsed, 1e-9) << "/s";
    if (!txOnly) std::cout << " | valid telemetry RX=" << counts.received / std::max(elapsed, 1e-9) << "/s";
    std::cout << "\nPlanned=" << planned << " attempted=" << attempted << " submitted=" << submitted
        << " host-unsent=" << planned - attempted << " write-errors=" << errors << '\n';
    if (!txOnly) std::cout << "Telemetry received=" << counts.received << " invalid=" << counts.invalid << " MotionActive reports=" << counts.motion
        << " max RX gap=" << counts.maxGapMs << " ms\n";
    else std::cout << "Telemetry: DISABLED (transmit-only test).\n";
    std::cout << "Mean write=" << (submitted ? totalWriteMs / submitted : 0)
        << " ms max write=" << maxWriteMs << " ms\n"
        << "Nano/Teensy command receipts, executed-command count, and packet loss: UNAVAILABLE.\n"
        << "TX RATE MET is successful BlueZ submissions within 1% of target, not a zero-loss or 250 Hz algorithm result.\n";
    if (sequenceTest) {
      if (attempted) std::cout << "Attempted sequence IDs: 0.." << attempted - 1 << "; completed writes=" << submitted << '\n';
      if (complete && !counts.invalid && submitted)
        std::cout << "Compare Nano serial Received (unique) with submitted=" << submitted
                  << ": loss% = 100 * (submitted - unique) / submitted.\n"
                  << "Nano's highest-ID estimate misses trailing loss. Host-unsent slots are not radio loss.\n";
      else std::cout << "Incomplete/invalid run: delivery is uncertain; do not calculate a definitive loss percentage.\n";
    }
    if (!failure.empty()) std::cerr << failure << '\n';
    if (interrupted) return 130;
    return !complete || counts.invalid ? 1 : (monitor || met ? 0 : 2);
  } catch (const std::exception& error) {
    std::cerr << "Trial error: " << error.what() << '\n'; return 1;
  }
}
