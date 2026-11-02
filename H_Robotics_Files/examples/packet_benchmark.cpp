// BBB -> Nano -> BBB diagnostic, using BlueZ through the existing GIO transport.
#include "../PacketGatt.hpp"
#include "../PacketTrial.hpp"
#include <iomanip>
#include <iostream>
#include <random>
#include <thread>

using namespace neuroexo;
using namespace packetdemo;
namespace {
PacketStats snapshot(PacketGatt& client, uint8_t kind, uint32_t token, uint32_t session) {
  client.writePacket(demoPacket(kind, token), true);
  const auto deadline = TrialClock::now() + std::chrono::seconds(5);
  while (TrialClock::now() < deadline) {
    const auto stats = decodePacketStats(client.readStatistics());
    if (stats[Session] == session && stats[Token] == token) {
      if (stats[Ready] != 2)
        throw std::runtime_error("Nano-only loopback required: upload nano33ble_packet_loopback");
      return stats;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(50));
  }
  throw std::runtime_error("timed out waiting for fresh Nano statistics");
}
double numeric(const std::string& text) {
  size_t consumed = 0;
  const double value = std::stod(text, &consumed);
  if (consumed != text.size() || !std::isfinite(value)) throw std::runtime_error("invalid number: " + text);
  return value;
}
void help() {
  std::cout << "Usage: neuroexo_packet_benchmark --address AA:BB:CC:DD:EE:FF\n"
               "       [--rate 250] [--duration 10] [--adapter hci0] [--write-response]\n"
               "Requires nano33ble_packet_loopback firmware (NeuroExoLoopback).\n"
               "9-byte numbered writes; compare Write Command with --write-response.\n"
               "Reports host scheduling, WriteValue, RTT and notification timing.\n"
               "Two seconds of drain follow the timed run. Exit: 0 PASS, 2 FAIL, 1 INCONCLUSIVE.\n";
}
}
int main(int argc, char** argv) {
  try {
    std::string address, adapter = "hci0";
    double rateValue = 250, duration = 10;
    bool writeResponse = false;
    for (int i = 1; i < argc; ++i) {
      const std::string option = argv[i];
      if (option == "--help" || option == "-h") { help(); return 0; }
      if (option == "--write-response") { writeResponse = true; continue; }
      if (++i >= argc) throw std::runtime_error("missing value for " + option);
      if (option == "--address") address = argv[i];
      else if (option == "--adapter") adapter = argv[i];
      else if (option == "--rate") rateValue = numeric(argv[i]);
      else if (option == "--duration") duration = numeric(argv[i]);
      else throw std::runtime_error("unknown option " + option);
    }
    if (address.empty()) { help(); return 1; }
    if (rateValue < 1 || rateValue > 1000 || std::floor(rateValue) != rateValue || duration < 1 || duration > 600)
      throw std::runtime_error("rate must be an integer 1..1000; duration must be 1..600 seconds");
    const unsigned rate = unsigned(rateValue);
    const uint32_t total = uint32_t(std::ceil(rate * duration));
    PacketAccounting accounting(rate, duration);
    // accounting outlives the transport and its notification thread.
    auto client = makePacketGatt([&](const uint8_t* raw, size_t size) { accounting.notification(raw, size); });
    std::cout << "Connecting through BlueZ to " << address << "..." << std::endl;
    client->connect(address, adapter);
    std::this_thread::sleep_for(std::chrono::milliseconds(500));
    uint32_t session = std::random_device{}();
    if (!session) session = 1;
    const auto initial = snapshot(*client, START, session, session);
    for (size_t i = NanoReceived; i < Ready; ++i)
      if (initial[i]) throw std::runtime_error("Nano counters did not reset");
    std::cout << "Nano-only BLE trial: " << rate << " packets/s for " << duration
              << " seconds; planned=" << total << "; "
              << (writeResponse ? "Write Request" : "Write Command")
              << ". Motors disabled.\n" << std::flush;
    const auto started = TrialClock::now();
    const auto end = started + std::chrono::duration_cast<TrialClock::duration>(std::chrono::duration<double>(duration));
    accounting.start(started);
    auto nextReport = started + std::chrono::seconds(1);
    uint32_t slot = 0, submitted = 0;
    std::string error;
    while (slot < total) {
      const auto due = started + std::chrono::duration_cast<TrialClock::duration>(
          std::chrono::duration<double>(double(slot) / rate));
      std::this_thread::sleep_until(due);
      auto now = TrialClock::now();
      if (now >= end) break;
      // Skip missed slots rather than sending catch-up bursts that inflate rate.
      slot = std::max(slot, uint32_t(std::chrono::duration<double>(now - started).count() * rate));
      if (slot >= total) break;
      const auto scheduled = started + std::chrono::duration_cast<TrialClock::duration>(
          std::chrono::duration<double>(double(slot) / rate));
      now = TrialClock::now();
      if (now >= end) break;
      accounting.attempted(slot + 1, scheduled, now);
      try {
        const auto raw = demoPacket(DATA, slot + 1);
        const auto writeStarted = TrialClock::now();
        client->writePacket(raw, false, writeResponse);
        accounting.submitted(slot + 1, writeStarted, TrialClock::now());
        ++submitted;
      } catch (const std::exception& e) {
        error = std::string("write outcome unknown: ") + e.what(); break;
      }
      ++slot;
      if (TrialClock::now() >= nextReport) {
        std::cout << "TX submitted=" << submitted << " RX echoes=" << accounting.received() << std::endl;
        nextReport = TrialClock::now() + std::chrono::seconds(1);
      }
    }
    if (error.empty()) std::this_thread::sleep_until(end);
    const double elapsed = std::chrono::duration<double>(TrialClock::now() - started).count();
    PacketStats stats{};
    bool complete = false;
    try {
      std::this_thread::sleep_for(std::chrono::seconds(2));
      stats = snapshot(*client, SNAPSHOT, session == UINT32_MAX ? 1 : session + 1, session);
      std::this_thread::sleep_for(std::chrono::milliseconds(200));
      complete = error.empty() && client->connected();
    } catch (const std::exception& e) {
      if (error.empty()) error = std::string("final counts unavailable: ") + e.what();
    }
    const auto result = accounting.finish(submitted, elapsed, stats, complete);
    client->disconnect();
    std::cout << std::fixed << std::setprecision(2)
              << "\n" << (!result.consistent ? "INCONCLUSIVE" : result.pass ? "PASS" : "FAIL")
              << " | TX=" << result.txHz << "/s | RX during sending=" << result.rxHz << "/s\n"
              << "Planned=" << total << " attempted=" << result.attempted << " submitted=" << submitted
              << " host-unsent=" << total - result.attempted << '\n'
              << "Schedule lateness n=" << result.scheduleLatenessSamples << " mean/max="
              << result.scheduleLatenessMeanMs << '/' << result.scheduleLatenessMaxMs
              << " ms | WriteValue n=" << result.writeCallSamples << " mean/max="
              << result.writeCallMeanMs << '/' << result.writeCallMaxMs << " ms\n"
              << "Round trip n=" << result.roundTripSamples << " mean/max="
              << result.roundTripMeanMs << '/' << result.roundTripMaxMs
              << " ms | RX inter-arrival n=" << result.interArrivalSamples
              << " mean/max/jitter(sd)="
              << result.interArrivalMeanMs << '/' << result.interArrivalMaxMs << '/'
              << result.interArrivalJitterMs << " ms | max consecutive missing="
              << result.maxConsecutiveMissing << '\n'
              << "Nano accepted=" << stats[NanoReceived] << " BBB echoes=" << result.received
              << " missing echoes=" << result.missing.size() << '\n'
              << "Nano queue drops=" << stats[QueueDrops] << " notify failures=" << stats[EchoFailures]
              << " Nano duplicates=" << stats[NanoDuplicates] << " Nano invalid=" << stats[NanoInvalid] << '\n'
              << "BBB duplicates=" << result.duplicates << " invalid=" << result.invalid
              << " unexpected=" << result.unexpected << '\n';
    if (result.consistent) {
      std::cout << "Not received by Nano=" << result.attempted - stats[NanoReceived]
                << " | Nano accepted but no return=" << stats[NanoReceived] - result.received
                << " | round-trip loss=" << (result.attempted ? 100.0 * result.missing.size() / result.attempted : 0) << "%\n";
    }
    if (!result.missing.empty()) {
      std::cout << "Missing echo sequence IDs:";
      for (auto sequence : result.missing) std::cout << ' ' << sequence;
      std::cout << '\n';
    }
    if (!result.consistent) std::cout << "Delivery uncertain: " << (error.empty() ? "receiver counts inconsistent" : error) << '\n';
    else if (!result.pass) std::cout << "FAIL means loss/errors or the requested TX/RX rate was not sustained.\n";
    return !result.consistent ? 1 : result.pass ? 0 : 2;
  } catch (const std::exception& error) {
    std::cerr << "INCONCLUSIVE: " << error.what() << '\n';
    return 1;
  }
}
