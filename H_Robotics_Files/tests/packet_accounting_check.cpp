#include "../PacketTrial.hpp"
#include <cassert>
#include <iostream>
using namespace neuroexo;
using namespace packetdemo;
TrialResult trial(bool lose, bool late, bool slow, bool uncertain, bool duplicate, bool corrupt) {
  PacketAccounting accounting(250, 10);
  const auto start = TrialClock::now();
  accounting.start(start);
  PacketStats stats{}; stats[Ready] = 2;
  const unsigned count = slow ? 2499 : 2500;
  for (unsigned i = 1; i <= count; ++i) {
    accounting.attempted(i);
    ++stats[NanoReceived]; ++stats[EchoAttempts];
    if (lose && i == 10) continue;
    uint8_t echo[ECHO_SIZE]; write32(echo + 3, i); echo[7] = 1; finish(echo, ECHO, ECHO_SIZE);
    if (corrupt && i == 11) echo[8] ^= 1;
    const auto now = start + std::chrono::milliseconds(late ? 11000 : 4 * (i - 1) + 1);
    accounting.notification(echo, sizeof(echo), now);
    if (duplicate && i == 12) accounting.notification(echo, sizeof(echo), now);
  }
  return accounting.finish(count, 10, stats, !uncertain);
}
int main() {
  auto r = trial(false, false, false, false, false, false);
  assert(r.pass && r.consistent && r.txHz == 250 && r.rxHz == 250 && r.missing.empty());
  r = trial(true, false, false, false, false, false);
  assert(!r.pass && r.consistent && r.missing == std::vector<uint32_t>{10});
  r = trial(false, true, false, false, false, false);
  assert(!r.pass && r.missing.empty() && r.rxHz == 0); // Drain cannot inflate RX rate.
  r = trial(false, false, true, false, false, false);
  assert(!r.pass && r.attempted == 2499 && r.missing.empty()); // Host skip is not loss.
  r = trial(false, false, false, true, false, false);
  assert(!r.pass && !r.consistent);
  r = trial(false, false, false, false, true, false);
  assert(!r.pass && r.duplicates == 1 && r.received == 2500);
  r = trial(false, false, false, false, false, true);
  assert(!r.pass && r.invalid == 1 && r.missing == std::vector<uint32_t>{11});
  std::cout << "Packet accounting: clean run, loss, late echoes, host skip, uncertain write, duplicate, CRC PASS\n";
}
