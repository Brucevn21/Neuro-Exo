// Native tests exercise the exact core included by the Nano sketch.
#include "../NanoBLEBenchmark/receiver_protocol.h"
#include <sstream>
// Independent serializer using the same default ostream operations as the
// read-only reference file. Verifies Python formatting on identical doubles.
extern "C" int format_eeg(const double* values, char* out, size_t capacity) {
  std::ostringstream record;
  for (int i = 0; i < 8; ++i) {
    record << values[i];
    if (i < 7) record << ";";
  }
  record << "\n";
  const std::string text = record.str();
  if (text.size() > capacity) return -1;
  memcpy(out, text.data(), text.size());
  return int(text.size());
}
static bench::Receiver receiver;
extern "C" {
void reset_receiver() { receiver = bench::Receiver(); }
int start_receiver(uint32_t id, uint16_t size, uint32_t count, uint32_t now) {
  return receiver.start(id, size, count, now);
}
void receive_fragment(const uint8_t* bytes, size_t size, uint32_t now) {
  receiver.receive(bytes, size, now);
}
void stop_receiver(uint32_t now) { receiver.stop(now); }
void status_receiver(uint8_t page, uint32_t now, uint8_t* bytes) {
  receiver.status(page, now, bytes);
}
}
