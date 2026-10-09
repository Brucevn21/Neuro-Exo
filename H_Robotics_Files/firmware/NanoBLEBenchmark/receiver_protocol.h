#ifndef NANO_BENCHMARK_RECEIVER_PROTOCOL_H
#define NANO_BENCHMARK_RECEIVER_PROTOCOL_H

#include <stdint.h>
#include <stddef.h>
#include <string.h>

namespace bench {
const uint8_t VERSION = 2;
const uint16_t MAX_PAYLOAD = 4096;
const uint32_t MAX_COUNT = 65536;
const size_t MAX_WRITE = 244;
enum State { IDLE = 0, RUNNING = 1, STOPPED = 2, ERROR = 3 };

inline uint16_t read16(const uint8_t* p) {
  return uint16_t(p[0]) | (uint16_t(p[1]) << 8);
}
inline uint32_t read32(const uint8_t* p) {
  return uint32_t(p[0]) | (uint32_t(p[1]) << 8) |
         (uint32_t(p[2]) << 16) | (uint32_t(p[3]) << 24);
}
inline void write32(uint8_t* p, uint32_t n) {
  for (unsigned i = 0; i < 4; ++i) p[i] = uint8_t(n >> (8 * i));
}
inline uint32_t crcUpdate(uint32_t crc, const uint8_t* data, size_t length) {
  for (size_t i = 0; i < length; ++i) {
    crc ^= data[i];
    for (unsigned bit = 0; bit < 8; ++bit)
      crc = (crc >> 1) ^ ((crc & 1) ? 0xedb88320UL : 0);
  }
  return crc;
}

// No allocation, serial printing, or hardware access in the receive path.
class Receiver {
public:
  uint8_t state = IDLE;
  uint32_t run = 0, expected = 0, received = 0, fragments = 0;
  uint32_t duplicates = 0, corrupt = 0, invalid = 0, incomplete = 0;
  uint32_t reordered = 0, foreign = 0, firstMs = 0, lastMs = 0;
  uint32_t maxPollGapUs = 0, startedMs = 0, elapsedMs = 0;
  uint16_t payloadSize = 0; // START size 0 permits variable-sized packets.
  uint32_t payloadBytesReceived = 0, minBytes = 0, maxBytes = 0;
  bool partial = false;

  bool start(uint32_t id, uint16_t size, uint32_t count, uint32_t now) {
    if (!id || size > MAX_PAYLOAD || !count || count > MAX_COUNT) {
      state = ERROR;
      return false;
    }
    state = RUNNING; run = id; payloadSize = size; expected = count;
    received = fragments = duplicates = corrupt = invalid = incomplete = 0;
    reordered = foreign = firstMs = lastMs = maxPollGapUs = elapsedMs = 0;
    payloadBytesReceived = minBytes = maxBytes = 0;
    startedMs = now; partial = false; nextOffset = 0; highest = 0;
    memset(seen, 0, sizeof(seen));
    return true;
  }

  void stop(uint32_t now) {
    if (state != RUNNING) return; // Idempotent; keep frozen final counters.
    if (partial) ++incomplete;
    partial = false;
    elapsedMs = now - startedMs;
    state = STOPPED;
  }

  void receive(const uint8_t* value, size_t length, uint32_t now) {
    if (state != RUNNING) return;
    ++fragments;
    if (length <= 10 || length > MAX_WRITE) { ++invalid; return; }
    if (read32(value) != run) { ++foreign; return; }
    const uint16_t sequence = read16(value + 4);
    const uint16_t offset = read16(value + 6);
    const uint16_t size = read16(value + 8);
    if (!size || size > MAX_PAYLOAD || (payloadSize && size != payloadSize)) {
      ++invalid; return;
    }
    const size_t chunk = length - 10, total = size_t(size) + 4;
    if (uint32_t(sequence) >= expected || offset >= total || chunk > total - offset) {
      ++invalid; return;
    }
    // Counts duplicate fragments; the unique packet bitmap prevents counting
    // duplicate delivery as a replacement for a missing packet.
    if (seen[sequence / 8] & (1U << (sequence % 8))) { ++duplicates; return; }
    if (offset == 0) {
      if (partial) ++incomplete;
      partial = true; current = sequence; currentSize = size; nextOffset = 0;
    }
    if (!partial || sequence != current || size != currentSize || offset != nextOffset) { ++invalid; return; }
    memcpy(buffer + offset, value + 10, chunk);
    nextOffset += uint16_t(chunk);
    if (nextOffset != total) return;
    partial = false;
    uint8_t identity[8];
    write32(identity, run);
    identity[4] = uint8_t(sequence); identity[5] = uint8_t(sequence >> 8);
    identity[6] = uint8_t(size); identity[7] = uint8_t(size >> 8);
    uint32_t crc = crcUpdate(0xffffffffUL, identity, sizeof(identity));
    crc = crcUpdate(crc, buffer, size) ^ 0xffffffffUL;
    if (crc != read32(buffer + size)) { ++corrupt; return; }
    if (received && sequence < highest) ++reordered;
    if (!received || sequence > highest) highest = sequence;
    if (!received) firstMs = now - startedMs;
    lastMs = now - startedMs;
    seen[sequence / 8] |= uint8_t(1U << (sequence % 8));
    if (!received || size < minBytes) minBytes = size;
    if (size > maxBytes) maxBytes = size;
    payloadBytesReceived += size;
    ++received;
  }

  void status(uint8_t page, uint32_t now, uint8_t out[20]) const {
    memset(out, 0, 20);
    out[0] = VERSION; out[1] = state; out[2] = page;
    write32(out + 4, run);
    uint32_t a = 0, b = 0, c = 0;
    switch (page) {
      case 0: a = expected; b = received; c = fragments; break;
      case 1: a = duplicates; b = corrupt; c = invalid; break;
      case 2: a = incomplete; b = reordered; c = foreign; break;
      case 3: a = firstMs; b = lastMs; c = maxPollGapUs; break;
      case 4:
        a = state == RUNNING ? now - startedMs : elapsedMs;
        b = payloadSize; c = partial ? 1 : 0; break;
      case 5: a = payloadBytesReceived; b = minBytes; c = maxBytes; break;
      default: out[1] = ERROR; break;
    }
    write32(out + 8, a); write32(out + 12, b); write32(out + 16, c);
  }

private:
  uint8_t seen[MAX_COUNT / 8] = {};
  uint8_t buffer[MAX_PAYLOAD + 4] = {};
  uint16_t current = 0, currentSize = 0, nextOffset = 0, highest = 0;
};
} // namespace bench
#endif
