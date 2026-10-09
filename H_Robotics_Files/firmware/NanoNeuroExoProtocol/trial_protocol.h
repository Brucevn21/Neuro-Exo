#ifndef NEUROEXO_TRIAL_PROTOCOL_H
#define NEUROEXO_TRIAL_PROTOCOL_H

#include <stdint.h>
#include <stddef.h>
#include <string.h>

namespace neuroexo {
static const uint8_t MAGIC = 0x4e, VERSION = 1, SIMULATED = 1;
static const uint32_t WATCHDOG_MS = 2000;
static const int32_t SCALE = 1000;
enum Kind { CALIBRATE = 1, SET_MAX = 2, CONFIGURE = 3, GET_POSITION = 4,
            START = 5, END = 6, ACK = 0x80, POSITION = 0x81, INFO = 0x82 };
enum State { BOOT = 0, IDLE = 1, READY = 2, RUNNING = 3, ENDED = 4, FAULT = 5 };
enum Status { OK = 0, BAD_STATE = 1, RANGE = 2, WRONG_TRIAL = 3,
              BAD_SEQUENCE = 4, UNSUPPORTED = 5 };

inline uint16_t read16(const uint8_t* p) {
  return uint16_t(p[0]) | uint16_t(uint16_t(p[1]) << 8);
}
inline int32_t read32(const uint8_t* p) {
  uint32_t bits = uint32_t(p[0]) | (uint32_t(p[1]) << 8) |
                  (uint32_t(p[2]) << 16) | (uint32_t(p[3]) << 24);
  int32_t value;
  memcpy(&value, &bits, sizeof(value));
  return value;
}
inline void write16(uint8_t* p, uint16_t v) {
  p[0] = uint8_t(v); p[1] = uint8_t(v >> 8);
}
inline void write32(uint8_t* p, int32_t v) {
  const uint32_t bits = uint32_t(v);
  for (int i = 0; i < 4; ++i) p[i] = uint8_t(bits >> (8 * i));
}
struct Frame {
  uint8_t kind, flags;
  uint16_t sequence, trial;
  int32_t a, b, c;
};
inline bool decode(const uint8_t* p, size_t length, Frame& f) {
  if (length != 20 || p[0] != MAGIC || p[1] != VERSION || p[3] > SIMULATED)
    return false;
  f.kind = p[2]; f.flags = p[3];
  f.sequence = read16(p + 4); f.trial = read16(p + 6);
  f.a = read32(p + 8); f.b = read32(p + 12); f.c = read32(p + 16);
  return true;
}
inline void encode(const Frame& f, uint8_t* p) {
  p[0] = MAGIC; p[1] = VERSION; p[2] = f.kind; p[3] = f.flags;
  write16(p + 4, f.sequence); write16(p + 6, f.trial);
  write32(p + 8, f.a); write32(p + 12, f.b); write32(p + 16, f.c);
}

// PROTOCOL SIMULATOR ONLY. No encoder reads, homing, motor outputs, or I2C writes.
// This codec can be reused by a future BBB C++ GATT client.
class Simulator {
 public:
  State state;
  uint16_t trial;
  int32_t maximum, target, velocity;
  int64_t positionScaled;
  uint32_t lastTick, lastCommand;
  uint16_t lastSequence;
  uint8_t lastRaw[20], lastReply[20];

  Simulator() { reset(); }
  void reset() {
    state = BOOT; trial = 0; maximum = target = velocity = 0;
    positionScaled = 0; lastTick = lastCommand = 0; lastSequence = 0;
    memset(lastRaw, 0, sizeof(lastRaw));
    memset(lastReply, 0, sizeof(lastReply));
  }
  int32_t position() const { return int32_t(positionScaled / SCALE); }
  void info(uint8_t* out) const {
    const Frame f = {INFO, SIMULATED, 0, 0, SCALE, int32_t(WATCHDOG_MS), state};
    encode(f, out);
  }
  void tick(uint32_t now) {
    if (state == RUNNING) {
      if (uint32_t(now - lastCommand) > WATCHDOG_MS) {
        state = FAULT;
      } else {
        const int64_t step = int64_t(velocity) * uint32_t(now - lastTick);
        const int64_t goal = int64_t(target) * SCALE;
        if (positionScaled < goal) {
          positionScaled += step;
          if (positionScaled > goal) positionScaled = goal;
        } else {
          positionScaled -= step;
          if (positionScaled < goal) positionScaled = goal;
        }
      }
    }
    lastTick = now;
  }
  void acknowledge(const Frame& f, Status status, uint8_t* out) const {
    const Frame reply = {ACK, SIMULATED, f.sequence, f.trial, f.kind, status, state};
    encode(reply, out);
  }
  bool process(const uint8_t* raw, size_t length, uint32_t now, uint8_t* out) {
    Frame f;
    if (!decode(raw, length, f) || f.flags || !f.sequence) return false;
    tick(now);
    if (f.sequence == lastSequence) {
      if (memcmp(raw, lastRaw, 20) == 0) memcpy(out, lastReply, 20);
      else acknowledge(f, BAD_SEQUENCE, out);
      return true;
    }
    const uint16_t expected = (lastSequence == 0 || lastSequence == 65535)
                               ? 1 : uint16_t(lastSequence + 1);
    if (f.sequence != expected) {
      acknowledge(f, BAD_SEQUENCE, out);
      return true;
    }
    Status status = OK;
    if ((f.kind == CALIBRATE || f.kind == SET_MAX) && f.trial) {
      status = WRONG_TRIAL;
    } else if (f.kind == CALIBRATE) {
      if (f.a || f.b || f.c) status = RANGE;
      else if (state == RUNNING) status = BAD_STATE;
      else {
        state = IDLE; trial = 0; maximum = 0; positionScaled = 0;
      }
    } else if (f.kind == SET_MAX) {
      if (state != IDLE && state != ENDED) status = BAD_STATE;
      else if (f.a <= 0 || f.a > 360000 || f.b || f.c) status = RANGE;
      else { maximum = f.a; trial = 0; state = IDLE; }
    } else if (f.kind == CONFIGURE) {
      if ((state != IDLE && state != ENDED) || !maximum) status = BAD_STATE;
      else if (!f.trial) status = WRONG_TRIAL;
      else if (f.a < 0 || f.a > maximum || f.b < 0 || f.b > maximum ||
               f.c <= 0 || f.c > 360000) status = RANGE;
      else {
        target = f.a; velocity = f.c; positionScaled = int64_t(f.b) * SCALE;
        trial = f.trial; state = READY;
      }
    } else if (f.kind == GET_POSITION || f.kind == START || f.kind == END) {
      if (f.trial != trial) status = WRONG_TRIAL;
      else if (f.a || f.b || f.c) status = RANGE;
      else if (f.kind == START) {
        if (state != READY) status = BAD_STATE;
        else state = RUNNING;
      } else if (f.kind == END) {
        if (state != READY && state != RUNNING && state != ENDED && state != FAULT)
          status = BAD_STATE;
        else if (state != FAULT) state = ENDED;
      }
    } else status = UNSUPPORTED;
    if (status == OK) lastCommand = now;
    if (status == OK && f.kind == GET_POSITION) {
      int32_t timestamp;
      memcpy(&timestamp, &now, sizeof(timestamp));
      const Frame reply = {POSITION, SIMULATED, f.sequence, f.trial,
                           position(), timestamp, state};
      encode(reply, out);
    } else acknowledge(f, status, out);
    lastSequence = f.sequence;
    memcpy(lastRaw, raw, 20); memcpy(lastReply, out, 20);
    return true;
  }
};
}  // namespace neuroexo
#endif
