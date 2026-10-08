#include "../NanoNeuroExoProtocol/trial_protocol.h"
static neuroexo::Simulator nano;
extern "C" {
void trial_reset() { nano.reset(); }
int trial_receive(const uint8_t* raw, size_t length, uint32_t now, uint8_t* out) {
  return nano.process(raw, length, now, out);
}
void trial_tick(uint32_t now) { nano.tick(now); }
int trial_state() { return nano.state; }
int trial_position() { return nano.position(); }
}
