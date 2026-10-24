#include "../../NeuroExoFirmware/src/commProtocol.h"
extern "C" {
int control_encode(unsigned mode, unsigned speed, int angle, uint8_t* out) {
  return neuroexo::encodeControl({neuroexo::Mode(mode), neuroexo::Speed(speed), int16_t(angle)}, out);
}
int control_decode(const uint8_t* raw, size_t size, int* out) {
  neuroexo::Control c;
  if (!neuroexo::decodeControl(raw, size, c)) return 0;
  out[0] = int(c.mode); out[1] = int(c.speed); out[2] = c.targetAngleDeg;
  return 1;
}
void telemetry_encode(int angle, int current, unsigned status, uint8_t* out) {
  neuroexo::encodeTelemetry({int16_t(angle), int16_t(current), uint8_t(status)}, out);
}
int telemetry_decode(const uint8_t* raw, size_t size, int* out) {
  neuroexo::Telemetry t;
  if (!neuroexo::decodeTelemetry(raw, size, t)) return 0;
  out[0] = t.currentAngleDeg; out[1] = t.currentMilliAmps; out[2] = t.status;
  return 1;
}
}
