// Exposes the exact firmware codec (firmware/commProtocol) to the Python tests.
#include "../firmware/commProtocol/commProtocol.cpp"
using namespace NeuroExoProtocol;
extern "C" {
int comm_crc8(const uint8_t* data, int length) { return crc8(data, uint8_t(length)); }
int comm_encode_control(int mode, int speed, int target, uint8_t* out) {
  ControlPacket packet;
  packet.mode = Mode(mode);
  packet.speed = Speed(speed);
  packet.targetAngleDeg = int16_t(target);
  return encodeControlFrame(packet, out);
}
int comm_encode_telemetry(int angle, int milliamps, int status, uint8_t* out) {
  TelemetryPacket packet;
  packet.currentAngleDeg = int16_t(angle);
  packet.currentMilliAmps = int16_t(milliamps);
  packet.status = uint8_t(status);
  return encodeTelemetryFrame(packet, out);
}
// Feeds one buffer through a fresh FrameParser, as the Nano and Teensy do.
// Returns 1 for a control frame, 2 for telemetry, 0 when nothing decodes.
int comm_parse(const uint8_t* raw, int length, int* a, int* b, int* c) {
  FrameParser parser;
  for (int i = 0; i < length; ++i) parser.push(raw[i]);
  Frame frame;
  if (!parser.takeFrame(frame)) return 0;
  ControlPacket control;
  if (decodeControlFrame(frame, control)) {
    *a = int(control.mode); *b = int(control.speed); *c = control.targetAngleDeg;
    return 1;
  }
  TelemetryPacket telemetry;
  if (decodeTelemetryFrame(frame, telemetry)) {
    *a = telemetry.currentAngleDeg; *b = telemetry.currentMilliAmps; *c = telemetry.status;
    return 2;
  }
  return 0;
}
}
