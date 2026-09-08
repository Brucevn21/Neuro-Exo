/*
 * JS port of lib/commProtocol/src/commProtocol.cpp's 7-byte JointPacket.
 * Must stay byte-for-byte in sync with that file.
 *
 * Byte layout (56 bits total):
 *   Byte 0     : Mode (bits 1:0) + Speed (bits 3:2)
 *   Bytes 1-2  : Current Angle [degrees], int16, MSB first
 *   Bytes 3-4  : Target Angle [degrees], int16, MSB first
 *   Bytes 5-6  : Current [mA], int16, MSB first
 */

const NeuroExoProtocol = {
  PACKET_SIZE: 7,

  // BLE UUIDs, shared with Nano33BLEFirmware.ino and re-served under the
  // same names by tools/beaglebone-bridge (the app now connects to the
  // BeagleBone Black, not the Nano, but the wire format is unchanged).
  SERVICE_UUID: 0x180c,
  COMMAND_CHAR_UUID: 0x2a56,
  TELEMETRY_CHAR_UUID: 0x2a57,
  STOP_CHAR_UUID: 0x2a58,

  Mode: { Resistive: 0, Assistive: 1, Neutral: 2 },
  Speed: { Slow: 0, Medium: 1, High: 2 },

  encodeJointPacket({ mode, speed, currentAngleDeg = 0, targetAngleDeg = 0, currentMilliAmps = 0 }) {
    const buf = new Uint8Array(NeuroExoProtocol.PACKET_SIZE);
    const view = new DataView(buf.buffer);
    buf[0] = ((speed & 0x03) << 2) | (mode & 0x03);
    view.setInt16(1, currentAngleDeg, false);
    view.setInt16(3, targetAngleDeg, false);
    view.setInt16(5, currentMilliAmps, false);
    return buf;
  },

  decodeJointPacket(dataView) {
    const b0 = dataView.getUint8(0);
    return {
      mode: b0 & 0x03,
      speed: (b0 >> 2) & 0x03,
      currentAngleDeg: dataView.getInt16(1, false),
      targetAngleDeg: dataView.getInt16(3, false),
      currentMilliAmps: dataView.getInt16(5, false),
    };
  },
};
