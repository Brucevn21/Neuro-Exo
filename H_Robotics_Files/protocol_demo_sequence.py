"""Telemetry/GATT adapter for the Nano's stationary BLE_PACKET_TEST mode.

Control frames retain the 9-byte Neutral/Slow layout; their angle field is a
sequence ID ONLY when the Nano has BLE_PACKET_TEST=1. Normal firmware can move
in response to these frames. This module does not detect the flashed mode.
"""
from dataclasses import dataclass
import struct

from neuroexo_protocol import Status, Telemetry as LegacyTelemetry, crc8

SERVICE = "0000180c-0000-1000-8000-00805f9b34fb"
COMMAND = "00002a56-0000-1000-8000-00805f9b34fb"
TELEMETRY = "00002a57-0000-1000-8000-00805f9b34fb"


@dataclass(frozen=True)
class Telemetry(LegacyTelemetry):
    patient_effort_centinm: int = 0

    @classmethod
    def decode(cls, raw):
        raw = bytes(raw)
        if (len(raw) != 12 or raw[:3] != b"\x02\x11\x07" or
                raw[-1] != 3 or raw[-2] != crc8(raw[1:-2])):
            raise ValueError("invalid ProtocolDemo telemetry length, markers or CRC")
        angle, current, status, effort = struct.unpack(">hhBh", raw[3:-2])
        return cls(angle, current, Status(status), effort)

    def encode(self):
        body = b"\x11\x07" + struct.pack(">hhBh", self.current_angle_deg,
            self.current_milliamps, self.status, self.patient_effort_centinm)
        return b"\x02" + body + bytes((crc8(body), 3))
