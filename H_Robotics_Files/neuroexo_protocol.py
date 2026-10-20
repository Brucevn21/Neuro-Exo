"""NeuroExo control/telemetry codec matching the Teensy commProtocol.cpp."""
from dataclasses import dataclass
from enum import IntEnum, IntFlag
import struct

SERVICE = "b7e20000-6a2b-4f10-9c31-8b674045a901"
COMMAND = "b7e20001-6a2b-4f10-9c31-8b674045a901"  # BBB TX, Nano write
TELEMETRY = "b7e20002-6a2b-4f10-9c31-8b674045a901"  # BBB RX, Nano notify
STX, ETX = 0x02, 0x03
CONTROL_TYPE, TELEMETRY_TYPE = 0x10, 0x11
CONTROL_SIZE, TELEMETRY_SIZE = 9, 10
COMMAND_TIMEOUT_MS = 250


class Mode(IntEnum):
    Resistive = 0
    Assistive = 1
    Neutral = 2


class Speed(IntEnum):
    Slow = 0
    Medium = 1
    High = 2


class Status(IntFlag):
    MotionActive = 1
    CommandTimeout = 2
    InvalidCommand = 4
    SafetyFault = 8


def crc8(data):
    crc = 0
    for value in data:
        crc ^= value
        for _ in range(8):
            crc = ((crc << 1) ^ (0x07 if crc & 0x80 else 0)) & 0xff
    return crc


def _encode(kind, payload):
    body = bytes((kind, len(payload))) + payload
    return bytes((STX,)) + body + bytes((crc8(body), ETX))


def _payload(raw, kind, size):
    raw = bytes(raw)
    if len(raw) != size:
        raise ValueError(f"expected exactly {size} bytes, received {len(raw)}")
    if raw[0] != STX or raw[-1] != ETX or raw[1] != kind or raw[2] != size - 5:
        raise ValueError("invalid frame markers, type, or payload length")
    if raw[-2] != crc8(raw[1:-2]):
        raise ValueError("CRC8 mismatch")
    return raw[3:-2]


def _int16(value):
    if not isinstance(value, int) or isinstance(value, bool) or not -32768 <= value <= 32767:
        raise ValueError("angle/current must be an integer in -32768..32767")
    return value


@dataclass(frozen=True)
class Control:
    mode: Mode
    speed: Speed
    target_angle_deg: int

    def encode(self):
        return _encode(CONTROL_TYPE, struct.pack(">BBh", Mode(self.mode), Speed(self.speed),
                                                 _int16(self.target_angle_deg)))

    @classmethod
    def decode(cls, raw):
        mode, speed, angle = struct.unpack(">BBh", _payload(raw, CONTROL_TYPE, CONTROL_SIZE))
        return cls(Mode(mode), Speed(speed), angle)

    def describe(self):
        return (f"mode={Mode(self.mode).name} speed={Speed(self.speed).name} "
                f"target={self.target_angle_deg} deg")


@dataclass(frozen=True)
class Telemetry:
    current_angle_deg: int
    current_milliamps: int
    status: Status

    def encode(self):
        if not isinstance(self.status, int) or not 0 <= self.status <= 255:
            raise ValueError("status must be a byte")
        return _encode(TELEMETRY_TYPE, struct.pack(">hhB", _int16(self.current_angle_deg),
                                                  _int16(self.current_milliamps), self.status))

    @classmethod
    def decode(cls, raw):
        angle, current, status = struct.unpack(">hhB", _payload(raw, TELEMETRY_TYPE, TELEMETRY_SIZE))
        return cls(angle, current, Status(status))

    def describe(self):
        flags = " ".join(f"{bit.name}={int(bool(self.status & bit))}" for bit in Status)
        return (f"angle={self.current_angle_deg} deg current={self.current_milliamps} mA "
                f"status=0x{int(self.status):02x} {flags}")


def decode(raw):
    raw = bytes(raw)
    if len(raw) < 2 or raw[1] not in (CONTROL_TYPE, TELEMETRY_TYPE):
        raise ValueError("unknown frame type")
    return Control.decode(raw) if raw[1] == CONTROL_TYPE else Telemetry.decode(raw)
