"""Wire-compatible implementation of lib/commProtocol (no GUI dependencies)."""

from dataclasses import dataclass
from enum import IntEnum, IntFlag
import struct

SERVICE_UUID = "0000180c-0000-1000-8000-00805f9b34fb"
COMMAND_UUID = "00002a56-0000-1000-8000-00805f9b34fb"
TELEMETRY_UUID = "00002a57-0000-1000-8000-00805f9b34fb"
MIN_ANGLE, MAX_ANGLE = -150, 80


class Mode(IntEnum):
    RESISTIVE = 0
    ASSISTIVE = 1
    NEUTRAL = 2
    CALIBRATE = 3
    DEMO = 4
    HOME = 5


class Status(IntFlag):
    ACTIVE = 1
    COMMAND_TIMEOUT = 2
    INVALID_COMMAND = 4
    SAFETY_FAULT = 8
    TARGET_REACHED = 16
    TRIAL_TIMEOUT = 32
    CALIBRATED = 64
    PATIENT_RESISTING = 128


def crc8(data):
    crc = 0
    for value in data:
        crc ^= value
        for _ in range(8):
            crc = ((crc << 1) ^ (0x07 if crc & 0x80 else 0)) & 0xFF
    return crc


def frame(kind, payload=b""):
    body = bytes([kind, len(payload)]) + payload
    return b"\x02" + body + bytes([crc8(body), 3])


@dataclass(frozen=True)
class Command:
    mode: Mode
    setting: int
    target: int

    def encode(self):
        mode = Mode(self.mode)
        if type(self.setting) is not int or type(self.target) is not int:
            raise ValueError("Setting and target must be whole numbers")
        if not MIN_ANGLE <= self.target <= MAX_ANGLE:
            raise ValueError(f"Target must be {MIN_ANGLE} to {MAX_ANGLE} degrees")
        if mode in (Mode.ASSISTIVE, Mode.RESISTIVE):
            valid = 1 <= self.setting <= 9
        elif mode == Mode.CALIBRATE:
            valid = self.setting == 0
        else:
            valid = 0 <= self.setting <= 2
        if not valid:
            raise ValueError("Invalid setting for this mode")
        target = 0 if mode in (Mode.DEMO, Mode.HOME) else self.target
        return frame(0x10, struct.pack(">BBh", mode, self.setting, target))


STOP_FRAME = frame(0x12)


@dataclass(frozen=True)
class Telemetry:
    angle: float
    current_a: float
    status: Status
    effort_nm: float

    @property
    def active(self):
        return bool(self.status & Status.ACTIVE)

    @classmethod
    def decode(cls, data):
        data = bytes(data)
        if (len(data) != 12 or data[:3] != b"\x02\x11\x07"
                or data[-1] != 3 or crc8(data[1:-2]) != data[-2]):
            raise ValueError("Invalid telemetry frame (length, markers or CRC)")
        angle, current, status, effort = struct.unpack(">hhBh", data[3:-2])
        return cls(angle, current / 1000, Status(status), effort / 100)


def end_reason(sample):
    for flag, text in ((Status.SAFETY_FAULT, "safety_fault"),
                       (Status.PATIENT_RESISTING, "patient_resisting"),
                       (Status.TRIAL_TIMEOUT, "trial_timeout"),
                       (Status.INVALID_COMMAND, "invalid_command"),
                       (Status.TARGET_REACHED, "target_reached")):
        if sample.status & flag:
            return text
    return "device_stopped"
