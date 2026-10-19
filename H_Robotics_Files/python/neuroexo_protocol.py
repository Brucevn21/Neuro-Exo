"""NeuroExo framed control/telemetry protocol, no hardware imports.

Python port of firmware/commProtocol (copied from NeuroExoFirmware on the
ProtocolDemo branch). Every frame is STX | TYPE | LENGTH | PAYLOAD | CRC8 | ETX;
the CRC-8 (polynomial 0x07, initial value 0) covers TYPE, LENGTH and PAYLOAD.
Signed 16-bit fields are big-endian.
"""
from dataclasses import dataclass
from enum import IntEnum, IntFlag
import struct

DEVICE_NAME = "Nano33BLE_Master"
SERVICE = "0000180c-0000-1000-8000-00805f9b34fb"
COMMAND = "00002a56-0000-1000-8000-00805f9b34fb"
TELEMETRY = "00002a57-0000-1000-8000-00805f9b34fb"

START_BYTE, STOP_BYTE = 0x02, 0x03
MESSAGE_CONTROL, MESSAGE_TELEMETRY = 0x10, 0x11
CONTROL_PAYLOAD = struct.Struct(">BBh")
TELEMETRY_PAYLOAD = struct.Struct(">hhB")
FRAME_OVERHEAD = 5
CONTROL_FRAME_SIZE = FRAME_OVERHEAD + CONTROL_PAYLOAD.size      # 9
TELEMETRY_FRAME_SIZE = FRAME_OVERHEAD + TELEMETRY_PAYLOAD.size  # 10
COMMAND_TIMEOUT_MS = 250
TELEMETRY_INTERVAL_MS = 20  # the Nano polls the Teensy at 50 Hz


class Mode(IntEnum):
    RESISTIVE = 0
    ASSISTIVE = 1
    NEUTRAL = 2


class Speed(IntEnum):
    SLOW = 0
    MEDIUM = 1
    HIGH = 2


class Status(IntFlag):
    NONE = 0
    MOTION_ACTIVE = 1 << 0
    COMMAND_TIMEOUT = 1 << 1
    INVALID_COMMAND = 1 << 2
    SAFETY_FAULT = 1 << 3
    TARGET_REACHED = 1 << 4  # proposed firmware addition; see docs/NEUROEXO_PROTOCOL.md


def crc8(data):
    crc = 0
    for byte in data:
        crc ^= byte
        for _ in range(8):
            crc = ((crc << 1) ^ 0x07) & 0xFF if crc & 0x80 else (crc << 1) & 0xFF
    return crc


def _frame(kind, payload):
    body = bytes((kind, len(payload))) + payload
    return bytes((START_BYTE,)) + body + bytes((crc8(body), STOP_BYTE))


def status_names(status):
    names = [flag.name for flag in Status if flag and status & flag]
    unknown = status & ~0x1F
    if unknown:
        names.append(f"UNKNOWN(0x{unknown:02X})")
    return "|".join(names) or "NONE"


@dataclass(frozen=True)
class Control:
    mode: Mode
    speed: Speed
    target_deg: int

    def encode(self):
        if self.mode not in tuple(Mode) or self.speed not in tuple(Speed):
            raise ValueError("mode or speed out of range")
        try:
            payload = CONTROL_PAYLOAD.pack(self.mode, self.speed, self.target_deg)
        except struct.error as error:
            raise ValueError(f"target must be a signed 16-bit angle: {error}") from error
        return _frame(MESSAGE_CONTROL, payload)

    def describe(self):
        return (f"CONTROL mode={Mode(self.mode).name} speed={Speed(self.speed).name} "
                f"target={self.target_deg} deg")


@dataclass(frozen=True)
class Telemetry:
    angle_deg: int
    milliamps: int
    status: int

    def encode(self):
        try:
            payload = TELEMETRY_PAYLOAD.pack(self.angle_deg, self.milliamps, self.status)
        except struct.error as error:
            raise ValueError(f"telemetry field out of range: {error}") from error
        return _frame(MESSAGE_TELEMETRY, payload)

    def has(self, flag):
        return bool(self.status & flag)

    def describe(self):
        return (f"TELEMETRY angle={self.angle_deg} deg current={self.milliamps} mA "
                f"status={status_names(self.status)}")


def decode(raw):
    """Decode exactly one complete frame; raise ValueError for anything else."""
    raw = bytes(raw)
    if len(raw) < FRAME_OVERHEAD or raw[0] != START_BYTE or raw[-1] != STOP_BYTE:
        raise ValueError("missing STX/ETX or truncated frame")
    kind, length = raw[1], raw[2]
    if len(raw) != FRAME_OVERHEAD + length:
        raise ValueError(f"LENGTH {length} does not match a {len(raw)}-byte frame")
    if crc8(raw[1:-2]) != raw[-2]:
        raise ValueError("CRC-8 mismatch")
    payload = raw[3:-2]
    if kind == MESSAGE_CONTROL and length == CONTROL_PAYLOAD.size:
        mode, speed, target = CONTROL_PAYLOAD.unpack(payload)
        if mode not in tuple(Mode) or speed not in tuple(Speed):
            raise ValueError("control mode or speed out of range")
        return Control(Mode(mode), Speed(speed), target)
    if kind == MESSAGE_TELEMETRY and length == TELEMETRY_PAYLOAD.size:
        return Telemetry(*TELEMETRY_PAYLOAD.unpack(payload))
    raise ValueError(f"unknown TYPE 0x{kind:02X} or wrong LENGTH {length}")


# Teensy main.cpp: interpCyclesForSpeed() at the 2 ms motor-control ISR.
TRAJECTORY_SECONDS = {Speed.SLOW: 60000 * 0.002, Speed.MEDIUM: 9000 * 0.002, Speed.HIGH: 1750 * 0.002}
# Assumed TargetReached rule until the firmware defines its own values.
TARGET_TOLERANCE_DEG = 2.0
TARGET_SETTLE_S = 0.2


class SimulatedJoint:
    """Behavioral model of Nano33BLEFirmware plus the proposed Teensy rules.

    A control frame that differs from the active command, or any frame after
    a timeout, starts a linear trajectory from the current angle (unless
    SafetyFault is latched). An identical frame only refreshes the 250 ms
    watchdog. At the end of the trajectory the joint holds the target and sets
    TargetReached after settling within TARGET_TOLERANCE_DEG for
    TARGET_SETTLE_S. 250 ms without a valid frame stops motion (motor disabled)
    and clears SafetyFault; TargetReached stays set until the next trajectory.
    The angle follows the setpoint exactly; no motor or current dynamics.
    """
    def __init__(self, angle_deg=0.0):
        self.angle = float(angle_deg)
        self.begin = self.angle
        self.command = None
        self.target = self.angle
        self.duration = 1.0
        self.started = 0.0
        self.motion = False
        self.timed_out = True
        self.invalid = False
        self.fault = False
        self.reached = False
        self.settle_since = None
        self.last_valid = None
        self.trajectories = 0
        self.fault_on_motion = False  # test hook: latch SafetyFault as soon as motion starts

    def receive(self, raw, now):
        """BLE write -> Nano validation -> I2C -> Teensy. Returns the frame or None."""
        self.tick(now)
        try:
            frame = decode(raw)
        except ValueError:
            return None  # the Nano discards it; nothing reaches the Teensy
        if not isinstance(frame, Control):
            return None
        restart = not self.fault and (self.timed_out or frame != self.command)
        self.invalid = False
        self.timed_out = False
        self.last_valid = now
        if restart:
            self.command = frame
            self.begin, self.target, self.started = self.angle, frame.target_deg, now
            self.duration = TRAJECTORY_SECONDS[frame.speed]
            self.motion, self.reached, self.settle_since = True, False, None
            self.trajectories += 1
        return frame

    def tick(self, now):
        expired = self.last_valid is None or (now - self.last_valid) * 1000 > COMMAND_TIMEOUT_MS
        if self.motion and self.fault_on_motion:
            self.fault, self.motion = True, False
        if self.motion:
            # Motion runs until the moment the command timeout fires.
            until = self.last_valid + COMMAND_TIMEOUT_MS / 1000 if expired else now
            fraction = min(1.0, (until - self.started) / self.duration)
            self.angle = self.begin + (self.target - self.begin) * fraction
            if abs(self.angle - self.target) <= TARGET_TOLERANCE_DEG:
                if self.settle_since is None:
                    self.settle_since = until
                if until - self.settle_since >= TARGET_SETTLE_S:
                    self.reached = True
            else:
                self.settle_since = None
        if expired:
            self.timed_out, self.motion, self.fault = True, False, False

    def telemetry(self, now):
        self.tick(now)
        status = ((Status.MOTION_ACTIVE if self.motion else 0)
                  | (Status.COMMAND_TIMEOUT if self.timed_out else 0)
                  | (Status.INVALID_COMMAND if self.invalid else 0)
                  | (Status.SAFETY_FAULT if self.fault else 0)
                  | (Status.TARGET_REACHED if self.reached else 0))
        milliamps = 800 if self.motion else 0  # placeholder, not a current model
        return Telemetry(int(round(self.angle)), milliamps, int(status)).encode()
