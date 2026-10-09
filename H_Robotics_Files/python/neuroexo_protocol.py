"""NeuroExo trial protocol v1: fixed 20-byte BLE values, no hardware imports.

Positions and velocity use milli-degrees and milli-degrees/second in this
proposed simulator protocol. These units are not inferred from the old firmware.
"""
from dataclasses import dataclass
from enum import IntEnum
import struct

SERVICE = "b7e20000-6a2b-4f10-9c31-8b674045a901"
COMMAND = "b7e20001-6a2b-4f10-9c31-8b674045a901"
EVENT = "b7e20002-6a2b-4f10-9c31-8b674045a901"
INFO = "b7e20003-6a2b-4f10-9c31-8b674045a901"
MAGIC, VERSION, SIMULATED = 0x4E, 1, 1
WIRE = struct.Struct("<BBBBHHiii")
WATCHDOG_MS = 2000
SCALE = 1000


class Kind(IntEnum):
    CALIBRATE = 1
    SET_MAX = 2
    CONFIGURE = 3
    GET_POSITION = 4
    START = 5
    END = 6
    ACK = 0x80
    POSITION = 0x81
    INFO = 0x82


class State(IntEnum):
    BOOT = 0
    IDLE = 1
    READY = 2
    RUNNING = 3
    ENDED = 4
    FAULT = 5


class Status(IntEnum):
    OK = 0
    BAD_STATE = 1
    RANGE = 2
    WRONG_TRIAL = 3
    BAD_SEQUENCE = 4
    UNSUPPORTED = 5


def label(enum, value):
    try:
        return enum(value).name
    except ValueError:
        return f"UNKNOWN({value})"


def signed32(value):
    value &= 0xFFFFFFFF
    return value if value < 0x80000000 else value - 0x100000000


@dataclass(frozen=True)
class Frame:
    kind: int
    seq: int = 0
    trial: int = 0
    a: int = 0
    b: int = 0
    c: int = 0
    flags: int = 0

    def encode(self):
        if self.flags not in (0, SIMULATED):
            raise ValueError("unknown frame flags")
        try:
            return WIRE.pack(MAGIC, VERSION, self.kind, self.flags,
                             self.seq, self.trial, self.a, self.b, self.c)
        except struct.error as error:
            raise ValueError(f"frame field out of range: {error}") from error

    @classmethod
    def decode(cls, raw):
        if len(raw) != WIRE.size:
            raise ValueError(f"expected 20 bytes; got {len(raw)}")
        magic, version, kind, flags, seq, trial, a, b, c = WIRE.unpack(raw)
        if magic != MAGIC or version != VERSION or flags not in (0, SIMULATED):
            raise ValueError("wrong magic, protocol version, or flags")
        return cls(kind, seq, trial, a, b, c, flags)

    def describe(self):
        prefix = f"{label(Kind, self.kind)} seq={self.seq} trial={self.trial}"
        if self.kind == Kind.CONFIGURE:
            detail = (f"target={self.a / SCALE:.3f} deg "
                      f"start={self.b / SCALE:.3f} deg "
                      f"assistance={self.c / SCALE:.3f} deg/s")
        elif self.kind == Kind.SET_MAX:
            detail = f"max_position={self.a / SCALE:.3f} deg"
        elif self.kind == Kind.ACK:
            detail = (f"command={label(Kind, self.a)} status={label(Status, self.b)} "
                      f"state={label(State, self.c)}")
        elif self.kind == Kind.POSITION:
            detail = (f"position={self.a / SCALE:.3f} deg "
                      f"device_ms={self.b & 0xFFFFFFFF} state={label(State, self.c)}")
        elif self.kind == Kind.INFO:
            detail = f"scale={self.a} watchdog_ms={self.b} state={label(State, self.c)}"
        else:
            detail = ""
        return f"{prefix} {detail}{' [SIMULATED]' if self.flags & SIMULATED else ''}".rstrip()


class SimulatedNano:
    """Behavioral counterpart of NanoNeuroExoProtocol/trial_protocol.h.

    CALIBRATE sets virtual zero. CONFIGURE instantly places the virtual arm at
    its starting position. No encoder, motor, EEG acquisition, or I2C is modeled.
    """
    def __init__(self):
        self.state = State.BOOT
        self.trial = 0
        self.maximum = 0
        self.target = 0
        self.velocity = 0
        self.position_scaled = 0  # milli-degrees * 1000; retain fractional steps
        self.last_tick = 0
        self.last_command = 0
        self.last_raw = None
        self.last_reply = None
        self.last_seq = 0

    @property
    def position(self):
        return self.position_scaled // SCALE

    def info(self):
        return Frame(Kind.INFO, a=SCALE, b=WATCHDOG_MS,
                     c=self.state, flags=SIMULATED).encode()

    def tick(self, now):
        now &= 0xFFFFFFFF
        if self.state == State.RUNNING:
            if ((now - self.last_command) & 0xFFFFFFFF) > WATCHDOG_MS:
                self.state = State.FAULT
            else:
                step = self.velocity * ((now - self.last_tick) & 0xFFFFFFFF)
                goal = self.target * SCALE
                if self.position_scaled < goal:
                    self.position_scaled = min(goal, self.position_scaled + step)
                else:
                    self.position_scaled = max(goal, self.position_scaled - step)
        self.last_tick = now

    def process(self, raw, now):
        frame = Frame.decode(raw)
        if frame.flags or not frame.seq:
            raise ValueError("commands require flags=0 and a nonzero sequence")
        self.tick(now)
        if frame.seq == self.last_seq:
            if raw == self.last_raw:
                return self.last_reply
            return self._ack(frame, Status.BAD_SEQUENCE)
        expected = 1 if self.last_seq in (0, 65535) else self.last_seq + 1
        if frame.seq != expected:
            return self._ack(frame, Status.BAD_SEQUENCE)

        status = Status.OK
        global_command = frame.kind in (Kind.CALIBRATE, Kind.SET_MAX)
        if global_command and frame.trial:
            status = Status.WRONG_TRIAL
        elif frame.kind == Kind.CALIBRATE:
            if frame.a or frame.b or frame.c:
                status = Status.RANGE
            elif self.state == State.RUNNING:
                status = Status.BAD_STATE
            else:
                self.state, self.trial, self.maximum = State.IDLE, 0, 0
                self.position_scaled = 0
        elif frame.kind == Kind.SET_MAX:
            if self.state not in (State.IDLE, State.ENDED):
                status = Status.BAD_STATE
            elif not 0 < frame.a <= 360000 or frame.b or frame.c:
                status = Status.RANGE
            else:
                self.maximum, self.trial, self.state = frame.a, 0, State.IDLE
        elif frame.kind == Kind.CONFIGURE:
            if self.state not in (State.IDLE, State.ENDED) or not self.maximum:
                status = Status.BAD_STATE
            elif not frame.trial:
                status = Status.WRONG_TRIAL
            elif not (0 <= frame.a <= self.maximum and 0 <= frame.b <= self.maximum
                      and 0 < frame.c <= 360000):
                status = Status.RANGE
            else:
                self.target, self.velocity = frame.a, frame.c
                self.position_scaled = frame.b * SCALE
                self.trial, self.state = frame.trial, State.READY
        elif frame.kind in (Kind.GET_POSITION, Kind.START, Kind.END):
            if frame.trial != self.trial:
                status = Status.WRONG_TRIAL
            elif frame.a or frame.b or frame.c:
                status = Status.RANGE
            elif frame.kind == Kind.START:
                if self.state != State.READY:
                    status = Status.BAD_STATE
                else:
                    self.state = State.RUNNING
            elif frame.kind == Kind.END:
                if self.state not in (State.READY, State.RUNNING, State.ENDED, State.FAULT):
                    status = Status.BAD_STATE
                elif self.state != State.FAULT:
                    self.state = State.ENDED
        else:
            status = Status.UNSUPPORTED

        if status == Status.OK:
            self.last_command = now & 0xFFFFFFFF
        if status == Status.OK and frame.kind == Kind.GET_POSITION:
            reply = Frame(Kind.POSITION, frame.seq, frame.trial, self.position,
                          signed32(now), self.state, SIMULATED).encode()
        else:
            reply = self._ack(frame, status)
        self.last_seq, self.last_raw, self.last_reply = frame.seq, bytes(raw), reply
        return reply

    def _ack(self, frame, status):
        return Frame(Kind.ACK, frame.seq, frame.trial, frame.kind,
                     status, self.state, SIMULATED).encode()
