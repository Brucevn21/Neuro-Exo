"""NeuroExo joint protocol CLI (service 180C). See docs/NEUROEXO_PROTOCOL.md."""
import argparse
import asyncio
import json
import math
import sys
import time

from neuroexo_protocol import (COMMAND, COMMAND_TIMEOUT_MS, DEVICE_NAME, SERVICE, TELEMETRY,
                               TELEMETRY_INTERVAL_MS, Control, Mode, SimulatedJoint, Speed,
                               Status, Telemetry, decode)

TELEMETRY_STALE_S = 0.5
STOP_CONFIRM_S = 1.0
FIRST_TELEMETRY_S = 1.0


def positive(value):
    result = float(value)
    if not math.isfinite(result) or result <= 0:
        raise argparse.ArgumentTypeError("must be finite and greater than zero")
    return result


def angle(value):
    try:
        result = int(value, 10)
    except ValueError:
        raise argparse.ArgumentTypeError("use a whole number of degrees")
    if not -32768 <= result <= 32767:
        raise argparse.ArgumentTypeError("must fit a signed 16-bit value")
    return result


def named(enum):
    def parse(value):
        try:
            return enum[value.upper()]
        except KeyError:
            raise argparse.ArgumentTypeError(
                "use " + ", ".join(member.name.lower() for member in enum))
    return parse


def add_command_fields(parser):
    # dest avoids args.mode, which holds the subcommand name
    parser.add_argument("--mode", dest="joint_mode", type=named(Mode), default=Mode.ASSISTIVE,
                        help="resistive, assistive, or neutral")
    parser.add_argument("--speed", type=named(Speed), default=Speed.MEDIUM,
                        help="slow, medium, or high")
    parser.add_argument("--target", type=angle, default=45, help="target angle in whole degrees")


def add_modes(modes):
    scan = modes.add_parser("protocol-scan", help=f"find {DEVICE_NAME} (service 180C)")
    scan.add_argument("--timeout", type=positive, default=10.0)
    decoder = modes.add_parser("protocol-decode", help="decode one control or telemetry frame")
    decoder.add_argument("hex", help="hex bytes, quoted if they contain spaces")
    encoder = modes.add_parser("protocol-encode", help="print the 9-byte control frame for a command")
    add_command_fields(encoder)
    monitor = modes.add_parser("protocol-monitor", help="print telemetry only; never commands motion")
    monitor.add_argument("--address", required=True)
    monitor.add_argument("--duration", type=positive, default=5.0, help="seconds to listen")
    for mode in ("protocol-demo", "protocol-run"):
        parser = modes.add_parser(mode, help=(
            "simulate trials in virtual time without hardware" if mode.endswith("demo")
            else "run trials over BLE; COMMANDS REAL MOTOR MOTION"))
        if mode == "protocol-run":
            parser.add_argument("--address", required=True)
            parser.add_argument("--write-without-response", action="store_true",
                                help="skip the ATT write response (2A56 supports both)")
        add_command_fields(parser)
        parser.add_argument("--trials", type=int, default=1)
        parser.add_argument("--time-limit", type=positive, default=150.0,
                            help="seconds to wait for TargetReached before stopping (Slow takes ~120 s)")
        parser.add_argument("--interval-ms", type=int, default=100,
                            help="keepalive period, 20..150 ms (Teensy timeout is 250 ms)")
        parser.add_argument("--rest", type=float, default=1.0,
                            help="silent seconds between trials, >= 0.3")
    for parser in [monitor] + [modes.choices[m] for m in ("protocol-demo", "protocol-run")]:
        parser.add_argument("--hex", action="store_true", help="also display the exact frame bytes")
        parser.add_argument("--print-every", type=int, default=5,
                            help="print every Nth TX/RX frame plus every status change (log keeps all)")
        parser.add_argument("--log", help="new JSONL file for decoded TX/RX and exact bytes")


def validate(args):
    if args.print_every < 1:
        raise ValueError("--print-every must be at least 1")
    if args.mode == "protocol-monitor":
        if args.duration > 3600:
            raise ValueError("--duration must be at most 3600 seconds")
        return
    if not 1 <= args.trials <= 1000:
        raise ValueError("--trials must be 1..1000")
    if not 20 <= args.interval_ms <= 150:
        raise ValueError("--interval-ms must be 20..150 to stay inside the 250 ms command timeout")
    if args.time_limit > 3600:
        raise ValueError("--time-limit must be at most 3600 seconds")
    if not math.isfinite(args.rest) or not 0.3 <= args.rest <= 3600:
        raise ValueError("--rest must be 0.3..3600 seconds")


class Trace:
    def __init__(self, show_hex=False, stream=None, simulated=False, every=1):
        self.show_hex, self.stream, self.simulated, self.every = show_hex, stream, simulated, every
        self.counts = {"TX": 0, "RX": 0}
        self.last_status = None

    def frame(self, direction, raw, stamp):
        frame = decode(raw)
        count = self.counts[direction]
        self.counts[direction] += 1
        changed = isinstance(frame, Telemetry) and frame.status != self.last_status
        if changed:
            self.last_status = frame.status
        if changed or count % self.every == 0:
            print(f"{stamp:9.3f}s {direction:2s} {frame.describe()}"
                  + (f" | {bytes(raw).hex(' ')}" if self.show_hex else ""), flush=True)
        if self.stream:
            record = dict(time_s=stamp, clock="virtual" if self.simulated else "host_monotonic",
                          direction=direction, hex=bytes(raw).hex(), decoded=frame.describe())
            if isinstance(frame, Control):
                record.update(mode=int(frame.mode), speed=int(frame.speed), target_deg=frame.target_deg)
            else:
                record.update(angle_deg=frame.angle_deg, milliamps=frame.milliamps, status=frame.status)
            self.stream.write(json.dumps(record) + "\n")
            self.stream.flush()
        return frame


class RealClock:
    def __init__(self):
        self.origin = time.monotonic()

    def now(self):
        return time.monotonic() - self.origin

    async def sleep(self, seconds):
        await asyncio.sleep(max(0, seconds))


class DemoLink:
    """Virtual time: SimulatedJoint publishes telemetry every 20 ms of virtual time."""
    failure = None

    def __init__(self, joint=None):
        self.joint = joint or SimulatedJoint()
        self.seconds = 0.0
        self.next_telemetry = TELEMETRY_INTERVAL_MS / 1000
        self.pending = []

    def now(self):
        return self.seconds

    async def sleep(self, seconds):
        end = self.seconds + max(0, seconds)
        while self.next_telemetry <= end + 1e-12:
            self.seconds = self.next_telemetry
            self.pending.append((self.seconds, self.joint.telemetry(self.seconds)))
            self.next_telemetry += TELEMETRY_INTERVAL_MS / 1000
        self.seconds = end

    async def write(self, raw):
        self.joint.receive(raw, self.seconds)

    def take(self):
        frames, self.pending = self.pending, []
        return frames


class BleLink:
    def __init__(self, clock, response, timeout=1.0):
        self.client, self.clock, self.response, self.timeout = None, clock, response, timeout
        self.pending = []
        self.failure = None

    def notification(self, characteristic, raw):
        if len(self.pending) >= 1024:
            self.pending.pop(0)
        self.pending.append((self.clock.now(), bytes(raw)))

    def disconnected(self, client):
        self.failure = "Nano disconnected"

    async def write(self, raw):
        if self.failure:
            raise ConnectionError(self.failure)
        await asyncio.wait_for(self.client.write_gatt_char(COMMAND, raw, response=self.response),
                               self.timeout)

    def take(self):
        frames, self.pending = self.pending, []
        return frames


class Session:
    def __init__(self, link, clock, trace):
        self.link, self.clock, self.trace = link, clock, trace
        self.commands = self.telemetry = self.rejected = self.skipped = 0
        self.max_gap = 0.0
        self.last_telemetry = None

    def drain(self, stopping=False):
        frames = []
        for stamp, raw in self.link.take():
            try:
                frame = decode(raw)
                if not isinstance(frame, Telemetry):
                    raise ValueError("control frame on the telemetry characteristic")
            except ValueError as error:
                self.rejected += 1
                print(f"{stamp:9.3f}s RX rejected ({error}) | {raw.hex(' ')}", flush=True)
                continue
            self.trace.frame("RX", raw, stamp)
            if self.last_telemetry is not None:
                self.max_gap = max(self.max_gap, stamp - self.last_telemetry)
            self.last_telemetry = stamp
            self.telemetry += 1
            frames.append((stamp, frame))
            if not stopping and frame.has(Status.SAFETY_FAULT):
                raise RuntimeError("Teensy safety fault (e-stop latched); commands stopped")
        if self.link.failure:
            raise ConnectionError(self.link.failure)
        return frames

    async def wait_for_telemetry(self, timeout):
        deadline = self.clock.now() + timeout
        while self.clock.now() < deadline:
            await self.clock.sleep(0.01)
            frames = self.drain(stopping=True)
            if frames:
                return frames[-1][1]
        raise TimeoutError("no telemetry from the Nano; check the Teensy I2C link and power")

    async def trial(self, control):
        """Start signal, keepalives until TargetReached, then silence and idle."""
        raw = control.encode()
        initial = await self.wait_for_telemetry(FIRST_TELEMETRY_S)
        if initial.has(Status.SAFETY_FAULT):
            raise RuntimeError("Teensy safety fault is latched; it clears after 250 ms without commands")
        period = self.args.interval_ms / 1000
        started = self.clock.now()
        deadline = started + self.args.time_limit
        due = started
        last_send = started
        sent = reached = False
        timeout_window = (COMMAND_TIMEOUT_MS + 150) / 1000
        # Writes happen only in this loop, so any exception also ends the
        # keepalives and the Teensy disables motion within 250 ms.
        while True:
            await self.clock.sleep(due - self.clock.now())
            for stamp, frame in self.drain():
                # A TargetReached left over from an earlier move arrives with
                # CommandTimeout set; only a frame from this active command counts.
                if (sent and frame.has(Status.TARGET_REACHED) and frame.has(Status.MOTION_ACTIVE)
                        and not frame.has(Status.COMMAND_TIMEOUT)):
                    reached = True
                elif stamp - started > timeout_window and frame.has(Status.COMMAND_TIMEOUT):
                    raise RuntimeError("Teensy reports CommandTimeout while commands are being sent")
            if reached or self.clock.now() >= deadline:
                break
            if sent and self.clock.now() - self.last_telemetry > TELEMETRY_STALE_S:
                raise RuntimeError("telemetry stopped during the trial; commands stopped")
            last_send = self.clock.now()
            self.trace.frame("TX", raw, last_send)
            await self.link.write(raw)
            self.commands += 1
            sent = True
            due += period
            now = self.clock.now()
            if due < now:
                skipped = int((now - due) // period) + 1
                self.skipped += skipped
                due += skipped * period  # never send a burst to catch up
        elapsed = self.clock.now() - started
        if reached:
            print(f"{self.clock.now():9.3f}s    TargetReached after {elapsed:.2f} s; keepalives stopped")
        confirm = self.clock.now() + STOP_CONFIRM_S
        confirmed = False
        while self.clock.now() < confirm and not confirmed:
            await self.clock.sleep(0.01)
            for stamp, frame in self.drain(stopping=True):
                if (stamp > last_send + COMMAND_TIMEOUT_MS / 1000 and frame.has(Status.COMMAND_TIMEOUT)
                        and not frame.has(Status.MOTION_ACTIVE)):
                    confirmed = True
            if self.clock.now() - self.last_telemetry > TELEMETRY_STALE_S:
                break
        if not confirmed:
            raise RuntimeError("stop not confirmed: no CommandTimeout telemetry after keepalives ceased")
        print(f"{self.clock.now():9.3f}s    idle confirmed: CommandTimeout, motor disabled")
        if not reached:
            raise RuntimeError(f"target not reached within {self.args.time_limit:g} s; commands stopped")

    async def run(self, args):
        self.args = args
        control = Control(args.joint_mode, args.speed, args.target)
        if isinstance(self.link, DemoLink):
            print("Virtual-time demo: SimulatedJoint stands in for the Nano and Teensy "
                  "(proposed TargetReached firmware); no Bluetooth or motor timing is measured.")
        else:
            print("Commanding REAL motion through the Nano and Teensy. Ctrl+C stops the keepalives.")
        for trial in range(1, args.trials + 1):
            if trial > 1:
                await self.clock.sleep(args.rest)  # silence also clears a latched SafetyFault
                self.drain(stopping=True)
            print(f"Trial {trial}: {control.describe()}, keepalive every {args.interval_ms} ms")
            await self.trial(control)
        self.summary()
        return 2 if self.skipped else 0

    async def monitor(self, duration):
        print("Monitoring telemetry only; no commands are written.")
        await self.wait_for_telemetry(FIRST_TELEMETRY_S)
        end = self.clock.now() + duration
        while self.clock.now() < end:
            await self.clock.sleep(TELEMETRY_INTERVAL_MS / 1000)
            self.drain(stopping=True)
        self.summary()
        return 0

    def summary(self):
        print(f"Completed: commands={self.commands} telemetry={self.telemetry} "
              f"rejected={self.rejected} skipped_keepalives={self.skipped} "
              f"max_telemetry_gap={self.max_gap * 1000:.1f} ms")


async def scan(args):
    try:
        from bleak import BleakScanner
    except ImportError as error:
        raise RuntimeError("install python/requirements-ble.txt in the active Python environment") from error
    devices = await BleakScanner.discover(timeout=args.timeout, return_adv=True)
    count = 0
    for device, advertisement in devices.values():
        name = advertisement.local_name or device.name
        if SERVICE in [uuid.lower() for uuid in advertisement.service_uuids] or name == DEVICE_NAME:
            print(f"{device.address}  {name}  RSSI {advertisement.rssi}")
            count += 1
    if not count:
        print(f"No {DEVICE_NAME} found. Check power and Nano33BLEFirmware, and disconnect other BLE clients.")
        return 1
    return 0


async def execute(args, stream=None):
    if args.mode == "protocol-demo":
        link = DemoLink()
        trace = Trace(args.hex, stream, simulated=True, every=args.print_every)
        return await Session(link, link, trace).run(args)
    try:
        from bleak import BleakClient
    except ImportError as error:
        raise RuntimeError("install python/requirements-ble.txt in the active Python environment") from error
    clock = RealClock()
    link = BleLink(clock, response=not getattr(args, "write_without_response", False))
    client = BleakClient(args.address, timeout=15.0, disconnected_callback=link.disconnected)
    link.client = client
    async with client:
        for uuid, needed in ((COMMAND, ("write", "write-without-response")), (TELEMETRY, ("notify",))):
            characteristic = client.services.get_characteristic(uuid)
            if characteristic is None or not set(needed) <= set(characteristic.properties):
                raise RuntimeError("Nano GATT layout mismatch: load Nano33BLEFirmware (service 180C)")
        await asyncio.wait_for(client.start_notify(TELEMETRY, link.notification), 10.0)
        session = Session(link, clock, Trace(args.hex, stream, every=args.print_every))
        if args.mode == "protocol-monitor":
            return await session.monitor(args.duration)
        return await session.run(args)


def run(args):
    stream = None
    try:
        if args.mode == "protocol-decode":
            print(decode(bytes.fromhex(args.hex)).describe())
            return 0
        if args.mode == "protocol-encode":
            print(Control(args.joint_mode, args.speed, args.target).encode().hex(" "))
            return 0
        if args.mode == "protocol-scan":
            return asyncio.run(scan(args))
        validate(args)
        if args.log:
            stream = open(args.log, "x", encoding="utf-8")
        return asyncio.run(execute(args, stream))
    except KeyboardInterrupt:
        print("\nProtocol run interrupted; keepalives stopped.", file=sys.stderr)
        return 130
    except Exception as error:
        print(f"Protocol error: {type(error).__name__}: {error}", file=sys.stderr)
        return 1
    finally:
        if stream:
            stream.close()
