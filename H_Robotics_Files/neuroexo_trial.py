"""Readable trial-protocol demo and BLE client. See NEUROEXO_PROTOCOL.md."""
import argparse
import asyncio
import json
import math
import sys
import time
from decimal import Decimal, InvalidOperation

from neuroexo_protocol import (COMMAND, EVENT, INFO, SERVICE, SIMULATED, SCALE,
                               WATCHDOG_MS, Frame, Kind, SimulatedNano, State, Status)


def positive(value):
    result = float(value)
    if not math.isfinite(result) or result <= 0:
        raise argparse.ArgumentTypeError("must be finite and greater than zero")
    return result


def milli(value):
    try:
        number = Decimal(value) * SCALE
        if not number.is_finite() or number != number.to_integral_value():
            raise ValueError()
        result = int(number)
        if not 0 <= result <= 360000:
            raise ValueError()
        return result
    except (InvalidOperation, ValueError, OverflowError):
        raise argparse.ArgumentTypeError("use 0..360 with at most three decimal places")


def add_modes(modes):
    scan = modes.add_parser("protocol-scan", help="find NanoNeuroExo trial-protocol receivers")
    scan.add_argument("--timeout", type=positive, default=10.0)
    decode = modes.add_parser("protocol-decode", help="decode one 20-byte trial-protocol value")
    decode.add_argument("hex", help="hex bytes, quoted if they contain spaces")
    for mode in ("protocol-demo", "protocol-run"):
        parser = modes.add_parser(mode, help=("simulate the diagram without hardware" if mode.endswith("demo")
                                             else "run the diagram over BLE with the simulated Nano sketch"))
        if mode == "protocol-run":
            parser.add_argument("--address", required=True)
        parser.add_argument("--trials", type=int, default=1, help="trial count; use 20 for the diagram")
        parser.add_argument("--duration", type=positive, default=2.0, help="running seconds per trial")
        parser.add_argument("--interval-ms", type=int, default=40, help="position request period, 20..1000 ms")
        parser.add_argument("--start", type=milli, default=0, help="simulated starting position in degrees")
        parser.add_argument("--target", type=milli, default=60000, help="simulated target in degrees")
        parser.add_argument("--assistance", type=milli, default=30000, help="simulated velocity in degrees/second")
        parser.add_argument("--max-position", type=milli, default=90000, help="simulated upper limit in degrees")
        parser.add_argument("--baseline-seconds", type=float, default=0.0,
                            help="local pause for the dot/blink stage; does not collect EEG")
        parser.add_argument("--timeout", type=positive, default=1.0, help="per-command write/reply timeout")
        parser.add_argument("--hex", action="store_true", help="also display the exact BLE value bytes")
        parser.add_argument("--log", help="new JSONL file for decoded TX/RX and exact bytes")


def validate(args):
    if not 1 <= args.trials <= 1000:
        raise ValueError("--trials must be 1..1000")
    if not 20 <= args.interval_ms <= 1000:
        raise ValueError("--interval-ms must be 20..1000")
    if not 0 < args.assistance <= 360000:
        raise ValueError("--assistance must be greater than zero")
    if not 0 < args.max_position <= 360000 or max(args.start, args.target) > args.max_position:
        raise ValueError("start and target must fit within the positive maximum position")
    if not math.isfinite(args.baseline_seconds) or not 0 <= args.baseline_seconds <= 3600:
        raise ValueError("--baseline-seconds must be 0..3600")
    if args.duration > 3600:
        raise ValueError("--duration must be at most 3600 seconds")


class Trace:
    def __init__(self, clock, show_hex=False, stream=None, simulated=False):
        self.clock, self.show_hex, self.stream = clock, show_hex, stream
        self.simulated = simulated

    def frame(self, direction, raw):
        frame = Frame.decode(raw)
        stamp = self.clock.now()
        print(f"{stamp:9.3f}s {direction:4s} {frame.describe()}"
              + (f" | {bytes(raw).hex(' ')}" if self.show_hex else ""), flush=True)
        if self.stream:
            record = dict(time_s=stamp, clock="virtual" if self.simulated else "host_monotonic",
                          direction=direction, kind=frame.kind, sequence=frame.seq,
                          trial=frame.trial, a=frame.a, b=frame.b, c=frame.c,
                          flags=frame.flags, hex=bytes(raw).hex(), decoded=frame.describe())
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
    def __init__(self):
        self.nano = SimulatedNano()
        self.seconds = 0.0

    def now(self):
        return self.seconds

    async def sleep(self, seconds):
        self.seconds += max(0, seconds)
        self.nano.tick(round(self.seconds * 1000))

    async def info(self):
        return self.nano.info()

    async def exchange(self, raw, timeout):
        return self.nano.process(raw, round(self.seconds * 1000))


class BleLink:
    def __init__(self, client):
        self.client = client
        self.queue = asyncio.Queue(maxsize=32)
        self.failure = None

    def notification(self, characteristic, raw):
        try:
            self.queue.put_nowait(bytes(raw))
        except asyncio.QueueFull:
            self.failure = "notification queue overflow"

    def disconnected(self, client):
        self.failure = "Nano disconnected"
        try:
            self.queue.put_nowait(None)
        except asyncio.QueueFull:
            pass

    async def info(self):
        return await asyncio.wait_for(self.client.read_gatt_char(INFO), 10.0)

    async def exchange(self, raw, timeout):
        frame = Frame.decode(raw)
        deadline = asyncio.get_running_loop().time() + timeout
        if self.failure:
            raise ConnectionError(self.failure)
        await asyncio.wait_for(self.client.write_gatt_char(COMMAND, raw, response=True), timeout)
        while True:
            if self.failure:
                raise ConnectionError(self.failure)
            remaining = deadline - asyncio.get_running_loop().time()
            if remaining <= 0:
                raise TimeoutError("no matching application reply before command deadline")
            value = await asyncio.wait_for(self.queue.get(), remaining)
            if value is None:
                raise ConnectionError("Nano disconnected")
            reply = Frame.decode(value)
            if reply.seq == frame.seq and reply.trial == frame.trial:
                return value
            # No automatic retries; an unexpected reply indicates a protocol mismatch.
            raise ValueError(f"unexpected response sequence/trial: {reply.describe()}")


class Session:
    def __init__(self, link, clock, trace, timeout):
        self.link, self.clock, self.trace, self.timeout = link, clock, trace, timeout
        self.sequence = 0
        self.requests = self.responses = self.skipped = self.polls = 0
        self.max_rtt = 0.0
        self.max_poll_gap = 0.0
        self.last_poll = None

    async def command(self, kind, trial=0, a=0, b=0, c=0):
        self.sequence = self.sequence % 65535 + 1
        frame = Frame(kind, self.sequence, trial, a, b, c)
        raw = frame.encode()
        self.trace.frame("TX", raw)
        start = self.clock.now()
        self.requests += 1
        reply = self.trace.frame("RX", await self.link.exchange(raw, self.timeout))
        self.responses += 1
        self.max_rtt = max(self.max_rtt, self.clock.now() - start)
        if reply.seq != frame.seq or reply.trial != frame.trial or reply.flags != SIMULATED:
            raise ValueError("response identity or simulator flag mismatch")
        expected = Kind.POSITION if kind == Kind.GET_POSITION else Kind.ACK
        if reply.kind == Kind.ACK and reply.b != Status.OK:
            raise RuntimeError(f"Nano rejected command: {reply.describe()}")
        if reply.kind != expected or (expected == Kind.ACK and reply.a != kind):
            raise ValueError("wrong response type or acknowledged command")
        if reply.c not in tuple(State):
            raise ValueError("unknown receiver state")
        if reply.c == State.FAULT:
            raise RuntimeError("Nano simulator watchdog fault; trial stopped")
        return reply

    async def run(self, args):
        info = self.trace.frame("INFO", await self.link.info())
        if (info.kind != Kind.INFO or info.flags != SIMULATED or info.a != SCALE
                or info.b != WATCHDOG_MS or info.c != State.BOOT):
            raise ValueError("requires a freshly connected NanoNeuroExo simulator v1")
        print("Protocol prototype: positions are SIMULATED; no arm or EEG hardware is driven.")
        if isinstance(self.clock, DemoLink):
            print("Virtual time demo: this does not measure Bluetooth throughput or timing.")
        await self.command(Kind.CALIBRATE)
        await self.command(Kind.SET_MAX, a=args.max_position)
        print(f"LOCAL dot/blink stage: {args.baseline_seconds:g}s pause; no EEG acquisition.")
        await self.clock.sleep(args.baseline_seconds)
        active = 0
        try:
            for trial in range(1, args.trials + 1):
                active = trial
                await self.command(Kind.CONFIGURE, trial, args.target, args.start, args.assistance)
                position = await self.command(Kind.GET_POSITION, trial)
                if position.c != State.READY or position.a != args.start:
                    raise RuntimeError("simulated arm is not ready at the starting position")
                await self.command(Kind.START, trial)
                self.last_poll = None
                period = args.interval_ms / 1000.0
                started = self.clock.now()
                end = started + args.duration
                due = started + period
                while due <= end + 1e-9:
                    await self.clock.sleep(due - self.clock.now())
                    sent_at = self.clock.now()
                    if self.last_poll is not None:
                        self.max_poll_gap = max(self.max_poll_gap, sent_at - self.last_poll)
                    self.last_poll = sent_at
                    self.polls += 1
                    position = await self.command(Kind.GET_POSITION, trial)
                    if position.c != State.RUNNING:
                        raise RuntimeError("receiver left RUNNING during the trial")
                    due += period
                    now = self.clock.now()
                    if due < now:
                        skipped = int((now - due) // period) + 1
                        self.skipped += skipped
                        due += skipped * period  # never send a burst to catch up
                await self.clock.sleep(end - self.clock.now())
                await self.command(Kind.END, trial)
                active = 0
        finally:
            if active:
                try:
                    await self.command(Kind.END, active)
                except BaseException as error:
                    print(f"Cleanup END could not be confirmed: {type(error).__name__}: {error}",
                          file=sys.stderr)
        print(f"Completed: commands={self.requests} replies={self.responses} "
              f"position_polls={self.polls} skipped_poll_deadlines={self.skipped}")
        if not isinstance(self.clock, DemoLink):
            print(f"Observed host round-trip max={self.max_rtt * 1000:.2f} ms; "
                  f"position-request gap max={self.max_poll_gap * 1000:.2f} ms "
                  "(not a sensor sampling or one-way latency measurement)")
        return 2 if self.skipped else 0


async def scan(args):
    try:
        from bleak import BleakScanner
    except ImportError as error:
        raise RuntimeError("install requirements-ble.txt in the active Python environment") from error
    devices = await BleakScanner.discover(timeout=args.timeout, return_adv=True)
    count = 0
    for device, advertisement in devices.values():
        if SERVICE in [uuid.lower() for uuid in advertisement.service_uuids]:
            print(f"{device.address}  {advertisement.local_name or device.name}")
            count += 1
    if not count:
        print("No NanoNeuroExo found. Load NanoNeuroExoProtocol and disconnect other BLE clients.")
        return 1
    return 0


async def execute(args, stream=None):
    if args.mode == "protocol-demo":
        link = DemoLink()
        trace = Trace(link, args.hex, stream, simulated=True)
        return await Session(link, link, trace, args.timeout).run(args)
    try:
        from bleak import BleakClient
    except ImportError as error:
        raise RuntimeError("install requirements-ble.txt in the active Python environment") from error
    clock = RealClock()
    link = BleLink(None)
    # Set the public callback via the constructor, including unexpected disconnects.
    client = BleakClient(args.address, timeout=15.0, disconnected_callback=link.disconnected)
    link.client = client
    async with client:
        for uuid, prop in ((COMMAND, "write"), (EVENT, "notify"), (INFO, "read")):
            characteristic = client.services.get_characteristic(uuid)
            if characteristic is None or prop not in characteristic.properties:
                raise RuntimeError("wrong Nano firmware: load NanoNeuroExoProtocol")
        await asyncio.wait_for(client.start_notify(EVENT, link.notification), 10.0)
        trace = Trace(clock, args.hex, stream)
        return await Session(link, clock, trace, args.timeout).run(args)


def run(args):
    stream = None
    try:
        if args.mode == "protocol-decode":
            print(Frame.decode(bytes.fromhex(args.hex)).describe())
            return 0
        if args.mode == "protocol-scan":
            return asyncio.run(scan(args))
        validate(args)
        if args.log:
            stream = open(args.log, "x", encoding="utf-8")
        return asyncio.run(execute(args, stream))
    except KeyboardInterrupt:
        print("\nProtocol run interrupted.", file=sys.stderr)
        return 130
    except Exception as error:
        print(f"Protocol error: {type(error).__name__}: {error}", file=sys.stderr)
        return 1
    finally:
        if stream:
            stream.close()
