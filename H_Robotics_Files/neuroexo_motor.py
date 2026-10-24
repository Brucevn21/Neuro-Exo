"""Independent command TX and live motor telemetry RX over one BLE connection."""
import asyncio
import json
import math
import sys
import time
from dataclasses import asdict
from neuroexo_protocol import SERVICE, COMMAND, TELEMETRY, Control, Telemetry, Mode, Speed, Status, decode


def add_modes(modes):
    scan = modes.add_parser("ble-scan", help="find NanoNeuroExo bridges")
    scan.add_argument("--timeout", type=float, default=5)
    decoder = modes.add_parser("protocol-decode", help="decode a control or telemetry frame")
    decoder.add_argument("hex", help="quoted, space-separated hex bytes")
    for name in ("ble-monitor", "ble-control"):
        parser = modes.add_parser(name, help="receive motor telemetry" if name == "ble-monitor"
                                  else "send periodic controls and receive motor telemetry")
        parser.add_argument("--address", required=True)
        parser.add_argument("--duration", type=float, default=0 if name == "ble-monitor" else None,
                            required=name == "ble-control", help="seconds; monitor 0 means until Ctrl+C")
        parser.add_argument("--telemetry-timeout", type=float, default=0.5)
        parser.add_argument("--hex", action="store_true")
        parser.add_argument("--log", help="new JSONL file for TX/RX records")
        if name == "ble-control":
            parser.add_argument("--mode", dest="motor_mode", choices=[m.name.lower() for m in Mode], required=True)
            parser.add_argument("--speed", choices=[s.name.lower() for s in Speed], required=True)
            parser.add_argument("--angle", type=int, required=True, help="signed integer degrees")
            parser.add_argument("--period-ms", type=int, default=100, help="command interval, 20..200 ms")


class BleLink:
    def __init__(self):
        self.client = None
        self.queue = asyncio.Queue(maxsize=1)
        self.latest = None
        self.failure = None
        self.invalid_frames = 0

    def notification(self, characteristic, raw):
        try:
            telemetry = Telemetry.decode(raw)
        except ValueError:
            self.invalid_frames += 1
            return  # Corruption never refreshes feedback freshness.
        update = (time.monotonic(), telemetry, bytes(raw))
        self.latest = update
        if self.queue.full():
            self.queue.get_nowait()
        self.queue.put_nowait(update)

    def disconnected(self, client):
        self.failure = "Nano disconnected"
        self.latest = None
        if self.queue.full():
            self.queue.get_nowait()
        self.queue.put_nowait(None)

    async def receive(self, timeout):
        if self.failure:
            raise ConnectionError(self.failure)
        try:
            update = await asyncio.wait_for(self.queue.get(), timeout)
        except asyncio.TimeoutError as error:
            raise TimeoutError("Teensy telemetry missing or stale") from error
        if update is None or self.failure:
            raise ConnectionError(self.failure or "Nano disconnected")
        if time.monotonic() - update[0] > timeout:
            raise TimeoutError("Teensy telemetry is stale")
        return update

    async def send(self, command):
        if self.failure:
            raise ConnectionError(self.failure)
        # This completion is an ATT response from Nano, not a Teensy acknowledgment.
        await asyncio.wait_for(self.client.write_gatt_char(COMMAND, command.encode(), response=True), 0.1)


class Trace:
    def __init__(self, show_hex=False, stream=None):
        self.origin = time.monotonic()
        self.show_hex, self.stream = show_hex, stream

    def frame(self, direction, frame, raw, received_at=None):
        stamp = (time.monotonic() if received_at is None else received_at) - self.origin
        print(f"{stamp:9.3f}s {direction} {frame.describe()}"
              + (f" | {raw.hex(' ')}" if self.show_hex else ""), flush=True)
        if self.stream:
            record = dict(time_s=stamp, clock="host_monotonic", direction=direction,
                          **asdict(frame), hex=raw.hex())
            self.stream.write(json.dumps(record) + "\n")
            self.stream.flush()


async def monitor(link, trace, timeout, controlling=False):
    while True:
        stamp, telemetry, raw = await link.receive(timeout)
        trace.frame("RX", telemetry, raw, stamp)
        if controlling and telemetry.status & (Status.SafetyFault | Status.InvalidCommand):
            raise RuntimeError("Teensy reported SafetyFault or InvalidCommand; control transmission stopped")


async def transmit(link, trace, command, period, freshness):
    due = asyncio.get_running_loop().time()
    while True:
        if link.latest is None or time.monotonic() - link.latest[0] > freshness:
            raise TimeoutError("Teensy telemetry missing or stale; control transmission stopped")
        if link.latest[1].status & (Status.SafetyFault | Status.InvalidCommand):
            raise RuntimeError("Teensy fault; control transmission stopped")
        await link.send(command)
        trace.frame("TX", command, command.encode())
        due += period
        now = asyncio.get_running_loop().time()
        if due <= now:
            due = now + period  # Never send a catch-up burst.
        await asyncio.sleep(due - now)


async def run_connected(link, args, trace):
    # Subscribe first and require real telemetry before transmitting any motion.
    first = await link.receive(args.telemetry_timeout)
    trace.frame("RX", first[1], first[2], first[0])
    controlling = args.mode == "ble-control"
    if controlling and first[1].status & (Status.SafetyFault | Status.InvalidCommand):
        raise RuntimeError("Teensy fault; no control sent")
    tasks = [asyncio.create_task(monitor(link, trace, args.telemetry_timeout, controlling))]
    if controlling:
        command = Control(Mode[args.motor_mode.title()], Speed[args.speed.title()], args.angle)
        tasks.append(asyncio.create_task(transmit(link, trace, command, args.period_ms / 1000,
                                                 args.telemetry_timeout)))
    if args.duration:
        tasks.append(asyncio.create_task(asyncio.sleep(args.duration)))
    try:
        done, _ = await asyncio.wait(tasks, return_when=asyncio.FIRST_COMPLETED)
        for task in done:
            task.result()
    finally:
        for task in tasks:
            task.cancel()
        await asyncio.gather(*tasks, return_exceptions=True)
        # Receive-only monitoring never sends a stop. Avoid extending a fault latch.
        fault = link.latest and link.latest[1].status & Status.SafetyFault
        if controlling and not link.failure and not fault:
            try:
                neutral = Control(Mode.Neutral, Speed.Slow, 0)
                await link.send(neutral)
                trace.frame("TX", neutral, neutral.encode())
            except Exception as error:
                print(f"Neutral delivery unconfirmed: {error}; Teensy watchdog remains responsible for stopping",
                      file=sys.stderr)


async def execute(args, stream=None):
    try:
        from bleak import BleakClient, BleakScanner
    except ImportError as error:
        raise RuntimeError("install requirements-ble.txt in the active Python environment") from error
    if args.mode == "ble-scan":
        devices = await BleakScanner.discover(timeout=args.timeout, return_adv=True)
        found = False
        for device, adv in devices.values():
            if SERVICE in [uuid.lower() for uuid in adv.service_uuids]:
                print(f"{device.address}  {adv.local_name or device.name}")
                found = True
        if not found:
            print("No NanoNeuroExo bridge found.")
        return 0 if found else 1
    link = BleLink()
    client = BleakClient(args.address, timeout=15, disconnected_callback=link.disconnected)
    link.client = client
    async with client:
        for uuid, prop in ((COMMAND, "write"), (TELEMETRY, "notify")):
            char = client.services.get_characteristic(uuid)
            if char is None or prop not in char.properties or char.service_uuid.lower() != SERVICE:
                raise RuntimeError("Nano motor service mismatch; flash NeuroExoFirmware with the nano33ble environment")
        await asyncio.wait_for(client.start_notify(TELEMETRY, link.notification), 10)
        trace = Trace(args.hex, stream)
        await run_connected(link, args, trace)
        if link.invalid_frames:
            print(f"Discarded {link.invalid_frames} invalid telemetry frames.", file=sys.stderr)
    return 0


def run(args):
    stream = None
    try:
        if args.mode == "protocol-decode":
            print(decode(bytes.fromhex(args.hex)).describe())
            return 0
        if args.mode == "ble-scan":
            if not math.isfinite(args.timeout) or args.timeout <= 0:
                raise ValueError("scan timeout must be finite and positive")
        else:
            if not math.isfinite(args.duration) or not 0 <= args.duration <= 3600:
                raise ValueError("duration must be 0..3600 seconds")
            if not math.isfinite(args.telemetry_timeout) or not 0.05 <= args.telemetry_timeout <= 10:
                raise ValueError("telemetry timeout must be 0.05..10 seconds")
            if args.mode == "ble-control":
                if args.duration <= 0 or not 20 <= args.period_ms <= 200:
                    raise ValueError("control requires positive duration and period 20..200 ms")
                Control(Mode[args.motor_mode.title()], Speed[args.speed.title()], args.angle).encode()
            if args.log:
                stream = open(args.log, "x", encoding="utf-8")
        return asyncio.run(execute(args, stream))
    except KeyboardInterrupt:
        print("\nMotor interface stopped.", file=sys.stderr)
        return 130
    except Exception as error:
        print(f"Motor interface error: {error}", file=sys.stderr)
        return 1
    finally:
        if stream:
            stream.close()
