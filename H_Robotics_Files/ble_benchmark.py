"""BLE application-packet load test. See BLE_BENCHMARK.md for setup/protocol."""

import argparse
import asyncio
import csv
import math
import secrets
import struct
import time
import zlib
from datetime import datetime, timezone

from hardware_traffic import SOURCE_SLEEP_SECONDS, eeg_record


SERVICE = "b7e10000-6a2b-4f10-9c31-8b674045a901"
CONTROL = "b7e10001-6a2b-4f10-9c31-8b674045a901"
DATA = "b7e10002-6a2b-4f10-9c31-8b674045a901"
STATUS = "b7e10003-6a2b-4f10-9c31-8b674045a901"
VERSION = 2
START, SNAPSHOT, STOP = 1, 2, 3
RUNNING, STOPPED = 1, 2
MAX_PAYLOAD, MAX_COUNT, MAX_WRITE = 4096, 65536, 244
HEADER = struct.Struct("<IHHH")
PAGE = struct.Struct("<BBHIIII")
PAGE_FIELDS = (
    ("expected", "received", "fragments"),
    ("duplicates", "corrupt", "invalid"),
    ("incomplete", "reordered", "foreign"),
    ("first_ms", "last_ms", "max_poll_gap_us"),
    ("elapsed_ms", "payload_size", "partial"),
    ("received_payload_bytes", "received_min_bytes", "received_max_bytes"),
)
CSV_FIELDS = (
    "timestamp address run_id profile pacing processing_delay_ms payload_bytes requested_pps repeat expected "
    "write_size write_mode result reason sent received lost loss_percent "
    "delivered_pps goodput_bytes_s sending_seconds completion_seconds "
    "writes gatt_value_bytes write_p95_ms write_max_ms schedule_lag_p95_ms "
    "sender_cpu_percent fragments duplicates corrupt invalid incomplete "
    "reordered foreign first_ms last_ms max_poll_gap_us elapsed_ms payload_size partial "
    "payload_total_bytes payload_min_bytes payload_max_bytes received_payload_bytes received_min_bytes received_max_bytes"
).split()


def positive(value):
    number = float(value)
    if not math.isfinite(number) or number <= 0:
        raise argparse.ArgumentTypeError("must be finite and greater than zero")
    return number


def rate_value(value):
    number = float(value)
    if not math.isfinite(number) or number < 0:
        raise argparse.ArgumentTypeError("must be finite and nonnegative (0 means unpaced)")
    return number


def add_modes(modes):
    preview = modes.add_parser("ble-preview", help="preview synthetic Hardware_Interface EEG records without Bluetooth")
    preview.add_argument("--count", type=int, default=3)
    preview.add_argument("--write-size", type=int, default=20)
    scan = modes.add_parser("ble-scan", help="find Nano BLE benchmark receivers")
    scan.add_argument("--timeout", type=positive, default=10)
    bench = modes.add_parser("ble-benchmark", help="send test packets to a Nano 33 BLE")
    bench.add_argument("--address", required=True, help="Nano address from ble-scan")
    bench.add_argument("--profile", choices=["pattern", "hardware-eeg"], default="pattern",
                       help="arbitrary byte payloads or the C++ debug EEG/EOG text format")
    bench.add_argument("--pacing", choices=["rate", "source-loop"], default="rate",
                       help="target packet rate, or C++ style sleep/process/send (hardware-eeg only)")
    bench.add_argument("--processing-delay-ms", type=rate_value, default=0,
                       help="extra simulated processing wait per source-loop iteration; not a CPU model")
    bench.add_argument("--sizes", type=int, nargs="+", default=None,
                       help="application payload bytes per packet, 1..4096")
    bench.add_argument("--rates", type=rate_value, nargs="+", default=None,
                       help="application packets/second; 0 sends without pacing")
    length = bench.add_mutually_exclusive_group()
    length.add_argument("--duration", type=positive,
                        help="nominal seconds per paced run (default 10)")
    length.add_argument("--count", type=int, help="packets per run, 1..65536; required for rate 0")
    bench.add_argument("--repeats", type=int, default=3)
    bench.add_argument("--write-size", type=int, default=20,
                       help="GATT value bytes including 10-byte header; 0 selects negotiated maximum")
    bench.add_argument("--write-mode", choices=["without-response", "with-response"],
                       default="without-response")
    bench.add_argument("--rate-tolerance", type=rate_value, default=0.05,
                       help="allowed fractional throughput shortfall (default 0.05)")
    bench.add_argument("--max-write-ms", type=positive, help="optional p95 write-call latency limit")
    bench.add_argument("--max-poll-gap-ms", type=positive, help="optional Nano BLE.poll() gap limit")
    bench.add_argument("--drain-timeout", type=positive, default=3,
                       help="seconds to wait for queued data after sending")
    bench.add_argument("--timeout", type=positive, default=10, help="individual BLE operation timeout")
    bench.add_argument("--run-timeout", type=positive, default=300, help="maximum seconds for one run")
    bench.add_argument("--output", default="ble_benchmark.csv", help="new CSV file; refuses overwrite")


def normalize(args):
    if getattr(args, "_normalized", False):
        return args
    if args.profile == "hardware-eeg":
        if args.sizes is not None:
            raise ValueError("hardware-eeg uses actual variable text lengths; use --profile pattern for --sizes")
        args.sizes = [0]  # Protocol 2: START size 0 permits variable payload lengths.
    else:
        args.sizes = [20] if args.sizes is None else args.sizes
    if args.pacing == "source-loop":
        if args.profile != "hardware-eeg" or args.rates is not None or args.count is None:
            raise ValueError("source-loop requires --profile hardware-eeg and --count; omit --rates")
        args.rates = [0]  # No offered-rate claim: this loop naturally slows with work.
    else:
        if args.processing_delay_ms:
            raise ValueError("--processing-delay-ms applies only to source-loop")
        if args.rates is None:
            args.rates = [250] if args.profile == "hardware-eeg" else [10, 25, 50, 100, 200, 400]
    args._normalized = True
    return args


def packet_count(args, rate):
    if args.count is not None:
        return args.count
    count = rate * (args.duration or 10)
    if not math.isfinite(count) or count > MAX_COUNT:
        raise ValueError("each run must contain at most 65536 packets; reduce duration/rate")
    return math.ceil(count)


def validate(args):
    normalize(args)
    if args.profile == "pattern" and any(size < 1 or size > MAX_PAYLOAD for size in args.sizes):
        raise ValueError("--sizes must be between 1 and 4096 payload bytes")
    if args.repeats < 1 or args.rate_tolerance >= 1:
        raise ValueError("--repeats must be positive and --rate-tolerance must be less than 1")
    if args.write_size != 0 and not HEADER.size < args.write_size <= MAX_WRITE:
        raise ValueError("--write-size must be 0 (auto) or between 11 and 244")
    if 0 in args.rates and args.count is None:
        raise ValueError("rate 0 requires --count")
    if any(not 1 <= packet_count(args, rate) <= MAX_COUNT for rate in args.rates):
        raise ValueError("each run must contain 1..65536 packets")


def make_payload(profile, run_id, sequence, size):
    if profile == "hardware-eeg":
        return eeg_record(sequence)
    pattern = bytes((sequence * 17 + index * 31 + run_id) & 255 for index in range(256))
    return (pattern * math.ceil(size / 256))[:size]


def fragments(run_id, sequence, size, write_size, payload=None):
    """Protocol 2: variable payload + CRC32 split into bounded GATT writes."""
    if payload is None:
        payload = make_payload("pattern", run_id, sequence, size)
    if not 1 <= len(payload) <= MAX_PAYLOAD or not HEADER.size < write_size <= MAX_WRITE:
        raise ValueError("invalid payload or GATT write size")
    crc = zlib.crc32(struct.pack("<IHH", run_id, sequence, len(payload)) + payload)
    message = payload + struct.pack("<I", crc)
    capacity = write_size - HEADER.size
    for offset in range(0, len(message), capacity):
        yield HEADER.pack(run_id, sequence, offset, len(payload)) + message[offset:offset + capacity]


def packet_wait(args, rate, due, now):
    if args.pacing == "source-loop":
        return SOURCE_SLEEP_SECONDS + args.processing_delay_ms / 1000
    return max(0, due - now) if rate else 0


def preview(args):
    if not 1 <= args.count <= 100 or not HEADER.size < args.write_size <= MAX_WRITE:
        raise ValueError("preview needs --count 1..100 and --write-size 11..244")
    print("Synthetic hardware-eeg records; no hardware, filters, or Bluetooth accessed.")
    print("C++-style fields: eeg1;eeg2;eeg3;eeg4;eeg5;eog1;eog2;eog3 followed by newline.")
    for sequence in range(args.count):
        payload = eeg_record(sequence)
        writes = list(fragments(1, sequence, 0, args.write_size, payload))
        print(f"record={sequence} payload={len(payload)} B fragments={len(writes)} "
              f"GATT_value_bytes={sum(map(len, writes))} text={payload.decode('ascii')!r}")
    return 0


class Distribution:
    """Bounded log histogram: conservative upper percentile, ~1% resolution."""

    def __init__(self):
        self.bins = [0] * 2048
        self.count = 0
        self.maximum = 0.0

    def add(self, milliseconds):
        index = min(len(self.bins) - 1,
                    math.ceil(math.log1p(max(0, milliseconds) * 1000) / math.log(1.01)))
        self.bins[index] += 1
        self.count += 1
        self.maximum = max(self.maximum, milliseconds)

    def percentile(self, fraction):
        target, total = math.ceil(self.count * fraction), 0
        for index, count in enumerate(self.bins):
            total += count
            if total >= target:
                return min(self.maximum, (1.01 ** index - 1) / 1000)
        return self.maximum


class Link:
    def __init__(self, client, timeout):
        self.client, self.timeout = client, timeout

    async def write(self, characteristic, value, response=True):
        await asyncio.wait_for(
            self.client.write_gatt_char(characteristic, value, response=response), self.timeout)

    async def page(self, run_id, page, state):
        await self.write(CONTROL, struct.pack("<BBIB", VERSION, SNAPSHOT, run_id, page))
        raw = await asyncio.wait_for(self.client.read_gatt_char(STATUS), self.timeout)
        if len(raw) != PAGE.size:
            raise RuntimeError("Nano returned an invalid status length")
        version, actual_state, actual_page, actual_run, *values = PAGE.unpack(raw)
        if (version, actual_state, actual_page, actual_run) != (VERSION, state, page, run_id):
            raise RuntimeError("Nano status version/state/page/run mismatch; check receiver firmware")
        return dict(zip(PAGE_FIELDS[page], values))


def evaluate(row, args):
    reasons = []
    if row["sent"] != row["expected"]:
        reasons.append("sender_incomplete")
    if row["received"] != row["expected"]:
        reasons.append("packet_loss")
    if any(row[key] for key in ("duplicates", "corrupt", "invalid", "incomplete", "reordered", "foreign")):
        reasons.append("receiver_errors")
    if row["received"] == row["expected"] and row["received_payload_bytes"] != row["payload_total_bytes"]:
        reasons.append("payload_byte_mismatch")
    if args.pacing == "rate" and row["requested_pps"] and row["delivered_pps"] < row["requested_pps"] * (1 - args.rate_tolerance):
        reasons.append("rate_shortfall")
    if args.max_write_ms is not None and row["write_p95_ms"] > args.max_write_ms:
        reasons.append("write_latency")
    if args.max_poll_gap_ms is not None and row["max_poll_gap_us"] / 1000 > args.max_poll_gap_ms:
        reasons.append("receiver_poll_gap")
    row.update(result="FAIL" if reasons else "PASS", reason=";".join(reasons))


async def run_trial(link, args, row):
    run_id, size, rate = row["run_id"], row["payload_bytes"], row["requested_pps"]
    expected, write_size = row["expected"], row["write_size"]
    row.update(sent=0, writes=0, gatt_value_bytes=0,
               payload_total_bytes=0, payload_min_bytes=MAX_PAYLOAD, payload_max_bytes=0)
    try:
        await link.write(CONTROL, struct.pack("<BBIHI", VERSION, START, run_id, size, expected))
        initial = await link.page(run_id, 0, RUNNING)
        if initial["expected"] != expected or initial["received"] != 0:
            raise RuntimeError("Nano did not reset counters for this run")
        timings, lags = Distribution(), Distribution()
        cpu_started = time.process_time()
        started = due = last_progress = time.perf_counter()
        for sequence in range(expected):
            if rate or args.pacing == "source-loop":
                await asyncio.sleep(packet_wait(args, rate, due, time.perf_counter()))
            packet_started = time.perf_counter()
            lags.add(max(0, packet_started - due) * 1000 if rate else 0)
            # Avoid catch-up bursts when the sender cannot sustain the target.
            due = packet_started + 1 / rate if rate else packet_started
            payload = make_payload(args.profile, run_id, sequence, size)
            for value in fragments(run_id, sequence, size, write_size, payload):
                before = time.perf_counter()
                await link.write(DATA, value, response=args.write_mode == "with-response")
                timings.add((time.perf_counter() - before) * 1000)
                row["writes"] += 1
                row["gatt_value_bytes"] += len(value)
            row["sent"] += 1
            row["payload_total_bytes"] += len(payload)
            row["payload_min_bytes"] = min(row["payload_min_bytes"], len(payload))
            row["payload_max_bytes"] = max(row["payload_max_bytes"], len(payload))
            if time.perf_counter() - last_progress >= 5:
                print(f"  submitted {row['sent']}/{expected} packets", flush=True)
                last_progress = time.perf_counter()
        row["sending_seconds"] = time.perf_counter() - started
        row["sender_cpu_percent"] = 100 * (time.process_time() - cpu_started) / max(row["sending_seconds"], 1e-9)
        deadline = time.perf_counter() + args.drain_timeout
        while True:
            status = await link.page(run_id, 0, RUNNING)
            if status["received"] == expected or time.perf_counter() >= deadline:
                break
            await asyncio.sleep(min(0.1, max(0, deadline - time.perf_counter())))
        # Time until Nano confirmation, not just until the OS accepts writes.
        row["completion_seconds"] = time.perf_counter() - started
        await link.write(CONTROL, struct.pack("<BBI", VERSION, STOP, run_id))
        for page in range(len(PAGE_FIELDS)):
            snapshot = await link.page(run_id, page, STOPPED)
            if page == 0 and snapshot["expected"] != expected:
                raise RuntimeError("Nano final expected count changed")
            row.update(snapshot)
        if row["payload_size"] != size or row["received"] > expected:
            raise RuntimeError("Nano final counters do not match this run")
        if status["received"] != row["received"]:
            row["completion_seconds"] = time.perf_counter() - started
        row.update(lost=expected - row["received"],
                   loss_percent=100 * (expected - row["received"]) / expected,
                   delivered_pps=row["received"] / row["completion_seconds"],
                   goodput_bytes_s=row["received_payload_bytes"] / row["completion_seconds"],
                   write_p95_ms=timings.percentile(0.95), write_max_ms=timings.maximum,
                   schedule_lag_p95_ms=lags.percentile(0.95))
        evaluate(row, args)
    finally:
        # No retries of test data. Freeze partial runs on errors/Ctrl+C.
        if link.client.is_connected:
            try:
                await link.write(CONTROL, struct.pack("<BBI", VERSION, STOP, run_id))
            except Exception:
                pass


def summarize(rows):
    loop_rows = [row for row in rows if row.get("pacing") == "source-loop"]
    if loop_rows:
        outcome = "PASS" if all(row["result"] == "PASS" for row in loop_rows) else "FAIL"
        print(f"Source-loop {outcome}: observed delivery "
              f"{min(row['delivered_pps'] for row in loop_rows):.2f}.."
              f"{max(row['delivered_pps'] for row in loop_rows):.2f} records/s.")
        print("The loop waits 4 ms plus configured processing delay and send time; this is not a fixed-rate capacity test.")
    rows = [row for row in rows if row.get("pacing") != "source-loop"]
    for size in sorted({row["payload_bytes"] for row in rows}):
        groups = {}
        for row in rows:
            if row["payload_bytes"] == size:
                groups.setdefault(row["requested_pps"], []).append(row)
        passing = [(rate, runs) for rate, runs in groups.items()
                   if all(row["result"] == "PASS" for row in runs)]
        paced = [(rate, runs) for rate, runs in passing if rate > 0]
        label = "hardware-eeg (variable bytes)" if size == 0 else f"{size} bytes"
        if paced:
            rate, runs = max(paced, key=lambda group: group[0])
            print(f"{label}: highest passing tested rate = {rate:g} packets/s; "
                  f"worst measured delivery = {min(row['delivered_pps'] for row in runs):.2f} packets/s.")
        else:
            print(f"{label}: no paced rate passed every repeat.")
        for rate, runs in passing:
            if rate == 0:
                print(f"  Unpaced loss-free delivery: {min(row['delivered_pps'] for row in runs):.2f}"
                      f"..{max(row['delivered_pps'] for row in runs):.2f} packets/s.")
    print("These are observed results for the tested load/duration, not a guaranteed hardware maximum.")


async def benchmark(args, scanner, client_factory):
    validate(args)
    # Refuse overwrites. Flush every completed/error row to preserve evidence.
    with open(args.output, "x", newline="", encoding="utf-8") as report:
        writer = csv.DictWriter(report, fieldnames=CSV_FIELDS)
        writer.writeheader()
        report.flush()
        device = await scanner.find_device_by_address(args.address, timeout=args.timeout)
        if device is None:
            raise RuntimeError("Nano not found; run ble-scan and check its power/address")
        async with client_factory(device, timeout=args.timeout) as client:
            chars = {uuid: client.services.get_characteristic(uuid) for uuid in (CONTROL, DATA, STATUS)}
            if any(value is None for value in chars.values()):
                raise RuntimeError("Benchmark characteristics missing; flash NanoBLEBenchmark")
            needed = "write" if args.write_mode == "with-response" else "write-without-response"
            if needed not in chars[DATA].properties or "write" not in chars[CONTROL].properties or "read" not in chars[STATUS].properties:
                raise RuntimeError("Nano characteristic properties do not match the benchmark")
            # Use the conservative no-response limit for both modes to avoid
            # accidentally benchmarking ATT long-write procedures.
            await asyncio.sleep(0.5)
            limit = min(MAX_WRITE, chars[DATA].max_write_without_response_size)
            write_size = args.write_size or limit
            if write_size > limit or write_size <= HEADER.size:
                raise ValueError(f"GATT write size {write_size} unsupported; current limit is {limit} bytes")
            print(f"Connected to {device.address}; GATT value size <= {write_size} bytes (10-byte header).")
            print("Payload sizes exclude headers and CRC.")
            if args.profile == "hardware-eeg":
                print("Modeling eight-channel C++ debug text over BLE; requires benchmark receiver v2, not angle/I2C firmware.")
            if args.pacing == "source-loop":
                print(f"Source-loop pacing: 4 ms + {args.processing_delay_ms:g} ms simulated work + formatting/send time.")
            else:
                print("Fixed-rate pacing; rate 0 means unpaced.")
            link, rows = Link(client, args.timeout), []
            for size in dict.fromkeys(args.sizes):
                for rate in dict.fromkeys(args.rates):
                    for repeat in range(1, args.repeats + 1):
                        row = dict(timestamp=datetime.now(timezone.utc).isoformat(), address=args.address,
                                   run_id=secrets.randbits(32) or 1, profile=args.profile, pacing=args.pacing,
                                   processing_delay_ms=args.processing_delay_ms, payload_bytes=size,
                                   requested_pps=rate, repeat=repeat, expected=packet_count(args, rate),
                                   write_size=write_size, write_mode=args.write_mode)
                        label = "variable" if size == 0 else str(size)
                        print(f"Testing profile={args.profile} size={label} pacing={args.pacing} rate={rate:g} "
                              f"repeat={repeat}/{args.repeats} count={row['expected']}...", flush=True)
                        try:
                            await asyncio.wait_for(run_trial(link, args, row), args.run_timeout)
                        except BaseException as error:
                            row.update(result="ERROR", reason=f"{type(error).__name__}: {error}")
                            # No partial receiver snapshot is reported as final.
                            for key in ("received", "lost", "loss_percent", "delivered_pps", "goodput_bytes_s"):
                                row.pop(key, None)
                            writer.writerow(row)
                            report.flush()
                            print(f"Incomplete run recorded as ERROR in {args.output}.", flush=True)
                            raise
                        writer.writerow(row)
                        report.flush()
                        rows.append(row)
                        print(f"{row['result']}: received={row['received']}/{row['expected']} lost={row['lost']} "
                              f"delivery={row['delivered_pps']:.2f} packets/s goodput={row['goodput_bytes_s']:.1f} B/s "
                              f"write_p95={row['write_p95_ms']:.2f} ms {row['reason']}", flush=True)
            summarize(rows)
            print(f"Saved {args.output}")
            return 0 if all(row["result"] == "PASS" for row in rows) else 2


async def execute(args):
    if args.mode == "ble-preview":
        return preview(args)
    if args.mode == "ble-benchmark":
        validate(args)
    try:
        from bleak import BleakClient, BleakScanner
    except ImportError as error:
        raise RuntimeError("BLE modes need Bleak: python3 -m pip install -r H_Robotics_Files/requirements-ble.txt") from error
    if args.mode == "ble-scan":
        found = await BleakScanner.discover(timeout=args.timeout, return_adv=True)
        matches = 0
        for device, advertisement in found.values():
            if SERVICE in [uuid.lower() for uuid in advertisement.service_uuids]:
                print(f"{device.address}  {advertisement.local_name or device.name}  RSSI={advertisement.rssi} dBm")
                matches += 1
        if not matches:
            print("No benchmark receiver found. Flash/power the Nano and ensure it is advertising.")
        return 0
    return await benchmark(args, BleakScanner, BleakClient)


def run(args):
    try:
        return asyncio.run(execute(args))
    except KeyboardInterrupt:
        print("\nBLE test interrupted; an unfinished run is not a passing result.")
        return 130
    except Exception as error:
        print(f"BLE benchmark error: {error}")
        return 1
