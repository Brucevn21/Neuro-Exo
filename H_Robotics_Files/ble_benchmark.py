"""Numbered packet demo for the two PlatformIO *_packet_demo environments.

Invoked through read_outputs.py; uses the existing Bleak dependency only.
"""
import asyncio
import json
import math
import secrets
import struct
import sys
import time

from neuroexo_protocol import crc8

SERVICE = "b7e21000-6a2b-4f10-9c31-8b674045a901"
TX = "b7e21001-6a2b-4f10-9c31-8b674045a901"
RX = "b7e21002-6a2b-4f10-9c31-8b674045a901"
CONTROL = "b7e21003-6a2b-4f10-9c31-8b674045a901"
STATS = "b7e21004-6a2b-4f10-9c31-8b674045a901"
START, DATA, SNAPSHOT, ECHO, REPLY, STATISTICS = range(0x30, 0x36)
FIELDS = ("session", "token", "nano_received", "nano_duplicates", "nano_invalid",
          "queue_drops", "i2c_errors", "teensy_received", "teensy_duplicates",
          "teensy_invalid", "echo_attempts", "echo_failures", "queued",
          "teensy_last_sequence", "ready")


def frame(kind, payload):
    body = bytes((kind, len(payload))) + payload
    return b"\x02" + body + bytes((crc8(body), 3))


def packet(kind, value):
    return frame(kind, struct.pack(">I", value))


def payload(raw, kind, size):
    raw = bytes(raw)
    if (len(raw) != size or raw[0:3] != bytes((2, kind, size - 5)) or
            raw[-1] != 3 or raw[-2] != crc8(raw[1:-2])):
        raise ValueError("invalid demo frame length/type/CRC")
    return raw[3:-2]


def statistics(raw):
    values = struct.unpack(">15I", payload(raw, STATISTICS, 65))
    return dict(zip(FIELDS, values))


def add_mode(modes):
    parser = modes.add_parser("ble-benchmark", help="250 Hz numbered Nano/Teensy packet demo")
    parser.add_argument("--address", help="Nano address; automatically find one demo if omitted")
    parser.add_argument("--scan", action="store_true", help="list demo firmware devices and exit")
    parser.add_argument("--rate", type=int, default=250, help="requested packets/s (default: 250)")
    parser.add_argument("--rates", help="test a list, e.g. 50,100,150,200,250,300")
    parser.add_argument("--duration", type=float, default=10, help="seconds per run (default: 10)")
    parser.add_argument("--repeat", type=int, default=1, help="runs at each rate (default: 1)")
    parser.add_argument("--drain", type=float, default=2, help="seconds for late echoes (default: 2)")
    parser.add_argument("--write-response", action="store_true",
                        help="compare the acknowledged write mode used by production controls")
    parser.add_argument("--json", help="new file for complete results and missing echo IDs")


class Receiver:
    def __init__(self):
        self.sent = {}
        self.echoes = {}
        self.latencies_ms = []
        self.duplicates = self.invalid = self.unexpected = 0
        self.active = False

    def notification(self, characteristic, raw):
        if not self.active:
            return
        try:
            sequence, accepted = struct.unpack(">IB", payload(raw, ECHO, 10))
            if accepted != 1:
                raise ValueError("Teensy did not accept packet")
        except ValueError:
            self.invalid += 1
            return
        now = time.monotonic()
        if sequence not in self.sent:
            self.unexpected += 1
        elif sequence in self.echoes:
            self.duplicates += 1
        else:
            self.echoes[sequence] = now
            self.latencies_ms.append((now - self.sent[sequence]) * 1000)


async def snapshot(client, kind, token, session, timeout=5):
    # Control uses a response even when the timed data stream does not.
    await asyncio.wait_for(client.write_gatt_char(CONTROL, packet(kind, token), response=True), timeout)
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        raw = await asyncio.wait_for(client.read_gatt_char(STATS), timeout)
        info = statistics(raw)
        if info["session"] == session and info["token"] == token:
            if not info["ready"]:
                raise RuntimeError("Teensy demo did not answer: check its firmware, USB power and I2C wiring")
            return info
        await asyncio.sleep(0.05)
    raise TimeoutError("receiver statistics did not become ready; result is inconclusive")


def summarize(receiver, info, rate, duration, elapsed, submitted, write_errors, error, started):
    sent = len(receiver.sent)
    echoed = len(receiver.echoes)
    expected = math.ceil(rate * duration)
    missing = sorted(set(receiver.sent) - set(receiver.echoes))
    consistent = (info is not None and info["ready"] == 1 and info["queued"] == 0 and
                  0 <= echoed <= info["echo_attempts"] <= info["teensy_received"] <=
                  info["nano_received"] <= sent)
    complete = consistent and not error and write_errors == 0
    active_echoes = sum(stamp < started + duration for stamp in receiver.echoes.values())
    tx_target_met = submitted == expected and submitted / elapsed >= rate * 0.99
    rx_target_met = active_echoes / duration >= rate * 0.99
    target_met = tx_target_met and rx_target_met
    clean = (complete and not missing and receiver.duplicates == 0 and
             receiver.invalid == 0 and receiver.unexpected == 0 and
             all(info[key] == 0 for key in ("nano_duplicates", "nano_invalid", "queue_drops",
                                          "i2c_errors", "teensy_duplicates", "teensy_invalid",
                                          "echo_failures")))
    latency = sorted(receiver.latencies_ms)
    return {
        "result": "INCONCLUSIVE" if not complete else ("PASS" if clean and target_met else "FAIL"),
        "requested_rate_hz": rate, "requested_duration_s": duration,
        "measured_tx_duration_s": elapsed, "planned_packets": expected,
        "attempted": sent, "submitted": submitted,
        "host_unsent_slots": expected - sent, "write_errors": write_errors,
        "actual_tx_hz": submitted / elapsed,
        "rx_during_send_hz": active_echoes / duration,
        "echoes_after_drain": echoed, "missing_echo_count": len(missing),
        "attempted_sequences": sorted(receiver.sent),
        "received_sequences": sorted(receiver.echoes),
        "missing_echo_sequences": missing,
        "roundtrip_loss_percent": 100 * len(missing) / sent if complete and sent else None,
        "not_seen_by_nano": sent - info["nano_received"] if complete else None,
        "nano_to_teensy_missing": info["nano_received"] - info["teensy_received"] if complete else None,
        "teensy_to_host_missing": info["teensy_received"] - echoed if complete else None,
        "duplicate_echoes": receiver.duplicates, "invalid_echoes": receiver.invalid,
        "unexpected_echoes": receiver.unexpected,
        "rtt_mean_ms": sum(latency) / len(latency) if latency else None,
        "rtt_p95_ms": latency[max(0, math.ceil(len(latency) * 0.95) - 1)] if latency else None,
        "rtt_max_ms": max(latency) if latency else None,
        "target_met": target_met, "tx_target_met": tx_target_met, "rx_target_met": rx_target_met,
        "receiver_statistics": info,
        "error": error or (None if consistent else "receiver counts inconsistent or unavailable"),
    }


def print_result(result):
    info = result["receiver_statistics"]
    print("\n{}: requested {} Hz | actual TX {:.1f}/s | RX during sending {:.1f}/s".format(
        result["result"], result["requested_rate_hz"], result["actual_tx_hz"], result["rx_during_send_hz"]))
    print("  Attempted={} submitted={} host-unsent={} write-errors={}".format(
        result["attempted"], result["submitted"], result["host_unsent_slots"], result["write_errors"]))
    if info is not None:
        print("  Nano accepted={} | Teensy accepted={} | echoes received={}".format(
            info["nano_received"], info["teensy_received"], result["echoes_after_drain"]))
        print("  Nano queue drops={} | I2C errors={} | echo submission failures={}".format(
            info["queue_drops"], info["i2c_errors"], info["echo_failures"]))
    if result["roundtrip_loss_percent"] is not None:
        print("  Missing: host->Nano={} | Nano->Teensy={} | Teensy->host={} | round-trip={:.3f}%".format(
            result["not_seen_by_nano"], result["nano_to_teensy_missing"],
            result["teensy_to_host_missing"], result["roundtrip_loss_percent"]))
    print("  Echo duplicates={} invalid={} unexpected={}".format(
        result["duplicate_echoes"], result["invalid_echoes"], result["unexpected_echoes"]))
    if result["rtt_mean_ms"] is not None:
        print("  Round-trip time: mean={:.2f} ms p95={:.2f} ms max={:.2f} ms".format(
            result["rtt_mean_ms"], result["rtt_p95_ms"], result["rtt_max_ms"]))
    if result["missing_echo_sequences"]:
        print("  Missing echo IDs (first 20):", result["missing_echo_sequences"][:20])
    if result["error"]:
        print("  Incomplete/uncertain delivery:", result["error"])
    elif not result["target_met"]:
        print("  Requested TX/RX rate was not sustained; host-unsent slots and late echoes are separate from loss.")
    sys.stdout.flush()


async def run_trial(client, args, rate):
    receiver = Receiver()
    await client.start_notify(RX, receiver.notification)
    try:
        # A fresh nonzero session plus a confirmed reset prevents cross-run counts.
        session = secrets.randbelow(0xffffffff) + 1
        initial = await snapshot(client, START, session, session)
        if any(initial[key] for key in ("nano_received", "teensy_received", "queued")):
            raise RuntimeError("receiver counters did not reset")
        receiver.active = True
        total = math.ceil(rate * args.duration)
        print("\nTesting {} Hz for {:g}s: {} packets, {}".format(
            rate, args.duration, total,
            "write with response" if args.write_response else "write without response"), flush=True)
        started = time.monotonic()
        end = started + args.duration
        next_report = started + 1
        slot = submitted = write_errors = 0
        error = None
        while slot < total:
            now = time.monotonic()
            due = started + slot / rate
            if now < due:
                await asyncio.sleep(due - now)
            now = time.monotonic()
            if now >= end:
                break
            # Skip missed scheduling slots; never hide a slow host with catch-up bursts.
            slot = max(slot, int((now - started) * rate))
            if slot >= total:
                break
            sequence = slot + 1
            receiver.sent[sequence] = time.monotonic()
            try:
                await asyncio.wait_for(client.write_gatt_char(
                    TX, packet(DATA, sequence), response=args.write_response), 1)
                submitted += 1
            except Exception as exc:
                write_errors += 1
                error = "write failed (delivery unknown): {}".format(exc)
                break
            slot += 1
            now = time.monotonic()
            if now >= next_report:
                print("  {:.1f}s TX={} RX={} pending={} actual TX={:.1f}/s".format(
                    now - started, submitted, len(receiver.echoes),
                    len(receiver.sent) - len(receiver.echoes), submitted / (now - started)), flush=True)
                next_report = now + 1
        if not error:
            await asyncio.sleep(max(0, end - time.monotonic()))
        elapsed = max(time.monotonic() - started, 1e-9)
        info = None
        try:
            # These seconds do not inflate the timed send rate.
            await asyncio.sleep(args.drain)
            token = (session % 0xffffffff) + 1
            info = await snapshot(client, SNAPSHOT, token, session)
            await asyncio.sleep(0.2)  # Deliver notifications already queued on the host.
        except Exception as exc:
            error = error or "final statistics unavailable: {}".format(exc)
        receiver.active = False
        result = summarize(receiver, info, rate, args.duration, elapsed, submitted, write_errors, error, started)
        result["write_with_response"] = args.write_response
        print_result(result)
        return result
    finally:
        receiver.active = False
        if client.is_connected:
            await client.stop_notify(RX)


async def execute(args, rates, results):
    from bleak import BleakClient, BleakScanner
    if args.scan or not args.address:
        devices = await BleakScanner.discover(timeout=5, return_adv=True)
        matches = [(device, adv) for device, adv in devices.values()
                   if SERVICE in [uuid.lower() for uuid in adv.service_uuids]]
        for device, adv in matches:
            print("{}  {}".format(device.address, adv.local_name or device.name))
        if args.scan:
            return 0 if matches else 1
        if len(matches) != 1:
            raise RuntimeError("expected one NeuroExoPacketDemo; flash demo firmware or specify --address")
        device = matches[0][0]
    else:
        device = args.address
    async with BleakClient(device, timeout=15) as client:
        required = ((TX, "write" if args.write_response else "write-without-response"),
                    (RX, "notify"), (CONTROL, "write"), (STATS, "read"))
        for uuid, prop in required:
            char = client.services.get_characteristic(uuid)
            if char is None or prop not in char.properties or char.service_uuid.lower() != SERVICE:
                raise RuntimeError("wrong firmware/service: upload nano33ble_packet_demo and teensy41_packet_demo")
        await asyncio.sleep(0.5)  # Allow the requested connection parameters to settle.
        for rate in rates:
            for repeat in range(args.repeat):
                print("Run {}/{} at {} Hz".format(repeat + 1, args.repeat, rate))
                result = await run_trial(client, args, rate)
                results.append(result)
                if result["result"] == "INCONCLUSIVE":
                    return 1
    passing = [rate for rate in rates if all(
        r["result"] == "PASS" for r in results if r["requested_rate_hz"] == rate)]
    if passing:
        print("\nHighest tested rate passing every repetition: {} Hz (zero observed loss).".format(max(passing)))
    else:
        print("\nNo tested rate passed all repetitions.")
    return 0 if all(result["result"] == "PASS" for result in results) else 2


def run(args):
    results = []
    output = None
    try:
        rates = list(dict.fromkeys(int(value) for value in args.rates.split(","))) if args.rates else [args.rate]
        if not rates or any(not 1 <= rate <= 1000 for rate in rates):
            raise ValueError("rates must be integers from 1 to 1000")
        if not math.isfinite(args.duration) or not 1 <= args.duration <= 600:
            raise ValueError("duration must be 1..600 seconds")
        if not 1 <= args.repeat <= 10:
            raise ValueError("repeat must be 1..10")
        if not math.isfinite(args.drain) or not 0.1 <= args.drain <= 30:
            raise ValueError("drain must be 0.1..30 seconds")
        if args.json:
            output = open(args.json, "x", encoding="utf-8")
        return asyncio.run(execute(args, rates, results))
    except KeyboardInterrupt:
        print("\nBenchmark interrupted; unfinished run is not a passing measurement.", file=sys.stderr)
        return 130
    except Exception as exc:
        print("Benchmark error: {}".format(exc), file=sys.stderr)
        return 1
    finally:
        if output:
            json.dump({"format_version": 1, "runs": results}, output, indent=2)
            output.write("\n")
            output.close()
