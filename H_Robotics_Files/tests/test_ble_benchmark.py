"""No radio required. g++ compiles the Nano's actual protocol core for tests."""
import argparse
import asyncio
import contextlib
import csv
import ctypes
import io
import pathlib
import random
import struct
import subprocess
import sys
import tempfile
import unittest

ROOT = pathlib.Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
import ble_benchmark as b
import read_outputs
from hardware_traffic import eeg_values, eeg_record, serialize_eeg

BUILD = None
NATIVE = None


def setUpModule():
    global BUILD, NATIVE
    BUILD = tempfile.TemporaryDirectory(prefix="nano-benchmark-test-")
    try:
        binary = pathlib.Path(BUILD.name) / "receiver.so"
        subprocess.run(["g++", "-std=c++11", "-Wall", "-Wextra", "-Werror",
                        "-shared", "-fPIC", "-O2",
                        str(ROOT / "tests/receiver_bridge.cpp"), "-o", str(binary)], check=True)
        NATIVE = ctypes.CDLL(str(binary))
        NATIVE.start_receiver.argtypes = [ctypes.c_uint32, ctypes.c_uint16, ctypes.c_uint32, ctypes.c_uint32]
        NATIVE.start_receiver.restype = ctypes.c_int
        NATIVE.receive_fragment.argtypes = [ctypes.c_void_p, ctypes.c_size_t, ctypes.c_uint32]
        NATIVE.stop_receiver.argtypes = [ctypes.c_uint32]
        NATIVE.status_receiver.argtypes = [ctypes.c_uint8, ctypes.c_uint32, ctypes.c_void_p]
        NATIVE.format_eeg.argtypes = [ctypes.POINTER(ctypes.c_double), ctypes.c_void_p, ctypes.c_size_t]
        NATIVE.format_eeg.restype = ctypes.c_int
    except BaseException:
        BUILD.cleanup()
        raise


def tearDownModule():
    BUILD.cleanup()


def parse(*extra):
    parser = argparse.ArgumentParser()
    b.add_modes(parser.add_subparsers(dest="mode", required=True))
    return b.normalize(parser.parse_args(["ble-benchmark", "--address", "test", "--count", "3",
                                         "--rates", "0", "--repeats", "1", *extra]))


def parse_hardware(*extra):
    parser = argparse.ArgumentParser()
    b.add_modes(parser.add_subparsers(dest="mode", required=True))
    return b.normalize(parser.parse_args(["ble-benchmark", "--address", "test", "--count", "3",
                                         "--repeats", "1", "--profile", "hardware-eeg",
                                         "--pacing", "source-loop", *extra]))


def receive(value, now=100):
    NATIVE.receive_fragment(value, len(value), now)


def page(number, now=100):
    out = ctypes.create_string_buffer(20)
    NATIVE.status_receiver(number, now, out)
    fields = b.PAGE.unpack(out.raw)
    return fields[:4], dict(zip(b.PAGE_FIELDS[number], fields[4:]))


def stats():
    result = {}
    for number in range(len(b.PAGE_FIELDS)):
        result.update(page(number)[1])
    return result


class ProtocolTests(unittest.TestCase):
    def setUp(self):
        NATIVE.reset_receiver()

    def test_python_packets_decoded_by_actual_cpp_receiver(self):
        for size in (1, 8, 12, 20, 64, 256, 4096):
            for write_size in (11, 20, 244):
                with self.subTest(size=size, write_size=write_size):
                    self.assertTrue(NATIVE.start_receiver(0x12345678, size, 65536, 1))
                    values = list(b.fragments(0x12345678, 65535, size, write_size))
                    self.assertTrue(all(b.HEADER.size < len(v) <= write_size for v in values))
                    for value in values:
                        receive(value)
                    self.assertEqual(stats()["received"], 1)
                    self.assertEqual(stats()["corrupt"], 0)

    def test_missing_first_middle_and_last_packets(self):
        for missing in range(3):
            NATIVE.start_receiver(7, 20, 3, 0)
            for seq in range(3):
                if seq != missing:
                    for value in b.fragments(7, seq, 20, 20):
                        receive(value)
            NATIVE.stop_receiver(150)
            self.assertEqual(stats()["expected"] - stats()["received"], 1)

    def test_duplicate_cannot_hide_a_missing_packet(self):
        NATIVE.start_receiver(7, 20, 3, 0)
        for seq in (0, 0, 2):
            for value in b.fragments(7, seq, 20, 20):
                receive(value)
        self.assertEqual(stats()["received"], 2)
        self.assertGreater(stats()["duplicates"], 0)

    def test_crc_rejects_corrupt_payload_and_sequence(self):
        for corrupt_header in (False, True):
            NATIVE.start_receiver(7, 20, 2, 0)
            for original in b.fragments(7, 0, 20, 20):
                value = bytearray(original)
                if corrupt_header:
                    value[4] = 1
                else:
                    value[b.HEADER.size] ^= 1
                receive(bytes(value))
            self.assertEqual(stats()["received"], 0)
            self.assertEqual(stats()["corrupt"], 1)

    def test_missing_fragment_and_trailing_partial_are_counted(self):
        NATIVE.start_receiver(7, 40, 2, 0)
        values = list(b.fragments(7, 0, 40, 20))
        for value in values[:1] + values[2:]:
            receive(value)
        receive(next(b.fragments(7, 1, 40, 20)))
        NATIVE.stop_receiver(150)
        self.assertEqual(stats()["incomplete"], 2)
        self.assertEqual(stats()["received"], 0)
        self.assertGreater(stats()["invalid"], 0)

    def test_foreign_run_reorder_and_reset(self):
        NATIVE.start_receiver(7, 20, 3, 0)
        for run, seq in ((8, 0), (7, 2), (7, 0)):
            for value in b.fragments(run, seq, 20, 20):
                receive(value)
        self.assertGreater(stats()["foreign"], 0)
        self.assertEqual(stats()["reordered"], 1)
        self.assertEqual(stats()["received"], 2)
        NATIVE.start_receiver(9, 20, 1, 0)
        self.assertEqual(stats()["received"], 0)
        self.assertEqual(stats()["foreign"], 0)

    def test_stop_freezes_counters_and_handles_millis_wrap(self):
        NATIVE.start_receiver(7, 1, 1, 0xfffffff0)
        for value in b.fragments(7, 0, 1, 20):
            receive(value, 5)
        NATIVE.stop_receiver(10)
        self.assertEqual(stats()["last_ms"], 21)
        before = stats()
        NATIVE.stop_receiver(1000)
        receive(next(b.fragments(7, 0, 1, 20)))
        self.assertEqual(stats(), before)

    def test_invalid_bounds_and_malformed_frames(self):
        for size, count in ((4097, 1), (1, 0), (1, 65537)):
            self.assertFalse(NATIVE.start_receiver(7, size, count, 0))
        NATIVE.start_receiver(7, 20, 1, 0)
        rng = random.Random(1)
        for length in range(301):
            receive(bytes(rng.randrange(256) for _ in range(length)))
        for value in (b.HEADER.pack(7, 0, 65535, 20) + b"x",
                      b.HEADER.pack(7, 1, 0, 20) + b"x",
                      b.HEADER.pack(7, 0, 0, 20) + b"x" * 237):
            receive(value)
        self.assertEqual(stats()["received"], 0)
        self.assertGreater(stats()["invalid"], 0)

    def test_variable_size_packets_and_actual_byte_count(self):
        payloads = [b"x", eeg_record(0), eeg_record(1), b"x" * 4096]
        self.assertTrue(NATIVE.start_receiver(7, 0, len(payloads), 0))
        for seq, payload in enumerate(payloads):
            for value in b.fragments(7, seq, 0, 20, payload):
                receive(value)
        self.assertEqual(stats()["received"], len(payloads))
        self.assertEqual(stats()["received_payload_bytes"], sum(map(len, payloads)))
        self.assertEqual(stats()["received_min_bytes"], 1)
        self.assertEqual(stats()["received_max_bytes"], 4096)

    def test_declared_length_changes_and_fixed_size_mismatch(self):
        NATIVE.start_receiver(7, 0, 1, 0)
        values = list(b.fragments(7, 0, 20, 20))
        receive(values[0])
        changed = bytearray(values[1])
        changed[8:10] = struct.pack("<H", 21)
        receive(bytes(changed))
        for value in values[2:]:
            receive(value)
        self.assertEqual(stats()["received"], 0)
        self.assertGreater(stats()["invalid"], 0)
        NATIVE.start_receiver(7, 21, 1, 0)
        for value in values:
            receive(value)
        self.assertEqual(stats()["received"], 0)
        self.assertGreater(stats()["invalid"], 0)

    def test_eeg_serialization_matches_cpp_default_ostream(self):
        cases = [eeg_values(seq) for seq in range(250)]
        cases.append((0.0, -0.0, 1e-7, 1e6, 1.23456789, -9.99999e-5, 0.0001, 999999.))
        for values in cases:
            array = (ctypes.c_double * 8)(*values)
            out = ctypes.create_string_buffer(512)
            length = NATIVE.format_eeg(array, out, len(out))
            self.assertGreater(length, 0)
            self.assertEqual(serialize_eeg(values), out.raw[:length])
        record = eeg_record(1)
        self.assertTrue(record.endswith(b"\n"))
        self.assertEqual(record.count(b";"), 7)
        self.assertEqual(read_outputs.decode_line(record.decode())[0], "EEG/EOG")
        self.assertGreater(len({len(eeg_record(seq)) for seq in range(100)}), 1)

    def test_source_loop_configuration_and_preview(self):
        args = parse_hardware("--processing-delay-ms", "2.5")
        self.assertAlmostEqual(b.packet_wait(args, 0, 0, 100), .0065)
        self.assertAlmostEqual(b.packet_wait(args, 0, 100, 0), .0065)
        self.assertEqual(args.sizes, [0])
        b.validate(args)
        for extra in (["--sizes", "128"], ["--rates", "250"]):
            with self.assertRaises(ValueError):
                parse_hardware(*extra)
        output = io.StringIO()
        with contextlib.redirect_stdout(output):
            result = b.preview(argparse.Namespace(count=2, write_size=20))
        self.assertEqual(result, 0)
        self.assertIn("record=1", output.getvalue())
        self.assertIn("no hardware", output.getvalue())

    def test_validation_existing_reader_and_histogram(self):
        for extra in (["--sizes", "0"], ["--sizes", "4097"], ["--count", "65537"],
                      ["--write-size", "8"], ["--rate-tolerance", "1"], ["--repeats", "0"]):
            with self.assertRaises(ValueError):
                b.validate(parse(*extra))
        kind, text = read_outputs.decode_line("1;2;3;4;5;6;7;8")
        self.assertEqual(kind, "EEG/EOG")
        self.assertIn("eog3=8", text)
        hist = b.Distribution()
        for value in range(1, 101):
            hist.add(value)
        self.assertGreaterEqual(hist.percentile(.95), 95)
        self.assertLess(hist.percentile(.95), 96)


class FakeClient:
    """Fake transport; payloads are processed by the actual C++ receiver."""
    def __init__(self, device=None, timeout=None, drop=None, delay=0, disconnect=False, stale=False):
        self.is_connected = True
        self.drop, self.delay = drop, delay
        self.disconnect, self.stale = disconnect, stale
        self.selected_page = 0
        self.response_flags = []
        self.queued = []
        self.hold_last = False

    async def __aenter__(self):
        NATIVE.reset_receiver()
        return self

    async def __aexit__(self, *args):
        self.is_connected = False

    @property
    def services(self):
        return self

    def get_characteristic(self, uuid):
        return type("Characteristic", (), {
            "properties": ["write", "write-without-response", "read"],
            "max_write_without_response_size": 20})()

    async def write_gatt_char(self, uuid, value, response):
        if uuid == b.DATA:
            self.response_flags.append(response)
            if self.disconnect:
                self.is_connected = False
                raise OSError("simulated disconnect")
            if self.delay:
                await asyncio.sleep(self.delay)
            seq = b.HEADER.unpack_from(value)[1]
            if seq == self.drop:
                return
            if self.hold_last and seq == 2:
                self.queued.append(value)
            else:
                receive(value)
        else:
            version, operation, run = struct.unpack_from("<BBI", value)
            if operation == b.START:
                size, count = struct.unpack_from("<HI", value, 6)
                NATIVE.start_receiver(run, size, count, 0)
            elif operation == b.SNAPSHOT:
                for packet in self.queued:
                    receive(packet)
                self.queued.clear()
                self.selected_page = value[6]
            elif operation == b.STOP:
                NATIVE.stop_receiver(150)

    async def read_gatt_char(self, uuid):
        out = ctypes.create_string_buffer(20)
        NATIVE.status_receiver(self.selected_page, 100, out)
        value = bytearray(out.raw)
        if self.stale:
            value[4] ^= 1
        return value


class AsyncTests(unittest.IsolatedAsyncioTestCase):
    def setUp(self):
        NATIVE.reset_receiver()

    def row(self, args):
        return dict(run_id=7, payload_bytes=args.sizes[0], requested_pps=args.rates[0],
                    expected=args.count, write_size=20)

    async def test_hardware_profile_receives_variable_records(self):
        args = parse_hardware()
        row = self.row(args)
        await b.run_trial(b.Link(FakeClient(), 1), args, row)
        self.assertEqual(row["result"], "PASS")
        self.assertEqual(row["received"], 3)
        total = sum(len(eeg_record(seq)) for seq in range(3))
        self.assertEqual(row["payload_total_bytes"], total)
        self.assertEqual(row["received_payload_bytes"], total)
        self.assertEqual(row["goodput_bytes_s"], total / row["completion_seconds"])

    async def test_hardware_profile_drop_and_fixed_rate_shortfall(self):
        for client, args, reason in (
            (FakeClient(drop=2), parse_hardware("--drain-timeout", ".001"), "packet_loss"),
            (FakeClient(delay=.005),
             parse_hardware("--pacing", "rate", "--rates", "1000"), "rate_shortfall"),
        ):
            row = self.row(args)
            await b.run_trial(b.Link(client, 1), args, row)
            self.assertEqual(row["result"], "FAIL")
            self.assertIn(reason, row["reason"])

    async def test_success_and_delayed_delivery(self):
        for mode in ("without-response", "with-response"):
            args = parse("--write-mode", mode)
            row = self.row(args)
            client = FakeClient()
            client.hold_last = True
            await b.run_trial(b.Link(client, 1), args, row)
            self.assertEqual(row["result"], "PASS")
            self.assertEqual(row["received"], 3)
            self.assertTrue(all(flag == (mode == "with-response") for flag in client.response_flags))

    async def test_loss_and_backpressure_are_failures(self):
        for client, options, reason in (
            (FakeClient(drop=2), ["--drain-timeout", ".001"], "packet_loss"),
            (FakeClient(delay=.005), ["--rates", "1000"], "rate_shortfall"),
        ):
            args = parse(*options)
            row = self.row(args)
            await b.run_trial(b.Link(client, 1), args, row)
            self.assertEqual(row["result"], "FAIL")
            self.assertIn(reason, row["reason"])

    async def test_stale_snapshot_and_disconnect_raise(self):
        for client in (FakeClient(stale=True), FakeClient(disconnect=True)):
            with self.assertRaises((RuntimeError, OSError)):
                await b.run_trial(b.Link(client, 1), parse(), self.row(parse()))

    async def test_error_run_saved_without_claiming_zero_loss(self):
        scanner = type("Scanner", (), {})
        async def find(*args, **kwargs):
            return type("Device", (), {"address": "test"})()
        scanner.find_device_by_address = find
        with tempfile.TemporaryDirectory() as tmp:
            path = pathlib.Path(tmp) / "report.csv"
            args = parse("--output", str(path))
            with contextlib.redirect_stdout(io.StringIO()):
                with self.assertRaises(OSError):
                    await b.benchmark(args, scanner, lambda *a, **k: FakeClient(disconnect=True))
            with path.open() as stream:
                rows = list(csv.DictReader(stream))
            self.assertEqual(rows[0]["result"], "ERROR")
            self.assertEqual(rows[0]["lost"], "")
            with self.assertRaises(FileExistsError):
                await b.benchmark(args, scanner, FakeClient)

    async def test_complete_sweep_writes_verified_results(self):
        scanner = type("Scanner", (), {})
        async def find(*args, **kwargs):
            return type("Device", (), {"address": "test"})()
        scanner.find_device_by_address = find
        with tempfile.TemporaryDirectory() as tmp:
            path = pathlib.Path(tmp) / "report.csv"
            args = parse("--output", str(path), "--sizes", "1", "64", "--repeats", "2")
            with contextlib.redirect_stdout(io.StringIO()):
                result = await b.benchmark(args, scanner, FakeClient)
            self.assertEqual(result, 0)
            with path.open() as stream:
                rows = list(csv.DictReader(stream))
            self.assertEqual(len(rows), 4)
            for row in rows:
                self.assertEqual(row["result"], "PASS")
                self.assertEqual(row["received"], "3")
                self.assertEqual(row["lost"], "0")
                self.assertGreater(float(row["goodput_bytes_s"]), 0)

    async def test_operation_timeout_stops_receiver(self):
        args = parse()
        with self.assertRaises(asyncio.TimeoutError):
            await b.run_trial(b.Link(FakeClient(delay=.1), .001), args, self.row(args))
        self.assertEqual(page(0)[0][1], b.STOPPED)

    async def test_optional_performance_thresholds(self):
        args = parse("--max-write-ms", ".001", "--max-poll-gap-ms", ".001")
        row = self.row(args)
        await b.run_trial(b.Link(FakeClient(delay=.003), 1), args, row)
        self.assertIn("write_latency", row["reason"])
        row["max_poll_gap_us"] = 100
        b.evaluate(row, args)
        self.assertIn("receiver_poll_gap", row["reason"])

    async def test_summary_requires_every_repeat_to_pass(self):
        rows = [
            dict(payload_bytes=20, requested_pps=10, result="PASS", delivered_pps=10),
            dict(payload_bytes=20, requested_pps=20, result="PASS", delivered_pps=20),
            dict(payload_bytes=20, requested_pps=20, result="FAIL", delivered_pps=18),
        ]
        output = io.StringIO()
        with contextlib.redirect_stdout(output):
            b.summarize(rows)
        self.assertIn("highest passing tested rate = 10", output.getvalue())


if __name__ == "__main__":
    unittest.main()
