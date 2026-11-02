"""Real firmware with host substitutes, codec parity, and loss-report behavior."""
import argparse
import asyncio
import contextlib
import io
from pathlib import Path
import struct
import subprocess
import sys
import tempfile
from types import SimpleNamespace
import unittest
from unittest.mock import patch

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
import ble_benchmark as b


class FirmwareTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.directory = tempfile.TemporaryDirectory(prefix="neuroexo-packet-demo-")
        cls.executable = str(Path(cls.directory.name) / "check")
        subprocess.run([
            "g++", "-std=c++17", "-Wall", "-Wextra", "-Werror",
            "-I", str(ROOT / "tests/packet_demo_stubs"),
            "-I", str(ROOT.parent / "NeuroExoFirmware/lib/pinMap/src"),
            str(ROOT / "tests/packet_demo_check.cpp"),
            str(ROOT.parent / "NeuroExoFirmware/src/commProtocol.cpp"),
            "-o", cls.executable], check=True)

    @classmethod
    def tearDownClass(cls):
        cls.directory.cleanup()

    def test_firmware_forward_return_and_failure_accounting(self):
        result = subprocess.run([self.executable], check=True, capture_output=True, text=True)
        self.assertIn("PASS", result.stdout)

    def test_python_and_firmware_bytes_match(self):
        lines = subprocess.check_output([self.executable, "vectors"], text=True).splitlines()
        self.assertEqual(bytes.fromhex(lines[0]), b.packet(b.DATA, 0x01020304))
        self.assertEqual(bytes.fromhex(lines[1]), b.frame(b.ECHO, struct.pack(">IB", 0x01020304, 1)))
        self.assertEqual(list(b.statistics(bytes.fromhex(lines[2])).values()), list(range(100, 115)))
        raw = bytes.fromhex(lines[2])
        for size in range(len(raw)):
            with self.assertRaises(ValueError):
                b.statistics(raw[:size])
        for index in range(len(raw)):
            damaged = bytearray(raw)
            damaged[index] ^= 1
            with self.assertRaises(ValueError):
                b.statistics(damaged)


class FakeClient:
    is_connected = True

    def __init__(self, drop=0, slow=0, write_error=0, missing_stats=False):
        self.callback = None
        self.info = dict.fromkeys(b.FIELDS, 0)
        self.drop, self.slow, self.write_error = drop, slow, write_error
        self.missing_stats = missing_stats
        self.data_modes = []

    async def start_notify(self, uuid, callback):
        assert uuid == b.RX
        self.callback = callback

    async def stop_notify(self, uuid):
        self.callback = None

    async def write_gatt_char(self, uuid, raw, response):
        kind = raw[1]
        value, = struct.unpack(">I", b.payload(raw, kind, 9))
        if uuid == b.CONTROL:
            assert response is True
            if kind == b.START:
                self.info = dict.fromkeys(b.FIELDS, 0)
                self.info.update(session=value, token=value, ready=1)
            else:
                self.info.update(token=value, ready=int(not self.missing_stats))
        else:
            assert uuid == b.TX and kind == b.DATA
            self.data_modes.append(response)
            if value != self.drop:
                self.info["nano_received"] += 1
                self.info["teensy_received"] += 1
                self.info["teensy_last_sequence"] = value
                self.info["echo_attempts"] += 1
                self.callback(None, b.frame(b.ECHO, struct.pack(">IB", value, 1)))
            if self.slow:
                await b.asyncio.sleep(self.slow)
            if value == self.write_error:
                raise TimeoutError("ambiguous write timeout after receipt")

    async def read_gatt_char(self, uuid):
        assert uuid == b.STATS
        return b.frame(b.STATISTICS, struct.pack(">15I", *(self.info[key] for key in b.FIELDS)))


class TrialTests(unittest.IsolatedAsyncioTestCase):
    async def trial(self, client, rate=20, response=False):
        args = argparse.Namespace(duration=0.2, drain=0.01, write_response=response)
        # Test scheduling deterministically; OS sleep jitter is measured on hardware,
        # not asserted by a unit test running on a loaded development computer.
        clock = SimpleNamespace(now=10.0)
        async def sleep(seconds):
            clock.now += max(0, seconds)
            await asyncio.sleep(0)
        with contextlib.redirect_stdout(io.StringIO()), \
                patch.object(b, "time", SimpleNamespace(monotonic=lambda: clock.now)), \
                patch.object(b, "asyncio", SimpleNamespace(sleep=sleep, wait_for=asyncio.wait_for)):
            return await b.run_trial(client, args, rate)

    async def test_full_roundtrip_can_pass(self):
        client = FakeClient()
        result = await self.trial(client)
        self.assertEqual(result["result"], "PASS")
        self.assertEqual(result["received_sequences"], [1, 2, 3, 4])
        self.assertEqual(result["roundtrip_loss_percent"], 0)
        self.assertEqual(client.data_modes, [False] * 4)

    async def test_exact_missing_sequence_and_forward_loss(self):
        result = await self.trial(FakeClient(drop=2))
        self.assertEqual(result["result"], "FAIL")
        self.assertEqual(result["missing_echo_sequences"], [2])
        self.assertEqual(result["not_seen_by_nano"], 1)
        self.assertEqual(result["nano_to_teensy_missing"], 0)
        self.assertEqual(result["teensy_to_host_missing"], 0)
        self.assertEqual(result["roundtrip_loss_percent"], 25)

    async def test_slow_sender_is_not_mistaken_for_passing_requested_rate(self):
        result = await self.trial(FakeClient(slow=0.07), rate=50)
        self.assertEqual(result["result"], "FAIL")
        self.assertGreater(result["host_unsent_slots"], 0)
        self.assertFalse(result["target_met"])
        self.assertEqual(result["roundtrip_loss_percent"], 0)

    async def test_timeout_is_unknown_even_if_echo_arrived(self):
        result = await self.trial(FakeClient(write_error=2))
        self.assertEqual(result["result"], "INCONCLUSIVE")
        self.assertEqual(result["write_errors"], 1)
        self.assertEqual(result["missing_echo_count"], 0)
        self.assertIsNone(result["roundtrip_loss_percent"])

    async def test_missing_final_statistics_cannot_pass(self):
        result = await self.trial(FakeClient(missing_stats=True))
        self.assertEqual(result["result"], "INCONCLUSIVE")
        self.assertIsNone(result["receiver_statistics"])

    async def test_response_mode_preserved(self):
        client = FakeClient()
        await self.trial(client, response=True)
        self.assertTrue(all(client.data_modes))

    def test_callback_counts_duplicates_corruption_and_unexpected_ids(self):
        receiver = b.Receiver()
        receiver.active = True
        receiver.sent[1] = b.time.monotonic()
        raw = b.frame(b.ECHO, struct.pack(">IB", 1, 1))
        receiver.notification(None, raw)
        receiver.notification(None, raw)
        receiver.notification(None, b"bad")
        receiver.notification(None, b.frame(b.ECHO, struct.pack(">IB", 99, 1)))
        self.assertEqual(len(receiver.echoes), 1)
        self.assertEqual((receiver.duplicates, receiver.invalid, receiver.unexpected), (1, 1, 1))

    def test_inconsistent_receiver_counts_cannot_produce_negative_loss(self):
        receiver = b.Receiver()
        now = b.time.monotonic()
        receiver.sent = {1: now}
        info = dict.fromkeys(b.FIELDS, 0)
        info.update(ready=1, nano_received=10, teensy_received=10)
        result = b.summarize(receiver, info, 1, 1, 1, 1, 0, None, now)
        self.assertEqual(result["result"], "INCONCLUSIVE")
        self.assertIsNone(result["roundtrip_loss_percent"])

    def test_backlog_drained_after_test_is_not_sustained_receive_rate(self):
        receiver = b.Receiver()
        receiver.sent = {i: i * 0.01 for i in range(1, 101)}
        receiver.echoes = {i: 2 + i * 0.01 for i in range(1, 101)}
        info = dict.fromkeys(b.FIELDS, 0)
        info.update(ready=1, nano_received=100, teensy_received=100, echo_attempts=100)
        result = b.summarize(receiver, info, 100, 1, 1, 100, 0, None, 0)
        self.assertEqual(result["roundtrip_loss_percent"], 0)
        self.assertEqual(result["result"], "FAIL")
        self.assertTrue(result["tx_target_met"])
        self.assertFalse(result["rx_target_met"])


if __name__ == "__main__":
    unittest.main()
