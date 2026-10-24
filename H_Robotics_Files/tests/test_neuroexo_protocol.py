"""Wire compatibility and independent BLE TX/RX behavior; no hardware needed."""
import argparse
import asyncio
import contextlib
import ctypes
import io
from pathlib import Path
import random
import subprocess
import sys
import tempfile
import time
import unittest

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
import neuroexo_protocol as p
import neuroexo_motor as cli


class CodecTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.build = tempfile.TemporaryDirectory(prefix="neuroexo-protocol-")
        library = Path(cls.build.name) / "protocol.so"
        subprocess.run(["g++", "-std=c++17", "-Wall", "-Wextra", "-Werror", "-shared", "-fPIC",
                        str(ROOT / "tests/protocol_bridge.cpp"),
                        str(ROOT.parent / "NeuroExoFirmware/src/commProtocol.cpp"), "-o", str(library)], check=True)
        cls.native = ctypes.CDLL(str(library))
        for prefix in ("control", "telemetry"):
            getattr(cls.native, prefix + "_decode").argtypes = [ctypes.c_void_p, ctypes.c_size_t, ctypes.c_void_p]

    @classmethod
    def tearDownClass(cls):
        cls.build.cleanup()

    def compare(self, frame):
        raw = frame.encode()
        out = ctypes.create_string_buffer(len(raw))
        values = (ctypes.c_int * 3)()
        if isinstance(frame, p.Control):
            self.assertEqual(self.native.control_encode(int(frame.mode), int(frame.speed), frame.target_angle_deg, out), 1)
            self.assertEqual(self.native.control_decode(raw, len(raw), values), 1)
            self.assertEqual(list(values), [frame.mode, frame.speed, frame.target_angle_deg])
        else:
            self.native.telemetry_encode(frame.current_angle_deg, frame.current_milliamps, int(frame.status), out)
            self.assertEqual(self.native.telemetry_decode(raw, len(raw), values), 1)
            self.assertEqual(list(values), [frame.current_angle_deg, frame.current_milliamps, frame.status])
        self.assertEqual(raw, out.raw)
        self.assertEqual(p.decode(raw), frame)

    def test_crc_and_reference_examples(self):
        self.assertEqual(p.crc8(b"123456789"), 0xf4)
        control = p.Control(p.Mode.Assistive, p.Speed.High, -123)
        telemetry = p.Telemetry(45, 1200, p.Status.MotionActive)
        # README originally used AA and 14: these contradict the supplied C++ CRC.
        self.assertEqual(control.encode().hex(), "0210040102ff859403")
        self.assertEqual(telemetry.encode().hex(), "021105002d04b0015203")
        self.compare(control)
        self.compare(telemetry)

    def test_signed_boundaries_and_all_flags(self):
        for number in (-32768, -123, -1, 0, 2, 3, 45, 32767):
            for mode in p.Mode:
                for speed in p.Speed:
                    self.compare(p.Control(mode, speed, number))
            for status in range(256):
                self.compare(p.Telemetry(number, number, p.Status(status)))

    def test_random_wire_parity(self):
        rng = random.Random(42)
        for _ in range(1000):
            self.compare(p.Control(p.Mode(rng.randrange(3)), p.Speed(rng.randrange(3)), rng.randint(-32768, 32767)))
            self.compare(p.Telemetry(rng.randint(-32768, 32767), rng.randint(-32768, 32767), p.Status(rng.randrange(256))))

    def test_corruption_truncation_and_oversize(self):
        for frame, native in ((p.Control(p.Mode.Neutral, p.Speed.Slow, -123), self.native.control_decode),
                              (p.Telemetry(-123, -1200, p.Status(15)), self.native.telemetry_decode)):
            raw = frame.encode()
            bad = [raw[:i] for i in range(len(raw))] + [raw + b"\0", raw * 2]
            for index in range(len(raw)):
                for bit in range(8):
                    mutated = bytearray(raw)
                    mutated[index] ^= 1 << bit
                    bad.append(bytes(mutated))
            for value in bad:
                with self.assertRaises(ValueError):
                    type(frame).decode(value)
                self.assertEqual(native(value, len(value), (ctypes.c_int * 3)()), 0)

    def test_invalid_mode_speed_with_valid_crc(self):
        for index in (3, 4):
            raw = bytearray(p.Control(p.Mode.Assistive, p.Speed.High, 45).encode())
            raw[index] = 3
            raw[-2] = p.crc8(raw[1:-2])
            with self.assertRaises(ValueError):
                p.Control.decode(raw)
            self.assertEqual(self.native.control_decode(bytes(raw), len(raw), (ctypes.c_int * 3)()), 0)
        for angle in (-32769, 32768, 1.5):
            with self.assertRaises(ValueError):
                p.Control(p.Mode.Neutral, p.Speed.Slow, angle).encode()


class FakeClient:
    def __init__(self):
        self.writes = []

    async def write_gatt_char(self, uuid, raw, response):
        assert uuid == p.COMMAND and response is True
        self.writes.append(p.Control.decode(raw))


def args(mode, duration=0.12):
    return argparse.Namespace(mode=mode, duration=duration, telemetry_timeout=0.5,
                              motor_mode="assistive", speed="high", angle=-123, period_ms=20)


class LinkTests(unittest.IsolatedAsyncioTestCase):
    async def asyncSetUp(self):
        self.link = cli.BleLink()
        self.link.client = FakeClient()
        self.trace = cli.Trace(stream=io.StringIO())

    def feedback(self, status=p.Status.MotionActive, angle=45):
        self.link.notification(None, p.Telemetry(angle, -1200, status).encode())

    async def pump(self):
        while True:
            self.feedback()
            await asyncio.sleep(0.01)

    async def test_telemetry_without_any_command(self):
        self.feedback()
        update = await self.link.receive(0.05)
        self.assertEqual(update[1].current_milliamps, -1200)
        self.assertEqual(self.link.client.writes, [])

    async def test_tx_does_not_wait_for_telemetry_reply(self):
        await self.link.send(p.Control(p.Mode.Assistive, p.Speed.High, -123))
        self.assertEqual(len(self.link.client.writes), 1)
        self.assertTrue(self.link.queue.empty())

    async def test_corruption_does_not_replace_latest_or_refresh_age(self):
        self.feedback()
        before = self.link.latest
        self.link.notification(None, b"bad")
        self.assertIs(self.link.latest, before)
        self.assertEqual(self.link.invalid_frames, 1)
        await self.link.receive(0.05)
        with self.assertRaises(TimeoutError):
            await self.link.receive(0.01)

    async def test_bounded_queue_keeps_newest(self):
        for angle in range(1000):
            self.feedback(angle=angle)
        self.assertEqual(self.link.queue.qsize(), 1)
        self.assertEqual((await self.link.receive(0.05))[1].current_angle_deg, 999)

    async def test_disconnect_wakes_waiter_and_discards_cache(self):
        waiter = asyncio.create_task(self.link.receive(1))
        await asyncio.sleep(0)
        self.link.disconnected(None)
        with self.assertRaises(ConnectionError):
            await waiter
        self.assertIsNone(self.link.latest)

    async def test_monitor_never_writes(self):
        pump = asyncio.create_task(self.pump())
        try:
            with contextlib.redirect_stdout(io.StringIO()):
                await cli.run_connected(self.link, args("ble-monitor"), self.trace)
            self.assertEqual(self.link.client.writes, [])
        finally:
            pump.cancel()
            await asyncio.gather(pump, return_exceptions=True)

    async def test_control_heartbeat_and_neutral_cleanup(self):
        pump = asyncio.create_task(self.pump())
        try:
            with contextlib.redirect_stdout(io.StringIO()):
                await cli.run_connected(self.link, args("ble-control"), self.trace)
            self.assertGreaterEqual(len(self.link.client.writes), 3)
            self.assertTrue(all(c.target_angle_deg == -123 for c in self.link.client.writes[:-1]))
            self.assertEqual(self.link.client.writes[-1].mode, p.Mode.Neutral)
        finally:
            pump.cancel()
            await asyncio.gather(pump, return_exceptions=True)

    async def test_safety_fault_stops_heartbeat_without_rearming(self):
        self.feedback()
        async def fault():
            await asyncio.sleep(0.03)
            self.feedback(p.Status.SafetyFault)
        task = asyncio.create_task(fault())
        with contextlib.redirect_stdout(io.StringIO()), self.assertRaises(RuntimeError):
            await cli.run_connected(self.link, args("ble-control", 1), self.trace)
        await task
        count = len(self.link.client.writes)
        await asyncio.sleep(0.04)
        self.assertEqual(len(self.link.client.writes), count)
        self.assertTrue(all(c.mode == p.Mode.Assistive for c in self.link.client.writes))

    async def test_missing_telemetry_sends_nothing(self):
        with self.assertRaises(TimeoutError):
            await cli.run_connected(self.link, args("ble-control"), self.trace)
        self.assertEqual(self.link.client.writes, [])


if __name__ == "__main__":
    unittest.main()
