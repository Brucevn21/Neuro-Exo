"""Protocol tests against the exact firmware codec (firmware/commProtocol)."""
import argparse
import asyncio
import contextlib
import ctypes
import io
import json
import pathlib
import random
import subprocess
import sys
import tempfile
import unittest

ROOT = pathlib.Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "python"))
import neuroexo_protocol as p
import neuroexo_trial as cli

BUILD = None
NATIVE = None

# Frames produced by the firmware for the documented examples.
CONTROL_EXAMPLE = bytes.fromhex("02 10 04 01 02 FF 85 94 03")
TELEMETRY_EXAMPLE = bytes.fromhex("02 11 05 00 2D 04 B0 01 52 03")


def setUpModule():
    global BUILD, NATIVE
    BUILD = tempfile.TemporaryDirectory(prefix="neuroexo-comm-")
    library = pathlib.Path(BUILD.name) / "comm.so"
    subprocess.run(["g++", "-std=c++11", "-Wall", "-Wextra", "-Werror", "-shared", "-fPIC", "-O2",
                    str(ROOT / "tests/comm_bridge.cpp"), "-o", str(library)], check=True)
    NATIVE = ctypes.CDLL(str(library))
    NATIVE.comm_crc8.argtypes = [ctypes.c_char_p, ctypes.c_int]
    NATIVE.comm_parse.argtypes = [ctypes.c_char_p, ctypes.c_int] + [ctypes.POINTER(ctypes.c_int)] * 3


def tearDownModule():
    BUILD.cleanup()


def native_control(mode, speed, target):
    out = ctypes.create_string_buffer(16)
    length = NATIVE.comm_encode_control(mode, speed, target, out)
    return out.raw[:length]


def native_telemetry(angle, milliamps, status):
    out = ctypes.create_string_buffer(16)
    length = NATIVE.comm_encode_telemetry(angle, milliamps, status, out)
    return out.raw[:length]


def native_parse(raw):
    a, b, c = ctypes.c_int(), ctypes.c_int(), ctypes.c_int()
    kind = NATIVE.comm_parse(bytes(raw), len(raw), ctypes.byref(a), ctypes.byref(b), ctypes.byref(c))
    return {1: ("control", a.value, b.value, c.value),
            2: ("telemetry", a.value, b.value, c.value)}.get(kind)


def python_parse(raw):
    try:
        frame = p.decode(raw)
    except ValueError:
        return None
    if isinstance(frame, p.Control):
        return ("control", int(frame.mode), int(frame.speed), frame.target_deg)
    return ("telemetry", frame.angle_deg, frame.milliamps, frame.status)


def args(mode="protocol-demo", *extra):
    parser = argparse.ArgumentParser()
    cli.add_modes(parser.add_subparsers(dest="mode", required=True))
    base = {"protocol-demo": ["--speed", "high"],
            "protocol-run": ["--address", "AA:BB:CC:DD:EE:FF"]}.get(mode, [])
    return parser.parse_args([mode, *base, *extra])


class CodecTests(unittest.TestCase):
    def test_documented_examples_match_firmware(self):
        control = p.Control(p.Mode.ASSISTIVE, p.Speed.HIGH, -123)
        self.assertEqual(control.encode(), CONTROL_EXAMPLE)
        self.assertEqual(native_control(1, 2, -123), CONTROL_EXAMPLE)
        self.assertEqual(p.decode(CONTROL_EXAMPLE), control)
        telemetry = p.Telemetry(45, 1200, int(p.Status.MOTION_ACTIVE))
        self.assertEqual(telemetry.encode(), TELEMETRY_EXAMPLE)
        self.assertEqual(native_telemetry(45, 1200, 1), TELEMETRY_EXAMPLE)
        self.assertEqual(len(CONTROL_EXAMPLE), p.CONTROL_FRAME_SIZE)
        self.assertEqual(len(TELEMETRY_EXAMPLE), p.TELEMETRY_FRAME_SIZE)

    def test_crc_bytes_from_the_pasted_readme_are_rejected_by_firmware(self):
        # The README examples show CRC 0xAA and 0x14; firmware computes 0x94 and 0x52.
        for raw in (bytes.fromhex("02 10 04 01 02 FF 85 AA 03"),
                    bytes.fromhex("02 11 05 00 2D 04 B0 01 14 03")):
            self.assertIsNone(native_parse(raw))
            self.assertIsNone(python_parse(raw))

    def test_crc_and_encoders_match_native_for_random_values(self):
        rng = random.Random(7)
        for _ in range(2000):
            data = bytes(rng.randrange(256) for _ in range(rng.randrange(0, 12)))
            self.assertEqual(p.crc8(data), NATIVE.comm_crc8(data, len(data)))
            mode, speed, target = rng.randrange(3), rng.randrange(3), rng.randrange(-32768, 32768)
            self.assertEqual(p.Control(p.Mode(mode), p.Speed(speed), target).encode(),
                             native_control(mode, speed, target))
            angle, milliamps, status = rng.randrange(-32768, 32768), rng.randrange(-32768, 32768), rng.randrange(256)
            self.assertEqual(p.Telemetry(angle, milliamps, status).encode(),
                             native_telemetry(angle, milliamps, status))

    def test_corrupted_frames_are_rejected_exactly_like_firmware(self):
        rng = random.Random(11)
        accepted = 0
        for _ in range(5000):
            raw = bytearray(rng.choice((CONTROL_EXAMPLE, TELEMETRY_EXAMPLE)))
            for _ in range(rng.randrange(1, 3)):
                raw[rng.randrange(len(raw))] = rng.randrange(256)
            expected = native_parse(raw)
            self.assertEqual(python_parse(raw), expected, raw.hex(" "))
            accepted += expected is not None
        self.assertLess(accepted, 100)  # nearly every corruption must be caught

    def test_out_of_range_and_truncated_frames(self):
        bad_mode = bytearray(CONTROL_EXAMPLE)
        bad_mode[3] = 3
        bad_mode[7] = p.crc8(bad_mode[1:7])
        for raw in (bytes(bad_mode), CONTROL_EXAMPLE[:-1], b""):
            self.assertIsNone(python_parse(raw))
            self.assertIsNone(native_parse(raw))
        # The firmware parser ignores bytes after a complete frame; the BBB
        # accepts only exact frames, matching decodeTelemetry() in C++.
        self.assertIsNotNone(native_parse(CONTROL_EXAMPLE + b"\x00"))
        self.assertIsNone(python_parse(CONTROL_EXAMPLE + b"\x00"))
        with self.assertRaises(ValueError):
            p.Control(p.Mode.NEUTRAL, p.Speed.SLOW, 40000).encode()
        self.assertEqual(p.status_names(0x0B), "MOTION_ACTIVE|COMMAND_TIMEOUT|SAFETY_FAULT")
        self.assertEqual(p.status_names(0x31), "MOTION_ACTIVE|TARGET_REACHED|UNKNOWN(0x20)")


class SimulatedJointTests(unittest.TestCase):
    def command(self, speed=p.Speed.HIGH, target=45):
        return p.Control(p.Mode.ASSISTIVE, speed, target).encode()

    def test_command_timeout_after_250_ms(self):
        joint = p.SimulatedJoint()
        self.assertTrue(p.decode(joint.telemetry(0)).has(p.Status.COMMAND_TIMEOUT))
        joint.receive(self.command(), 1.0)
        self.assertTrue(p.decode(joint.telemetry(1.25)).has(p.Status.MOTION_ACTIVE))
        status = p.decode(joint.telemetry(1.26))
        self.assertTrue(status.has(p.Status.COMMAND_TIMEOUT))
        self.assertFalse(status.has(p.Status.MOTION_ACTIVE))

    def test_identical_frames_are_keepalives_and_new_commands_restart(self):
        joint = p.SimulatedJoint()
        for step in range(11):
            joint.receive(self.command(), step * 0.1)
        self.assertEqual(joint.trajectories, 1)
        self.assertAlmostEqual(joint.angle, 45 * 1.0 / 3.5)  # linear, not restarted
        joint.receive(self.command(target=-10), 1.1)
        self.assertEqual(joint.trajectories, 2)
        self.assertEqual(joint.target, -10)

    def test_target_reached_after_settling_and_holds(self):
        joint = p.SimulatedJoint()
        t = 0.0
        while not p.decode(joint.telemetry(t)).has(p.Status.TARGET_REACHED):
            if t > 5:
                self.fail("TargetReached never set")
            joint.receive(self.command(), t)
            t = round(t + 0.02, 3)
        # Within 2 deg at 3.5 - 2/45*3.5 s, plus 0.2 s settle.
        self.assertAlmostEqual(t, 3.5 - 2 / 45 * 3.5 + 0.2, delta=0.05)
        for _ in range(20):  # keepalives hold the target
            joint.receive(self.command(), t)
            t += 0.1
        status = p.decode(joint.telemetry(t))
        self.assertEqual(status.angle_deg, 45)
        self.assertTrue(status.has(p.Status.MOTION_ACTIVE | p.Status.TARGET_REACHED))

    def test_target_reached_survives_idle_until_next_trajectory(self):
        joint = p.SimulatedJoint(angle_deg=45)
        joint.receive(self.command(), 0)
        idle = p.decode(joint.telemetry(1.0))
        self.assertTrue(idle.has(p.Status.TARGET_REACHED | p.Status.COMMAND_TIMEOUT))
        self.assertFalse(idle.has(p.Status.MOTION_ACTIVE))
        joint.receive(self.command(), 1.0)  # after a timeout, the same frame starts a new move
        self.assertEqual(joint.trajectories, 2)
        self.assertFalse(p.decode(joint.telemetry(1.0)).has(p.Status.TARGET_REACHED))

    def test_safety_fault_latches_until_silence(self):
        joint = p.SimulatedJoint()
        joint.fault_on_motion = True
        joint.receive(self.command(), 0.0)
        self.assertTrue(p.decode(joint.telemetry(0.02)).has(p.Status.SAFETY_FAULT))
        joint.fault_on_motion = False
        joint.receive(self.command(), 0.1)  # ignored while latched
        self.assertFalse(joint.motion)
        self.assertFalse(p.decode(joint.telemetry(0.4)).has(p.Status.SAFETY_FAULT))

    def test_invalid_bytes_never_reach_the_teensy(self):
        joint = p.SimulatedJoint()
        self.assertIsNone(joint.receive(bytes.fromhex("02 10 04 01 02 FF 85 AA 03"), 0))
        self.assertIsNone(joint.receive(TELEMETRY_EXAMPLE, 0))
        self.assertTrue(joint.timed_out)


class SessionTests(unittest.IsolatedAsyncioTestCase):
    async def run_demo(self, link, *extra, stream=None):
        with contextlib.redirect_stdout(io.StringIO()):
            session = cli.Session(link, link, cli.Trace(stream=stream, simulated=True))
            result = await session.run(args("protocol-demo", *extra))
        return session, result

    async def test_demo_runs_to_target_reached_then_idles_and_logs_bytes(self):
        class Recording(cli.DemoLink):
            sends = []
            async def write(self, raw):
                self.sends.append(self.seconds)
                await super().write(raw)
        link = Recording()
        stream = io.StringIO()
        session, result = await self.run_demo(link, "--trials", "2", stream=stream)
        self.assertEqual(result, 0)
        self.assertEqual(link.joint.trajectories, 2)
        self.assertEqual(round(link.joint.angle), 45)
        self.assertTrue(link.joint.timed_out and not link.joint.motion)
        gaps = [b - a for a, b in zip(link.sends, link.sends[1:]) if b - a < 0.3]
        self.assertTrue(gaps and max(gaps) <= 0.1 + 1e-9)
        # Trial 1 runs ~3.6 s; trial 2 starts at the target and only settles.
        self.assertLess(link.sends[-1] - link.sends[0], 3.7 + 1.0 + 0.3 + 0.5)
        records = [json.loads(line) for line in stream.getvalue().splitlines()]
        tx = [row for row in records if row["direction"] == "TX"]
        self.assertEqual(len(tx), session.commands)
        self.assertTrue(all(bytes.fromhex(row["hex"]) == p.Control(p.Mode.ASSISTIVE, p.Speed.HIGH, 45).encode()
                            for row in tx))
        self.assertTrue(all(len(bytes.fromhex(row["hex"])) == 10 for row in records if row["direction"] == "RX"))
        self.assertTrue(all(row["clock"] == "virtual" for row in records))

    async def test_time_limit_stops_keepalives_and_raises(self):
        link = cli.DemoLink()
        with self.assertRaisesRegex(RuntimeError, "target not reached"):
            await self.run_demo(link, "--time-limit", "1")
        self.assertTrue(link.joint.timed_out and not link.joint.reached)
        self.assertLess(link.joint.angle, 45)

    async def test_safety_fault_stops_keepalives(self):
        link = cli.DemoLink()
        link.joint.fault_on_motion = True
        writes = []
        original = link.write
        async def write(raw):
            writes.append(link.seconds)
            await original(raw)
        link.write = write
        with self.assertRaisesRegex(RuntimeError, "safety fault"):
            await self.run_demo(link, "--time-limit", "2")
        self.assertEqual(len(writes), 1)

    async def test_rejected_commands_raise_command_timeout(self):
        class Rejecting(cli.DemoLink):
            async def write(self, raw):
                pass  # the Teensy never sees a valid frame
        with self.assertRaisesRegex(RuntimeError, "CommandTimeout"):
            await self.run_demo(Rejecting(), "--time-limit", "2")

    async def test_silence_raises_and_corrupt_telemetry_is_counted(self):
        class Flaky(cli.DemoLink):
            async def sleep(self, seconds):
                await super().sleep(seconds)
                if self.seconds > 0.3:
                    self.pending = []  # telemetry stops
                elif self.pending:
                    stamp, raw = self.pending[0]
                    self.pending.insert(0, (stamp, raw[:-2] + b"\x00\x03"))
        link = Flaky()
        with contextlib.redirect_stdout(io.StringIO()):
            session = cli.Session(link, link, cli.Trace(simulated=True))
            with self.assertRaisesRegex(RuntimeError, "telemetry stopped"):
                await session.run(args("protocol-demo", "--time-limit", "2"))
        self.assertGreater(session.rejected, 0)

    async def test_overrun_skips_deadlines_without_catchup_burst(self):
        class Slow(cli.DemoLink):
            async def write(self, raw):
                await self.sleep(0.130)
                await super().write(raw)
        session, result = await self.run_demo(Slow(), "--interval-ms", "100")
        self.assertEqual(result, 2)
        self.assertGreater(session.skipped, 0)

    async def test_ble_link_write_type_and_disconnect(self):
        class Client:
            async def write_gatt_char(self, uuid, raw, response):
                self.last = (uuid, bytes(raw), response)
        clock = cli.RealClock()
        for response in (True, False):
            link = cli.BleLink(clock, response=response)
            link.client = Client()
            await link.write(CONTROL_EXAMPLE)
            self.assertEqual(link.client.last, (p.COMMAND, CONTROL_EXAMPLE, response))
        link.notification(None, bytearray(TELEMETRY_EXAMPLE))
        self.assertEqual(link.take()[0][1], TELEMETRY_EXAMPLE)
        link.disconnected(None)
        with self.assertRaises(ConnectionError):
            await link.write(CONTROL_EXAMPLE)


class CliTests(unittest.TestCase):
    def test_validation_and_log_no_overwrite(self):
        for extra in (("--trials", "0"), ("--interval-ms", "200"), ("--interval-ms", "10"),
                      ("--rest", "0.1"), ("--time-limit", "4000"), ("--print-every", "0")):
            with self.assertRaises(ValueError):
                cli.validate(args("protocol-demo", *extra))
        for extra in (("--target", "40000"), ("--mode", "fast"), ("--speed", "turbo")):
            with self.assertRaises(SystemExit), contextlib.redirect_stderr(io.StringIO()):
                args("protocol-demo", *extra)
        with tempfile.TemporaryDirectory() as directory:
            path = pathlib.Path(directory) / "trace.jsonl"
            path.write_text("keep")
            with contextlib.redirect_stderr(io.StringIO()):
                self.assertEqual(cli.run(args("protocol-demo", "--log", str(path))), 1)
            self.assertEqual(path.read_text(), "keep")

    def test_encode_decode_and_mode_option_does_not_replace_subcommand(self):
        parsed = args("protocol-encode", "--mode", "assistive", "--speed", "high", "--target", "-123")
        self.assertEqual(parsed.mode, "protocol-encode")
        out = io.StringIO()
        with contextlib.redirect_stdout(out):
            self.assertEqual(cli.run(parsed), 0)
            self.assertEqual(cli.run(args("protocol-decode", TELEMETRY_EXAMPLE.hex(" "))), 0)
        self.assertEqual(out.getvalue().splitlines(),
                         [CONTROL_EXAMPLE.hex(" "),
                          "TELEMETRY angle=45 deg current=1200 mA status=MOTION_ACTIVE"])

    def test_demo_cli_end_to_end(self):
        out = io.StringIO()
        with contextlib.redirect_stdout(out):
            self.assertEqual(cli.run(args("protocol-demo", "--hex")), 0)
        self.assertIn("TargetReached after", out.getvalue())
        self.assertIn("idle confirmed", out.getvalue())
        self.assertIn(CONTROL_EXAMPLE[:3].hex(" "), out.getvalue())


if __name__ == "__main__":
    unittest.main()
