"""Cross-language protocol tests using the exact Nano receiver core."""
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
sys.path.insert(0, str(ROOT))
import neuroexo_protocol as p
import neuroexo_trial as cli

BUILD = None
NATIVE = None


def setUpModule():
    global BUILD, NATIVE
    BUILD = tempfile.TemporaryDirectory(prefix="neuroexo-trial-")
    library = pathlib.Path(BUILD.name) / "trial.so"
    subprocess.run(["g++", "-std=c++11", "-Wall", "-Wextra", "-Werror", "-shared",
                    "-fPIC", "-O2", str(ROOT / "tests/trial_bridge.cpp"), "-o", str(library)],
                   check=True)
    NATIVE = ctypes.CDLL(str(library))
    NATIVE.trial_receive.argtypes = [ctypes.c_void_p, ctypes.c_size_t, ctypes.c_uint32, ctypes.c_void_p]
    NATIVE.trial_receive.restype = ctypes.c_int
    NATIVE.trial_tick.argtypes = [ctypes.c_uint32]


def tearDownModule():
    BUILD.cleanup()


def args(*extra):
    parser = argparse.ArgumentParser()
    cli.add_modes(parser.add_subparsers(dest="mode", required=True))
    return parser.parse_args(["protocol-demo", "--duration", "0.16", *extra])


class ProtocolTests(unittest.TestCase):
    def setUp(self):
        self.model = p.SimulatedNano()
        NATIVE.trial_reset()
        self.seq = 0

    def exchange(self, kind, trial=0, a=0, b=0, c=0, now=0):
        self.seq = self.seq % 65535 + 1
        raw = p.Frame(kind, self.seq, trial, a, b, c).encode()
        self.assertEqual(len(raw), 20)
        return self.compare(raw, now)

    def compare(self, raw, now):
        out = ctypes.create_string_buffer(20)
        self.assertEqual(NATIVE.trial_receive(raw, len(raw), now, out), 1)
        self.assertEqual(out.raw, self.model.process(raw, now))
        self.assertEqual(NATIVE.trial_state(), self.model.state)
        self.assertEqual(NATIVE.trial_position(), self.model.position)
        return p.Frame.decode(out.raw)

    def setup_trial(self, now=0, start=0, target=60000):
        self.exchange(p.Kind.CALIBRATE, now=now)
        self.exchange(p.Kind.SET_MAX, a=90000, now=now)
        self.exchange(p.Kind.CONFIGURE, 1, target, start, 30000, now=now)

    def test_full_twenty_trial_diagram_and_40ms_feedback(self):
        self.exchange(p.Kind.CALIBRATE)
        self.exchange(p.Kind.SET_MAX, a=90000)
        now = 0
        for trial in range(1, 21):
            self.assertEqual(self.exchange(p.Kind.CONFIGURE, trial, 60000, 0, 30000, now).c,
                             p.State.READY)
            self.assertEqual(self.exchange(p.Kind.GET_POSITION, trial, now=now).a, 0)
            self.exchange(p.Kind.START, trial, now=now)
            for sample in range(1, 11):
                now += 40
                reply = self.exchange(p.Kind.GET_POSITION, trial, now=now)
                self.assertEqual(reply.a, sample * 1200)
                self.assertEqual(reply.c, p.State.RUNNING)
            self.assertEqual(self.exchange(p.Kind.END, trial, now=now).c, p.State.ENDED)
            now += 100

    def test_wrong_state_trial_range_and_unused_fields(self):
        self.assertEqual(self.exchange(p.Kind.START, 0).b, p.Status.BAD_STATE)
        self.setup_trial()
        self.assertEqual(self.exchange(p.Kind.START, 2).b, p.Status.WRONG_TRIAL)
        self.assertEqual(self.exchange(p.Kind.GET_POSITION, 1, a=1).b, p.Status.RANGE)
        self.exchange(p.Kind.START, 1)
        self.assertEqual(self.exchange(p.Kind.CALIBRATE).b, p.Status.BAD_STATE)
        self.exchange(p.Kind.END, 1)
        self.assertEqual(self.exchange(p.Kind.CONFIGURE, 2, 91000, 0, 30000).b, p.Status.RANGE)

    def test_duplicate_start_is_idempotent_and_conflict_rejected(self):
        self.setup_trial()
        self.exchange(p.Kind.START, 1)
        raw = p.Frame(p.Kind.START, self.seq, 1).encode()
        self.assertEqual(self.compare(raw, 40).c, p.State.RUNNING)
        conflict = p.Frame(p.Kind.END, self.seq, 1).encode()
        self.assertEqual(self.compare(conflict, 50).b, p.Status.BAD_SEQUENCE)
        self.exchange(p.Kind.END, 1, now=60)

    def test_watchdog_disconnect_semantics_and_timestamp_wrap(self):
        now = 0xFFFFFFF0
        self.setup_trial(now)
        self.exchange(p.Kind.START, 1, now=now)
        reply = self.exchange(p.Kind.GET_POSITION, 1, now=(now + 40) & 0xFFFFFFFF)
        self.assertEqual(reply.a, 1200)
        self.assertEqual(reply.b & 0xFFFFFFFF, 24)
        self.model.tick(2025)
        NATIVE.trial_tick(2025)
        self.assertEqual(self.model.state, p.State.FAULT)
        self.assertEqual(NATIVE.trial_state(), p.State.FAULT)
        self.assertEqual(self.exchange(p.Kind.END, 1, now=2025).c, p.State.FAULT)
        self.exchange(p.Kind.CALIBRATE, now=2025)
        self.assertEqual(self.model.state, p.State.IDLE)

    def test_motion_clamps_at_target_and_can_move_down(self):
        self.setup_trial(start=60000, target=0)
        self.exchange(p.Kind.START, 1)
        for time_ms in range(1000, 5001, 1000):
            reply = self.exchange(p.Kind.GET_POSITION, 1, now=time_ms)
        self.assertEqual(reply.a, 0)
        self.assertEqual(reply.c, p.State.RUNNING)  # Only End ends a trial.

    def test_malformed_values_and_random_commands_match(self):
        for raw in (b"", b"x" * 19, b"x" * 21,
                    p.Frame(p.Kind.START, 0).encode(),
                    p.Frame(p.Kind.START, 1, flags=1).encode()):
            out = ctypes.create_string_buffer(20)
            self.assertEqual(NATIVE.trial_receive(raw, len(raw), 0, out), 0)
            with self.assertRaises(ValueError):
                self.model.process(raw, 0)
        rng = random.Random(31)
        for _ in range(1000):
            self.exchange(rng.randrange(1, 10), rng.randrange(0, 3),
                          rng.randrange(-1, 100000), rng.randrange(-1, 100000),
                          rng.randrange(-1, 100000), now=rng.randrange(0, 1000))

    def test_sequence_wrap_and_missing_command(self):
        self.assertEqual(self.compare(p.Frame(p.Kind.CALIBRATE, 2).encode(), 0).b,
                         p.Status.BAD_SEQUENCE)
        for _ in range(65536):
            self.exchange(p.Kind.GET_POSITION)
        self.assertEqual(self.seq, 1)


class ClientTests(unittest.IsolatedAsyncioTestCase):
    async def test_demo_runs_diagram_and_logs_actual_bytes(self):
        link = cli.DemoLink()
        stream = io.StringIO()
        with contextlib.redirect_stdout(io.StringIO()):
            session = cli.Session(link, link, cli.Trace(link, stream=stream, simulated=True), 1)
            result = await session.run(args("--trials", "2"))
        self.assertEqual(result, 0)
        self.assertEqual(session.polls, 8)
        self.assertEqual(link.nano.state, p.State.ENDED)
        records = [json.loads(line) for line in stream.getvalue().splitlines()]
        self.assertTrue(all(len(bytes.fromhex(row["hex"])) == 20 for row in records))
        self.assertTrue(all(row["clock"] == "virtual" for row in records))
        self.assertEqual(session.requests, session.responses)

    async def test_overrun_skips_deadlines_without_catchup_burst(self):
        class Slow(cli.DemoLink):
            async def exchange(self, raw, timeout):
                self.seconds += 0.060
                return await super().exchange(raw, timeout)
        link = Slow()
        with contextlib.redirect_stdout(io.StringIO()):
            session = cli.Session(link, link, cli.Trace(link), 1)
            result = await session.run(args("--duration", ".4"))
        self.assertEqual(result, 2)
        self.assertGreater(session.skipped, 0)
        self.assertEqual(link.nano.state, p.State.ENDED)

    async def test_exception_attempts_end(self):
        class Broken(cli.DemoLink):
            async def exchange(self, raw, timeout):
                frame = p.Frame.decode(raw)
                if frame.kind == p.Kind.GET_POSITION and self.nano.state == p.State.RUNNING:
                    # Process the request but lose its reply, as on a real connection.
                    await super().exchange(raw, timeout)
                    raise TimeoutError("lost reply")
                return await super().exchange(raw, timeout)
        link = Broken()
        with contextlib.redirect_stdout(io.StringIO()):
            with self.assertRaises(TimeoutError):
                await cli.Session(link, link, cli.Trace(link), 1).run(args())
        self.assertEqual(link.nano.state, p.State.ENDED)

    async def test_ble_early_reply_and_timeout(self):
        class Fake:
            def __init__(self):
                self.nano = p.SimulatedNano()
                self.link = None
                self.drop = False
            async def write_gatt_char(self, uuid, raw, response):
                self.last = (uuid, response)
                reply = self.nano.process(raw, 0)
                if not self.drop:
                    self.link.notification(None, reply)
        fake = Fake()
        link = cli.BleLink(fake)
        fake.link = link
        reply = await link.exchange(p.Frame(p.Kind.CALIBRATE, 1).encode(), .1)
        self.assertEqual(p.Frame.decode(reply).b, p.Status.OK)
        self.assertEqual(fake.last, (p.COMMAND, True))
        fake.drop = True
        with self.assertRaises(asyncio.TimeoutError):
            await link.exchange(p.Frame(p.Kind.SET_MAX, 2, a=90000).encode(), .01)

    def test_cli_validation_and_log_no_overwrite(self):
        for extra in (("--trials", "0"), ("--interval-ms", "1"),
                      ("--target", "100"), ("--assistance", "0")):
            with self.assertRaises(ValueError):
                cli.validate(args(*extra))
        with tempfile.TemporaryDirectory() as directory:
            path = pathlib.Path(directory) / "trace.jsonl"
            path.write_text("keep")
            with contextlib.redirect_stderr(io.StringIO()):
                self.assertEqual(cli.run(args("--log", str(path))), 1)
            self.assertEqual(path.read_text(), "keep")
