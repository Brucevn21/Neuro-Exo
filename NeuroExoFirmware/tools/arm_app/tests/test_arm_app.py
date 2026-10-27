"""Run with: python -m unittest discover -s tests -v (from tools/arm_app)."""

import asyncio
import csv
import json
from pathlib import Path
import struct
import tempfile
import time
import unittest
from unittest.mock import AsyncMock, patch
from urllib.error import HTTPError
from urllib.request import Request, urlopen

from controller import Controller, Runtime, TrialRecorder
from devices import SimulatedDevice
from protocol import Command, Mode, Status, STOP_FRAME, Telemetry, frame
from server import MobileServer


class ProtocolTests(unittest.TestCase):
    def test_documented_command_vectors(self):
        for command, expected in [
            (Command(Mode.ASSISTIVE, 5, -123), "02 10 04 01 05 FF 85 82 03"),
            (Command(Mode.HOME, 1, 0), "02 10 04 05 01 00 00 34 03"),
            (Command(Mode.DEMO, 2, 0), "02 10 04 04 02 00 00 9F 03"),
            (Command(Mode.CALIBRATE, 0, -40), "02 10 04 03 00 FF D8 FA 03"),
        ]:
            self.assertEqual(command.encode(), bytes.fromhex(expected))
        self.assertEqual(STOP_FRAME, bytes.fromhex("02 12 00 7D 03"))

    def test_invalid_settings_and_angles_are_rejected(self):
        for command in (Command(Mode.ASSISTIVE, 0, 60), Command(Mode.RESISTIVE, 10, 60),
                        Command(Mode.NEUTRAL, 3, 60), Command(Mode.HOME, 1, 81),
                        Command(Mode.NEUTRAL, 1, -151), Command(Mode.ASSISTIVE, True, 60),
                        Command(Mode.NEUTRAL, 1, 12.5)):
            with self.assertRaises(ValueError):
                command.encode()

    def test_signed_telemetry_units_and_flags(self):
        sample = Telemetry.decode(bytes.fromhex("02 11 07 00 2D 04 B0 41 00 EB 19 03"))
        self.assertEqual((sample.angle, sample.current_a, sample.effort_nm), (45, 1.2, 2.35))
        self.assertTrue(sample.active)
        self.assertTrue(sample.status & Status.CALIBRATED)
        sample = Telemetry.decode(frame(0x11, struct.pack(">hhBh", -120, -250, 128, -345)))
        self.assertEqual((sample.angle, sample.current_a, sample.effort_nm), (-120, -.25, -3.45))

    def test_every_single_byte_corruption_and_truncation_is_rejected(self):
        valid = bytes.fromhex("02 11 07 00 2D 04 B0 41 00 EB 19 03")
        for index in range(len(valid)):
            corrupt = bytearray(valid)
            corrupt[index] ^= 1
            with self.assertRaises(ValueError):
                Telemetry.decode(corrupt)
        for length in range(len(valid)):
            with self.assertRaises(ValueError):
                Telemetry.decode(valid[:length])


class ControllerTests(unittest.IsolatedAsyncioTestCase):
    async def asyncSetUp(self):
        self.directory = tempfile.TemporaryDirectory()
        self.controller = Controller(self.directory.name)
        self.device = AsyncMock()
        self.controller.device = self.device
        self.controller.connected = True
        self.controller.source = "bluetooth"
        self.idle = Telemetry(0, 0, Status(0), 0)
        self.controller.on_sample(self.idle)

    async def asyncTearDown(self):
        self.controller._finish("test_cleanup")
        self.directory.cleanup()

    def active(self, angle=1, extra=Status(0)):
        return Telemetry(angle, .2, Status.ACTIVE | extra, .4)

    async def test_start_waits_for_motion_and_prevents_repeated_start(self):
        await self.controller.start("neutral", 0, 60)
        self.assertEqual(self.controller.pending, "start")
        self.assertIsNone(self.controller.recorder)
        with self.assertRaises(ValueError):
            await self.controller.start("neutral", 0, 30)
        self.controller.on_sample(self.active())
        self.assertIsNone(self.controller.pending)
        self.assertEqual(self.controller.recorder.rows, 1)
        with self.assertRaises(ValueError):
            await self.controller.start("neutral", 0, 30)

    async def test_calibration_fault_and_stale_telemetry_gate_motion(self):
        with self.assertRaisesRegex(ValueError, "Calibrate"):
            await self.controller.start()
        self.controller.on_sample(Telemetry(0, 0, Status.SAFETY_FAULT, 0))
        with self.assertRaisesRegex(ValueError, "Safety fault"):
            await self.controller.start("neutral", 0, 60)
        self.controller.on_sample(self.idle)
        self.controller.last_sample_at -= 2
        with self.assertRaisesRegex(ValueError, "Fresh"):
            await self.controller.start("neutral", 0, 60)
        self.device.start.assert_not_awaited()

    async def test_storage_failure_prevents_start(self):
        with patch("controller.tempfile.TemporaryFile", side_effect=OSError("disk full")):
            with self.assertRaises(OSError):
                await self.controller.start("neutral", 0, 60)
        self.device.start.assert_not_awaited()

    async def test_stop_sends_before_save_and_waits_for_inactive_telemetry(self):
        await self.controller.start("neutral", 0, 60)
        self.controller.on_sample(self.active())
        await self.controller.stop("operator_ended")
        self.device.stop.assert_awaited_once()
        self.assertIsNotNone(self.controller.recorder)
        self.assertEqual(self.controller.trials, [])
        self.controller.on_sample(self.idle)
        trial = self.controller.trials[0]
        self.assertEqual(trial["result"], "operator_ended")
        with (Path(self.directory.name) / trial["file"]).open() as handle:
            rows = list(csv.DictReader(handle))
        self.assertEqual(len(rows), 2)
        self.assertEqual(rows[0]["requested_target_deg"], "60")
        self.assertEqual(rows[-1]["status"], "0")
        self.assertIsNone(self.controller.pending)

    async def test_natural_completion_and_disconnect_keep_data(self):
        await self.controller.start("neutral", 0, 60)
        self.controller.on_sample(self.active(59))
        self.controller.on_sample(Telemetry(60, 0, Status.TARGET_REACHED, 0))
        self.assertEqual(self.controller.trials[0]["result"], "target_reached")
        await self.controller.start("neutral", 0, 0)
        self.controller.on_sample(self.active(58))
        self.controller.on_disconnect()
        self.assertEqual(self.controller.trials[0]["result"], "connection_lost_unconfirmed_stop")
        self.assertEqual(len(list(Path(self.directory.name).glob("*.csv"))), 2)
        restored = Controller(self.directory.name)
        self.assertEqual(len(restored.trials), 2)

    async def test_watchdog_requests_stop_on_lost_telemetry(self):
        await self.controller.start("neutral", 0, 60)
        self.controller.on_sample(self.active())
        self.controller.last_sample_at -= 2
        await self.controller.check_timeouts()
        self.device.stop.assert_awaited_once()
        self.assertEqual(self.controller.pending, "stop")
        self.assertEqual(self.controller.stop_reason, "telemetry_lost")

    async def test_unconfirmed_start_is_cancelled(self):
        await self.controller.start("neutral", 0, 60)
        self.controller.pending_at -= 4
        await self.controller.check_timeouts()
        self.device.stop.assert_awaited_once()
        self.assertEqual(self.controller.pending, "stop")

    async def test_stop_needs_notification_after_write_completion(self):
        async def stop_with_early_notification():
            self.controller.on_sample(self.idle)

        self.device.stop.side_effect = stop_with_early_notification
        await self.controller.stop()
        self.assertEqual(self.controller.pending, "stop")
        self.controller.on_sample(self.idle)
        self.assertIsNone(self.controller.pending)

    async def test_watchdog_rearms_after_telemetry_recovers(self):
        self.controller.last_sample_at -= 2
        await self.controller.check_timeouts()
        self.controller.on_sample(self.idle)
        self.controller.last_sample_at -= 2
        await self.controller.check_timeouts()
        self.assertEqual(self.device.stop.await_count, 2)

    async def test_safety_fault_is_not_hidden_by_operator_end(self):
        await self.controller.start("neutral", 0, 60)
        self.controller.on_sample(self.active())
        await self.controller.stop("operator_ended")
        self.controller.on_sample(Telemetry(1, 0, Status.SAFETY_FAULT, 0))
        self.assertEqual(self.controller.trials[0]["result"], "safety_fault")

    async def test_recording_error_requests_stop(self):
        await self.controller.start("neutral", 0, 60)
        self.controller.on_sample(self.active())
        with patch.object(self.controller.recorder, "append", side_effect=OSError("disk full")):
            self.controller.on_sample(self.active(2))
        await asyncio.sleep(0)
        self.device.stop.assert_awaited_once()
        self.assertTrue(self.controller.logging_failed)
        self.assertIn("Recording failed", self.controller.message)
        self.assertEqual(len(list(Path(self.directory.name).glob("*.csv"))), 1)

    async def test_crash_recovery_preserves_unfinished_trial(self):
        recorder = TrialRecorder(self.directory.name, "simulation", Command(Mode.NEUTRAL, 0, 60), time.monotonic())
        recorder.append(self.active(), time.monotonic())
        recorder.file.close()  # Model process exit before metadata was written.
        restored = Controller(self.directory.name)
        self.assertEqual(restored.trials[0]["result"], "recovered_unfinished")
        self.assertEqual(restored.trials[0]["samples"], 1)


class SimulatorTests(unittest.IsolatedAsyncioTestCase):
    async def test_simulator_status_and_calibration_range(self):
        samples = []
        device = SimulatedDevice(samples.append, lambda: None)
        await device.connect()
        try:
            await device.start(Command(Mode.ASSISTIVE, 5, 60))
            self.assertFalse(device.status & Status.ACTIVE)
            await device.start(Command(Mode.CALIBRATE, 0, 60))
            self.assertEqual(device.waypoints, [60, 0, 60, 0])
            await device.stop()
            self.assertIsNone(device.calibration_range)
            device.calibration_range = (0, 60)
            device.status |= Status.CALIBRATED
            await device.start(Command(Mode.ASSISTIVE, 9, 60))
            self.assertTrue(device.status & Status.ACTIVE)
            await asyncio.sleep(.03)
            self.assertGreater(device.angle, 0)
            await device.stop()
            self.assertFalse(samples[-1].active)
            await device.start(Command(Mode.ASSISTIVE, 5, -40))
            self.assertFalse(device.status & Status.ACTIVE)
            await device.start(Command(Mode.DEMO, 0, 0))
            self.assertEqual(device.waypoints, [0, 70, 0])
        finally:
            await device.disconnect()


class MobileIntegrationTests(unittest.TestCase):
    def setUp(self):
        self.directory = tempfile.TemporaryDirectory()
        self.runtime = Runtime(self.directory.name)
        self.server = MobileServer(("127.0.0.1", 0), self.runtime)
        self.thread = self.server.start()
        self.base = f"http://127.0.0.1:{self.server.server_address[1]}"

    def tearDown(self):
        self.server.shutdown()
        self.server.server_close()
        self.thread.join(timeout=2)
        self.runtime.close()
        self.directory.cleanup()

    def request(self, path, data=None, authenticated=True):
        headers = {"Authorization": f"Bearer {self.server.token}"} if authenticated else {}
        if data is not None:
            headers["Content-Type"] = "application/json"
        request = Request(self.base + path, data=json.dumps(data).encode() if data is not None else None, headers=headers)
        return urlopen(request, timeout=3)

    def wait_for(self, predicate):
        deadline = time.monotonic() + 3
        while time.monotonic() < deadline:
            state = self.runtime.snapshot()
            if predicate(state):
                return state
            time.sleep(.02)
        self.fail(f"Timed out; last state: {state}")

    def test_authentication_validation_and_static_assets(self):
        for path in ("/", "/style.css", "/app.js"):
            with self.request(path, authenticated=False) as response:
                self.assertEqual(response.status, 200)
                self.assertIn("frame-ancestors 'none'", response.headers["Content-Security-Policy"])
        with self.assertRaises(HTTPError) as error:
            self.request("/api/state", authenticated=False)
        self.assertEqual(error.exception.code, 401)
        error.exception.close()
        with self.assertRaises(HTTPError) as error:
            self.request("/api/action", {"action": "start", "mode": "neutral", "setting": True, "target": 60})
        self.assertEqual(error.exception.code, 400)
        error.exception.close()
        with self.assertRaises(HTTPError) as error:
            self.request("/api/trials/../../protocol.py")
        self.assertEqual(error.exception.code, 404)
        error.exception.close()

    def test_mobile_simulation_start_stop_and_csv_export(self):
        with self.request("/api/action", {"action": "connect", "source": "simulation"}) as response:
            self.assertEqual(response.status, 202)
        self.wait_for(lambda s: s["fresh"])
        self.request("/api/action", {"action": "start", "mode": "neutral", "setting": 2, "target": 60}).close()
        self.wait_for(lambda s: s["active"] and s["rows"] > 1)
        self.request("/api/action", {"action": "stop", "reason": "operator_ended"}).close()
        state = self.wait_for(lambda s: bool(s["trials"]))
        trial = state["trials"][0]
        self.assertEqual(trial["result"], "operator_ended")
        with self.request(f"/api/trials/{trial['id']}") as response:
            self.assertEqual(response.headers.get_content_type(), "text/csv")
            self.assertIn("attachment", response.headers["Content-Disposition"])
            rows = list(csv.DictReader(response.read().decode().splitlines()))
        self.assertGreater(len(rows), 1)
        self.assertEqual(rows[0]["source"], "simulation")
        self.assertEqual(rows[0]["requested_mode"], "neutral")
        self.assertEqual(rows[-1]["status"], "0")


if __name__ == "__main__":
    unittest.main()
