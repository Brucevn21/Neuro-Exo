"""Shared trial state, durable CSV recording, and background device runtime."""

import asyncio
from collections import deque
from datetime import datetime, timezone
import csv
import json
from pathlib import Path
import threading
import tempfile
import time
import uuid

from devices import BluetoothDevice, SimulatedDevice
from protocol import Command, Mode, Status, end_reason

CSV_COLUMNS = ["elapsed_ms", "host_timestamp_utc", "current_position_deg",
               "current_A", "patient_effort_Nm", "status", "requested_mode",
               "requested_setting", "requested_target_deg", "source"]


class TrialRecorder:
    """Never rotates or deletes data. Flushes every sample, including stop sample."""

    def __init__(self, directory, source, command, started):
        directory = Path(directory)
        directory.mkdir(parents=True, exist_ok=True)
        stamp = datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%S_%fZ")
        self.path = directory / f"trial_{stamp}_{uuid.uuid4().hex[:8]}.csv"
        self.file = self.path.open("x", newline="", encoding="utf-8")
        self.writer = csv.writer(self.file)
        self.writer.writerow(CSV_COLUMNS)
        self.file.flush()
        self.started = started
        self.source = source
        self.command = command
        self.rows = 0

    def append(self, sample, now):
        command = self.command
        self.writer.writerow([
            round((now - self.started) * 1000, 3),
            datetime.now(timezone.utc).isoformat(), round(sample.angle, 3),
            sample.current_a, sample.effort_nm, int(sample.status),
            command.mode.name.lower() if command else "unknown",
            command.setting if command else "", command.target if command else "", self.source,
        ])
        self.file.flush()
        self.rows += 1

    def finish(self, reason):
        self.file.close()
        summary = {"id": self.path.stem, "file": self.path.name,
                   "source": self.source, "samples": self.rows, "result": reason,
                   "mode": self.command.mode.name.lower() if self.command else "unknown"}
        # A sidecar also distinguishes interrupted trials from successful ones.
        self.path.with_suffix(".json").write_text(json.dumps(summary, indent=2), encoding="utf-8")
        return summary


class Controller:
    def __init__(self, log_dir):
        self.log_dir = Path(log_dir)
        self.device = None
        self.connected = False
        self.connecting = False
        self.source = "disconnected"
        self.sample = None
        self.last_sample_at = None
        self.pending = None
        self.pending_at = 0.0
        self.command = None
        self.recorder = None
        self.stop_reason = None
        self.stop_written = False
        self.message = "Connect to the simulator or the Nano 33 BLE."
        self.history = deque(maxlen=1500)
        self.trials = self._load_trials()
        self.stale_handled = False
        self.logging_failed = False
        self.trial_started = None

    def _load_trials(self):
        trials = []
        for path in sorted(self.log_dir.glob("trial_*.csv"), reverse=True):
            try:
                summary = json.loads(path.with_suffix(".json").read_text(encoding="utf-8"))
                if not isinstance(summary, dict) or any(key not in summary for key in ("source", "samples", "result", "mode")):
                    raise ValueError("Invalid summary")
            except (OSError, ValueError):
                try:
                    with path.open(newline="", encoding="utf-8") as handle:
                        reader = csv.DictReader(handle)
                        first = next(reader, {})
                        rows = (1 if first else 0) + sum(1 for _ in reader)
                    summary = {"source": first.get("source", "unknown"),
                               "mode": first.get("requested_mode", "unknown"),
                               "samples": rows, "result": "recovered_unfinished"}
                except (OSError, csv.Error, UnicodeError):
                    continue
            # Use actual on-disk names, never paths supplied by metadata.
            summary.update(id=path.stem, file=path.name)
            trials.append(summary)
        return trials

    async def connect(self, source="simulation", identifier=""):
        if self.connected or self.connecting:
            raise ValueError("Disconnect before selecting another device")
        if source not in ("simulation", "bluetooth"):
            raise ValueError("Unknown device type")
        self.connecting = True
        self.source = source
        self.sample = None
        self.last_sample_at = None
        self.stale_handled = False
        self.message = "Connecting…"
        kind = SimulatedDevice if source == "simulation" else BluetoothDevice
        device = kind(self.on_sample, self.on_disconnect)
        self.device = device
        try:
            await device.connect(identifier)
            self.connected = True
            self.message = "Connected. Ready for a trial." if self.last_sample_at is not None else "Connected; waiting for telemetry."
        except BaseException:
            self.device = None
            self.source = "disconnected"
            raise
        finally:
            self.connecting = False

    async def start(self, mode="assistive", setting=5, target=60):
        command = Command(Mode[mode.upper()], setting, target)
        command.encode()
        now = time.monotonic()
        if (not self.connected or self.last_sample_at is None
                or now - self.last_sample_at > 1.5):
            raise ValueError("Fresh device telemetry is required before starting")
        if self.pending or self.sample.active or self.recorder:
            raise ValueError("End the current trial and wait for stop confirmation")
        if self.sample.status & Status.SAFETY_FAULT:
            raise ValueError("Safety fault latched; send Stop before starting")
        if command.mode in (Mode.ASSISTIVE, Mode.RESISTIVE) and not self.sample.status & Status.CALIBRATED:
            raise ValueError("Calibrate the intended range before an assistance/resistance trial")
        if command.mode == Mode.CALIBRATE and abs(command.target - self.sample.angle) < 10:
            raise ValueError("Calibration needs at least 10 degrees of travel")
        # Catch basic storage failures before commanding motion.
        self.log_dir.mkdir(parents=True, exist_ok=True)
        with tempfile.TemporaryFile(dir=self.log_dir) as probe:
            probe.write(b"NeuroExo recording preflight\n")
            probe.flush()
        self.command = command
        self.pending, self.pending_at = "start", now
        self.stop_reason = None
        self.logging_failed = False
        self.message = "Start sent; waiting for MotionActive confirmation."
        try:
            await self.device.start(command)
        except Exception:
            # A write can fail after delivery: cancel any possible motion.
            await self.stop("start_write_failed")
            raise

    async def stop(self, reason="operator_stop"):
        if not self.connected:
            raise ValueError("No connected device; stop cannot be confirmed")
        # Set intent before awaiting; notifications can arrive during a GATT write.
        self.stop_reason = reason
        self.stop_written = False
        self.pending, self.pending_at = "stop", time.monotonic()
        self.message = "Stop requested; waiting for inactive telemetry."
        await self.device.stop()
        self.stop_written = True

    def on_sample(self, sample):
        now = time.monotonic()
        if self.last_sample_at is None and self.pending is None:
            self.message = "Connected. Ready for a trial."
        self.sample, self.last_sample_at = sample, now
        self.stale_handled = False
        if sample.active:
            if self.pending == "start":
                self.pending = None
            if self.trial_started is None:
                self.history.clear()
                self.trial_started = now
                if not self.logging_failed:
                    try:
                        self.recorder = TrialRecorder(self.log_dir, self.source, self.command, now)
                    except OSError as exc:
                        self._logging_error(exc)
                if not self.logging_failed:
                    self.message = "Trial running."
            self.history.append((round(now - self.trial_started, 3), sample.angle, sample.effort_nm))
        if self.recorder:
            try:
                self.recorder.append(sample, now)
            except OSError as exc:
                self._logging_error(exc)
        if not sample.active:
            if self.recorder:
                terminal = end_reason(sample)
                reason = terminal if terminal in ("safety_fault", "patient_resisting", "trial_timeout", "invalid_command") else self.stop_reason or terminal
                self._finish(reason)
            self.trial_started = None
            if self.pending == "stop" and self.stop_written:
                self.pending = None
                self.command = None
                self.message = "Stop confirmed." if not self.logging_failed else self.message
            elif self.pending != "start":
                self.command = None
            if sample.status & Status.SAFETY_FAULT:
                self.message = "Safety fault latched. Motion stopped; send Stop to clear."

    def _logging_error(self, exc):
        self.logging_failed = True
        self.message = f"Recording failed: {exc}. Requesting stop."
        if self.recorder:
            try:
                self.recorder.file.close()
            except OSError:
                pass
            self.recorder = None
        asyncio.create_task(self._stop_after_logging_error())

    async def _stop_after_logging_error(self):
        error = self.message
        try:
            await self.stop("recording_error")
        except Exception as exc:
            error += f" Stop failed: {exc}"
        self.message = error

    def _finish(self, reason):
        recorder, self.recorder = self.recorder, None
        if recorder:
            try:
                self.trials.insert(0, recorder.finish(reason))
                self.message = f"Trial saved: {reason.replace('_', ' ')}."
            except OSError as exc:
                self.message = f"CSV kept at {recorder.path}; could not save summary: {exc}"

    def on_disconnect(self):
        self.connected = False
        self.pending = None
        self.command = None
        self.trial_started = None
        self._finish("connection_lost_unconfirmed_stop")
        self.message = "Disconnected. Any stop on the device is unconfirmed."

    async def disconnect(self):
        if self.connecting:
            raise ValueError("Connection is still in progress")
        try:
            if self.connected:
                try:
                    await self.stop("disconnected")
                    await asyncio.sleep(0.1)
                finally:
                    await self.device.disconnect()
        finally:
            self.on_disconnect()
            self.device = None

    async def check_timeouts(self):
        now = time.monotonic()
        if self.connected and self.last_sample_at is not None and now - self.last_sample_at > 1.5 and not self.stale_handled:
            self.stale_handled = True
            await self.stop("telemetry_lost")
            self.message = "Telemetry lost. Stop requested; waiting for confirmation."
        if self.pending and now - self.pending_at > 3:
            if self.pending == "start":
                await self.stop("start_unconfirmed")
                self.message = "Start not confirmed. Stop requested; check calibration range and device."
            else:
                await self.disconnect()
                self.message = "Stop not confirmed. Disconnected; check the device."

    def snapshot(self):
        now = time.monotonic()
        age = None if self.last_sample_at is None else now - self.last_sample_at
        fresh = self.connected and age is not None and age <= 1.5
        active = bool(self.sample and self.sample.active)
        status = int(self.sample.status) if self.sample else 0
        return {"connected": self.connected, "connecting": self.connecting,
                "source": self.source, "fresh": fresh, "age_s": age,
                "active": active, "pending": self.pending, "status": status,
                "angle": self.sample.angle if self.sample else 0,
                "current_a": self.sample.current_a if self.sample else 0,
                "effort_nm": self.sample.effort_nm if self.sample else 0,
                "can_start": bool(fresh and not active and not self.pending and not status & Status.SAFETY_FAULT),
                "calibrated": bool(status & Status.CALIBRATED),
                "message": self.message, "history": list(self.history),
                "trials": list(self.trials),
                "target": self.command.target if self.command and self.command.mode not in (Mode.DEMO, Mode.HOME) else (0 if self.command and self.command.mode == Mode.HOME else None),
                "mode": self.command.mode.name.lower() if self.command else None,
                "elapsed_s": now - self.trial_started if self.trial_started is not None else 0,
                "rows": self.recorder.rows if self.recorder else 0}


class Runtime:
    """The device has one owner even when desktop and mobile are open together."""

    def __init__(self, log_dir):
        self.controller = Controller(log_dir)
        self.loop = asyncio.new_event_loop()
        self.lock = threading.Lock()
        self.cached = self.controller.snapshot()
        self.thread = threading.Thread(target=self._run, daemon=True)
        self.thread.start()

    def _run(self):
        asyncio.set_event_loop(self.loop)
        self.loop.create_task(self._tick())
        self.loop.run_forever()
        tasks = asyncio.all_tasks(self.loop)
        for task in tasks:
            task.cancel()
        if tasks:
            self.loop.run_until_complete(asyncio.gather(*tasks, return_exceptions=True))
        self.loop.close()

    async def _tick(self):
        while True:
            try:
                await self.controller.check_timeouts()
            except Exception as exc:
                self.controller.message = f"Device error: {exc}"
            with self.lock:
                self.cached = self.controller.snapshot()
            await asyncio.sleep(0.05)

    def snapshot(self):
        with self.lock:
            return self.cached

    def submit(self, action, **kwargs):
        if action not in ("connect", "start", "stop", "disconnect"):
            raise ValueError("Unknown action")

        async def invoke():
            try:
                await getattr(self.controller, action)(**kwargs)
                return {"ok": True}
            except Exception as exc:
                self.controller.message = str(exc)
                return {"ok": False, "error": str(exc)}

        return asyncio.run_coroutine_threadsafe(invoke(), self.loop)

    def close(self):
        try:
            self.submit("disconnect").result(timeout=8)
        finally:
            self.loop.call_soon_threadsafe(self.loop.stop)
            self.thread.join(timeout=2)
