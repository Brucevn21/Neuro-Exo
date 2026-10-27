"""Bluetooth and simulated devices; all operations run on one asyncio loop."""

import asyncio
import math
import time

from protocol import (COMMAND_UUID, TELEMETRY_UUID, SERVICE_UUID, STOP_FRAME,
                      Command, Mode, Status, Telemetry)


class BluetoothDevice:
    source = "bluetooth"

    def __init__(self, on_sample, on_disconnect):
        self.on_sample = on_sample
        self.on_disconnect = on_disconnect
        self.client = None

    async def connect(self, identifier=""):
        try:
            from bleak import BleakClient, BleakScanner
        except ImportError as exc:
            raise RuntimeError("Bluetooth needs bleak: pip install -r requirements.txt") from exc
        if identifier.strip():
            device = await BleakScanner.find_device_by_address(
                identifier.strip(), timeout=12, service_uuids=[SERVICE_UUID])
        else:
            device = await BleakScanner.find_device_by_name(
                "Nano33BLE_Master", timeout=12, service_uuids=[SERVICE_UUID])
        if device is None:
            raise RuntimeError("Nano33BLE_Master not found; check power and Bluetooth")
        self.client = BleakClient(device, disconnected_callback=lambda _: self.on_disconnect())
        try:
            await self.client.connect()
            await self.client.start_notify(TELEMETRY_UUID, self._notification)
        except BaseException:
            await self.disconnect()
            raise

    def _notification(self, _characteristic, data):
        try:
            sample = Telemetry.decode(data)
        except ValueError:
            return  # Invalid data must never acknowledge motion or get logged.
        self.on_sample(sample)

    async def start(self, command):
        await asyncio.wait_for(self.client.write_gatt_char(COMMAND_UUID, command.encode(), response=True), timeout=2)

    async def stop(self):
        if self.client is not None and self.client.is_connected:
            await asyncio.wait_for(self.client.write_gatt_char(COMMAND_UUID, STOP_FRAME, response=True), timeout=2)

    async def disconnect(self):
        if self.client is not None and self.client.is_connected:
            await asyncio.wait_for(self.client.disconnect(), timeout=3)


class SimulatedDevice:
    """Deterministic motion with the same command/status lifecycle as the arm.

    This models the interface, not the dynamics or efficacy of assistance.
    Calibration sweeps the selected range twice and is lost on reconnect.
    """

    source = "simulation"

    def __init__(self, on_sample, on_disconnect):
        self.on_sample = on_sample
        self.on_disconnect = on_disconnect
        self.angle = 0.0
        self.status = Status(0)
        self.command = None
        self.waypoints = []
        self.calibration_range = None
        self.task = None
        self.started = time.monotonic()

    async def connect(self, identifier=""):
        self.task = asyncio.create_task(self._run())

    async def start(self, command: Command):
        command.encode()
        if self.status & (Status.ACTIVE | Status.SAFETY_FAULT):
            return
        if command.mode in (Mode.ASSISTIVE, Mode.RESISTIVE):
            bounds = self.calibration_range
            if bounds is None or not bounds[0] <= min(self.angle, command.target) <= max(self.angle, command.target) <= bounds[1]:
                return  # Firmware rejects ranges not covered by calibration.
        if command.mode == Mode.CALIBRATE and abs(command.target - self.angle) < 10:
            return
        self.command = command
        self.status &= Status.CALIBRATED
        self.status |= Status.ACTIVE
        if command.mode == Mode.DEMO:
            self.waypoints = [0.0, 70.0, 0.0]
        elif command.mode == Mode.HOME:
            self.waypoints = [0.0]
        elif command.mode == Mode.CALIBRATE:
            self.calibration_range = None
            self.status &= ~Status.CALIBRATED
            self.waypoints = [float(command.target), self.angle, float(command.target), self.angle]
            self._new_range = (min(self.angle, command.target), max(self.angle, command.target))
        else:
            self.waypoints = [float(command.target)]

    async def stop(self):
        self.waypoints.clear()
        self.status &= Status.CALIBRATED
        self._emit()

    async def disconnect(self):
        if self.task:
            self.task.cancel()
            try:
                await self.task
            except asyncio.CancelledError:
                pass
        self.on_disconnect()

    def _emit(self):
        active = bool(self.status & Status.ACTIVE)
        effort = (0.6 + 0.2 * math.sin(time.monotonic() - self.started)) if active else 0.0
        self.on_sample(Telemetry(self.angle, 0.35 if active else 0.0, self.status, effort))

    async def _run(self):
        previous = time.monotonic()
        while True:
            now = time.monotonic()
            dt, previous = min(now - previous, 0.1), now
            if self.waypoints:
                command = self.command
                if command.mode in (Mode.NEUTRAL, Mode.DEMO, Mode.HOME):
                    speed = (3, 7, 15)[command.setting]
                elif command.mode == Mode.CALIBRATE:
                    speed = 5 if len(self.waypoints) > 2 else 20
                else:
                    speed = 10 if command.mode == Mode.RESISTIVE else 1 + command.setting
                target = self.waypoints[0]
                self.angle += max(-speed * dt, min(speed * dt, target - self.angle))
                if abs(self.angle - target) < 0.01:
                    self.waypoints.pop(0)
                    if not self.waypoints:
                        self.status &= ~Status.ACTIVE
                        if command.mode == Mode.CALIBRATE:
                            self.calibration_range = self._new_range
                            self.status |= Status.CALIBRATED
                        else:
                            self.status |= Status.TARGET_REACHED
            self._emit()
            await asyncio.sleep(0.02)
