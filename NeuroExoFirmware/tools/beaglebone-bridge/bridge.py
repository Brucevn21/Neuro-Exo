#!/usr/bin/env python3
"""
NeuroExo BeagleBone bridge.

Sits between the patient app (Web Bluetooth, tools/patient-app) and the
Nano 33 BLE (Nano33BLEFirmware.ino). It plays two BLE roles at once on the
BeagleBone's Bluetooth adapter:

  - Central toward the Nano 33 BLE: connects to "Nano33BLE_Master" and uses
    its joint service exactly like the old app.js did directly.
  - Peripheral toward the app: advertises its own copy of that same joint
    service (same UUIDs), so tools/patient-app's protocol.js/app.js don't
    need to change what they connect to - just who they connect to.

This is where trial randomization now lives: whenever the app requests a new
trial (a write to the command characteristic), this bridge - not the app -
picks the target angle and substitutes it into the packet forwarded to the
Nano. Telemetry and stop requests are otherwise passed straight through.

Requires: bleak (BLE central) and bless (BLE peripheral / GATT server, via
BlueZ on Linux). Simultaneous central+peripheral operation on one adapter
depends on your Bluetooth hardware and BlueZ version - this has not been
validated against real BeagleBone Black hardware; test on your device before
relying on it in a session.
"""

import asyncio
import logging
import random

from bleak import BleakClient, BleakScanner
from bless import (
    BlessServer,
    BlessGATTCharacteristic,
    GATTAttributePermissions,
    GATTCharacteristicProperties,
)

import protocol

logging.basicConfig(level=logging.INFO, format="%(asctime)s %(levelname)s %(message)s")
log = logging.getLogger("neuroexo-bridge")

NANO_DEVICE_NAME = "Nano33BLE_Master"
NANO_SCAN_TIMEOUT_S = 20.0

TARGET_MIN_DEG = 20
TARGET_MAX_DEG = 120


class Bridge:
    def __init__(self):
        self.nano_client: BleakClient | None = None
        self.app_server: BlessServer | None = None

    # ---------- Nano side (central) ----------

    async def connect_to_nano(self):
        log.info("Scanning for %s...", NANO_DEVICE_NAME)
        device = await BleakScanner.find_device_by_name(NANO_DEVICE_NAME, timeout=NANO_SCAN_TIMEOUT_S)
        if device is None:
            raise RuntimeError(f"Could not find a BLE device advertising as '{NANO_DEVICE_NAME}'")

        self.nano_client = BleakClient(device)
        await self.nano_client.connect()
        log.info("Connected to Nano 33 BLE at %s", device.address)

        await self.nano_client.start_notify(protocol.TELEMETRY_CHAR_UUID, self._on_nano_telemetry)

    def _on_nano_telemetry(self, _characteristic, data: bytearray):
        """Relay telemetry from the Nano straight up to the app, unmodified."""
        if self.app_server is None:
            return
        char = self.app_server.get_characteristic(protocol.TELEMETRY_CHAR_UUID)
        char.value = bytes(data)
        self.app_server.update_value(protocol.SERVICE_UUID, protocol.TELEMETRY_CHAR_UUID)

    # ---------- App side (peripheral) ----------

    async def start_app_server(self):
        server = BlessServer(name="NeuroExo-BBB-Bridge")
        server.write_request_func = self._on_app_write

        await server.add_new_service(protocol.SERVICE_UUID)

        write_props = GATTCharacteristicProperties.write | GATTCharacteristicProperties.write_without_response
        notify_props = GATTCharacteristicProperties.read | GATTCharacteristicProperties.notify
        perms = GATTAttributePermissions.readable | GATTAttributePermissions.writeable

        idle_telemetry = protocol.encode_joint_packet(protocol.Mode.NEUTRAL, protocol.Speed.MEDIUM)

        await server.add_new_characteristic(
            protocol.SERVICE_UUID, protocol.COMMAND_CHAR_UUID, write_props, None, perms
        )
        await server.add_new_characteristic(
            protocol.SERVICE_UUID, protocol.TELEMETRY_CHAR_UUID, notify_props, idle_telemetry, perms
        )
        await server.add_new_characteristic(
            protocol.SERVICE_UUID, protocol.STOP_CHAR_UUID, write_props, None, perms
        )

        await server.start()
        self.app_server = server
        log.info("Advertising NeuroExo joint service to the app.")

    def _on_app_write(self, characteristic: BlessGATTCharacteristic, value: bytearray):
        """
        bless invokes this synchronously (it is not awaited internally), so
        it must stay a plain function - async handling is scheduled as a
        task on the already-running asyncio loop instead of awaited here.
        """
        if characteristic.uuid == protocol.COMMAND_CHAR_UUID:
            asyncio.ensure_future(self._handle_start_request(bytes(value)))
        elif characteristic.uuid == protocol.STOP_CHAR_UUID:
            asyncio.ensure_future(self._handle_stop_request())

    async def _handle_start_request(self, raw: bytes):
        if len(raw) != protocol.PACKET_SIZE:
            log.warning("Ignoring malformed command packet (%d bytes, expected %d)", len(raw), protocol.PACKET_SIZE)
            return

        requested = protocol.decode_joint_packet(raw)
        target = random.randint(TARGET_MIN_DEG, TARGET_MAX_DEG)
        log.info(
            "New trial requested (mode=%d speed=%d) -> bridge-chosen target=%d deg",
            requested["mode"], requested["speed"], target,
        )

        outgoing = protocol.encode_joint_packet(requested["mode"], requested["speed"], target_angle_deg=target)
        await self.nano_client.write_gatt_char(protocol.COMMAND_CHAR_UUID, outgoing, response=False)

    async def _handle_stop_request(self):
        log.info("Stop requested by app - forwarding to Nano.")
        await self.nano_client.write_gatt_char(protocol.STOP_CHAR_UUID, b"\x01", response=False)

    # ---------- lifecycle ----------

    async def shutdown(self):
        if self.nano_client is not None and self.nano_client.is_connected:
            await self.nano_client.disconnect()
        if self.app_server is not None:
            await self.app_server.stop()


async def main():
    bridge = Bridge()
    await bridge.connect_to_nano()
    await bridge.start_app_server()

    log.info("Bridge running. Waiting for the app to connect...")
    try:
        while True:
            await asyncio.sleep(1)
    except (KeyboardInterrupt, asyncio.CancelledError):
        pass
    finally:
        await bridge.shutdown()


if __name__ == "__main__":
    asyncio.run(main())
