"""Check the sequence sender against the current firmware codec and fake BLE."""
import argparse
import asyncio
import contextlib
import io
import json
from pathlib import Path
import subprocess
import sys
import tempfile
import types
import unittest
from unittest.mock import patch

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
import neuroexo_motor as motor
import neuroexo_protocol as legacy
import protocol_demo_sequence as demo


class CodecTests(unittest.TestCase):
    def test_actual_firmware_control_and_telemetry(self):
        firmware = ROOT.parent / "NeuroExoFirmware/lib/commProtocol/src"
        source = r'''
#include "commProtocol.h"
#include <cstdio>
using namespace NeuroExoProtocol;
void print(const uint8_t* bytes, unsigned size) {
    for (unsigned i = 0; i < size; ++i) std::printf("%02x", bytes[i]);
    std::puts("");
}
int main() {
    uint8_t bytes[MAX_FRAME_SIZE];
    for (int angle : {0, 1, 20, 32767, -32768, -1}) {
        ControlPacket command;
        command.mode = Mode::Neutral;
        command.setting = 0;
        command.targetAngleDeg = angle;
        print(bytes, encodeControlFrame(command, bytes));
    }
    TelemetryPacket telemetry;
    telemetry.currentAngleDeg = -12;
    telemetry.currentMilliAmps = -345;
    telemetry.status = 64;
    telemetry.patientEffortCentiNm = -67;
    print(bytes, encodeTelemetryFrame(telemetry, bytes));
}
'''
        with tempfile.TemporaryDirectory() as folder:
            cpp, binary = Path(folder) / "reference.cpp", Path(folder) / "reference"
            cpp.write_text("#include <initializer_list>\n" + source)
            subprocess.run(["g++", "-std=c++11", "-Wall", "-Wextra", "-Werror",
                            "-I", str(firmware), str(cpp), str(firmware / "commProtocol.cpp"),
                            "-o", str(binary)], check=True, capture_output=True)
            lines = subprocess.check_output([str(binary)], text=True).splitlines()
        for angle, raw in zip((0, 1, 20, 32767, -32768, -1), lines):
            self.assertEqual(legacy.Control(legacy.Mode.Neutral, legacy.Speed.Slow, angle).encode().hex(), raw)
        decoded = demo.Telemetry.decode(bytes.fromhex(lines[-1]))
        self.assertEqual(decoded, demo.Telemetry(-12, -345, legacy.Status(64), -67))
        self.assertEqual(decoded.encode().hex(), lines[-1])
        bad = bytearray(decoded.encode())
        bad[4] ^= 1
        for raw in (bad, decoded.encode()[:-1], legacy.Telemetry(0, 0, 0).encode()):
            with self.assertRaises(ValueError):
                demo.Telemetry.decode(raw)


class SenderTests(unittest.IsolatedAsyncioTestCase):
    async def test_sequence_mode_selects_current_service_and_increments_ids(self):
        writes = []
        status = legacy.Status(0)

        class FakeClient:
            def __init__(self, *args, **kwargs):
                self.services = self
            def get_characteristic(self, uuid):
                if uuid == demo.COMMAND:
                    return types.SimpleNamespace(properties=["write", "write-without-response"], service_uuid=demo.SERVICE)
                if uuid == demo.TELEMETRY:
                    return types.SimpleNamespace(properties=["notify"], service_uuid=demo.SERVICE)
                return None
            async def __aenter__(self):
                return self
            async def __aexit__(self, *args):
                self.pump.cancel()
                await asyncio.gather(self.pump, return_exceptions=True)
            async def start_notify(self, uuid, callback):
                assert uuid == demo.TELEMETRY
                async def pump():
                    while True:
                        callback(None, demo.Telemetry(0, 0, status, 0).encode())
                        await asyncio.sleep(0.005)
                self.pump = asyncio.create_task(pump())
            async def write_gatt_char(self, uuid, raw, response):
                assert uuid == demo.COMMAND and response is False
                writes.append(legacy.Control.decode(raw).target_angle_deg)

        parser = argparse.ArgumentParser()
        motor.add_modes(parser.add_subparsers(dest="mode"))
        args = parser.parse_args(["ble-control", "--address", "test", "--mode", "neutral",
            "--speed", "slow", "--angle", "0", "--duration", "0.08", "--period-ms", "4",
            "--sequence-test", "--write-without-response"])
        fake_bleak = types.SimpleNamespace(BleakClient=FakeClient, BleakScanner=object)
        log = io.StringIO()
        with patch.dict(sys.modules, {"bleak": fake_bleak}), contextlib.redirect_stdout(io.StringIO()):
            self.assertEqual(await motor.execute(args, log), 0)
            self.assertGreater(len(writes), 1)
            self.assertEqual(writes, list(range(len(writes))))
            records = [json.loads(line) for line in log.getvalue().splitlines()]
            self.assertEqual(sum(record["direction"] == "TX" for record in records), len(writes))
            writes.clear()
            status = legacy.Status.SafetyFault
            with self.assertRaisesRegex(RuntimeError, "fault"):
                await motor.execute(args)
            self.assertEqual(writes, [])

    async def test_bad_telemetry_does_not_refresh_feedback(self):
        link = motor.BleLink(demo.COMMAND, demo.Telemetry)
        link.notification(None, demo.Telemetry(0, 0, 0, 0).encode())
        previous = link.latest
        link.notification(None, b"corrupt")
        self.assertIs(link.latest, previous)
        self.assertEqual(link.invalid_frames, 1)


if __name__ == "__main__":
    unittest.main()
