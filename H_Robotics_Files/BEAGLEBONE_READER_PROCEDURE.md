# BeagleBone motor control and telemetry

For the **250 packets/s BeagleBone/Nano trial using C++ and BlueZ**, start with
[BLUEZ_TRIAL.md](BLUEZ_TRIAL.md). It includes the matching Nano firmware, a source
transfer bundle, fresh-terminal build commands and packet-loss reporting, without Bleak.

For the communication-only 250 Hz send/receive test, use the separate
[packet demo guide](BLE_BENCHMARK.md). It starts from a fresh terminal and covers
PlatformIO activation in WSL/Windows, both firmware uploads, copying the scripts,
SSH from WSL to the BeagleBone, packet-loss reporting and restoration instructions.

Use both firmware environments from [NeuroExoFirmware](../NeuroExoFirmware/README.md),
integrated from `ProtocolDemo`. The Teensy now accepts the framed commands and
returns I2C telemetry described in [NEUROEXO_PROTOCOL.md](NEUROEXO_PROTOCOL.md).
Neutral disables motion. Motion remains disabled until the Teensy's
`ESCON_FULL_SCALE_CURRENT_A` is configured for the actual motor driver.

## Nano

Connect Nano SDA/SCL and common ground to the Teensy's configured I2C bus.
The Nano is master, and Teensy must answer at `0x08`. Build/upload from the
repository root:

```sh
pio run -d NeuroExoFirmware -e nano33ble
pio run -d NeuroExoFirmware -e nano33ble -t upload
```

The Nano entry point is `NeuroExoFirmware/src/nanoBridge.cpp`. The shared codec
lives alongside it in `commProtocol.cpp` and `commProtocol.h`. PlatformIO selects
these files for the `nano33ble` environment and installs ArduinoBLE automatically.
Serial diagnostics use 115200 baud. The bridge does not wait for USB Serial.
Opening its monitor prints a ready message and periodic angle/current/status
readings, or a waiting-for-Teensy message when I2C telemetry is missing/invalid.

Build and upload the Teensy environment separately:

```sh
pio run -d NeuroExoFirmware -e teensy41
pio run -d NeuroExoFirmware -e teensy41 -t upload
```

Uploading is required on both boards after this firmware update. The Teensy
encoder pins from `ProtocolDemo` are DATA=0, CLK=1, CS=2; verify the
[connection map](../NeuroExoFirmware/lib/pinMap/README.md) against the actual wiring.

## Python on the BeagleBone

```sh
cd H_Robotics_Files
python3 -m venv .venv
. .venv/bin/activate
python3 -m pip install -r requirements-ble.txt
bluetoothctl power on
python3 read_outputs.py ble-scan
python3 read_outputs.py ble-monitor --address AA:BB:CC:DD:EE:FF --hex
```

Replace the address with the Nano's scan result. Monitoring only subscribes to
telemetry. It sends no motor commands. Expect output like:

```text
    0.040s RX angle=45 deg current=1200 mA status=0x01 MotionActive=1 CommandTimeout=0 InvalidCommand=0 SafetyFault=0
```

To send controls while displaying the independent telemetry stream:

```sh
python3 read_outputs.py ble-control --address AA:BB:CC:DD:EE:FF \
  --mode assistive --speed slow --angle 30 --duration 5 --period-ms 100 --hex
```

When the Nano sketch has `BLE_PACKET_TEST` enabled, use this diagnostic
command to send distinct packet sequence values over Write Without Response:

For the current ProtocolDemo firmware, follow [the BBB sequence-test update
procedure](SEQUENCE_TEST_BBB.md). The local `--sequence-test` adapter uses the
current BLE UUIDs and 12-byte telemetry. Verify the Nano's test-mode startup
banner before sending: normal firmware can interpret the sequence as an angle.

```sh
python3 read_outputs.py ble-control --address AA:BB:CC:DD:EE:FF \
  --mode neutral --speed slow --angle 0 --duration 10 --period-ms 20 \
  --write-without-response --sequence-test
```

In this firmware mode the Nano counts packets and does not forward them to the
Teensy. `--sequence-test` increments the target angle in each frame because the
Nano uses that field as its test sequence number; a fixed angle is counted as
one unique packet followed by duplicates. Sequence-test frames use Neutral mode
and the run still requires live telemetry for the control client's safety checks.

This sends actual motor controls, with degrees and speed interpreted by your
Teensy controller. All three control fields and a positive duration are required.
The wire supports −32768..32767 degrees; this is an encoding range, not a
mechanical travel limit. The Teensy remains responsible for configured limits.
`neutral` is mode 2 and disables motion. At completion or Ctrl+C, the control
tool attempts Neutral. Identical heartbeat frames keep the watchdog alive
without restarting the trajectory.

Add `--log motor-session.jsonl` to either mode to write a new JSONL file of decoded
values and raw hex. Existing files are not overwritten. `--duration 10` limits a
monitoring session; otherwise it runs until Ctrl+C. The default feedback timeout
is 0.5 seconds (`--telemetry-timeout`). Run one BLE client at a time.

Decode bytes without Bluetooth:

```sh
python3 read_outputs.py protocol-decode '02 10 04 01 02 FF 85 94 03'
python3 read_outputs.py protocol-decode '02 11 05 00 2D 04 B0 01 52 03'
```

## C++ on the BeagleBone

```sh
sudo apt-get install build-essential cmake pkg-config libglib2.0-dev
cmake -S . -B build
cmake --build build
(cd build && ctest --output-on-failure)
./build/neuroexo_motor --address AA:BB:CC:DD:EE:FF
./build/neuroexo_motor --address AA:BB:CC:DD:EE:FF \
  --mode assistive --speed slow --angle 30 --duration 5 --period-ms 100
```

For high-rate testing, add `--write-without-response` to the C++ control
command. The client checks the Nano's GATT flags before using BLE Write Without
Response for periodic controls. Those writes have no ATT delivery confirmation;
the final Neutral stop still requests a response. For a 50 Hz trial, set
`--period-ms 20` and verify command reception and telemetry on the Nano/Teensy
before relying on this mode.

The standalone C++ target requires BlueZ on the system D-Bus and uses the same
two UUIDs as Python. `--adapter hci1` selects a different adapter.

For the EEG application, link `neuroexo_ble` and use:

```cpp
hardware.setBluetoothDevice("AA:BB:CC:DD:EE:FF");
if (!hardware.connectToBluetoothDevice()) throw std::runtime_error("BLE unavailable");
// Call periodically (<250 ms) only while control is intended:
hardware.sendMotorControl(neuroexo::Mode::Assistive, neuroexo::Speed::Slow, 30);
// This consumes a notification; it sends no control or polling packet:
auto update = hardware.readMotorTelemetry(100);
if (update) std::cout << neuroexo::describe(update->value) << '\n';
// Cached feedback throws if missing, stale, or disconnected:
auto current = hardware.latestMotorTelemetry();
hardware.stopMotor();
hardware.disconnectBluetooth();
```

`Arm_setup()` now only connects. The old simulated calibration, maximum-position,
start/end-trial, and assistance-velocity methods are removed because those
commands do not exist in this protocol. Application code using them must migrate
to the explicit mode/speed/target control fields. The copied EEG application
still lacks several original drivers/modules; the standalone client builds
without them. Its position matrix now reads validated telemetry instead of
`gV.POSITION`; arrange for the acquiring interface to own the BLE connection.

## Checks and troubleshooting

```sh
python3 -m unittest discover -s tests -p 'test_*.py' -v
```

Tests compare Python and C++ wire bytes, including signed boundaries, every
status combination, malformed/oversized/truncated frames, independent TX/RX,
fault handling, stale feedback, and disconnects. CTest exercises the actual
BlueZ code against a private fake D-Bus service and the Nano bridge against host
I2C/BLE substitutes. These checks do not measure or validate physical motor behavior.

- No Nano found: verify power, the loaded sketch, and adapter power.
- Service mismatch: load the updated Nano sketch and disconnect other clients.
- Missing telemetry: verify the Teensy is running the I2C firmware, address
  `0x08`, shared ground, and the correct bus. No measurements are synthesized.
- CommandTimeout: monitor mode sends nothing; control mode needs valid frames
  at intervals below 250 ms, including actual BLE/Linux scheduling delays.
- SafetyFault: the Teensy has latched its e-stop. The control tool exits and
  does not resume automatically. Refer to the Teensy controller for the cause.
