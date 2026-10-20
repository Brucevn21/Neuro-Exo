# BeagleBone motor control and telemetry

Use the updated Nano bridge with the Teensy I2C firmware described in
[NEUROEXO_PROTOCOL.md](NEUROEXO_PROTOCOL.md). The legacy SPI sketches elsewhere
in the repository are incompatible with this path.

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

This sends actual motor controls, with degrees and speed interpreted by your
Teensy controller. All three control fields and a positive duration are required.
The wire supports −32768..32767 degrees; this is an encoding range, not a
mechanical travel limit. The Teensy remains responsible for configured limits.
`neutral` is mode 2. At completion or Ctrl+C, the control tool attempts Neutral.

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
