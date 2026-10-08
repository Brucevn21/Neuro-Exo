# NeuroExo arm protocol: BeagleBone Black Wireless ↔ Nano 33 BLE

This implements the communication sequence in the supplied NeuroExo diagram.
It is separate from the existing packet-throughput benchmark.

**The earlier 180C/2A56 angle/I2C firmware cannot decode this protocol. Upload
NanoNeuroExoProtocol first. The supplied receiver simulates arm position and
does not drive a motor, calibrate real hardware, or issue I2C commands.**
It runs on a physical Nano over real BLE, but its position values are simulated.

## What is implemented

- Hardware_Interface.cpp/hpp now use BleTrialClient and BluezGatt for arm commands.
  The class's TCP helpers, Python socket bridge, and Classic RFCOMM socket code
  have been removed. main.cpp's visible cleanup call uses disconnectBluetooth().
- BluezGatt uses the official BlueZ GATT D-Bus API through GIO. BlueZ must be
  running on the BBB. Linking libbluetooth alone would not implement this GATT
  client. The old code already used BlueZ, but through Classic RFCOMM.
- read_outputs.py has protocol-demo, protocol-decode, protocol-scan, and
  protocol-run modes, with decoded TX/RX, sequence numbers, optional hex, and
  optional JSONL traces. Existing benchmark/legacy reader modes remain available.
- NanoNeuroExoProtocol uses ArduinoBLE's BLEWritten callback and BLE.poll().
  This is a BLE event callback, not a GPIO interrupt service routine.
- The C++ and Python clients use the same 20-byte packet layout and Nano parser.
- The original sensor acquisition remains local to the BBB. Debug EEG text now
  goes to stdout (read_outputs.py stdin can label it), not to the arm characteristic.
  The acquisition loop still sleeps 4 ms plus work; exact 250 Hz is unverified.

## Diagram mapping

| Diagram stage | BBB sends | Nano replies |
|---|---|---|
| Calibrate arm | CALIBRATE | ACK |
| Set maximum position | SET_MAX | ACK |
| Look at dot / blink | Local application phase; no arm message | None |
| 1. Target, starting position, assistance | CONFIGURE | ACK READY |
| 2. Initial arm position | GET_POSITION | POSITION READY |
| 3. Start command | START | ACK RUNNING |
| 4. Arm-position feedback | GET_POSITION every 40 ms | POSITION for each request |
| 5. End command | END | ACK ENDED |
| Repeat trials | A new nonzero trial ID | Same sequence |

This interprets the earlier requirement as **a BBB request every 40 ms**, with
one Nano position response per request. Configure, Start, and End happen once
per trial. No EEG samples are sent to the Nano by this arm-control protocol.
EEG acquisition and SVM processing may continue on their own application thread.

The polling schedule uses monotonic deadlines. Slow requests cause missed
periods to be counted and skipped; there are no catch-up bursts. This is a
25 Hz target, not a hard real-time guarantee. A successful ATT write is followed
by a matching application reply before the command is considered accepted.
The duration argument controls trial length, not the time to reach the target.

## Try the readable protocol without hardware

From the repository root in WSL, Linux, or another Python environment:

~~~bash
python3 H_Robotics_Files/read_outputs.py protocol-demo --duration 0.16 --hex
~~~

This runs in **virtual time**, with simulated arm position and no Bleak
dependency. It exercises calibration, maximum, configuration, initial position,
Start, four 40 ms position requests, and End. It does not measure performance.

~~~bash
python3 H_Robotics_Files/read_outputs.py protocol-demo \
  --trials 20 --duration 2 --start 0 --target 60 --assistance 30 \
  --log protocol_trace_demo.jsonl
~~~

Every JSONL record includes raw hex and decoded fields. Existing log files are
never overwritten. To decode a single captured 20-byte value:

~~~bash
python3 H_Robotics_Files/read_outputs.py protocol-decode \
  "4e 01 03 00 03 00 01 00 60 ea 00 00 00 00 00 00 30 75 00 00"
~~~

That example is CONFIGURE, sequence 3, trial 1, target 60 degrees, starting
position 0 degrees, and assistance velocity 30 degrees/second.

## Load the matching Nano receiver

1. Open NanoNeuroExoProtocol/NanoNeuroExoProtocol.ino in Arduino IDE. Keep
   trial_protocol.h alongside it.
2. Use Arduino Nano 33 BLE, the Arduino Mbed OS Nano Boards package, and the
   ArduinoBLE library (the project specifies version 1.3.7).
3. Compile and upload. Uploading replaces the existing angle/I2C sketch.
4. Power the board. It advertises **NanoNeuroExo**, not NanoBLEBench.
   The Serial Monitor is optional; it does not print each packet.
5. Disconnect other BLE clients before running either sender.

A PlatformIO project is also supplied in that directory. Target compilation
and upload require the Arduino toolchain/dependencies and have not been run
in this workspace.

## Run Python over the BBB's Bluetooth

Copy H_Robotics_Files to the BBB's project checkout. Do not reuse a laptop's
virtual environment or compiled binaries. Use WSL as an SSH terminal to the
BBB (commonly ssh debian@192.168.7.2 over USB); run the following on the BBB.

From the repository root, with Python 3.9+ and BlueZ 5.55+:

~~~bash
bluetoothctl show
bluetoothctl power on
python3 -m venv H_Robotics_Files/.venv
source H_Robotics_Files/.venv/bin/activate
python3 -m pip install -r H_Robotics_Files/requirements-ble.txt
python3 H_Robotics_Files/read_outputs.py protocol-scan
~~~

Replace the example address with the address printed beside NanoNeuroExo:

~~~bash
python3 H_Robotics_Files/read_outputs.py protocol-run \
  --address AA:BB:CC:DD:EE:FF --trials 1 --duration 2 \
  --interval-ms 40 --start 0 --target 60 --assistance 30 \
  --max-position 90 --hex --log protocol_trace_live.jsonl
~~~

The position remains simulated by the Nano. This tests the actual BLE path
and parsing. Use --trials 20 for the diagram. --baseline-seconds adds a local
pause for the dot/blink phase; it does not collect EEG or display a visual cue.

Run only one sender at a time. The Python prototype requires the simulation
flag; it deliberately rejects an unrecognized or different receiver protocol.
Ctrl+C attempts End and disconnects. Do not automatically retry Start after
an uncertain write. Reconnect and restart the protocol after an error.

## Build and run the new C++ sender

On the BBB, install the compiler and GIO/BlueZ development dependencies.
The C++ build supports CMake 3.13+, including Buster's 3.13.4. On Buster,
the dbus package supplies the dbus-daemon executable. Run CTest from inside
the build directory; the newer --test-dir option is not supported there.

~~~bash
sudo apt-get update
sudo apt-get install bluez libglib2.0-dev pkg-config cmake make g++ dbus
cmake -S H_Robotics_Files -B H_Robotics_Files/build-ble -DCMAKE_BUILD_TYPE=Release
cmake --build H_Robotics_Files/build-ble -j2
(cd H_Robotics_Files/build-ble && ctest --output-on-failure)
~~~

The standalone target does not require the missing EEG/IMU application modules:

~~~bash
H_Robotics_Files/build-ble/neuroexo_ble_trial \
  --address AA:BB:CC:DD:EE:FF --trials 20 --duration 2 \
  --period-ms 40 --start 0 --target 60 --assistance 30 --max-position 90
~~~

It uses the same trial protocol as Python. The displayed a/b/c integers follow
the mapping below. It reports position in degrees and marks simulated replies.

To integrate with the original application, link it to the neuroexo_ble CMake
library (BleTrialClient.cpp, BluezGatt.cpp, GIO, and Threads). The full acquisition
application still requires the missing IMU, channel, FIR filter, H-infinity,
global-variable, calibration, and SVM implementations. The independent BLE
executable is provided so communication can be tested before restoring those.

Example of the new Hardware_Interface API:

~~~cpp
Hardware_Interface hardware;
hardware.setBluetoothDevice("AA:BB:CC:DD:EE:FF");
hardware.calibrateArm();
hardware.setMaxPosition(90.0);
neuroexo::TrialSettings trial{1, 60.0, 0.0, 30.0}; // id, target, start, velocity
auto timing = hardware.runArmTrial(trial, std::chrono::seconds(2));
hardware.disconnectBluetooth();
~~~

runArmTrial blocks the calling arm-control thread and polls every 40 ms.
Run EEG acquisition on its own thread/object lifecycle as appropriate to the
complete application. Do not call arm-control methods concurrently.
Individual configureTrial(), startTrial(), readArmPosition(), and endTrial()
methods are also available for the SVM/therapy state machine.
lastArmPositionDegrees() is an atomic cached position for other threads.

Arm_setup(maxPosition) now takes a caller-supplied positive limit in degrees;
it does not detect a real maximum. The missing calibration/testing modules must
be migrated from their old TCP helper calls to this typed API. main.cpp accepts
NEUROEXO_NANO_ADDRESS for its hardware objects; it does not reconstruct the
missing trial modules automatically.

## Wire specification (proposed version 1)

This is an application protocol we defined from the diagram, not a Bluetooth
SIG standard and not the old I2C packet format. All values are little-endian.
Each complete command or reply occupies one 20-byte characteristic value.

| Byte offset | Size | Field |
|---|---:|---|
| 0 | 1 | Magic 0x4e |
| 1 | 1 | Version 1 |
| 2 | 1 | Message kind |
| 3 | 1 | Flags: 0 for commands; bit 0 means simulated in replies |
| 4 | 2 | Command sequence (uint16) |
| 6 | 2 | Trial ID (uint16); 0 for session-level commands |
| 8 | 4 | a (int32) |
| 12 | 4 | b (int32) |
| 16 | 4 | c (int32) |

| Kind | Name | a | b | c |
|---|---|---|---|---|
| 1 | CALIBRATE | 0 | 0 | 0 |
| 2 | SET_MAX | Maximum position | 0 | 0 |
| 3 | CONFIGURE | Target position | Starting position | Assistance velocity |
| 4 | GET_POSITION | 0 | 0 | 0 |
| 5 | START | 0 | 0 | 0 |
| 6 | END | 0 | 0 | 0 |
| 128 | ACK | Command kind | Status | Receiver state |
| 129 | POSITION | Position | Device milliseconds, uint32 bit pattern | Receiver state |
| 130 | INFO | Scale (1000) | Watchdog milliseconds (2000) | Receiver state |

Position is in milli-degrees and velocity in milli-degrees/second. Thus 60000
means 60 degrees. These are **proposed simulation units**; physical units,
ranges, polarity, homing behavior, and controller conversion remain to be
confirmed. The simulator accepts positions 0..maximum, maximum up to 360 degrees,
and positive velocity up to 360 degrees/second. Those are format bounds, not
validated limits for a real arm.

ACK status: 0 OK, 1 BAD_STATE, 2 RANGE, 3 WRONG_TRIAL, 4 BAD_SEQUENCE,
5 UNSUPPORTED. Receiver state: 0 BOOT, 1 IDLE, 2 READY, 3 RUNNING, 4 ENDED,
5 FAULT. A POSITION response itself acknowledges GET_POSITION.

Sequences begin at 1 on each connection and wrap 65535 → 1. Replies echo both
sequence and trial. An exact retransmission of the most recent command returns
its cached reply without repeating the action. Reusing its sequence with other
bytes or skipping a sequence is rejected. The clients do not automatically
retry commands. Accepted-frame sequence tracking is also advanced for command
rejections, allowing a subsequent valid command or End.

Malformed lengths, magic/version/flags, and sequence zero are ignored by the
Nano and therefore cause a client timeout. These are binary values: do not use
toInt(), text concatenation, or the old 200 ms quiet-time delimiter.

The simulator resets its session on connect/disconnect. During RUNNING, more
than 2000 ms without a successful new command changes it to FAULT. Cached
duplicates do not refresh that timer. End cannot clear FAULT; recalibration or
a new connection is required. This watchdog currently stops simulated motion.

GATT UUIDs (all share suffix -6a2b-4f10-9c31-8b674045a901):

| Prefix | Use | Properties |
|---|---|---|
| b7e20000 | Service | — |
| b7e20001 | Command | Write with response |
| b7e20002 | Application reply | Notify |
| b7e20003 | Protocol information | Read |

Twenty bytes fit the minimum ATT payload. Larger EEG batches belong to a
separate stream; do not put them into these command fields. The existing
benchmark has different UUIDs and framing and cannot use this receiver.

## What remains before real arm control

The earlier Nano sketch only specifies an I2C transaction to address 0x08:
opcode 0x01 followed by a big-endian uint16 angle. It does not define actual
calibration, velocity control, End/stop, or position readback.

The supplied receiver completely parses the proposed BLE protocol but its
CALIBRATE and CONFIGURE operate on virtual state; CONFIGURE places the simulated
arm at the start instantly. A real receiver must replace those operations with
a controller adapter, report readiness only after positioning finishes, return
encoder measurements, and stop the controller on End/disconnect/watchdog.
The BLE message IDs are not automatically the I2C opcodes.

Do not interpret simulated acceptance as successful physical actuation.
No physical arm motion, Nano upload, or live BBB radio timing was tested here.

## Verification and sources

~~~bash
python3 -m unittest discover -s H_Robotics_Files/tests -v
~~~

Python tests send real encoded values into the actual C++ Nano parser, including
all 20 trials, illegal states, bounds, duplicates, sequence wrap, timer wrap,
watchdog faults, and client timeouts. The BlueZ integration test runs the real
C++ D-Bus client against an isolated org.bluez service with the Nano parser.
It covers discovery, characteristic flags, notifications arriving before the
ATT write completes, the trial flow, failures, and cleanup.

The hardware-interface compile/logic check uses clearly marked test substitutes
in tests/hardware_stubs for missing drivers and filters. It verifies compilation,
24-bit conversion, gain selection, and persistence of filter objects; it does
not validate the real amplifier, IMU, filter algorithms, or SVM.

API references:
- [BlueZ GATT API](https://bluez.readthedocs.io/en/latest/gatt-api/)
- [BlueZ Device API](https://bluez.readthedocs.io/en/latest/device-api/)
- [Bleak Linux/BlueZ backend](https://bleak.readthedocs.io/en/latest/backends/linux.html)
- [Bleak client write/notification API](https://bleak.readthedocs.io/en/latest/api/client.html)

To reproduce that isolated hardware-class check, install libeigen3-dev and configure
CMake with -DNEUROEXO_TEST_SENSOR_STUBS=ON, then build and run CTest. Never use
tests/hardware_stubs as production sensor drivers.
