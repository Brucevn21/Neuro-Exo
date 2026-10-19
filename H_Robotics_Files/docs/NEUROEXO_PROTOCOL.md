# NeuroExo joint protocol: BeagleBone Black Wireless ↔ Nano 33 BLE ↔ Teensy 4.1

The BeagleBone Black (BBB) is the BLE central. It writes framed control
messages to the Nano 33 BLE. The Nano validates each frame and forwards it
over I2C to the Teensy 4.1 motor controller. It also polls the Teensy for
telemetry and notifies the BBB with each valid frame.

```text
 BeagleBone Black ──BLE write 2A56──► Nano 33 BLE ──I2C 0x08──► Teensy 4.1 ──► ESCON ──► motor
   (BLE central)  ◄─BLE notify 2A57── "Nano33BLE_Master" ◄─I2C read── (PID loop, AS5045 encoder)
```

The Nano and Teensy firmware live in `NeuroExoFirmware/` on the `ProtocolDemo`
branch (`src/Nano33BLEFirmware.ino`, `src/main.cpp`, `lib/commProtocol/`).
This folder contains the BBB side, plus a vendored copy of the shared codec in
[firmware/commProtocol](../firmware/commProtocol/README.md).

**`protocol-run`, `neuroexo_ble_trial` without `--monitor`, `runArmTrial()`,
and `sendArmCommand()` all command real motor motion.**

## BLE interface

| Item | Value |
|---|---|
| Local name | `Nano33BLE_Master` |
| Service | `180C` (`0000180c-0000-1000-8000-00805f9b34fb`) |
| Command | `2A56`: Write and Write Without Response. BBB → Nano control frames, 9 bytes. |
| Telemetry | `2A57`: Read and Notify. Nano → BBB telemetry frames, 10 bytes, 50 Hz while connected. |

The Nano publishes telemetry only after a successful I2C read that validates.
If the Teensy is unpowered or unwired, the BBB connects but receives no
telemetry. Both BBB clients treat that as an error.

## Frame format

```text
STX | TYPE | LENGTH | PAYLOAD | CRC8 | ETX
```

| Field | Size | Description |
|---|---:|---|
| STX | 1 | Start marker, always `0x02` |
| TYPE | 1 | `0x10` control or `0x11` telemetry |
| LENGTH | 1 | Payload length: 4 for control, 5 for telemetry |
| PAYLOAD | 4 or 5 | Message-specific, signed 16-bit values big-endian |
| CRC8 | 1 | CRC-8, polynomial `0x07`, initial value `0`, over TYPE, LENGTH and PAYLOAD |
| ETX | 1 | Stop marker, always `0x03` |

### Control frame (9 bytes, BBB → Nano → Teensy)

| Byte | Field | Values |
|---:|---|---|
| 3 | MODE | `0` Resistive, `1` Assistive, `2` Neutral |
| 4 | SPEED | `0` Slow, `1` Medium, `2` High |
| 5–6 | targetAngleDeg | int16, big-endian, degrees |

Example: Assistive, High, target −123° (`0xFF85`):

```text
02 10 04 01 02 FF 85 94 03
```

### Telemetry frame (10 bytes, Teensy → Nano → BBB)

| Byte | Field | Description |
|---:|---|---|
| 3–4 | currentAngleDeg | int16, big-endian, degrees |
| 5–6 | currentMilliAmps | int16, big-endian, milliamps |
| 7 | status | Bit field below |

| Bit | Name | Meaning |
|---:|---|---|
| 0 | MotionActive | The controller is driving the joint: moving along a trajectory or holding the target |
| 1 | CommandTimeout | No valid control frame for 250 ms; motor disabled (idle) |
| 2 | InvalidCommand | The most recent I2C command was invalid |
| 3 | SafetyFault | Controller e-stop latched (over-speed, overcurrent, invalid sensor data, or tracking error). Commands are ignored until none arrives for 250 ms. |
| 4 | TargetReached | **Proposed firmware addition.** The measured angle has settled within tolerance of the target. Cleared when a new trajectory starts; may stay set while idle. |

> **Firmware status:** bit 4 and the keepalive rules below are what the BBB
> code now expects. The firmware on `ProtocolDemo` (`39eac5b`) does not
> implement them yet. Against that firmware, a trial never sees TargetReached
> and ends at its time limit with an error.

Example: angle 45°, 1200 mA, MotionActive:

```text
02 11 05 00 2D 04 B0 01 52 03
```

> **CRC note:** the protocol README in `NeuroExoFirmware/lib/commProtocol` shows
> these examples with CRC bytes `AA` and `14`. The firmware's `crc8()` computes
> `94` and `52`, and the firmware rejects the README's bytes. The values above
> come from the firmware code, and tests check them against it.

## How the BBB drives a trial

The Teensy's control algorithm owns the motion: the trajectory, the control
loop, and deciding when the target is reached. The BBB only relays a start
signal, keeps it alive, and goes quiet when the Teensy says it's done.

Teensy rules (proposed; see Firmware status above):

- **Start.** A control frame that differs from the active command (mode,
  speed, or target), or any valid frame after a timeout, starts a new
  trajectory from the current angle and clears TargetReached.
- **Keepalive.** An identical frame only refreshes the 250 ms watchdog. It
  never restarts or alters the trajectory.
- **Done.** When the angle settles within tolerance of the target, the Teensy
  sets TargetReached and holds the target while keepalives continue.
- **Idle.** After 250 ms without a valid frame, the Teensy sets
  CommandTimeout, clears MotionActive, and disables the motor (joint
  backdrivable). The watchdog also stops the joint if the BBB or BLE link
  fails mid-move.

BBB sequence (`BleJointClient::runTrial`, `protocol-run`):

1. **Precondition.** Wait up to 1 s for telemetry. Refuse to start if
   SafetyFault is latched.
2. **Start and keepalive.** Write the control frame, then repeat it every
   `period` (default 100 ms, allowed 20–150 ms).
3. **Watch for done.** Stop repeating at the first telemetry frame with
   TargetReached and MotionActive set and CommandTimeout clear. A
   TargetReached left over from the previous move arrives with CommandTimeout
   set, so it is ignored.
4. **Abort.** Stop writing and report an error if any of these occurs:
   - The time limit passes before TargetReached. The default is 150 s, which
     covers Slow's 120 s trajectory.
   - SafetyFault is set.
   - No telemetry arrives for 500 ms.
   - CommandTimeout is still set more than 400 ms into the trial, which means
     the Teensy is not accepting the frames.
   - The BLE link fails.
5. **Confirm idle.** Wait up to 1 s for telemetry showing CommandTimeout
   without MotionActive, received more than 250 ms after the last write.

Writes happen only on the thread that runs the trial, and there is no
background keepalive. If the program crashes, hangs, or throws, the writes stop,
and the Teensy disables motion within 250 ms. Ctrl+C has the same effect.
After a SafetyFault, wait at least 250 ms without writing before the next
trial. The CLIs enforce `--rest ≥ 0.3 s` between trials.

## Try it without hardware

From the repository root:

~~~bash
python3 H_Robotics_Files/python/read_outputs.py protocol-encode --mode assistive --speed high --target -123
python3 H_Robotics_Files/python/read_outputs.py protocol-decode "02 11 05 00 2D 04 B0 01 52 03"
python3 H_Robotics_Files/python/read_outputs.py protocol-demo --speed high --target 45 --hex
~~~

`protocol-demo` runs in **virtual time**. A `SimulatedJoint` model stands in
for the Nano and Teensy and follows the proposed rules above: keepalive,
TargetReached (within 2° for 0.2 s, assumed values), the timeout, and faults.
Its angle follows the setpoint exactly, and its current value is a placeholder.
It does not measure Bluetooth or motor timing. `--print-every N` (default 5)
thins the terminal output, but status changes always print.
`--log trace.jsonl` writes every frame's bytes and decoded fields. Existing
logs are never overwritten.

## Run over the BBB's Bluetooth (Python)

Run this on the BBB from the repository root, with Python 3.9+ and BlueZ 5.55+:

~~~bash
bluetoothctl power on
python3 -m venv H_Robotics_Files/.venv
source H_Robotics_Files/.venv/bin/activate
python3 -m pip install -r H_Robotics_Files/python/requirements-ble.txt
python3 H_Robotics_Files/python/read_outputs.py protocol-scan
~~~

Start with telemetry only. This never writes a command:

~~~bash
python3 H_Robotics_Files/python/read_outputs.py protocol-monitor --address AA:BB:CC:DD:EE:FF --duration 5
~~~

Then, with the joint clear and the e-stop within reach:

~~~bash
python3 H_Robotics_Files/python/read_outputs.py protocol-run \
  --address AA:BB:CC:DD:EE:FF --mode assistive --speed medium --target 45 \
  --time-limit 60 --trials 1 --hex --log protocol_trace_live.jsonl
~~~

`--write-without-response` uses an ATT write command instead of a write
request. Only one central can connect to the Nano at a time, so close the
patient app or any other client first.

## Build and run the C++ client

On the BBB (CMake 3.13+, including Buster's 3.13.4):

~~~bash
sudo apt-get install bluez libglib2.0-dev pkg-config cmake make g++ dbus
cmake -S H_Robotics_Files -B H_Robotics_Files/build-ble -DCMAKE_BUILD_TYPE=Release
cmake --build H_Robotics_Files/build-ble -j2
(cd H_Robotics_Files/build-ble && ctest --output-on-failure)
H_Robotics_Files/build-ble/neuroexo_ble_trial --address AA:BB:CC:DD:EE:FF --monitor --duration 5
H_Robotics_Files/build-ble/neuroexo_ble_trial --address AA:BB:CC:DD:EE:FF \
  --mode assistive --speed medium --target 45 --time-limit 60 --trials 1 --trace
~~~

The `neuroexo_ble` library contains:

- `ble/BleJointClient.cpp`: keepalive, telemetry, and the stop rules.
- `ble/BluezGatt.cpp`: BlueZ GATT access over D-Bus/GIO.
- `firmware/commProtocol/commProtocol.cpp`: the codec.

The Hardware_Interface API:

~~~cpp
Hardware_Interface hardware;
hardware.setBluetoothDevice("AA:BB:CC:DD:EE:FF");   // or NEUROEXO_NANO_ADDRESS in main.cpp
hardware.debugArm();                                 // read-only: prints one telemetry frame
neuroexo::JointCommand command{NeuroExoProtocol::Mode::Assistive,
                               NeuroExoProtocol::Speed::Medium, 45};
// Blocks until TargetReached (or throws after the 60 s limit), then the joint idles.
auto result = hardware.runArmTrial(command, std::chrono::seconds(60),
    [](const neuroexo::JointTelemetry& t) { /* angleDeg, milliAmps, status */ });
hardware.disconnectBluetooth();
~~~

`runArmTrial()` blocks the calling arm-control thread. Do not call arm methods
concurrently. `lastArmPositionDegrees()` is an atomic cached angle that other
threads can read. To issue commands yourself, call `sendArmCommand()` and
repeat it within 250 ms. Read telemetry with `takeArmTelemetry()`.

`Arm_setup()` now only confirms that telemetry arrives, because the protocol
has no calibration or limit command. The removed calls (`calibrateArm`,
`setMaxPosition`, `configureTrial`, `startTrial`, `readArmPosition`,
`endTrial`) have no equivalent. The missing calibration and testing modules
need to move to `runArmTrial()` or `sendArmCommand()`.

## Verification

~~~bash
python3 -m unittest discover -s H_Robotics_Files/tests -v
~~~

- **Python codec and simulator.** The Python tests compile the vendored
  firmware codec and compare the Python codec against it: CRC, encoders,
  thousands of corrupted frames, and the documented examples. They also test
  the simulator's timeout, fault, and restart rules, and the session's
  keepalive, stop-confirmation, and abort paths.
- **C++ BlueZ client.** `bluez_integration` (CTest) runs the real BlueZ client
  against an isolated fake `org.bluez` service. The fake publishes 50 Hz
  telemetry and models the 250 ms timeout. It covers discovery, characteristic
  flags, a keepalive trial with stop confirmation, both write types, CRC
  rejection, SafetyFault, CommandTimeout, telemetry loss, and disconnect.

Not yet tested: real BLE radio timing on the BBB, the Nano/Teensy firmware on
hardware, and any physical motion.

API references:
[BlueZ GATT API](https://bluez.readthedocs.io/en/latest/gatt-api/),
[BlueZ Device API](https://bluez.readthedocs.io/en/latest/device-api/),
[Bleak client API](https://bleak.readthedocs.io/en/latest/api/client.html).

The hardware-interface compile check uses the test-only substitutes in
tests/hardware_stubs. To run it, install libeigen3-dev and configure CMake
with -DNEUROEXO_TEST_SENSOR_STUBS=ON. Never use tests/hardware_stubs as
production sensor drivers.
