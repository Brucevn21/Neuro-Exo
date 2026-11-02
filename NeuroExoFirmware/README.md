# NeuroExo Firmware

Firmware for the NeuroExo joint controller, integrated from `ProtocolDemo`
commit `e3ee8443e194e306e2733899a79f2d9e215ffac3` onto `prep`.
The Nano 33 BLE bridges the BeagleBone Black and Teensy 4.1.

## Communication overview

Control commands travel from the BeagleBone to the Nano over BLE, then from the Nano to the Teensy over I2C. Telemetry travels from the Teensy to the Nano over I2C and is forwarded to the BeagleBone over BLE.

Messages use framed binary packets with start/stop markers, payload length, and CRC-8 validation. Control frames are 9 bytes; telemetry frames are 10 bytes. See the detailed [communication protocol documentation](../H_Robotics_Files/NEUROEXO_PROTOCOL.md).

The shared codec remains in `src/commProtocol.h` and `.cpp`. Both firmware
environments compile it, and the existing BeagleBone C++ client uses the same
implementation. The Nano keeps the `NanoNeuroExo` name and the existing BLE
service/command/telemetry UUIDs documented in the protocol guide.
The obsolete `Nano33BLEFirmware.ino` entry point is replaced by `src/nanoBridge.cpp`.

Integration differences from `ProtocolDemo`:

- **Neutral disables motion.** It is not the branch's powered passive-motion mode.
- Repeated identical commands refresh the 250 ms watchdog without restarting
  a running or completed trajectory. Change the command, send Neutral first,
  or allow a timeout to start another trajectory.
- Each BLE write and I2C transaction must contain exactly one valid frame.
  Oversized, truncated, corrupt, and invalid-enum commands are rejected.
- The Nano polls telemetry every 40 ms, including without a BLE connection.
  Valid frames are published only to a connected, subscribed BeagleBone.
- Nano power management retains the branch's A3 divider input, D5 relay,
  22/30 V cutoffs, and 0.5 V recovery hysteresis. The relay starts off.

The Teensy preserves the branch's current/torque controller and its configuration
guard: **`ESCON_FULL_SCALE_CURRENT_A` is zero by default, so motion stays disabled
until it is set to the actual ESCON calibration.** Telemetry and communication
can be checked with that guard in place.

## Wiring

Pin assignments and board-to-board connections for the Teensy 4.1 and Nano 33 BLE are in the [connection map](lib/pinMap/README.md).

## Build

This is a PlatformIO project with separate environments for the Teensy 4.1 and Nano 33 BLE:

```bash
pio run -e teensy41
pio run -e nano33ble
```

From the repository root, add `-d NeuroExoFirmware`. Building does not upload.
After building, upload each environment separately with `-t upload`; use
`--upload-port` with the Nano's actual USB serial port.

## USB diagnostics

Both boards use 115200 baud. The Nano prints a ready message when a monitor is
opened, then a status line about once per second:

```text
[Nano] BLE=connected subscribed=1 | angle=45 deg current=1200 mA status=0x1 MotionActive=1 CommandTimeout=0 InvalidCommand=0 SafetyFault=0
```

If no valid I2C response arrives, it prints `Waiting for valid Teensy telemetry`.
It also prints commands relayed to the Teensy. USB logging works without a BLE
central and does not generate motor commands. The Teensy prints its own
measurements every two seconds. Set `STREAM_JOINT_CSV = true` in `src/main.cpp`
to enable the optional 50 Hz [visualizer feed](tools/visualizer/README.md).

## Validation

The host tests exercise the real firmware code using inert Arduino/I2C/BLE
substitutes. They check Neutral stop, repeated-command behavior, malformed
transactions, the watchdog (including timer wraparound), safety fault latching,
signed telemetry, Nano diagnostics, and relay hysteresis.

Build and run them from the repository root:

```bash
g++ -std=c++17 -Wall -Wextra -Werror \
  -I H_Robotics_Files/tests/nano_stubs \
  H_Robotics_Files/tests/nano_bridge_check.cpp NeuroExoFirmware/src/commProtocol.cpp \
  -o /tmp/neuroexo-nano-check
/tmp/neuroexo-nano-check

g++ -std=c++17 -Wall -Wextra -Werror \
  -I NeuroExoFirmware/tests/stubs -I H_Robotics_Files/tests/nano_stubs \
  -I NeuroExoFirmware/lib/pinMap/src \
  NeuroExoFirmware/tests/teensy_protocol_check.cpp \
  NeuroExoFirmware/src/ControlAlgorithm.cpp NeuroExoFirmware/src/commProtocol.cpp \
  -o /tmp/neuroexo-teensy-check
/tmp/neuroexo-teensy-check

python3 -m unittest discover -s H_Robotics_Files/tests -p 'test_*.py' -v
```

The two native checks are also registered in the BeagleBone project's CTest
configuration. These checks do not validate physical motor behavior.
