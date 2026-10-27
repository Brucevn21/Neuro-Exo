# Neuro-Exo

This repository contains the University of Houston New NeuroExo Senior Design project, focused on restoring arm mobility for stroke patients using an electronically controlled assistive/resistive exoskeleton system.

## System Features

### Joint Control Modes

The Teensy 4.1 firmware drives the joint through an ESCON motor controller in current mode. Each trial is started by one command packet and runs in one of these modes:

- **Neutral**: robot-driven move to a target at a constant speed (Slow 3 °/s, Medium 7 °/s, High 15 °/s) for passive range of motion.
- **Assistive**: patient-driven. The robot creeps toward the target and speeds up as it detects the patient's effort.
- **Resistive**: the robot moves toward the target while the patient pushes against it.
- **Calibrate**: a relaxed sweep that records the torque the arm needs at each angle. Required before Assistive or Resistive.
- **Demo**: a 0° → 70° → 0° sweep with no one attached.
- **Home**: a move back to 0°.

Position tracking uses a PD controller with static/sliding friction compensation, so the geared joint moves smoothly at low speed. Safety checks stop the motor on over-speed, a tracking error over 10° (Neutral/Demo/Home), invalid encoder data, or a time limit, and the protocol's stop packet ends any trial.

Implementation references:
- NeuroExoFirmware/src/ControlAlgorithm.cpp
- NeuroExoFirmware/src/main.cpp
- NeuroExoFirmware/lib/commProtocol/README.md (packet format and mode details)

### Bluetooth to I2C Command Bridge

Motion commands are sent over Bluetooth from the BeagleBone Black to an Arduino Nano 33 BLE, which acts as a communication bridge and forwards commands over I2C to the Teensy 4.1 motor controller.

Data flow:
1. BeagleBone Black (BLE central) sends command payload to Nano 33 BLE.
2. Nano 33 BLE parses command values.
3. Nano 33 BLE transmits command bytes to Teensy 4.1 over I2C.
4. Teensy 4.1 executes interpolation and motor control.

Implementation references:
- NeuroExoFirmware/src/Nano33BLEFirmware.ino
- NeuroExoFirmware/src/main.cpp

### Power Management System Firmware

The Nano 33 BLE mediator also runs the power management subsystem for the shared rail, connected to:
- A relay for load connection/disconnection
- A buck-converted supply path
- Polarity protection hardware services

Current PMS logic includes under-voltage (22.0 V) and over-voltage (32.5 V) protection with hysteresis-based relay control, sampled independently of the BLE/I2C mediation loop so it keeps running whether or not a BLE central is connected.

Implementation reference:
- NeuroExoFirmware/src/Nano33BLEFirmware.ino

## Temporary Bench-Testing Changes

Two changes in the Teensy firmware are temporary and must be reverted before the device is used on a person. Both are marked `TEMP` in `NeuroExoFirmware/src/main.cpp`.

### Overcurrent trip disabled

The controller normally latches a safety stop when the measured motor current stays above 3 A for 50 ms. That check is currently disabled (`cfg.overcurrentTripA = 1.0e9f`) because the ESCON current-feedback pin (ESCON analog output 1 → Teensy pin 21, `ESCON_AN1`) is not connected. The unconnected pin floats and reads about 1.5 A even with the motor off, so the measured current, the current in telemetry, and the visualizer's `current_A` column are not meaningful.

While it is disabled:
- Nothing stops the motor on a stall or short-circuit current. The commanded current is still limited to 3.29 A (the ESCON's full scale), and the over-speed, tracking-error, encoder and time-limit checks still apply.
- Run trials only with no one attached, and keep a way to cut motor power within reach.

To restore it: wire ESCON analog output 1 to Teensy pin 21 (see the [connection map](NeuroExoFirmware/lib/pinMap/README.md)), confirm the reading is about 0 A with the motor idle, then set `cfg.overcurrentTripA` back to `3.0f`.

### Encoder status debug print

The once-per-second `[Teensy Telemetry]` debug line on the Teensy's USB serial port ends with `TEMP enc raw <value> status 0b<bits>`: the raw 12-bit AS5045 reading and its 5 status bits (OCF, COF, LIN, MagINC, MagDEC). It was added to diagnose encoder glitches and is not needed in normal use.

During testing the encoder reported `10111` or `10011`, meaning the magnet's field is outside the AS5045's recommended range (too far, too close, or off-center). The firmware tolerates this by rejecting only readings that fail parity or overflow plus any reading that jumps more than 5°, but repositioning the magnet is the real fix. The `Enc rejects` count on the same line shows how many readings were rejected each second.

To remove it: delete the four `Serial.print` lines starting at the `// TEMP diagnostic` comment in the debug block of `loop()`.
