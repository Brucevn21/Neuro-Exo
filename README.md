# Neuro-Exo

This repository contains the University of Houston New NeuroExo Senior Design project, focused on restoring arm mobility for stroke patients using an electronically controlled assistive/resistive exoskeleton system.

## System Features

### Dynamic Force Control for Motor Assistance

The Teensy 4.1 control firmware includes a dynamic assist algorithm that adjusts motor command force in real time using:
- Joint velocity
- Joint acceleration
- Motor current (used as a torque-related effort signal)

The control path combines trajectory/PID output with assist modulation to increase support when user movement is faster or requires more effort, and reduce abrupt changes through filtering and slew-rate limiting.

Implementation references:
- NeuroExoFirmware/src/ControlAlgorithm.cpp
- NeuroExoFirmware/src/main.cpp

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

Current PMS logic includes under-voltage protection with hysteresis-based relay control, sampled independently of the BLE/I2C mediation loop so it keeps running whether or not a BLE central is connected.

Implementation reference:
- NeuroExoFirmware/src/Nano33BLEFirmware.ino
