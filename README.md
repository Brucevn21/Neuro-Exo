# Neuro-Exo

This repository contains the University of Houston New NeuroExo Senior Design
project, focused on restoring arm mobility for stroke patients using an
electronically controlled assistive/resistive exoskeleton system.

## Motor communication

The BeagleBone client and Nano 33 BLE bridge use separate BLE command and
telemetry characteristics. The Nano validates 9-byte control frames, forwards
them over I2C to address `0x08`, and relays valid 10-byte telemetry frames back to
the BeagleBone. Telemetry contains motor angle, current, and status flags.

See the [setup guide](H_Robotics_Files/BEAGLEBONE_READER_PROCEDURE.md) and
[binary protocol](H_Robotics_Files/NEUROEXO_PROTOCOL.md).

Implementation references:

- [Nano bridge](NeuroExoFirmware/src/nanoBridge.cpp)
- [Protocol header](NeuroExoFirmware/src/commProtocol.h)
- [Protocol implementation](NeuroExoFirmware/src/commProtocol.cpp)
- [BeagleBone interface](H_Robotics_Files/Hardware_Interface.hpp)

PlatformIO's `nano33ble` environment builds the framed bridge. The older
[Nano sketch](NeuroExoFirmware/src/Nano33BLEFirmware.ino) is retained from `main`
but excluded from that build. The old BLE benchmark tools, simulated trial
protocol, and `DemoFolder` have been removed.

**Teensy compatibility:** the [Teensy controller](NeuroExoFirmware/src/main.cpp)
imported from `main` still accepts the legacy three-byte `0x01` angle command.
It does not yet decode the framed protocol or return I2C telemetry. The framed
Nano/BeagleBone path requires the separately supplied Teensy firmware implementing
the protocol above; the two firmware environments in this checkout are not yet
a matching control/telemetry pair.

## Dynamic force control for motor assistance

The Teensy 4.1 controller imported from `main` includes an assist algorithm that
adjusts motor output using joint velocity, acceleration, and motor current as an
effort signal. It combines trajectory/PID output with assist modulation,
filtering, and slew-rate limiting.

Implementation references:

- [Assist algorithm](NeuroExoFirmware/src/ControlAlgorithm.cpp)
- [Teensy controller](NeuroExoFirmware/src/main.cpp)
- [Serial joint visualizer](NeuroExoFirmware/tools/visualizer/README.md)

## Power management system

The project includes firmware for a power management subsystem on an Arduino
Nano 33 BLE, with a relay for load connection/disconnection, a buck-converted
supply path, and polarity protection hardware. Its firmware provides
under-voltage protection with hysteresis-based relay control.

Implementation reference:

- [Power management firmware](PMSFirmware/NeuroExo_PMS_Code.ino)
