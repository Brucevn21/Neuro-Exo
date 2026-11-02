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

PlatformIO's `nano33ble` and `teensy41` environments build the matching framed
bridge and controller, integrated from `ProtocolDemo` (`e3ee844`). The old BLE
benchmark tools, simulated trial protocol, `DemoFolder`, and obsolete Nano
sketch have been removed.

The [Teensy controller](NeuroExoFirmware/src/main.cpp) validates control frames,
returns angle/current/status telemetry, and disables motion after 250 ms without
a valid command. **Neutral is stop.** Nano USB diagnostics show telemetry or a
waiting-for-Teensy message once per second at 115200 baud.
The controller stays disabled until `ESCON_FULL_SCALE_CURRENT_A` matches the
actual ESCON configuration. See the [firmware guide](NeuroExoFirmware/README.md)
for build instructions and the integration differences.

## Dynamic force control for motor assistance

The Teensy 4.1 controller from `ProtocolDemo` computes assistive tracking torque
or resistive damping torque and converts it to motor current. It includes
velocity filtering, current/torque limits, and latched overspeed/overcurrent
faults. Neutral bypasses the controller and disables the motor.

Implementation references:

- [Assist algorithm](NeuroExoFirmware/src/ControlAlgorithm.cpp)
- [Teensy controller](NeuroExoFirmware/src/main.cpp)
- [Serial joint visualizer](NeuroExoFirmware/tools/visualizer/README.md)

## Power management system

The project includes firmware for a power management subsystem on an Arduino
Nano 33 BLE, with a relay for load connection/disconnection, a buck-converted
supply path, and polarity protection hardware. Its firmware provides
under/over-voltage protection with hysteresis-based relay control. The active
Nano bridge also includes this protection on A3 (voltage sense) and D5 (relay).

Implementation reference:

- [Power management firmware](PMSFirmware/NeuroExo_PMS_Code.ino)
