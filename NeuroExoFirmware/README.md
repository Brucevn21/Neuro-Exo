# NeuroExo Firmware

Firmware for the NeuroExo joint controller. The system uses a Nano 33 BLE as a bridge between the BeagleBone Black and a Teensy 4.1 motor controller.

## Communication overview

Control commands travel from the BeagleBone to the Nano over BLE, then from the Nano to the Teensy over I2C. Telemetry travels from the Teensy to the Nano over I2C and is forwarded to the BeagleBone over BLE.

Messages use framed binary packets with start/stop markers, payload length, and CRC-8 validation. Control frames are 9 bytes; telemetry frames are 12 bytes; stop frames are 5 bytes. See the detailed [communication protocol documentation](lib/commProtocol/README.md).

## Example packets

Write one packet once to the BLE command characteristic `2A56` (ideally as a write with response) to start a trial; it runs until the joint reaches the target or a stop packet arrives. A start packet sent while a trial is running is ignored. The fifth byte is the speed or level and the second-to-last byte is the CRC, which changes with it, so copy whole packets rather than editing a single byte. See the [protocol documentation](lib/commProtocol/README.md#trials) for how each mode behaves.

**Calibrate first.** Assistive and Resistive trials are refused until a calibration covers their range. With the patient relaxed, send a Calibrate packet whose target is the far end of the range you'll use (the sweep goes from the current angle to it and back, twice). Repeat after a Teensy reset.

| Calibrate to | Packet |
| --- | --- |
| 60° | `02 10 04 03 00 00 3C 9F 03` |
| -40° | `02 10 04 03 00 FF D8 FA 03` |

**Stop** (any mode): `02 12 00 7D 03`

### Demo and Home

Demo sweeps the arm 0° → 70° → 0° with **no one attached** (moving to 0° first if needed). Home moves the arm to 0°. Both ignore the target angle and, like other trials, are ignored while a trial is running, so send Stop first.

| Speed | Demo | Home |
| --- | --- | --- |
| Slow (3 °/s) | `02 10 04 04 00 00 00 49 03` | `02 10 04 05 00 00 00 5F 03` |
| Medium (7 °/s) | `02 10 04 04 01 00 00 22 03` | `02 10 04 05 01 00 00 34 03` |
| High (15 °/s) | `02 10 04 04 02 00 00 9F 03` | `02 10 04 05 02 00 00 89 03` |

### Assistive (patient-driven)

| Level | 60° | 0° | -40° |
| --- | --- | --- | --- |
| 1 (10%) | `02 10 04 01 01 00 3C D8 03` | `02 10 04 01 01 00 00 6C 03` | `02 10 04 01 01 FF D8 BD 03` |
| 2 (20%) | `02 10 04 01 02 00 3C 65 03` | `02 10 04 01 02 00 00 D1 03` | `02 10 04 01 02 FF D8 00 03` |
| 3 (30%) | `02 10 04 01 03 00 3C 0E 03` | `02 10 04 01 03 00 00 BA 03` | `02 10 04 01 03 FF D8 6B 03` |
| 4 (40%) | `02 10 04 01 04 00 3C 18 03` | `02 10 04 01 04 00 00 AC 03` | `02 10 04 01 04 FF D8 7D 03` |
| 5 (50%) | `02 10 04 01 05 00 3C 73 03` | `02 10 04 01 05 00 00 C7 03` | `02 10 04 01 05 FF D8 16 03` |
| 6 (60%) | `02 10 04 01 06 00 3C CE 03` | `02 10 04 01 06 00 00 7A 03` | `02 10 04 01 06 FF D8 AB 03` |
| 7 (70%) | `02 10 04 01 07 00 3C A5 03` | `02 10 04 01 07 00 00 11 03` | `02 10 04 01 07 FF D8 C0 03` |
| 8 (80%) | `02 10 04 01 08 00 3C E2 03` | `02 10 04 01 08 00 00 56 03` | `02 10 04 01 08 FF D8 87 03` |
| 9 (90%) | `02 10 04 01 09 00 3C 89 03` | `02 10 04 01 09 00 00 3D 03` | `02 10 04 01 09 FF D8 EC 03` |

### Resistive (patient pushes against the motion)

| Level | 60° | 0° | -40° |
| --- | --- | --- | --- |
| 1 (10%) | `02 10 04 00 01 00 3C CE 03` | `02 10 04 00 01 00 00 7A 03` | `02 10 04 00 01 FF D8 AB 03` |
| 2 (20%) | `02 10 04 00 02 00 3C 73 03` | `02 10 04 00 02 00 00 C7 03` | `02 10 04 00 02 FF D8 16 03` |
| 3 (30%) | `02 10 04 00 03 00 3C 18 03` | `02 10 04 00 03 00 00 AC 03` | `02 10 04 00 03 FF D8 7D 03` |
| 4 (40%) | `02 10 04 00 04 00 3C 0E 03` | `02 10 04 00 04 00 00 BA 03` | `02 10 04 00 04 FF D8 6B 03` |
| 5 (50%) | `02 10 04 00 05 00 3C 65 03` | `02 10 04 00 05 00 00 D1 03` | `02 10 04 00 05 FF D8 00 03` |
| 6 (60%) | `02 10 04 00 06 00 3C D8 03` | `02 10 04 00 06 00 00 6C 03` | `02 10 04 00 06 FF D8 BD 03` |
| 7 (70%) | `02 10 04 00 07 00 3C B3 03` | `02 10 04 00 07 00 00 07 03` | `02 10 04 00 07 FF D8 D6 03` |
| 8 (80%) | `02 10 04 00 08 00 3C F4 03` | `02 10 04 00 08 00 00 40 03` | `02 10 04 00 08 FF D8 91 03` |
| 9 (90%) | `02 10 04 00 09 00 3C 9F 03` | `02 10 04 00 09 00 00 2B 03` | `02 10 04 00 09 FF D8 FA 03` |

### Neutral (robot-driven constant-speed move)

| Speed | 60° | 0° | -40° |
| --- | --- | --- | --- |
| Slow (3 °/s) | `02 10 04 02 00 00 3C 89 03` | `02 10 04 02 00 00 00 3D 03` | `02 10 04 02 00 FF D8 EC 03` |
| Medium (7 °/s) | `02 10 04 02 01 00 3C E2 03` | `02 10 04 02 01 00 00 56 03` | `02 10 04 02 01 FF D8 87 03` |
| High (15 °/s) | `02 10 04 02 02 00 3C 5F 03` | `02 10 04 02 02 00 00 EB 03` | `02 10 04 02 02 FF D8 3A 03` |

## Wiring

Pin assignments and board-to-board connections for the Teensy 4.1 and Nano 33 BLE are in the [connection map](lib/pinMap/README.md).

## Build

This is a PlatformIO project with separate environments for the Teensy 4.1 and Nano 33 BLE:

```bash
pio run -e teensy41
pio run -e nano33ble
```
