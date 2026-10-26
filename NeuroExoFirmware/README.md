# NeuroExo Firmware

Firmware for the NeuroExo joint controller. The system uses a Nano 33 BLE as a bridge between the BeagleBone Black and a Teensy 4.1 motor controller.

## Communication overview

Control commands travel from the BeagleBone to the Nano over BLE, then from the Nano to the Teensy over I2C. Telemetry travels from the Teensy to the Nano over I2C and is forwarded to the BeagleBone over BLE.

Messages use framed binary packets with start/stop markers, payload length, and CRC-8 validation. Control frames are 9 bytes; telemetry frames are 10 bytes. See the detailed [communication protocol documentation](lib/commProtocol/README.md).

## Example packets

Ready-to-send trial packets in Assistive mode. Write one packet once to the BLE command characteristic `2A56` (ideally as a write with response) to start a trial; it runs until the joint reaches the target or a stop packet arrives. A start packet sent while a trial is running is ignored.

| Target | Low (20 s) | Medium (12 s) | High (6 s) |
| --- | --- | --- | --- |
| 60° | `02 10 04 01 00 00 3C B3 03` | `02 10 04 01 01 00 3C D8 03` | `02 10 04 01 02 00 3C 65 03` |
| 0° | `02 10 04 01 00 00 00 07 03` | `02 10 04 01 01 00 00 6C 03` | `02 10 04 01 02 00 00 D1 03` |
| -40° | `02 10 04 01 00 FF D8 D6 03` | `02 10 04 01 01 FF D8 BD 03` | `02 10 04 01 02 FF D8 00 03` |
| Stop | `02 12 00 7D 03` | | |

The fifth byte is the speed and the second-to-last byte is the CRC, which changes with it, so copy whole packets rather than editing a single byte.

## Wiring

Pin assignments and board-to-board connections for the Teensy 4.1 and Nano 33 BLE are in the [connection map](lib/pinMap/README.md).

## Build

This is a PlatformIO project with separate environments for the Teensy 4.1 and Nano 33 BLE:

```bash
pio run -e teensy41
pio run -e nano33ble
```
