# NeuroExo Firmware

Firmware for the NeuroExo joint controller. The system uses a Nano 33 BLE as a bridge between the BeagleBone Black and a Teensy 4.1 motor controller.

## Communication overview

Control commands travel from the BeagleBone to the Nano over BLE, then from the Nano to the Teensy over I2C. Telemetry travels from the Teensy to the Nano over I2C and is forwarded to the BeagleBone over BLE.

Messages use framed binary packets with start/stop markers, payload length, and CRC-8 validation. Control frames are 9 bytes; telemetry frames are 10 bytes. See the detailed [communication protocol documentation](lib/commProtocol/README.md).

## Wiring

Pin assignments and board-to-board connections for the Teensy 4.1 and Nano 33 BLE are in the [connection map](lib/pinMap/README.md).

## Build

This is a PlatformIO project with separate environments for the Teensy 4.1 and Nano 33 BLE:

```bash
pio run -e teensy41
pio run -e nano33ble
```
