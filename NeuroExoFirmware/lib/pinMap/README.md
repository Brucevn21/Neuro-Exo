# NeuroExo Connection Map

Wiring reference for one NeuroExo joint. It covers the Teensy 4.1 motor controller (pins defined in [`pinMap4.1.h`](src/pinMap4.1.h)) and the Nano 33 BLE bridge and power manager (pins defined in [`Nano33BLEFirmware.ino`](../../src/Nano33BLEFirmware.ino)).

> `pinMap.h` is the legacy Teensy 3.2 map. The current firmware ([`main.cpp`](../../src/main.cpp)) includes only `pinMap4.1.h`.

## System overview

```text
                         BLE (service 0x180C)
 ┌──────────────────┐   cmd  0x2A56 (write)  ┌──────────────────────────────┐
 │ BeagleBone Black │ ─────────────────────► │        Nano 33 BLE           │
 │   (BLE central)  │ ◄───────────────────── │ "Nano33BLE_Master"           │
 └──────────────────┘   tlm  0x2A57 (notify) │  BLE peripheral + I2C master │
                                             │  + power management (PMS)    │
                                             └──┬──────────┬──────────┬─────┘
                                       A4 SDA / │          │ A3       │ D5
                                       A5 SCL   │ I2C      │          │
                                                │ 0x08     │          ▼
                                                │          │     ┌─────────┐
          ┌─────────────────────────────────────┘          │     │  Relay  │──► load (power rail)
          │                                    ┌───────────┴─┐   └─────────┘
          ▼ 18 SDA / 19 SCL                    │ Divider     │◄── power rail (+V)
 ┌──────────────────────────────┐              │ 101k / 9.9k │
 │          Teensy 4.1          │              └─────────────┘
 │  I2C slave 0x08, PID loop    │
 └──┬───────────────┬───────────┘
    │ 7 PWM          │ 0 DATA / 1 CLK / 2 CS
    │ 8 ENABLE       ▼
    │ 9 DIR     ┌──────────┐
    │ 21 ◄ AN1  │  AS5045  │ joint angle encoder
    ▼           └──────────┘
 ┌──────────────┐
 │ ESCON driver │──► motor
 └──────────────┘
```

All grounds (Nano, Teensy, ESCON, encoder, PMS divider) must be tied together.

## Wire list

Every physical connection, point to point. "→" is the signal direction; "↔" is bidirectional.

| # | From | Pin | | To | Pin | Signal |
| ---: | --- | --- | :---: | --- | --- | --- |
| 1 | Nano 33 BLE | A4 (SDA) | ↔ | Teensy 4.1 | 18 (SDA) | I2C data |
| 2 | Nano 33 BLE | A5 (SCL) | → | Teensy 4.1 | 19 (SCL) | I2C clock |
| 3 | Nano 33 BLE | GND | — | Teensy 4.1 | GND | Common ground |
| 4 | Nano 33 BLE | D5 | → | Relay module | IN / control | Load enable (HIGH = on) |
| 5 | Divider midpoint (R1/R2) | — | → | Nano 33 BLE | A3 | Rail voltage sense |
| 6 | Power rail +V | — | → | Divider R1 (101 kΩ) | top | Rail to divider |
| 7 | Divider R2 (9.9 kΩ) | bottom | — | GND | — | Divider return |
| 8 | Power rail +V | — | → | Relay | COM | Switched supply in |
| 9 | Relay | NO | → | Load (ESCON / motor supply) | +V | Switched supply out |
| 10 | Teensy 4.1 | 7 | → | ESCON | PWM set-value input | Speed (10–90 % duty) |
| 11 | Teensy 4.1 | 8 | → | ESCON | Enable input | HIGH = enabled |
| 12 | Teensy 4.1 | 9 | → | ESCON | Direction input | LOW = forward, HIGH = backward |
| 13 | ESCON | Analog out 1 | → | Teensy 4.1 | 21 (A7) | Motor current feedback |
| 14 | ESCON | GND | — | Teensy 4.1 | GND | Common ground |
| 15 | Teensy 4.1 | 1 | → | AS5045 | CLK | Encoder clock |
| 16 | Teensy 4.1 | 2 | → | AS5045 | CSn | Encoder chip select |
| 17 | AS5045 | DO | → | Teensy 4.1 | 0 | Encoder data |
| 18 | Teensy 4.1 | 3.3 V | → | AS5045 | VDD | Encoder supply |
| 19 | Teensy 4.1 | GND | — | AS5045 | GND | Encoder ground |
| 20 | ESCON | Motor outputs | → | Motor | M+ / M− | Motor power |
| 21 | Nano 33 BLE | USB | ↔ | PC | — | Debug serial, 115200 baud |
| 22 | Teensy 4.1 | USB | ↔ | PC | — | Debug + visualizer serial, 115200 baud |

Rows 8, 9, 18 and 20 aren't set by the firmware. They show the usual way to wire this, so check them against your PCB or harness. The ESCON input and output names depend on how it's configured in ESCON Studio, so match each function to the digital or analog pin you assigned there.

**Reserved Teensy pins, not wired in current firmware:** 14 home switch, 15 forward button, 16 backward button, 17 control-mode select, 20 ESCON analog out 2 (velocity), 22 backward limit switch, 23 forward limit switch.

## Teensy 4.1: joint motor controller

Built from the `teensy41` environment ([`main.cpp`](../../src/main.cpp) + [`ControlAlgorithm.cpp`](../../src/ControlAlgorithm.cpp)).

### Connected and used by firmware

| Teensy pin | Macro / role | Direction | Connects to | Notes |
| ---: | --- | --- | --- | --- |
| 0 | `AS5045_DATA_PIN` | In | AS5045 DO | Bit-banged SSI data. Shares the Serial1 RX pin, so Serial1 is unavailable. |
| 1 | `AS5045_CLK_PIN` | Out | AS5045 CLK | Bit-banged SSI clock. Shares the Serial1 TX pin. |
| 2 | `AS5045_CS_PIN` | Out | AS5045 CSn | Chip select. |
| 7 | `PWM_PIN` | Out | ESCON PWM set-value input | `analogWrite` 8-bit, clamped to 10 %–90 % duty (`ESCON_mapping`). Duty sets speed magnitude only. |
| 8 | `ENABLE_PIN` | Out | ESCON enable input | HIGH = enabled, LOW = disabled. Driven LOW on safety stop or when idle. |
| 9 | `DIR_PIN` | Out | ESCON direction input | LOW = forward, HIGH = backward (`direction` in `setup()`). |
| 13 | `LED` | Out | Onboard LED | Set HIGH at boot as a power/alive indicator. |
| 18 | `Wire` SDA | I/O | Nano 33 BLE A4 (SDA) | I2C slave at address `0x08`. |
| 19 | `Wire` SCL | In | Nano 33 BLE A5 (SCL) | I2C clock from the Nano. |
| 21 (A7) | `ESCON_AN1` | Analog in | ESCON analog output 1 (current) | Read every loop, 10-bit, 3.3 V reference. Reported in telemetry as mA. |
| USB | `Serial` | — | PC | 115200 baud. Debug text plus `JOINT,...` CSV lines for [the visualizer](../../tools/visualizer/README.md). |

### Defined but not used by current firmware

| Teensy pin | Macro | Intended role |
| ---: | --- | --- |
| 14 | `HOME_SWITCH_PIN` | Home switch. Not on the PCB. Moved off pin 14 to avoid a clash with the old AS5045 clock pin. |
| 15 | `FW_PIN` | Manual "rotate forward" button |
| 16 | `BW_PIN` | Manual "rotate backward" button |
| 17 | `CONTROL_MODE_PIN` | Mode select: 1 = manual (open loop), 0 = closed loop |
| 20 (A6) | `ESCON_AN2` | ESCON analog output 2 (velocity) |
| 22 | `BW_SWITCH_PIN` | Backward limit switch |
| 23 | `FW_SWITCH_PIN` | Forward limit switch |

The limit switches are disabled in firmware: `main.cpp` sets `motorWiring.FWSwitchPin` and `BWSwitchPin` to `0`, so `motorDriver::init()` skips their interrupts. To enable them, assign `FW_SWITCH_PIN` / `BW_SWITCH_PIN` instead. The ISRs trigger on a HIGH level and disable the motor.

## Nano 33 BLE: BLE bridge and power management

Built from the `nano33ble` environment ([`Nano33BLEFirmware.ino`](../../src/Nano33BLEFirmware.ino)).

| Nano pin | Firmware name | Direction | Connects to | Notes |
| --- | --- | --- | --- | --- |
| A4 | `Wire` SDA | I/O | Teensy 4.1 pin 18 | I2C master. Writes control frames to `0x08`. |
| A5 | `Wire` SCL | Out | Teensy 4.1 pin 19 | I2C clock. Telemetry is polled every 20 ms (50 Hz). |
| A3 | `PMS_ANALOG_PIN` | Analog in | Voltage divider midpoint | Senses the power rail. See [Power management](#power-management-pms). |
| D5 | `PMS_RELAY_PIN` | Out | Relay control input | HIGH = load connected, LOW = load disconnected. LOW at boot. |
| Radio | BLE | — | BeagleBone Black | See [BLE interface](#ble-interface). |
| USB | `Serial` | — | PC | 115200 baud. Logs received commands and I2C errors. |

### BLE interface

| Item | Value |
| --- | --- |
| Local name | `Nano33BLE_Master` |
| Service UUID | `180C` |
| Command characteristic | `2A56`: Write / Write Without Response. BeagleBone → Nano control frames (9 bytes). |
| Telemetry characteristic | `2A57`: Read / Notify. Nano → BeagleBone telemetry frames (10 bytes), 50 Hz while connected. |

Frame layouts are documented in the [communication protocol README](../commProtocol/README.md).

### Power management (PMS)

The rail being protected feeds a voltage divider into A3:

```text
 Rail +V ──[ R1 = 101 kΩ ]──┬──[ R2 = 9.9 kΩ ]── GND
                            │
                            └──► Nano A3
```

`Vin = Vout × (R1 + R2) / R2 ≈ Vout × 11.2`. At the 3.3 V ADC limit this reads up to about 37 V. At 30 V the pin sees about 2.68 V.

The rail is sampled every 500 ms in every `loop()` iteration, whether or not BLE is connected:

| Rail voltage | Relay (D5) |
| --- | --- |
| < 22.0 V | LOW: load disconnected (under-voltage) |
| 22.0 – 22.5 V | Holds its last state (hysteresis) |
| 22.5 – 29.5 V | HIGH: load connected |
| 29.5 – 30.0 V | Holds its last state (hysteresis) |
| > 30.0 V | LOW: load disconnected (over-voltage) |

The relay starts LOW at power-up and stays LOW until the first in-range reading.

## I2C link (Nano ↔ Teensy)

| Signal | Nano 33 BLE | Teensy 4.1 |
| --- | --- | --- |
| SDA | A4 | 18 |
| SCL | A5 | 19 |
| GND | GND | GND |

- Address: `0x08` (`SLAVE_ADDR` on both boards).
- Both boards use 3.3 V logic, so no level shifter is needed. Fit pull-up resistors to 3.3 V on SDA and SCL (commonly 4.7 kΩ) if the bus doesn't already have them.
- The Nano writes one control frame per accepted BLE command. It reads up to `MAX_FRAME_SIZE` bytes every 20 ms, and the Teensy replies with its latest double-buffered telemetry frame.

## Electrical cautions

- **Teensy 4.1 and Nano 33 BLE pins are 3.3 V only and not 5 V tolerant.** Check that the ESCON analog output (pin 21) and digital outputs routed to the Teensy stay within 0 – 3.3 V.
- D5 should drive a relay module or a transistor/MOSFET driver, not a bare relay coil.
- Keep the divider resistor values in the firmware (`PMS_R1`, `PMS_R2`) in sync with the parts actually fitted. A mismatch shifts both cutoff thresholds.
