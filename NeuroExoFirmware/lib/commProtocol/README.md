# NeuroExo Communication Protocol

The Nano 33 BLE bridges framed control and telemetry messages between the BeagleBone Black and the Teensy 4.1.

## Frame format

Every message uses this binary layout:

```text
STX | TYPE | LENGTH | PAYLOAD | CRC8 | ETX
```

| Field | Size | Description |
| --- | ---: | --- |
| `STX` | 1 byte | Start marker, always `0x02` |
| `TYPE` | 1 byte | `0x10` control, `0x11` telemetry, or `0x12` stop |
| `LENGTH` | 1 byte | Payload length in bytes |
| `PAYLOAD` | variable | Message-specific binary payload |
| `CRC8` | 1 byte | CRC-8 using polynomial `0x07` |
| `ETX` | 1 byte | Stop marker, always `0x03` |

The CRC is calculated over `TYPE`, `LENGTH`, and every payload byte. Signed 16-bit values are encoded big-endian (most-significant byte first). The protocol uses fixed payload sizes, so a valid control frame is 9 bytes, a valid telemetry frame is 12 bytes, and a stop frame is 5 bytes.

## Control frame: 9 bytes

Control messages travel from the BeagleBone Black to the Nano, then from the Nano to the Teensy at I2C address `0x08`. A single control frame starts a trial (see [Trials](#trials)); it does not need to be repeated.

```text
Byte:  0     1       2        3       4       5          6          7      8
       STX   TYPE    LENGTH   MODE    SETTING ANGLE MSB  ANGLE LSB  CRC8   ETX
Value: 02    10      04       --      --      --         --         --     03
```

Payload fields:

| Byte | Field | Values |
| ---: | --- | --- |
| 3 | `MODE` | `0` Resistive, `1` Assistive, `2` Neutral, `3` Calibrate, `4` Demo, `5` Home |
| 4 | `SETTING` | Neutral / Demo / Home: speed `0` Slow, `1` Medium, `2` High. Assistive / Resistive: level `1`-`9` = 10 %-90 %. Calibrate: ignored (send `0`) |
| 5-6 | `targetAngleDeg` | Signed `int16_t`, big-endian, degrees. Calibrate: the far end of the sweep. Demo / Home: ignored (send `0`) |

A frame whose `SETTING` is out of range for its mode (e.g. Assistive with `0`) is invalid and discarded.

Example: Assistive, level 5 (50 %), target angle `-123` (`0xFF85`):

```text
02 10 04 01 05 FF 85 82 03
```

The CRC in this example covers:

```text
10 04 01 05 FF 85
```

Example: Home at Medium speed:

```text
02 10 04 05 01 00 00 34 03
```

## Stop frame: 5 bytes

Ends the current trial immediately and clears a latched safety fault. It is sent on the same BLE command characteristic (`2A56`) as control frames and forwarded to the Teensy over I2C.

```text
Byte:  0     1       2        3      4
       STX   TYPE    LENGTH   CRC8   ETX
Value: 02    12      00       7D     03
```

## Telemetry frame: 12 bytes

Telemetry messages travel from the Teensy to the Nano in response to an I2C request. The Nano forwards the same framed message to the BeagleBone over BLE.

```text
Byte:  0     1       2        3          4          5           6           7       8           9           10     11
       STX   TYPE    LENGTH   ANGLE MSB  ANGLE LSB  CURRENT MSB CURRENT LSB STATUS  EFFORT MSB  EFFORT LSB  CRC8   ETX
Value: 02    11      07       --         --         --          --          --      --          --          --     03
```

Payload fields:

| Byte | Field | Description |
| ---: | --- | --- |
| 3-4 | `currentAngleDeg` | Signed `int16_t`, big-endian, degrees |
| 5-6 | `currentMilliAmps` | Signed `int16_t`, big-endian, milliamps |
| 7 | `status` | Status bit field |
| 8-9 | `patientEffortCentiNm` | Signed `int16_t`, big-endian, 0.01 N·m: estimated patient torque toward the target (negative = resisting). 0 outside Assistive / Resistive trials |

Status bits:

| Bit | Name | Meaning |
| ---: | --- | --- |
| 0 | `MotionActive` | A trial is currently running |
| 1 | `CommandTimeout` | Unused in trial mode (reserved) |
| 2 | `InvalidCommand` | The most recent received I2C command was invalid |
| 3 | `SafetyFault` | Controller e-stop latched (over-speed, overcurrent, invalid sensor data, or tracking error). Control frames are ignored until a stop frame clears it |
| 4 | `TargetReached` | The last trial ended with the joint at its target (within 1 deg for Neutral / Demo / Home, 2 deg otherwise) |
| 5 | `TrialTimeout` | The last trial was aborted by the trial time limit |
| 6 | `Calibrated` | A baseline calibration is stored, so Assistive / Resistive trials can run within its range |
| 7 | `PatientResisting` | The last Assistive trial was aborted because the patient pushed against the motion |

Example: angle `45`, current `1200 mA`, motion active + calibrated, patient effort `2.35 N·m` (`0x00EB`):

```text
02 11 07 00 2D 04 B0 41 00 EB 19 03
```

The CRC in this example covers:

```text
11 07 00 2D 04 B0 41 00 EB
```

## Trials

A valid control frame starts a trial when no trial is running. What the robot does depends on the mode:

| Mode | What happens |
| --- | --- |
| Neutral | Robot-driven move from the current angle to the target at a constant speed (Slow 3 °/s, Medium 7 °/s, High 15 °/s), so the move time depends on the distance. The patient can stay passive. |
| Assistive | Patient-driven. The robot creeps toward the target (1 °/s at level 1 up to 5 °/s at level 9) and speeds up as it detects the patient helping: speed = creep + effort / ((1 - level) x 28.6 N·m·s/rad), up to 45 °/s, slowing near the target. The same push moves the arm faster at higher levels. |
| Resistive | The robot moves toward the target at 10 °/s while the patient pushes against it. It pushes at most level x 10 N·m beyond what the relaxed arm needs (level 3 = 3 N·m); resisting harder than that stalls the arm. |
| Demo | For showing the device with no one attached. A robot-driven sweep 0° → 70° → 0° at the Neutral speed for the setting. If the joint is not already within 1° of 0° it first moves to 0° the same way. Ends back at 0°. No calibration needed. |
| Home | Robot-driven move from the current angle to 0° at the Neutral speed for the setting. No calibration needed. |
| Calibrate | With the patient relaxed, the robot moves from the current angle to the target and back twice (5 °/s, then 20 °/s), recording the torque the relaxed arm needs at each angle. Run once per session; Assistive and Resistive trials are refused until a calibration covers their range. |

The joint isn't back-drivable, so the robot always does the moving and reads the patient's effort as the calibrated relaxed-arm torque minus the torque it is commanding. A Teensy reset loses the calibration, and calibration is rejected (status bit `Calibrated` stays clear) if the arm doesn't move steadily enough to measure.

A trial ends when the first of these happens:

- the joint is within 1 deg of the target in Neutral / Demo / Home, or within 2 deg of or past it in Assistive / Resistive (`TargetReached` is set). In Demo this applies to the last waypoint; reaching an earlier one starts the next leg,
- a stop frame arrives,
- a safety fault trips (`SafetyFault` is set),
- in Assistive, the patient pushes against the motion by more than 2 N·m for 0.3 s (`PatientResisting` is set),
- the time limit expires (`TrialTimeout` is set): the move time at the slowest speed the mode can run (Neutral / Home / Demo travel distance at the Neutral speed, Assistive creep, Resistive speed, or the calibration sweep) + 5 s,
- Calibrate: the sweep finishes.

Other rules:

- Control frames received while a trial is running are ignored. Send a stop frame first to start a different trial.
- `TargetReached`, `TrialTimeout` and `PatientResisting` describe the last trial and are cleared when the next trial starts.
- When the BLE central disconnects, the Nano sends a stop frame to the Teensy.
- The motor is disabled when a trial ends; the gearbox holds the arm in place.
- Senders should use a BLE write with response for control and stop frames, and confirm the result from the `MotionActive` bit in telemetry.

## Transport behavior

- The Nano validates BLE command frames before forwarding them over I2C.
- The Teensy validates the I2C control frame before applying motor state changes.
- I2C callbacks only copy or transmit bytes; decoding and motor updates happen in the normal firmware loop.
- The Nano retries a failed I2C write up to 3 times.
- Invalid, oversized, truncated, or CRC-corrupted frames are discarded.
- The Nano polls telemetry periodically and only publishes a telemetry frame after successful validation.

The implementation is in `src/commProtocol.cpp` and `src/commProtocol.h`.
