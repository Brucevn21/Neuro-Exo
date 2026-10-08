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
| `TYPE` | 1 byte | `0x10` control or `0x11` telemetry |
| `LENGTH` | 1 byte | Payload length in bytes |
| `PAYLOAD` | variable | Message-specific binary payload |
| `CRC8` | 1 byte | CRC-8 using polynomial `0x07` |
| `ETX` | 1 byte | Stop marker, always `0x03` |

The CRC is calculated over `TYPE`, `LENGTH`, and every payload byte. Signed 16-bit values are encoded big-endian (most-significant byte first). The protocol uses fixed payload sizes, so a valid control frame is 9 bytes and a valid telemetry frame is 10 bytes.

## Control frame: 9 bytes

Control messages travel from the BeagleBone Black to the Nano, then from the Nano to the Teensy at I2C address `0x08`.

```text
Byte:  0     1       2        3       4       5          6          7      8
       STX   TYPE    LENGTH   MODE    SPEED   ANGLE MSB  ANGLE LSB  CRC8   ETX
Value: 02    10      04       --      --      --         --         --     03
```

Payload fields:

| Byte | Field | Values |
| ---: | --- | --- |
| 3 | `MODE` | `0` Resistive, `1` Assistive, `2` Neutral |
| 4 | `SPEED` | `0` Slow, `1` Medium, `2` High |
| 5-6 | `targetAngleDeg` | Signed `int16_t`, big-endian, degrees |

Example: Assistive, High speed, target angle `-123` (`0xFF85`):

```text
02 10 04 01 02 FF 85 AA 03
```

The CRC in this example covers:

```text
10 04 01 02 FF 85
```

## Telemetry frame: 10 bytes

Telemetry messages travel from the Teensy to the Nano in response to an I2C request. The Nano forwards the same framed message to the BeagleBone over BLE.

```text
Byte:  0     1       2        3          4          5          6          7       8      9
       STX   TYPE    LENGTH   ANGLE MSB  ANGLE LSB  CURRENT MSB CURRENT LSB STATUS  CRC8   ETX
Value: 02    11      05       --         --         --         --         --      --     03
```

Payload fields:

| Byte | Field | Description |
| ---: | --- | --- |
| 3-4 | `currentAngleDeg` | Signed `int16_t`, big-endian, degrees |
| 5-6 | `currentMilliAmps` | Signed `int16_t`, big-endian, milliamps |
| 7 | `status` | Status bit field |

Status bits:

| Bit | Name | Meaning |
| ---: | --- | --- |
| 0 | `MotionActive` | A trajectory is currently active |
| 1 | `CommandTimeout` | No valid control command arrived within 250 ms |
| 2 | `InvalidCommand` | The most recent received I2C command was invalid |
| 3 | `SafetyFault` | Controller e-stop latched (over-speed, overcurrent, invalid sensor data, or tracking error). Motion commands are ignored until no command has arrived for 250 ms |

Example: angle `45`, current `1200 mA`, motion active:

```text
02 11 05 00 2D 04 B0 01 14 03
```

The CRC in this example covers:

```text
11 05 00 2D 04 B0 01
```

## Transport behavior

- The Nano validates BLE command frames before forwarding them over I2C.
- The Teensy validates the I2C control frame before applying motor state changes.
- I2C callbacks only copy or transmit bytes; decoding and motor updates happen in the normal firmware loop.
- The Teensy disables motion when a valid control command has not arrived for 250 ms.
- Invalid, oversized, truncated, or CRC-corrupted frames are discarded.
- The Nano polls telemetry periodically and only publishes a telemetry frame after successful validation.

The implementation is in `src/commProtocol.cpp` and `src/commProtocol.h`.
