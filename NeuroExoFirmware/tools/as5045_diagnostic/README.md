# AS5045 Diagnostic

This sketch bypasses `EncDeg`, `EncCalib`, PID control, telemetry, and the motor driver. It polls the AS5045 and prints the raw 12-bit position, the latest five-bit status field, the library's parity/status validity result, and idle pin levels.

## Run it

1. Disconnect or disable the motor power stage. Keep the encoder powered.
2. Open `tools/as5045_diagnostic/as5045_diagnostic.ino` in Arduino IDE, or copy it into a temporary Teensy sketch.
3. Install or expose this repository's `lib/AS5045` library.
4. Select Teensy 4.1, upload, and open a 115200 baud serial monitor.
5. Rotate the encoder shaft slowly while watching `raw` and `valid`.

The firmware's active pin map is `CS=2`, `CLK=1`, and `DATA=0`. The constructor order is CS, CLK, DATA.

## Interpret the result

- `raw=0` continuously with `valid=NO`: the `-65.21` symptom is explained by the current calibration. A raw zero maps to exactly `-65.21` degrees with `encOffset=65.21`, `encRange={180,-180}`. Check DATA wiring, ground, supply, CS, and clock first.
- A changing `raw` with `valid=NO`: inspect parity, magnet alignment, air gap, and the status bits. Do not trust the calibrated angle until validity is good.
- A changing `raw` with `valid=YES`: the sensor and serial link are working. The fault is in the application path, calibration, or displayed/telemetry value.
- `raw=4095` continuously: check for a DATA line pulled high, a missing sensor ground, or a miswired data connection.
- A constant nonzero value: compare it with the shaft position, then inspect CS timing, the DATA line, and whether the magnet is slipping on the shaft.

For this AS5045 interface there are no ABZ interrupt counts to inspect. The device is read synchronously through clock, data, and chip select pins.

## Application firmware checks

The current main loop calls `myAS5045.read()` but does not reject an invalid sample before applying calibration. Add logging around one sample while diagnosing:

```cpp
encBinary = myAS5045.read();
Serial.printf("raw=%u status=0x%02X valid=%s\n",
              encBinary, myAS5045.status(), myAS5045.valid() ? "YES" : "NO");
```

Then gate control use on `myAS5045.valid()`, retaining the last known good angle or entering a fault state instead of feeding a failed sample into velocity and PID calculations.

## Prioritized checklist

1. Confirm the serial monitor is 115200 baud and the diagnostic sketch is actually running.
2. Confirm the Teensy 4.1 wiring: encoder VCC, common GND, CS to 2, CLK to 1, DATA to 0. Keep wires short and ensure the connector has not shifted.
3. Confirm the encoder supply is within its datasheet range at the sensor while the motor is enabled. Look for reset or ground-bounce symptoms.
4. Confirm CS idles high and CLK idles high. During a read, CS must select the sensor before the clock transitions and DATA must be sampled on the expected edge.
5. Rotate the magnet by hand and check whether `raw` changes. A frozen zero points to DATA/CS/CLK or a failed sensor; a frozen count with valid status points to mechanical coupling or magnet alignment.
6. Check the diametric magnet orientation, axial/radial alignment, air gap, and shaft set screw. A magnet that rotates independently of the shaft produces misleading readings.
7. Use `status` and `valid`: parity failures, invalid cordic output, or magnetic-field increase/decrease flags indicate a sensor or magnet problem rather than an offset problem.
8. Only after raw data is valid, verify `EncDeg` scaling, `encOffset`, the reversed `{180,-180}` range, and every display/telemetry variable for shadowing or stale copies.
9. Keep the encoder read inside the main loop or a controlled timer and log the sample immediately after reading. Do not read once during setup and reuse that value.