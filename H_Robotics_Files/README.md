# NeuroExo BeagleBone interface

The BeagleBone uses one BLE connection with two directional characteristics:
command writes to the Nano, and telemetry notifications back from the Teensy
through the Nano. The interface displays real motor angle, signed current in mA,
and MotionActive, CommandTimeout, InvalidCommand, and SafetyFault status bits.

- [Protocol and UUIDs](NEUROEXO_PROTOCOL.md)
- [Build, flash, monitor, and control instructions](BEAGLEBONE_READER_PROCEDURE.md)
- [Nano BLE/I2C bridge](../NeuroExoFirmware/src/nanoBridge.cpp)
- [C++ application interface](Hardware_Interface.hpp)

From this directory, after installing `requirements-ble.txt`:

```sh
python3 read_outputs.py ble-scan
python3 read_outputs.py ble-monitor --address AA:BB:CC:DD:EE:FF --hex
```

The 9-byte command and 10-byte telemetry formats match the supplied Teensy
`commProtocol.cpp`. The former simulated trial protocol, synthetic traffic
generators, and benchmark sketches/tools have been retired.

`main.cpp` and `Hardware_Interface.*` are a partial copy of the original EEG/EOG
application. Several drivers, calibration/testing modules, and training scripts
are absent. Build the standalone `neuroexo_motor` CMake target or use the Python
reader independently. `Hardware_Interface` exposes explicit motor controls and
cached live telemetry; the old simulated trial APIs have been removed.

The reader also retains `stdin` for EEG/EOG text logs and the historical `tcp`
and Classic `bluetooth` readers. Those formats are separate from Nano BLE motor
communication. `manual.pdf` and `nsf-tcp-protocol.md` describe the legacy rebless
robot, not the current Teensy protocol.
