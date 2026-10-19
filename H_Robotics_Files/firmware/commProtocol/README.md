# commProtocol (vendored copy)

commProtocol.h and commProtocol.cpp are unmodified copies of
`NeuroExoFirmware/lib/commProtocol/src/` from the `ProtocolDemo` branch at
commit `39eac5b`. The Nano 33 BLE bridge (`Nano33BLEFirmware.ino`) and the
Teensy 4.1 controller (`main.cpp`) on that branch compile the same files.

The BeagleBone client (`ble/`) compiles this copy so all three devices share
one encoder, decoder, and CRC. Do not edit it here. When the firmware copy
changes, copy both files again and run the tests:

~~~bash
git show ProtocolDemo:NeuroExoFirmware/lib/commProtocol/src/commProtocol.h > H_Robotics_Files/firmware/commProtocol/commProtocol.h
git show ProtocolDemo:NeuroExoFirmware/lib/commProtocol/src/commProtocol.cpp > H_Robotics_Files/firmware/commProtocol/commProtocol.cpp
~~~

The frame format is described in [docs/NEUROEXO_PROTOCOL.md](../../docs/NEUROEXO_PROTOCOL.md).
