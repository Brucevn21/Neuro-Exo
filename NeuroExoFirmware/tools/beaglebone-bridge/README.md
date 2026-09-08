# NeuroExo BeagleBone Bridge

Runs on the BeagleBone Black, sitting between the patient app
(`tools/patient-app`) and the Nano 33 BLE (`Nano33BLEFirmware.ino`):

```
app (Web Bluetooth) <--BLE--> BeagleBone Black <--BLE--> Nano 33 BLE <--I2C--> Teensy 4.1
                              (this bridge)
```

The bridge plays **two BLE roles at once** on the BeagleBone's Bluetooth
adapter:

- **Central** toward the Nano 33 BLE - connects to it as a normal BLE client
  (via [bleak](https://github.com/hbldh/bleak)), the same way the app used to.
- **Peripheral** toward the app - advertises its own GATT server (via
  [bless](https://github.com/kevincar/bless)) using the *same* service/characteristic
  UUIDs as the Nano, so `tools/patient-app` doesn't need any UUID changes -
  it just connects to the BeagleBone instead of the Nano directly.

## Why the bridge, and not the app, picks the target angle

The app is now a pure visualizer. When the patient presses Start/Next Trial,
the app sends a command packet with only mode + speed (the target field is
ignored). The bridge decodes that, rolls its own random target angle between
20&deg; and 120&deg;, and forwards a new packet - same mode/speed, bridge-chosen
target - down to the Nano. Telemetry and stop requests pass through unmodified
in both directions.

## Characteristics (service `180C`)

| UUID   | Direction        | Purpose                                                        |
|--------|-------------------|-----------------------------------------------------------------|
| `2A56` | app -> bridge     | Start/next-trial request: 7-byte JointPacket (mode+speed used, target ignored) |
| `2A57` | bridge -> app     | Telemetry: 7-byte JointPacket (current angle, target angle, current), relayed from the Nano at 20Hz |
| `2A58` | app -> bridge     | Stop: any 1-byte write triggers an immediate mid-motion halt   |

`protocol.py` is a Python port of `lib/commProtocol/src/commProtocol.cpp` and
must stay in sync with it (and with `tools/patient-app/protocol.js`).

## Running it

```bash
cd tools/beaglebone-bridge
python3 -m venv venv
source venv/bin/activate
pip install -r requirements.txt
python3 bridge.py
```

The Nano 33 BLE must already be powered and running `Nano33BLEFirmware.ino`
(advertising as `Nano33BLE_Master`) before starting the bridge - it scans for
that name on startup and exits if it isn't found within 20 seconds.

Once running, the bridge advertises its own `NeuroExo-BBB-Bridge` peripheral;
point `tools/patient-app` at it exactly as you would have pointed it at the
Nano before (the app's device picker will now show the BeagleBone's name
instead).

## Known limitations / things to verify on real hardware

- **This hasn't been tested against a real BeagleBone Black or real BLE
  radios.** The bridge logic (target substitution, telemetry/stop relay) is
  covered by mocked unit tests, but the actual dual central+peripheral BLE
  role on your specific adapter is not verified here.
- **Simultaneous central + peripheral on one adapter is hardware/BlueZ
  dependent.** Some Bluetooth chipsets/drivers don't support running both
  roles at once. If `bridge.py` can't advertise while connected to the Nano,
  you likely need either a second BLE adapter (e.g. a USB dongle in addition
  to onboard BT) or a BlueZ/adapter combination that supports concurrent
  roles.
- **No reconnect logic.** If the Nano or the app disconnects mid-session, the
  bridge does not currently attempt to reconnect - it will need a restart.
- Run this as a long-lived service (e.g. a systemd unit) rather than an
  interactive script for real sessions, and make sure the BLE adapter has the
  necessary permissions (BlueZ typically needs the process to run as root or
  have the `cap_net_admin`/`cap_net_raw` capabilities).
