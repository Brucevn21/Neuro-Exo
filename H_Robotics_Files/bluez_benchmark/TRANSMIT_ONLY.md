# BeagleBone to Nano transmit-only benchmark

Uses C++/BlueZ, without Python, Bleak, pip or a Teensy. The BeagleBone sends
9-byte numbered writes without ATT responses. It does not subscribe to or wait
for telemetry. Standard BLE connection-management and link-layer traffic still
exists; only the application data is one-way. The Nano counts packets in its
BLE write callback and prints receiver statistics over USB serial.

## 1. Upload the dedicated Nano image (PC)

In a Windows PlatformIO Core CLI with the Nano connected by USB:

```powershell
cd C:\Users\bruce\SeniorProjects\Neuro-Exo\NeuroExoFirmware
pio run -e nano33ble_rx_only -t upload
pio device monitor -b 115200
```

If uploading from WSL instead, attach the Nano to WSL first, then use:

```bash
cd /mnt/c/users/bruce/seniorprojects/neuro-exo
/home/bruce/.platformio/penv/bin/pio run -d NeuroExoFirmware -e nano33ble_rx_only -t upload
/home/bruce/.platformio/penv/bin/pio device monitor -b 115200
```

Reset the Nano if necessary to see its startup messages. Require this banner:

```text
RX-ONLY BENCHMARK: no telemetry, no I2C, motor relay OFF.
```

Keep the USB serial monitor open for the results. Motor power should be off;
the dedicated image keeps its relay output LOW even with a healthy supply.
The Teensy and SDA/SCL connections are not required. The existing `nano33ble`
build still retains its previous behavior; select `nano33ble_rx_only` explicitly.
The host cannot identify the flashed build automatically. Do not send sequence
IDs to normal motor firmware, where they can be interpreted as target angles.

## 2. Copy the new source (local WSL terminal)

These commands run at the PC's `bruce@...` prompt, not on the BBB:

```bash
cd /mnt/c/users/bruce/seniorprojects/neuro-exo
bash H_Robotics_Files/bluez_benchmark/package.sh /tmp/neuroexo-tx-only.tar.gz
scp /tmp/neuroexo-tx-only.tar.gz debian@192.168.7.2:/home/debian/
ssh debian@192.168.7.2
```

Wait for scp to finish successfully. The archive includes local working-tree
changes; no Git commit or push is required to transfer them.

## 3. Build on the BBB (SSH terminal)

```bash
mkdir -p ~/neuroexo-tx-only
tar -xzf ~/neuroexo-tx-only.tar.gz -C ~/neuroexo-tx-only
cd ~/neuroexo-tx-only
cat SOURCE_VERSION.txt
sha256sum -c SHA256SUMS
cmake -S H_Robotics_Files/bluez_benchmark -B build -DBUILD_TESTING=OFF -DCMAKE_BUILD_TYPE=Release
cmake --build build --target neuroexo_bluez_benchmark -- -j1
./build/neuroexo_bluez_benchmark --help
```

Checksums must say OK. The help must say `v3` and include `--tx-only`. The
existing C++ compiler, CMake, pkg-config, GLib/GIO and BlueZ from your previous
C++ benchmark are sufficient. No Python environment or pip installation is
used. Build the standalone `bluez_benchmark` directory, not its legacy parent.

## 4. Run on the BBB

Close any other BLE clients connected to the Nano. Replace the MAC below if it
changed; the Nano should advertise as `Nano33BLE_Master`. Run separately:

```bash
bluetoothctl power on
./build/neuroexo_bluez_benchmark --address CA:EB:67:8F:31:59 --sequence-test --tx-only --rate 20 --duration 10
./build/neuroexo_bluez_benchmark --address CA:EB:67:8F:31:59 --sequence-test --tx-only --rate 250 --duration 10
```

The host reports `TX-ONLY` and `Telemetry: DISABLED`. It streams for ten seconds,
allows up to two seconds for pending writes, then disconnects. The Nano prints
the final receiver summary in its USB serial terminal. The host sends no final
stop frame in this stationary test, as that would be counted as a rejected test
packet. `--write-response` and motion-control options are rejected with TX-only.

To save a host report:

```bash
set -o pipefail
./build/neuroexo_bluez_benchmark --address CA:EB:67:8F:31:59 --sequence-test --tx-only --rate 250 --duration 10 2>&1 | tee "tx-only-250-$(date +%Y%m%d-%H%M%S).log"
```

Compare the host's actual `submitted` count S with the matching Nano final
`Received (unique)` count R. For a completed run without write errors, invalid
frames or inconsistent counts: observed loss % = 100 * (S - R) / S. Scheduled
host-unsent slots were never attempted and must not be called receiver loss.
The Nano's own highest-ID estimate misses trailing loss; use S to check that.
Writes that fail or are interrupted have uncertain delivery. TX RATE MET is
only a sender-rate result, not a receiver-loss PASS. The Nano's counters must
be collected separately; the host does not retrieve them over BLE.

For 250/s over ten seconds, require 2,500 submissions and at least 2,376 unique
receipts for strictly less than 5% loss. A run submitting only 334 packets does
not demonstrate 250/s, even if every submitted packet arrives.

In TX-only mode no telemetry is available to detect motion, firmware faults or
stale sensor values; it is restricted to this stationary receiver image. BLE
disconnect and write-error handling remains active. Other benchmark modes
retain their telemetry and stop/fault behavior. The synchronous sending path is
unchanged, so this mode does not guarantee that the previous TX limit is fixed.

## Local verification

The CTest suite checks the real firmware codec, the actual receiver sketch with
hardware stubs (no I2C, no notifications, relay LOW, valid sequence counters),
and the actual CLI against a fake BlueZ service with no telemetry characteristic.
It also covers disconnects and existing bidirectional modes. Firmware builds
can be checked with `pio run -d NeuroExoFirmware -e nano33ble_rx_only -e nano33ble`.
These checks do not replace a hardware throughput measurement.
