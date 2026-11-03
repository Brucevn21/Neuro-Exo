# C++ sequence benchmark: no Python or pip

For the **BBB -> Nano transmit-only test, with no telemetry or Teensy**, use
[TRANSMIT_ONLY.md](TRANSMIT_ONLY.md). It requires the separate
`nano33ble_rx_only` firmware and the host flags `--sequence-test --tx-only`.
The telemetry-enabled sequence procedure follows below.

This is the updated version of the earlier `neuroexo_bluez_benchmark`. It uses
BlueZ's D-Bus API through GLib/GIO and the actual firmware codec from
`NeuroExoFirmware/lib/commProtocol/src`. It needs the same C++17 compiler, CMake,
pkg-config, GLib development package and BlueZ as the earlier C++ trial.
No Bleak, Python package installation or virtual environment is used.

## Firmware prerequisite

The new `--sequence-test` option requires the Nano's existing
`BLE_PACKET_TEST=1` firmware. Verify this exact startup message in its serial
monitor before testing:

```text
BLE PACKET TEST MODE: commands are counted, NOT forwarded to the Teensy.
```

A local source flag does not prove which firmware was flashed. In this mode,
the Nano treats the 16-bit angle field as a sequence ID and does not forward
commands or stops to the Teensy. Start stationary with motor power disabled;
keep the Teensy powered and connected over I2C to supply the normal 12-byte
telemetry. Do not run sequence traffic against normal movement firmware: it
can interpret IDs as movement targets. The host cannot read back the firmware
mode. Unexpected reported motion aborts the test and triggers a stop attempt,
but this does not replace verifying the correct firmware and disabling motion.

No firmware changes are required if that packet-test image is already loaded.

## Package and transfer (local WSL terminal)

If pip is still building on the BBB, cancel it with Ctrl+C first so it does not
compete for CPU during the benchmark. In a local terminal showing `bruce@...`:

```bash
cd /mnt/c/users/bruce/seniorprojects/neuro-exo
bash H_Robotics_Files/bluez_benchmark/package.sh
scp /tmp/neuroexo-cpp-sequence.tar.gz debian@192.168.7.2:/home/debian/
ssh debian@192.168.7.2
```

Use your current BBB SSH address if it changed. Wait for scp to finish before
extracting. The source archive includes local uncommitted sequence changes;
`git archive HEAD` alone would omit them. The package excludes host binaries.

## Build (BBB SSH terminal)

Use a new directory to leave the previous executable and its results intact:

```bash
mkdir -p ~/neuroexo-cpp-sequence
tar -xzf ~/neuroexo-cpp-sequence.tar.gz -C ~/neuroexo-cpp-sequence
cd ~/neuroexo-cpp-sequence
cat SOURCE_VERSION.txt
sha256sum -c SHA256SUMS
cmake -S H_Robotics_Files/bluez_benchmark -B build -DBUILD_TESTING=OFF -DCMAKE_BUILD_TYPE=Release
cmake --build build --target neuroexo_bluez_benchmark -- -j1
./build/neuroexo_bluez_benchmark --help
```

All checksums must say OK. The help must say `v3` and list `--sequence-test`.
Build on the BBB: a binary built on the development PC is not an ARM binary.
Do not build the legacy parent `H_Robotics_Files/CMakeLists.txt` for this test.
The dependencies should already be present from the earlier successful C++
benchmark build. Only if CMake reports a missing tool/package, the Debian
packages are `build-essential cmake pkg-config libglib2.0-dev bluez`.

## Run and compare counts

Close other BLE clients connected to the Nano. Keep its serial monitor open.
Use the Nano's current MAC address, previously CA:EB:67:8F:31:59:

```bash
bluetoothctl power on
./build/neuroexo_bluez_benchmark --address CA:EB:67:8F:31:59 --sequence-test --rate 20 --duration 10
./build/neuroexo_bluez_benchmark --address CA:EB:67:8F:31:59 --sequence-test --rate 250 --duration 10
```

Run separately and save the Nano's final disconnect summary for each. Each
connection starts IDs at zero and resets the Nano counters. IDs advance per
attempted write, not per scheduled slot, so host-unsent slots do not inflate
Nano loss. The host stops after a failed write without retrying an uncertain
ID. It waits up to two seconds for pending writes before disconnecting. These
seconds are excluded from the measured sending rate. A normal sequence run
sends no final stop frame, because the Nano test mode would count it as rejected.
Runs are limited to 65,536 planned packets so there is no full ID wrap per run.

The report includes planned, attempted, submitted, host-unsent, write errors,
TX rate, telemetry rate, mean/max write-call duration and attempted ID range.
For a completed error-free run, compare host submitted count S with the matching
Nano final `Received (unique)` count R:

```text
missing at Nano = S - R
observed delivery loss % = 100 * (S - R) / S
```

Check `0 <= R <= S`, no rejected frames/duplicates and matching runs first.
If counters conflict, or a run is interrupted, disconnected or has write errors,
delivery is uncertain; do not report a definitive loss percentage. Nano's own
`Expected (highest seq + 1)` misses trailing loss, so reconcile with S.
Receiver and host counts are not automatically combined by this CLI.

For example, S=334 and R=334 means zero observed delivery loss, but only about
33.4 submissions/s over ten seconds. It does not demonstrate 250/s throughput.
For 250/s over ten seconds, the host should submit all 2,500 scheduled packets;
strictly less than 5% loss then means at least 2,376 unique Nano receipts.
`TX RATE MET` is a host submission result, not an automatic receiver-loss PASS.
Telemetry remains approximately 50/s and is not an acknowledgment of each ID.
This test does not measure Teensy receipt, command execution or motor throughput.

Optional saved host report:

```bash
set -o pipefail
./build/neuroexo_bluez_benchmark --address CA:EB:67:8F:31:59 --sequence-test --rate 250 --duration 10 2>&1 | tee "cpp-sequence-250-$(date +%Y%m%d-%H%M%S).log"
```

The existing synchronous BlueZ sending path is retained. Avoiding Bleak does
not by itself fix a host/adapter throughput limit. Use the measured mean/max
write time and host-unsent count to investigate that separately.

## Existing modes and local checks

The old monitor, movement-control, stop-only and write-response options remain.
Use movement-control modes only with matching normal firmware; they are not
sequence-loss tests. Original stop and fault handling remains for those modes.

```bash
cmake -S H_Robotics_Files/bluez_benchmark -B /tmp/neuroexo-cpp-sequence-build -DBUILD_TESTING=ON -DCMAKE_BUILD_TYPE=Release
cmake --build /tmp/neuroexo-cpp-sequence-build --parallel 2
ctest --test-dir /tmp/neuroexo-cpp-sequence-build --output-on-failure
```

Tests use the real firmware codec and a private fake BlueZ D-Bus service,
including the actual CLI with slow writes, failed writes and unexpected motion.
They do not require Bluetooth hardware and do not measure hardware throughput.
