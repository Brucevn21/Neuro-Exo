# Update the BBB for the Nano sequence test

For the C++ route using your existing BlueZ dependencies, with no Python or pip,
follow [the C++ sequence benchmark instructions](bluez_benchmark/README.md).
The Python route below remains available but is not required for that test.

This procedure uses the Python sender added in commit bb31eba, plus the local
protocol adapter for the current Nano service 180C / command 2A56 / telemetry
2A57. The adapter is selected by `--sequence-test` only. It accepts the current
12-byte telemetry; other legacy CLI modes still use their original protocol.
The old `neuroexo_bluez_benchmark` executable does not receive these updates.

## Prepare the Nano

This is a stationary Nano receive test. Verify `BLE_PACKET_TEST=1` in
`NeuroExoFirmware/src/Nano33BLEFirmware.ino`. Flash the Nano from the current
checkout using PlatformIO on the PC that has USB access:

```sh
pio run -d NeuroExoFirmware -e nano33ble -t upload
pio device monitor -b 115200
```

Confirm the Nano prints `BLE PACKET TEST MODE: commands are counted, NOT
forwarded to the Teensy.` Reset the Nano if needed to see the startup message.
Keep the Teensy powered and connected over I2C: the sender requires its live
telemetry. A software flag in the local source does not confirm the flashed
firmware. Do not run sequence traffic against normal motor firmware: Neutral
commands can move the joint, and sequence mode does not send a final stop.

The test firmware itself bypasses command and stop forwarding. Start with the
motor stationary and disabled; do not use this test to stop existing movement.

## Package and transfer from local WSL

```bash
cd /mnt/c/Users/bruce/SeniorProjects/Neuro-Exo
python3 H_Robotics_Files/package_sequence_test.py
read -r -p "BeagleBone IP address: " BBB_IP
scp /tmp/neuroexo-bbb-sequence.tar.gz "debian@${BBB_IP}:"
ssh "debian@${BBB_IP}"
```

The package includes local compatibility fixes. `git archive HEAD` alone does
not include uncommitted fixes. It contains no virtual environment or C++ binary.

## Install on the BBB, inside SSH

```bash
mkdir -p ~/neuroexo-sequence-bb31eba
tar -xzf ~/neuroexo-bbb-sequence.tar.gz -C ~/neuroexo-sequence-bb31eba
cd ~/neuroexo-sequence-bb31eba
cat SOURCE_VERSION.txt
sha256sum -c SHA256SUMS
sudo apt-get update
sudo apt-get install -y python3-venv python3-pip bluez
cd H_Robotics_Files
python3 -m venv .venv
source .venv/bin/activate
python3 -m pip install -r requirements-ble.txt
python3 read_outputs.py ble-control --help
bluetoothctl power on
```

All checksums must say OK. The help must list `--sequence-test`,
`--write-without-response`, and the 4..200 ms interval. Python 3.7 or newer is
required; the requirements file selects a Python 3.7-compatible Bleak release.

## Run a fresh 250/s target test

Use the Nano's current Bluetooth address. Close other BLE clients first. Keep
the Nano serial monitor open to capture its final receiver report.

```bash
read -r -p "Nano Bluetooth address: " NANO_ADDRESS
RUN_LOG="sequence-250-$(date +%Y%m%d-%H%M%S).log"
python3 -u read_outputs.py ble-control --address "$NANO_ADDRESS" \
  --mode neutral --speed slow --angle 0 --duration 10 --period-ms 4 \
  --write-without-response --sequence-test > "$RUN_LOG" 2>&1
cat "$RUN_LOG"
grep -c ' TX ' "$RUN_LOG"
```

The run prints to the log for about ten seconds, then the shell returns. This
avoids sending every TX/RX line through SSH during timing, though local logging
still adds overhead. Check for errors in the log before interpreting counts.
The TX lines must show IDs 0, 1, 2, ... (displayed as `target=... deg`). The Nano
disconnect summary should count distinct IDs instead of repeated target 20.
For a 20/s baseline, change `--period-ms 4` to `--period-ms 50` and use a new log.

250/s is requested, not guaranteed. The count from grep is completed host
writes, not a receipt count. If the ten-second run completes without errors,
compare that count with the Nano's unique count after pending data has settled.
The Nano's printed loss uses highest received sequence + 1 and cannot detect
missing packets at the end; use the completed host count to check that case.
Failed or interrupted writes have uncertain delivery. Telemetry notifications
are not per-command receipts, and this test measures only Nano reception, not
Teensy receipt or execution. No automatic less-than-5%-loss claim is made.

## Next session

SSH to the BBB, enter `~/neuroexo-sequence-bb31eba/H_Robotics_Files`, activate
`.venv`, and rerun the test commands. No C++ rebuild or BBB OS reflash is needed.
To deploy later source changes, package and transfer again, verify checksums,
and restart the Python program. Reflash a board when its firmware changes.
