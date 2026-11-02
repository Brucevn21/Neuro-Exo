# 250 packets/s trial using C++ and BlueZ

This trial measures **BeagleBone -> Arduino Nano 33 BLE -> BeagleBone**.
The BeagleBone uses the existing `BluezGatt.cpp` connection through BlueZ's
D-Bus GATT API and GLib/GIO. There is no Bleak, pip, or virtual environment.
The Nano uses ArduinoBLE firmware. A Teensy is not required for this trial.

Each numbered packet is 9 bytes; the Nano returns a 10-byte notification with
the same sequence number. At 250 packets/s for 10 seconds, the target is
2,500 packets sent and 2,500 echoes received. This tests application packets,
not individual radio packets or BLE retransmissions. It does not measure EEG
processing or motor-control throughput, and 250 is a test target, not a BLE limit.

## 1. Upload the matching Nano firmware from your PC

Connect the Nano to your PC by USB. Power it from USB during the trial.
The diagnostic firmware keeps the Nano's motor-supply relay output low.
It replaces the normal Nano application until you upload that application again.

If using **Windows VS Code**, open this repository, select PlatformIO's
**Open PlatformIO Core CLI**, then run:

```powershell
cd C:\Users\bruce\seniorprojects\neuro-exo
pio run -d NeuroExoFirmware -e nano33ble_packet_loopback -t upload
```

If using a **fresh WSL terminal** with the Nano already attached to WSL:

```bash
cd /mnt/c/users/bruce/seniorprojects/neuro-exo
source /home/bruce/.platformio/penv/bin/activate
pio run -d NeuroExoFirmware -e nano33ble_packet_loopback -t upload
```

Use the platform with USB access to the board. WSL requires USB passthrough;
the existing [USB setup instructions](BLE_BENCHMARK.md) cover that option.
For port-selection errors, run `pio device list`, then append
`--upload-port YOUR_PORT` to the upload command. A serial monitor is unnecessary.

The Nano should advertise as **NeuroExoLoopback**. The older
`nano33ble_packet_demo` image uses a Teensy and is not the firmware for this trial.

## 2. Transfer the matching C++ files from WSL

Open a local **WSL terminal**, not a terminal currently showing
`debian@beaglebone`. Run each command separately:

```bash
cd /mnt/c/users/bruce/seniorprojects/neuro-exo
```

```bash
bash H_Robotics_Files/package_bluez_trial.sh
```

Enter the same IP address you normally use for SSH when prompted:

```bash
read -r -p "BeagleBone IP address: " BBB_IP
```

```bash
scp /tmp/neuroexo-bluez-trial.tar.gz "debian@${BBB_IP}:neuroexo-bluez-trial.tar.gz"
```

```bash
ssh "debian@${BBB_IP}"
```

The next commands run on the BeagleBone. The trial goes in its own directory so
the older Python scripts in `~/neuroexo/H_Robotics_Files` cannot get mixed in.

## 3. Build on the BeagleBone

If a pip installation is still running in another terminal, you can cancel that
installation with Ctrl+C; it is not needed for this route. If `(.venv)` appears
in your current terminal, run `deactivate` once. A fresh SSH session normally
does not have that Python environment active.

```bash
sudo apt-get update
```

```bash
sudo apt-get install -y bluez build-essential cmake pkg-config libglib2.0-dev
```

These are Debian packages for the existing C++/BlueZ implementation; apt skips
packages already installed. No `libbluetooth-dev` or Python Bluetooth package
is needed because the program uses BlueZ's D-Bus API through GIO.

```bash
mkdir -p ~/neuroexo-bluez-trial
```

```bash
tar -xzf ~/neuroexo-bluez-trial.tar.gz -C ~/neuroexo-bluez-trial
```

```bash
cd ~/neuroexo-bluez-trial
```

```bash
cmake -S H_Robotics_Files -B build -DBUILD_TESTING=OFF -DCMAKE_BUILD_TYPE=Release
```

```bash
cmake --build build --target neuroexo_packet_benchmark -- -j1
```

```bash
./build/neuroexo_packet_benchmark --help
```

The help must show `--address`, `--rate`, and `--duration` and mention
`nano33ble_packet_loopback`. No `read_outputs.py` command is used for this trial.

## 4. Find the Nano address

Start the interactive Bluetooth tool:

```bash
bluetoothctl
```

At its `[bluetooth]#` prompt enter each command separately:

```text
power on
scan on
```

Wait for a line like this (the address is only an example):

```text
[NEW] Device AA:BB:CC:DD:EE:FF NeuroExoLoopback
```

Write down the actual address, then enter:

```text
scan off
quit
```

If there is no Bluetooth controller, run `bluetoothctl list` to confirm. The
BeagleBone needs a Linux-supported BLE adapter; a board without built-in
Bluetooth needs a USB Bluetooth adapter. If the Bluetooth service is stopped,
run `sudo systemctl start bluetooth` and try again. Stop any other program
connected to the Nano before the trial.

## 5. Run the trial

From the BeagleBone shell in `~/neuroexo-bluez-trial`, enter the actual Nano
Bluetooth address when prompted (this is different from the BeagleBone's SSH IP):

```bash
read -r -p "Nano Bluetooth address: " NANO_ADDRESS
```

```bash
./build/neuroexo_packet_benchmark --address "$NANO_ADDRESS" --rate 250 --duration 10
```

Expect connection/setup time, 10 seconds of sending, then roughly 2 seconds for
late echoes and the final receiver-count snapshot. Progress and results appear
directly in the WSL SSH terminal; no Nano serial terminal is required.

Example of a successful result, **illustrative, not a hardware measurement**:

```text
PASS | TX=250.00/s | RX during sending=249.80/s
Planned=2500 attempted=2500 submitted=2500 host-unsent=0
Nano accepted=2500 BBB echoes=2500 missing echoes=0
Nano queue drops=0 notify failures=0 Nano duplicates=0 Nano invalid=0
BBB duplicates=0 invalid=0 unexpected=0
Not received by Nano=0 | Nano accepted but no return=0 | round-trip loss=0.00%
```

- **Submitted**: BlueZ accepted the write call; this alone is not proof the Nano received it.
- **Nano accepted**: the Nano's own count of valid, unique incoming packets.
- **BBB echoes**: unique, valid responses actually received by the BeagleBone.
- **Host-unsent**: scheduled slots the BeagleBone missed; these are not radio loss.
- **Missing echoes**: attempted packet IDs without a valid response after the drain.
- **Nano accepted but no return**: includes Nano queue/notification failures and the return path.
- **PASS**: all scheduled packets were submitted and returned, no recorded errors,
  fresh consistent receiver counts, and measured TX and RX during the send window
  each at least 99% of the requested rate. Late arrivals can resolve loss, but
  cannot inflate the receive rate during the trial.
- **FAIL**: loss, errors, missed host slots, or insufficient TX/RX rate.
- **INCONCLUSIVE**: failed write with uncertain delivery, wrong firmware,
  disconnect, unavailable final counts, or inconsistent counts. Do not interpret
  an inconclusive result as a measured loss percentage. A Ctrl+C-aborted run is
  likewise not a completed measurement.

Exit status is 0 for PASS, 2 for FAIL, and 1 for INCONCLUSIVE/setup error.
Writes use BlueZ `WriteValue` with type `command` (without ATT response), while
reset/snapshot controls use `request`. Receiving uses `StartNotify` and a
callback for every notification, not a latest-value mailbox. See the
[BlueZ GATT API](https://bluez.readthedocs.io/en/latest/gatt-api/).

To save terminal output while preserving the benchmark exit status:

```bash
set -o pipefail
```

```bash
./build/neuroexo_packet_benchmark --address "$NANO_ADDRESS" --rate 250 --duration 10 2>&1 | tee "trial-250-$(date +%Y%m%d-%H%M%S).log"
```

Repeat the trial several times. If 250 fails, rerun with `--rate 100`, then
`--rate 200`, to establish a working baseline. This measures this software,
adapter, payload size and connection's achieved rate; it does not establish a
universal BLE maximum. A successful Nano-only test also does not prove the
Nano/Teensy/EEG motor application can sustain that rate under load.

## Later, from a freshly opened WSL terminal

SSH to the BeagleBone as usual, then:

```bash
cd ~/neuroexo-bluez-trial
read -r -p "Nano Bluetooth address: " NANO_ADDRESS
./build/neuroexo_packet_benchmark --address "$NANO_ADDRESS" --rate 250 --duration 10
```

No activation, package installation, rebuilding, or firmware upload is needed
again unless the relevant files/firmware change.

To restore the normal Nano application, run on the PC with Nano USB access:

```bash
pio run -d NeuroExoFirmware -e nano33ble -t upload
```
