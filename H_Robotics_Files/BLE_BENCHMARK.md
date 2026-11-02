# 250 Hz BeagleBone → Nano → Teensy packet demo

For a **BeagleBone/Nano-only trial using C++ and BlueZ, without Bleak**, follow
[BLUEZ_TRIAL.md](BLUEZ_TRIAL.md). The Python/Teensy instructions below are a
different test route.

Run `read_outputs.py ble-benchmark` to attempt **250 outgoing test packets and
250 returning echoes per second**. Every 9-byte test packet has a sequence ID
and CRC. The Nano forwards it over I2C; a 10-byte BLE echo is sent only after
the Teensy validates that sequence. The BeagleBone shows actual rates, received
counts, missing echo IDs, loss percentages, and round-trip timing.

This is an opt-in communication demo. It does not collect EEG or run the motor
controller. The demo holds the Teensy motor enable/PWM pins LOW and the Nano
motor-supply relay LOW. Both boards must have independent USB power. The usual
`teensy41` and `nano33ble` build environments remain available to restore normal
operation afterward.

## Start from a freshly opened terminal

**Yes, you can see the benchmark output in a WSL terminal.** Use that terminal
to SSH into the BeagleBone, then start the Python benchmark on the BeagleBone.
The BeagleBone sends/receives BLE packets; SSH carries the displayed results
back to your WSL window. WSL itself needs no Bluetooth adapter for this workflow.

There are three different kinds of environment here:

| Environment | Where it runs | How you select it |
| --- | --- | --- |
| PlatformIO tools | Development PC, in Windows or WSL | Open PlatformIO Core CLI, or activate its `penv` below |
| Firmware build environment | Selected by PlatformIO | `-e teensy41_packet_demo` or `-e nano33ble_packet_demo` |
| Benchmark Python environment | BeagleBone, after SSH login | `source .venv/bin/activate` in `H_Robotics_Files` |

You only need PlatformIO for building/uploading firmware. Once the boards are
flashed, an ordinary WSL terminal with SSH is sufficient to run the test again.

### 1. Wire and power the two boards

Keep the existing I2C wiring:

| Nano 33 BLE | Teensy 4.1 |
| --- | --- |
| A4 / SDA | 18 / SDA |
| A5 / SCL | 19 / SCL |
| GND | GND |

Power **both boards over USB**; no motor supply or encoder is needed for this
demo. Use the existing 3.3 V I2C pull-ups described in the
[connection map](../NeuroExoFirmware/lib/pinMap/README.md). A USB connection alone
does not replace the SDA/SCL wires. The Nano is the I2C master at 100 kHz, and
the Teensy responds at `0x08`.

### 2. Open the PlatformIO tools on your development PC

Choose the option matching the terminal you want to use. The project paths
below match Bruce's checkout; change the username/folder if yours is elsewhere.

**Option A — fresh WSL/Ubuntu terminal, using the existing installation:**

Open Windows Terminal and select your Ubuntu/WSL profile. If you opened
PowerShell instead, enter `wsl` first. Then run:

```bash
cd /mnt/c/users/bruce/seniorprojects/neuro-exo
source ~/.platformio/penv/bin/activate
command -v pio
pio --version
```

`command -v pio` should show `/home/bruce/.platformio/penv/bin/pio` here. The
prompt may also show `(penv)`. Activate it again in each freshly opened WSL
terminal. In this checkout the managed executable was verified as PlatformIO
Core 6.1.19; `/usr/bin/pio` is an older, broken installation that raises a
`PlatformioCLI ... resultcallback` error. Do not use that executable.

You can also call `~/.platformio/penv/bin/pio` directly without activation.
If the activation file is missing on a different computer, install PlatformIO
IDE in VS Code in that environment first; the IDE includes Core. See the
[PlatformIO shell setup](https://docs.platformio.org/en/latest/core/installation/shell-commands.html).

Check the build from this WSL terminal:

```bash
pio run -d NeuroExoFirmware -e teensy41_packet_demo -e nano33ble_packet_demo
```

This builds both images without needing USB access. For **uploading from WSL**,
the physical boards must also be attached to WSL using the USB steps below.
Opening the project under `/mnt/c` does not attach its USB devices.

**Option B — Windows VS Code, simplest upload path when USB is still owned by Windows:**

1. Open a local Windows VS Code window. If the lower-left corner says
   `WSL: ...`, use a local Windows window for this option.
2. Open `C:\Users\bruce\SeniorProjects\Neuro-Exo\NeuroExoFirmware` with
   **File → Open Folder**.
3. If needed, install the **PlatformIO IDE** extension in Windows and allow
   its initial setup to finish.
4. Click the PlatformIO ant icon, then **Quick Access → Miscellaneous →
   PlatformIO Core CLI**. This opens a terminal with PlatformIO available,
   as described in the [official IDE guide](https://docs.platformio.org/en/latest/integration/ide/vscode.html#platformio-core-cli).
5. In that terminal, move to the repository root and check the tools:

```powershell
cd C:\Users\bruce\SeniorProjects\Neuro-Exo
pio --version
pio device list
```

For an ordinary fresh Windows **PowerShell** window, after installing PlatformIO
in Windows, the equivalent activation for this window is:

```powershell
cd C:\Users\bruce\SeniorProjects\Neuro-Exo
$env:Path = "$env:USERPROFILE\.platformio\penv\Scripts;" + $env:Path
pio --version
```

The Windows and WSL installations are separate. Activating one does not
activate the other.

### 3. Upload both demo images once

In the Windows or WSL PlatformIO terminal selected above, from the repository
root, run these separately:

```sh
pio run -d NeuroExoFirmware -e teensy41_packet_demo -t upload
pio run -d NeuroExoFirmware -e nano33ble_packet_demo -t upload
```

For WSL, complete the USB attachment steps below first. For Windows, the boards
must be available to Windows; detach them from WSL if previously attached.
Wait for `SUCCESS` after each upload. If requested by the Teensy uploader,
press its program button. Close other programs holding a board's serial port.

If Nano port autodetection is ambiguous, run `pio device list` and append
`--upload-port COM5` on Windows or `--upload-port /dev/ttyACM0` on WSL/Linux to
the Nano upload command, replacing the example with the actual port. The Teensy
uses its USB bootloader; it may not appear as a serial port while programming.

### 4. Copy the current scripts and connect to the BeagleBone

Open a fresh **WSL terminal**. The BeagleBone must be powered, reachable over
your network/USB networking, and accepting SSH connections. Replace
`your-user@YOUR_BEAGLEBONE_IP` below with your actual SSH username and address;
it is a placeholder, not a discovered device address. If USB networking is not
reachable from WSL, use the BeagleBone's reachable LAN address.

```bash
cd /mnt/c/users/bruce/seniorprojects/neuro-exo
BBB_TARGET='your-user@YOUR_BEAGLEBONE_IP'
ssh "$BBB_TARGET" 'mkdir -p ~/Neuro-Exo/H_Robotics_Files'
scp H_Robotics_Files/read_outputs.py \
    H_Robotics_Files/ble_benchmark.py \
    H_Robotics_Files/neuroexo_motor.py \
    H_Robotics_Files/neuroexo_protocol.py \
    H_Robotics_Files/requirements-ble.txt \
    "${BBB_TARGET}:Neuro-Exo/H_Robotics_Files/"
ssh "$BBB_TARGET"
```

This copies the five files needed by the CLI, replacing those files in the
specified BeagleBone folder with the current versions. It does not copy a PC's
Python virtual environment. If you already updated these files on the board,
you can skip the copy and just SSH in. Use your configured SSH authentication.

**After SSH connects, the commands you type run on the BeagleBone**, even though
you are still looking at a WSL terminal window. `hostname` identifies the
machine you are controlling. Its home directory is different from the WSL
home directory; the Windows `/mnt/c/...` project path is not used here.

### 5. First-time Python setup on the BeagleBone, then run

Run the following **inside the SSH session**. If Python's venv/pip tools or
`bluetoothctl` are missing on the BeagleBone's Debian image, install them once:

```bash
sudo apt-get update
sudo apt-get install python3-venv python3-pip bluez
```

Then create the board's Python environment (skip creation if it already exists):

```bash
cd ~/Neuro-Exo/H_Robotics_Files
python3 -m venv .venv
source .venv/bin/activate
python3 -m pip install -r requirements-ble.txt
bluetoothctl list
bluetoothctl power on
```

`bluetoothctl list` must show a controller. The BLE adapter belongs to the
**BeagleBone**, not your PC. If none appears, check the board's BLE adapter and
Bluetooth service before running the test. There are no new Python dependencies
beyond the existing `requirements-ble.txt`.

**Older BeagleBone Python versions:** check `python3 --version` in the activated
environment. The scripts require Python 3.7 or newer. The updated requirements
select Bleak 0.20.2 on Python 3.7, and Bleak 0.22 or newer on Python 3.8+.
[Bleak 0.20.2](https://pypi.org/project/bleak/0.20.2/) supports Python 3.7;
[Bleak 0.22](https://pypi.org/project/bleak/0.22.0/) requires Python 3.8+.
If your copied requirements still say only `bleak>=0.22,<4`, a Python 3.7 board
reports “No matching distribution found”. Update that file from this checkout,
or install the compatible version directly in the already activated environment:

```bash
python3 -m pip install "bleak==0.20.2"
python3 -m pip check
```

Run commands one at a time. There is no need to recreate the virtual environment
for this version-selection fix. The local checkout's updated requirements file
does not automatically update a previously copied file on the BeagleBone.

With one demo Nano advertising nearby:

```bash
python3 -u read_outputs.py ble-benchmark --rate 250 --duration 10
```

The `-u` option makes Python output unbuffered. Progress and the final report
appear directly in this WSL/SSH terminal. It finds `NeuroExoPacketDemo`, confirms
the Teensy is responding, resets counters, sends for ten seconds, and then drains
pending echoes. **You do not need `pio device monitor`**: the benchmark report
comes from the Python program, not either board's USB serial port.

To save a new result file as well:

```bash
python3 -u read_outputs.py ble-benchmark --rate 250 --duration 10 \
  --json "benchmark-250-$(date +%Y%m%d-%H%M%S).json"
```

The JSON file is saved on the **BeagleBone** in the current folder. The timestamp
avoids reusing a filename; existing files are never overwritten. To copy a
result back, open another WSL terminal, substitute your SSH login, and run:

```bash
cd /mnt/c/users/bruce/seniorprojects/neuro-exo
BBB_TARGET='your-user@YOUR_BEAGLEBONE_IP'
scp "${BBB_TARGET}:Neuro-Exo/H_Robotics_Files/ACTUAL_RESULT_FILENAME.json" .
```

Replace `ACTUAL_RESULT_FILENAME.json` with a filename listed by
`ls benchmark-250-*.json` in the BeagleBone session.

To select a particular Nano:

```sh
python3 read_outputs.py ble-benchmark --scan
python3 read_outputs.py ble-benchmark --address AA:BB:CC:DD:EE:FF --rate 250 --duration 30
```

`ble-benchmark --scan` looks for the demo. The existing `ble-scan` command still
looks for normal `NanoNeuroExo` motor firmware.

### 6. Run again from a fresh WSL terminal

After the initial uploads, file copy, and Python setup, only these steps are
needed. Start in WSL, substituting your SSH login:

```bash
ssh your-user@YOUR_BEAGLEBONE_IP
```

After login, run on the BeagleBone:

```bash
cd ~/Neuro-Exo/H_Robotics_Files
source .venv/bin/activate
bluetoothctl power on
python3 -u read_outputs.py ble-benchmark --rate 250 --duration 10
```

Keep the session open until the summary appears. `Ctrl+C` interrupts a run;
`exit` after completion returns you to the local WSL shell. You do not need to
rebuild, reflash, or reinstall packages for each run.

## Optional: attach boards to WSL for uploading

This is needed only if you choose to **upload firmware from WSL**. Viewing
BeagleBone output through SSH does not need USB forwarding.

WSL 2 requires USB device forwarding for direct board access. Microsoft's
[USB connection guide](https://learn.microsoft.com/en-us/windows/wsl/connect-usb)
documents `usbipd-win` setup, requirements, and permissions. Keep a WSL terminal
open, then in an **administrator Windows PowerShell** window:

```powershell
# Install once, if usbipd is not already installed.
winget install --interactive --exact dorssel.usbipd-win
usbipd list
```

Use each board's actual BUSID from that list in place of `ACTUAL_BUS_ID`:

```powershell
usbipd bind --busid ACTUAL_BUS_ID
usbipd attach --wsl --busid ACTUAL_BUS_ID
```

Repeat for the other board. `bind` needs administrator privileges; `attach`
can also run from an ordinary PowerShell window. Then in **WSL**:

```bash
cd /mnt/c/users/bruce/seniorprojects/neuro-exo
source ~/.platformio/penv/bin/activate
lsusb
pio device list
pio run -d NeuroExoFirmware -e teensy41_packet_demo -t upload
pio run -d NeuroExoFirmware -e nano33ble_packet_demo -t upload
```

If `lsusb` is unavailable, install WSL's `usbutils` package. If an upload loses
the device after a reset, check `usbipd list` and reattach the current device as
needed. For Linux device permission errors, use the udev guidance in Microsoft's
USB guide. If forwarding becomes troublesome, use the Windows upload option
above. To release an attached board back to Windows, run in PowerShell:

```powershell
usbipd detach --busid ACTUAL_BUS_ID
```

## Can the benchmark itself run directly in WSL?

It can only do so when WSL has a usable Linux BLE controller, the necessary
kernel/USB support, and a working BlueZ service; Bleak's
[Linux backend](https://bleak.readthedocs.io/en/stable/backends/linux.html)
uses BlueZ through D-Bus. Seeing Bluetooth devices in
Windows does not establish that `bluetoothctl list` inside WSL has a controller.
USB forwarding alone also does not guarantee Bluetooth driver support.

For the **BeagleBone throughput measurement**, run the Python command after
SSH login as above. Starting it locally in WSL would measure the PC/WSL sender
and its adapter instead. The WSL terminal can display either process; `hostname`
before running helps confirm which machine is executing the benchmark.

## What the terminal means

While sending, the terminal prints a progress line once per second. `pending`
means an echo has not arrived yet; it is not declared missing until the drain
period and final snapshot finish. There is no per-packet console or disk I/O.

Example final output **for illustration, not a hardware measurement**:

```text
FAIL: requested 250 Hz | actual TX 249.9/s | RX during sending 248.7/s
  Attempted=2500 submitted=2500 host-unsent=0 write-errors=0
  Nano accepted=2498 | Teensy accepted=2495 | echoes received=2490
  Nano queue drops=3 | I2C errors=0 | echo submission failures=0
  Missing: host->Nano=2 | Nano->Teensy=3 | Teensy->host=5 | round-trip=0.400%
  Echo duplicates=0 invalid=0 unexpected=0
  Round-trip time: mean=8.10 ms p95=13.20 ms max=22.40 ms
  Missing echo IDs (first 20): [...]
```

| Field | Interpretation |
| --- | --- |
| Attempted | Numbered writes started by the BeagleBone |
| Submitted | Bleak calls completed; without-response writes are only queued locally at this point |
| Host-unsent | Scheduled slots the sender missed; these packets never entered the transport |
| Nano accepted | Valid, increasing sequence IDs observed by the Nano's BLE callback |
| Teensy accepted | Valid, increasing sequence IDs accepted by the Teensy's diagnostic handler |
| Echoes received | Unique valid echoes received at the BeagleBone, including the drain period |
| Host → Nano missing | Attempted minus Nano accepted, for a completed error-free write run |
| Nano → Teensy missing | Nano accepted minus Teensy accepted, including queue drops and I2C delivery failures |
| Teensy → host missing | Teensy accepted minus unique echoes, covering the I2C return, Nano notification, and host receive path |
| Round-trip loss | `(attempted - unique echoes) / attempted × 100`, after draining |

These are **application packet counts**, not counts of physical radio packets
or link-layer retransmissions. Return-path loss is not necessarily BLE radio
loss: it can also occur while reading the Teensy's I2C response.

The JSON file records every attempted ID, every received ID, every missing echo
ID, all receiver counters, timing metrics, and the result for each run. A missing
echo identifies an incomplete round trip; receiver totals locate the stage in
aggregate, not the exact forward/return fate of every missing ID.

- **PASS:** every scheduled write completed, no observed missing/corrupt/duplicate
  packets or receiver errors, and both actual TX and RX-during-send rates reach
  at least 99% of the request. The 1% allowance accounts for measurement boundaries
  and final echoes still in flight. Host-unsent slots must still be zero.
- **FAIL:** the completed test observed loss/errors or did not sustain the requested
  rate. Receiving all packets later during draining does not by itself pass the
  receive-rate check.
- **INCONCLUSIVE:** a write timed out, statistics were unavailable, or counters
  were inconsistent. A timed-out write might have arrived; it is not silently
  classified as a dropped packet.

Exit status: 0 for all runs passing; 2 for a completed test with a failing run;
1 for an error/inconclusive run; 130 for Ctrl+C. Interrupted runs are not reported
as passing measurements.

## Find the highest rate that passes

```sh
python3 read_outputs.py ble-benchmark \
  --rates 50,100,150,200,250,300 --duration 30 --repeat 3 \
  --json benchmark-sweep.json
```

Each rate/repetition gets a fresh session and reset counters. The final line
reports the highest tested rate that passed **every repetition**. Add smaller
steps near the transition if desired. This is zero observed loss over those
tests, not a guarantee for every future radio condition or workload.

To compare the acknowledged BLE write mode used by existing motor commands:

```sh
python3 read_outputs.py ble-benchmark --rate 250 --duration 10 --write-response
```

The default demo uses `BLEWriteWithoutResponse` and notifications so it can
attempt 250 Hz without waiting for an ATT response per data write. The normal
motor service still uses acknowledged writes. Bleak documents the difference
between queuing a write and waiting for its response in its
[write API](https://bleak.readthedocs.io/en/stable/api/client.html#bleak.BleakClient.write_gatt_char).
Receiver counters, not successful write calls alone, establish delivery.

The demo requests a 7.5 ms BLE connection interval; the central chooses the
actual parameters. It uses 9-byte BLE TX and 10-byte BLE RX frames, matching
the sizes of normal motor frames. The diagnostic I2C reply is **25 bytes** to
include receiver counters, rather than the normal 10-byte telemetry frame.
The Teensy validates diagnostic packets in a bounded callback; the motor
control loop and sensor workload do not run. Thus a passing demo establishes
this diagnostic communication path's result, not the full application's
maximum under load. No fixed BLE-wide limit of 250 samples/s is assumed.

## Files involved

| File | Purpose |
| --- | --- |
| [read_outputs.py](read_outputs.py) | Existing CLI; repaired duplicate conditional/missing import and registered benchmark alongside normal modes |
| [ble_benchmark.py](ble_benchmark.py) | One new Python module: pacing, sequence tracking, reporting, scan and rate sweep |
| [packetDemo.cpp](../NeuroExoFirmware/src/packetDemo.cpp) | One new firmware source with Nano/Teensy build branches; motor outputs remain disabled |
| [packetDemoProtocol.h](../NeuroExoFirmware/src/packetDemoProtocol.h) | Demo frame codec, statistic fields and bounded queue |
| [platformio.ini](../NeuroExoFirmware/platformio.ini) | Two opt-in demo environments |
| [test_ble_benchmark.py](tests/test_ble_benchmark.py), [packet_demo_check.cpp](tests/packet_demo_check.cpp) | Host validation with inert BLE/I2C substitutes |

The production `main.cpp`, `nanoBridge.cpp` and command/telemetry codecs need no
demo edits. `requirements-ble.txt` selects a Bleak version for the installed
Python, including Python 3.7 on older BeagleBone images. The demo reuses the existing CRC
implementation in `commProtocol.cpp` and `neuroexo_protocol.py`.

The separate demo GATT service is `b7e21000-6a2b-4f10-9c31-8b674045a901`.
Its characteristic UUID first groups are `b7e21001` (data write), `b7e21002`
(echo notify), `b7e21003` (start/snapshot write), and `b7e21004` (read-only
statistics); all remaining UUID groups match the service. Frame types
`0x30..0x35` are separate from motor
control types. Integers are big-endian. Statistics are a frozen 65-byte framed
snapshot containing the 15 uint32 fields defined by `FIELDS` in the Python
module; BLE may read that snapshot in multiple MTU fragments.

## Validation and restoration

Run all host checks (Python plus the compiled demo-firmware harness):

```sh
python3 -m unittest discover -s H_Robotics_Files/tests -p 'test_*.py' -v
```

These checks require Python and `g++` but no Bluetooth hardware. They exercise
queue overflow, missing/duplicate/corrupt packets, I2C errors, notification
failures, slow senders, late echoes, ambiguous writes, and Python/C++ wire parity.
Hardware throughput must be measured on the actual BeagleBone and boards.

After the demo, restore the normal controller and bridge from the repository root:

```sh
pio run -d NeuroExoFirmware -e teensy41 -t upload
pio run -d NeuroExoFirmware -e nano33ble -t upload
```
