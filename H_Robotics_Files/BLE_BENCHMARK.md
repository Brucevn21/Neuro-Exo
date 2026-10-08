This benchmark measures how much synthetic application data a **BeagleBone
Black Wireless running Python 3** can deliver to an **Arduino Nano 33 BLE over
Bluetooth Low Energy**. It runs independently of the incomplete EEG application.

For traffic modeled on Hardware_Interface.cpp, see the
[hardware-eeg profile and comparison](HARDWARE_INTERFACE_BENCHMARK.md).
The profile models the earlier EEG text format. For the current C++ arm protocol,
see [NEUROEXO_PROTOCOL.md](NEUROEXO_PROTOCOL.md).

Python is the BLE central/sender. The Nano is the BLE peripheral/receiver.
The sender selects payload sizes, rates, run lengths, and repetitions. The Nano
reassembles packets, validates CRC32, and counts unique sequence numbers.
Python reads the Nano counters and prints results in the terminal and a CSV.

Use **ble-scan** and **ble-benchmark** for live tests, and **ble-preview**
for a hardware-free preview of synthetic EEG/EOG records.
The original demo/stdin/tcp/Classic Bluetooth reader modes remain available.
The earlier [EEG procedure](BEAGLEBONE_READER_PROCEDURE.md) describes a different
transport/application and does not apply to this benchmark.

**1. Load the receiver onto the Nano.**

Open [NanoBLEBenchmark.ino](NanoBLEBenchmark/NanoBLEBenchmark.ino) in Arduino
IDE. Keep receiver_protocol.h in the same folder. Install the official
Arduino Mbed OS Nano Boards package and ArduinoBLE library version 1.3.7.
Select **Arduino Nano 33 BLE** and its USB port, then compile and upload.
Sender and receiver must both use protocol version 2; re-upload if the old
benchmark receiver is installed. The 180C/2A56 angle/I2C sketch is not compatible.
Uploading replaces the application currently on the Nano.

Alternatively, use the standalone PlatformIO project:

~~~bash
pio run -d H_Robotics_Files/NanoBLEBenchmark
pio run -d H_Robotics_Files/NanoBLEBenchmark -t upload
~~~

Power the Nano. It advertises as **NanoBLEBench**. The sketch runs without
the Serial Monitor. It avoids printing individual packets during measurement
because terminal output would affect throughput. USB carries upload/power;
the test data travels over BLE.

**2. Prepare Python on the BeagleBone.**

Copy H_Robotics_Files to a checkout of this project on the board. Run the
following from that checkout's repository root. Use Python 3.9 or newer and
a working BlueZ installation (5.55 or newer). Older board images may need
updating. The BlueZ requirement follows [Bleak support documentation](https://github.com/hbldh/bleak#features). Check the versions and adapter:

~~~bash
python3 --version
bluetoothctl --version
bluetoothctl show
~~~

If needed, enable the adapter:

~~~bash
bluetoothctl power on
~~~

Install Bleak in a virtual environment:

~~~bash
python3 -m venv H_Robotics_Files/.venv
source H_Robotics_Files/.venv/bin/activate
python3 -m pip install -r H_Robotics_Files/requirements-ble.txt
~~~

If venv is missing, install the OS's python3-venv package, then repeat setup.
No EEG amplifier, SVM software, robot arm, or Classic RFCOMM pairing is needed.
Use the BeagleBone's own Linux Bluetooth adapter to measure that board;
running Python on a PC measures the PC-to-Nano path. WSL without a Bluetooth
controller cannot perform this test.

**3. Find the Nano.**

~~~bash
python3 H_Robotics_Files/read_outputs.py ble-scan
~~~

Copy the address beside NanoBLEBench and replace AA:BB:CC:DD:EE:FF in the
commands below. The sketch uses an open custom GATT service and does not
require separate pairing. Disconnect other BLE apps from the Nano.

**4. Run a short connection check.**

~~~bash
python3 H_Robotics_Files/read_outputs.py ble-benchmark \
  --address AA:BB:CC:DD:EE:FF \
  --sizes 20 --rates 10 --count 100 --repeats 1 \
  --output ble_benchmark_check.csv
~~~

This sends 100 application packets, each carrying 20 data bytes, at a requested
10 packets/second. Expect received=100/100 and lost=0 if delivery succeeds.
The CSV filename must be new; existing files are never overwritten.

**5. Sweep sizes and rates.**

~~~bash
python3 H_Robotics_Files/read_outputs.py ble-benchmark \
  --address AA:BB:CC:DD:EE:FF \
  --sizes 20 64 128 256 \
  --rates 10 25 50 100 200 400 \
  --duration 10 --repeats 3 \
  --output ble_benchmark_sweep.csv
~~~

Each repeat sends ceil(rate * duration) packets. A slow sender takes longer
and fails the rate criterion; duration is a nominal target, not an exact wall
time. The default run timeout is 300 seconds. This example has 72 runs and
takes at least about 12 minutes. Longer runs print submission progress.

The summary identifies the highest **tested** rate that passes **every repeat**
at each size. Test finer rates near the transition between passing and failing.
Extend the range if all rates pass. A finite sweep cannot prove an absolute
maximum; results can vary between runs and environments.

For an unpaced saturation test, use rate 0 with an explicit count:

~~~bash
python3 H_Robotics_Files/read_outputs.py ble-benchmark \
  --address AA:BB:CC:DD:EE:FF \
  --sizes 20 64 --rates 0 --count 2000 --repeats 3 \
  --output ble_benchmark_unpaced.csv
~~~

This sends as fast as Python/the BLE API permits. An external source might
produce data faster than this sender can accept; paced sweeps identify that
limitation as a throughput shortfall.

**6. Interpret packet size and performance.**

For the default pattern profile, sizes are **application payload bytes**, from
1 to 4096. The hardware-eeg profile instead generates variable-length text.
Each packet also has a 4-byte CRC32. Each GATT write adds a 10-byte fragment
header. Large packets are split and reconstructed automatically. At the default
20-byte write size, a 20-byte payload plus CRC uses three GATT writes (20, 20,
and 14 bytes), totaling 54 GATT value bytes.

The **--write-size** argument controls GATT value bytes including the header.
It defaults to 20 for compatibility. A value of 0 uses the reported negotiated
maximum, capped at the receiver's limit of 244. Explicit values above the
negotiated limit are rejected. Application payloads can exceed that limit
through fragmentation. GATT writes may themselves span multiple radio frames.

The script uses Bleak's max_write_without_response_size property. Older BlueZ
versions can report only 20 bytes. See
[Bleak's characteristic documentation](https://bleak.readthedocs.io/en/latest/api/index.html#bleak.backends.characteristic.BleakGATTCharacteristic.max_write_without_response_size).

Data defaults to **--write-mode without-response**. Configuration/status use
writes with response. Successful OS submission is not treated as proof of
delivery: the Nano counters determine loss. The sender does not retry test
data at the application level. Compare **--write-mode with-response** in a
separate run if needed. See
[Bleak's write API](https://bleak.readthedocs.io/en/latest/api/client.html#bleak.BleakClient.write_gatt_char).

This measures application delivery, not individual radio-frame loss or the
BLE controller's hidden retransmissions.

| Result | Meaning |
|---|---|
| expected / sent | Configured packets / packets fully submitted by Python. |
| received / lost | Unique CRC-valid packets at the Nano / expected minus received, including missing final packets. |
| duplicates / corrupt / invalid | Fragments for already completed sequences / complete packets with bad CRC / malformed or out-of-order fragments. |
| incomplete / reordered / foreign | Abandoned partial packets / completed packets below the highest completed sequence / fragments from another run. |
| delivered_pps / goodput_bytes_s | Unique packets or useful payload bytes divided by the time from first send attempt to receiver confirmation. |
| sending_seconds / completion_seconds | Submission window / delivery-confirmation window. |
| profile / pacing | Pattern or hardware-eeg data; rate or source-loop timing. |
| payload_bytes | Configured pattern size, or 0 for variable hardware-eeg records. |
| payload_total_bytes / payload_min_bytes / payload_max_bytes | Actual submitted data bytes and record size range. |
| received_payload_bytes / received_min_bytes / received_max_bytes | Nano counts of CRC-valid data bytes and received record size range. |
| write_p95_ms / write_max_ms | BLE API call duration on the BeagleBone, not one-way radio latency. p95 has approximately 1% histogram resolution. |
| schedule_lag_p95_ms | Sender lateness relative to its next planned packet; no catch-up bursts after missed deadlines. |
| max_poll_gap_us | Longest gap between Nano BLE.poll() calls during a run; a responsiveness indicator. |
| sender_cpu_percent | Python process CPU as a percentage of the sending window; excludes BlueZ/kernel CPU. |
| writes / fragments / gatt_value_bytes | GATT writes submitted / handled by the Nano / submitted value bytes including headers and CRC. |

In rate pacing, a default **PASS** requires all expected packets, zero receiver errors, and
delivery of at least 95% of the requested rate. Set **--rate-tolerance 0.01**
for 1% allowed shortfall, or 0 for none. Confirmation-query overhead is included
in the measured time, so very short runs can fail this criterion. Source-loop
pacing has no offered-rate criterion: it waits 4 ms plus any configured work
delay each iteration and reports achieved throughput. Use a rate sweep to
establish whether a fixed producer rate can be sustained.

Optional **--max-write-ms** limits p95 write-call duration and
**--max-poll-gap-ms** limits the Nano's polling gap. These thresholds depend
on your workload. A receiver-only sketch cannot prove that a later sensor,
classifier, or control workload will maintain its performance.

The sender waits up to **--drain-timeout** seconds (default 3) after submission
for queued data, then freezes the receiver counters. Individual BLE operations
also have a timeout. Disconnect, timeout, or Ctrl+C records an unfinished run
as **ERROR**, without claiming zero loss. Completed CSV rows are flushed
immediately. A setup error before any run can leave a header-only CSV.

**7. Confirm the candidate rate over a longer run.**

For example, if 100 packets/second at 64 bytes passes:

~~~bash
python3 H_Robotics_Files/read_outputs.py ble-benchmark \
  --address AA:BB:CC:DD:EE:FF \
  --sizes 64 --rates 100 --duration 120 --repeats 3 \
  --run-timeout 180 --rate-tolerance 0.01 \
  --output ble_benchmark_soak.csv
~~~

Record board/OS/library versions, GATT write size, distance, antenna
orientation, power sources, and nearby traffic. Repeat under deployment
conditions, including Wi-Fi activity and the intended Nano workload. Leave
operating margin below a boundary that varies between runs.

Press Ctrl+C to abort. Python attempts to stop/disconnect; the Nano freezes
on disconnect and advertises again. This sketch has no robot motor commands.

Exit codes: 0 = all runs passed; 2 = completed run failed; 1 = setup or
communication error; 130 = keyboard interruption.

**Protocol reference and local validation**

All integers are little-endian. The service UUID is
b7e10000-6a2b-4f10-9c31-8b674045a901. The first group changes to b7e10001,
b7e10002, and b7e10003 for control, data, and status, respectively.

| Message | Layout |
|---|---|
| START, 12 bytes | version u8=2, op u8=1, run u32, size u16 (0 permits variable sizes), expected u32 |
| SNAPSHOT, 7 bytes | version u8=2, op u8=2, run u32, page u8 |
| STOP, 6 bytes | version u8=2, op u8=3, run u32 |
| Data | run u32, sequence u16, offset u16, payload length u16, payload/CRC fragment |
| Status, 20 bytes | version u8, state u8, page u16, run u32, three u32 counters |

CRC32 covers run u32 + sequence u16 + payload length u16 + payload; its little-endian value is
appended before fragmentation. Sequences range from 0 to count-1; each run
supports at most 65536 packets. An 8 KiB bitmap tracks unique completions.
One packet is reassembled at a time with ordered fragments. Pages 0–5 match
PAGE_FIELDS in [ble_benchmark.py](ble_benchmark.py). STOP freezes the counters.

Run the local tests from the repository root:

~~~bash
python3 -m unittest discover -s H_Robotics_Files/tests -v
~~~

Tests need g++ and compile the same C++ reassembly/CRC/counter core used by the
Nano into a temporary native library. Python exercises it with dropped,
duplicated, reordered, and corrupted data. Simulated BLE calls check rate
failures and error reporting. These tests do not measure the radio or replace
compiling and uploading the firmware for the actual Nano.
