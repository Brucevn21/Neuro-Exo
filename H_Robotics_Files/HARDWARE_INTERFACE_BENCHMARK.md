> Historical traffic profile: hardware-eeg models the eight-channel text stream
> from the pre-refactor Hardware_Interface.cpp. The current arm-control protocol
> is separate; see [NEUROEXO_PROTOCOL.md](NEUROEXO_PROTOCOL.md). This benchmark does
> not execute the C++ application or measure its filters/IMU/SVM.

The **hardware-eeg** benchmark profile models the active EEG/EOG debug traffic
in [Hardware_Interface.cpp](Hardware_Interface.cpp). That file is a read-only
reference; this implementation does not edit, build, or run it.

The purpose is to measure whether BLE between the BeagleBone Black Wireless
and Nano 33 BLE can carry a comparable application stream. It does not emulate
the complete hardware acquisition program or measure its Classic Bluetooth
transport.

**What matches the reference**

| Reference behavior | Benchmark representation |
|---|---|
| measureEEGEOG() builds one row of eight values per iteration. The old 25-sample loop is commented out. | One application record per iteration, containing five EEG values followed by three EOG values. |
| sendEEGEOGToApp() prints eight numbers separated by semicolons and ends with newline. | The same eight-field ASCII layout and terminating newline. |
| The sender uses default std::ostringstream precision. | Six significant digits using general notation. Local tests compare Python output with C++ ostream output for identical double values. |
| Printed lengths depend on the numeric values. | Synthetic values change each iteration; record lengths vary naturally, with no padding or truncation. |
| The acquisition loop sleeps 4000 microseconds and also does SPI, IMU, conversion/filtering, and sending work. | Source-loop mode waits 4 ms per iteration, then generates/formats/sends the record. An optional extra delay approximates measured elapsed processing time. |
| Debug mode sends all eight channels when connected. | A connected BLE sender transmits the complete record, fragmented as needed, to the instrumented receiver. |

For example, the record layout is:

~~~text
eeg1;eeg2;eeg3;eeg4;eeg5;eog1;eog2;eog3\n
~~~

These field names describe positions; transmitted fields are numbers. The
newline is one byte. The C++ amplifier's 27-byte SPI transaction is an internal
input frame, not the outgoing Bluetooth message size.

Synthetic values come from deterministic sine/cosine expressions in
[hardware_traffic.py](hardware_traffic.py). Their amplitudes are illustrative.
They are not captured measurements, a physiological model, or the result of
the original filters. Keeping the same sample sequence between runs makes
message lengths and contents repeatable while comparing transport performance.

**Preview records without either board**

Run from the repository root:

~~~bash
python3 H_Robotics_Files/read_outputs.py ble-preview --count 3
~~~

This requires no Bleak installation or Bluetooth adapter. It prints each
synthetic text record, its actual payload length, fragment count, and total
GATT value bytes. The displayed text uses an escaped newline for readability.

**Run a source-style baseline**

First follow the [BLE setup procedure](BLE_BENCHMARK.md) to install the Python
dependency and upload the matching NanoBLEBenchmark receiver. Both sender and
receiver now use benchmark protocol version 2; recompile/upload the receiver
if an earlier version is installed.

~~~bash
python3 H_Robotics_Files/read_outputs.py ble-scan

python3 H_Robotics_Files/read_outputs.py ble-benchmark \
  --address AA:BB:CC:DD:EE:FF \
  --profile hardware-eeg --pacing source-loop \
  --count 2500 --repeats 3 \
  --output ble_benchmark_cpp_loop.csv
~~~

Replace the address with the Nano's scan result. The loop is:

~~~text
wait 4 ms + configured extra processing delay
generate eight synthetic values and format one text record
send the record's BLE fragments
repeat
~~~

This follows the reference's sequential timing structure. A 4 ms sleep alone
corresponds to a nominal 250 samples/second, but actual throughput is lower
when processing and sending take time. Source-loop mode reports the achieved
records/second; it does not claim to sustain an offered rate of 250.

If measurements from the actual acquisition program show, for example, an
additional 2 ms of SPI/IMU/filter work per iteration, add:

~~~text
--processing-delay-ms 2
~~~

That option adds waiting time, not equivalent CPU work. It is available only
in source-loop mode. The example value is an assumption, not a measured value.

Source-loop PASS checks delivery/integrity and any optional performance
thresholds. A loop can remain loss-free by slowing down, so this mode by itself
cannot establish maximum sustainable input rate.

**Find the capacity boundary with a fixed-rate sweep**

~~~bash
python3 H_Robotics_Files/read_outputs.py ble-benchmark \
  --address AA:BB:CC:DD:EE:FF \
  --profile hardware-eeg --pacing rate \
  --rates 50 100 150 200 250 300 \
  --duration 10 --repeats 3 \
  --output ble_benchmark_cpp_rates.csv
~~~

This sends the same kind of records at increasing requested rates. It checks
whether the full stream arrives and whether delivered throughput meets the
target within the chosen tolerance. The default allows 5% rate shortfall;
use --rate-tolerance 0.01 for 1% or 0 for none. Default rate-mode hardware-eeg
traffic uses 250 records/second if --rates is omitted.

The test reports the highest tested rate that passed all repeats. Investigate
finer rates near the boundary, then repeat longer runs under realistic operating
conditions. A successful finite trial is not a guarantee of future zero loss.

**Keep arbitrary packet-size testing available**

The hardware-eeg profile always contains exactly eight values, so it rejects
--sizes instead of padding or changing the reference format. For artificial
size/load sweeps, use the original pattern profile:

~~~bash
python3 H_Robotics_Files/read_outputs.py ble-benchmark \
  --address AA:BB:CC:DD:EE:FF \
  --profile pattern --sizes 20 64 128 256 \
  --rates 50 100 250 --duration 10 --repeats 3 \
  --output ble_benchmark_size_sweep.csv
~~~

Pattern data tests byte-volume limits, while hardware-eeg tests a specific
application record format. Both report application packets/second and useful
payload bytes/second, separately from GATT write count and transferred bytes.

**Measurement overhead and receiver requirements**

Protocol 2 wraps each text record with a CRC32 and includes a 10-byte header
on each GATT fragment. The header includes run ID, sequence, offset, and record
length. These fields allow the Nano to detect missing, duplicate, partial, and
corrupted records, including loss of the final record. They are benchmark
instrumentation, absent from the reference C++ stream.

At the default 20-byte GATT write size, each write carries up to 10 message
bytes. A record of N bytes therefore takes ceil((N + 4) / 10) writes. Its total
GATT value bytes are N + 4 + 10 * number_of_writes. BLE link-layer overhead is
additional and is not included in this count. A negotiated larger write size
can be selected with --write-size 0.

The CSV uses payload_bytes=0 to indicate variable-length hardware-eeg records.
Use payload_min_bytes, payload_max_bytes, and payload_total_bytes for actual
submitted sizes. received_payload_bytes and received_min_bytes/max_bytes come
from the Nano. Goodput uses the actual number of CRC-valid payload bytes the
Nano received.

Your existing **180C / 2A56 angle firmware is not this benchmark receiver**.
It waits for 200 ms of silence, parses the accumulated string as an angle,
and forwards it over I2C. It neither understands these EEG records nor returns
delivery counters. The benchmark connects only to its separate custom service;
it does not send test records into the angle/I2C command path. The receiver
sketch must be compiled and uploaded for this benchmark to operate.

**What this model cannot establish**

- The historical C++ transport was Classic RFCOMM (BTPROTO_RFCOMM). This test uses BLE GATT.
  Similar data volume does not make those transports equivalent.
- SPI/ADC timing, IMU access, high-pass/H-infinity/low-pass processing, and
  application CPU use are not reproduced. Extra processing delay models
  elapsed time only; Python CPU measurements do not estimate the C++ CPU load.
- Calibration/testing matrices are internal to the original process. The
  implemented full-data CSV helper has no caller in the supplied code. Neither
  is presented as existing debug Bluetooth traffic.
- No movement classification, angle command, or I2C forwarding is inferred
  from EEG values.
- The benchmark adds framing/checksum/status work. Its measurements include
  that cost and the Python/BlueZ/Nano implementation, not an isolated radio limit.
- A dedicated receiver sketch cannot establish performance with your complete
  Nano application running. That application would need an independent test
  receive path and counters alongside its normal workload.

This is a reproducible representation of the reference's **debug output format
and sequential pacing**, useful for evaluating a proposed BLE transport. It
does not validate the current C++ arm-control program or its communication with the Nano
over BLE or that its full processing pipeline will meet a deadline.
