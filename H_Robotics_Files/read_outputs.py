#!/usr/bin/env python3
"""Display the text formats in this folder. Run with --help for input sources.

Original reader modes use only the standard library. TCP sends command 0 only.
Bluetooth mode receives Classic RFCOMM; it does not connect to a BLE service.
ble-scan / ble-benchmark use Bleak to test a Nano 33 BLE (see BLE_BENCHMARK.md).
ble-preview displays synthetic Hardware_Interface records without hardware.
protocol-* modes explain and exercise the bidirectional NeuroExo trial protocol.
CAL, TEST, RAW, and IMU records require an added exporter in the C++ producer.
"""

import argparse
import math
import socket
import sys
import time
from datetime import datetime


EEG = tuple(f"eeg{i}" for i in range(1, 6))
EOG = tuple(f"eog{i}" for i in range(1, 4))
ACCEL = ("accel_x", "accel_y", "accel_z")
CAL = EEG + EOG + ACCEL + ("position", "imagine", "move")
TEST = EEG + EOG + ACCEL + ("position", "move_predicted", "imagine", "move")
MAX_LINE = 4096


def numeric_record(kind, fields, names):
    """Validate a record before assigning labels; preserve its printed precision."""
    if len(fields) != len(names):
        return None
    try:
        if not all(math.isfinite(float(value)) for value in fields):
            return None
    except ValueError:
        return None
    if kind in ("CAL", "TEST") and all(float(value) == 9999 for value in fields):
        return kind, "end-of-trial/baseline sentinel (9999), not a measurement"
    return kind, "  ".join(f"{name}={value}" for name, value in zip(names, fields))


def decode_line(line):
    """Recognize documented streams; retain unknown or malformed lines as text."""
    line = line.strip()
    if not line:
        return "LOG", "(blank line)"

    # These explicit tags are optional future C++ stdout exporters, not existing
    # network messages. See README.md for their column order and an example.
    tagged = {"CAL": CAL, "TEST": TEST, "RAW": EEG + EOG}
    tag, separator, rest = line.partition(",")
    if separator and tag in tagged:
        record = numeric_record(tag, rest.split(","), tagged[tag])
        return record or ("UNPARSED", line)
    if line.startswith("IMU:"):
        names = ACCEL + ("gyro_x", "gyro_y", "gyro_z")
        return numeric_record("IMU", line[4:].split(), names) or ("UNPARSED", line)
    if ";" in line:
        record = numeric_record("EEG/EOG", line.split(";"), EEG + EOG)
        if record:
            return record
    if "," in line:
        names = EEG + ("position", "imagine", "move")
        record = numeric_record("FULL", line.split(","), names)
        if record:
            return record
    if "\t" in line:
        names = ("drive_mode", "actual_curr", "actual_pos", "max_curr")
        record = numeric_record("ARM", line.split("\t"), names)
        if record:
            return record
    if "MODEL_FILE:" in line:
        return "MODEL", line.split("MODEL_FILE:", 1)[1].strip()
    return "LOG", line


def display(line):
    kind, message = decode_line(line)
    # This timestamp is receipt/display time, not a sensor acquisition timestamp.
    stamp = datetime.now().isoformat(timespec="milliseconds")
    print(f"{stamp} [{kind}] {message}", flush=True)


def read_record(stream):
    """Buffered readline handles split packets and multiple lines per packet."""
    line = stream.readline(MAX_LINE + 1)
    if not line:
        raise ConnectionError("peer disconnected")
    if len(line) > MAX_LINE:
        raise ValueError(f"received a line longer than {MAX_LINE} characters")
    if not line.endswith("\n"):
        raise ConnectionError("peer disconnected during an incomplete record")
    return line


def poll_arm(args):
    print(f"Polling {args.host}:{args.port} with status command 0.", file=sys.stderr)
    with socket.create_connection((args.host, args.port), timeout=args.timeout) as peer:
        with peer.makefile("r", encoding="utf-8", errors="replace") as stream:
            received = 0
            while True:
                peer.sendall(b"0\n")
                display(read_record(stream))
                received += 1
                if args.count and received >= args.count:
                    return
                time.sleep(args.interval)


def listen_bluetooth(args):
    if not hasattr(socket, "AF_BLUETOOTH") or not hasattr(socket, "BTPROTO_RFCOMM"):
        raise RuntimeError("Bluetooth mode requires Python with Linux BlueZ RFCOMM support")
    with socket.socket(socket.AF_BLUETOOTH, socket.SOCK_STREAM, socket.BTPROTO_RFCOMM) as server:
        server.bind((args.bind, args.channel))
        server.listen(1)
        print(
            f"Waiting for one RFCOMM sender on {args.bind}, channel {args.channel}.\n"
            "Configure the C++ sender with this computer's Bluetooth MAC address.",
            file=sys.stderr,
        )
        peer, address = server.accept()
        print(f"Bluetooth sender connected: {address}", file=sys.stderr)
        with peer:
            with peer.makefile("r", encoding="utf-8", errors="replace") as stream:
                while True:
                    display(read_record(stream))


def demo():
    print("Synthetic format examples only; no hardware is accessed.", file=sys.stderr)
    examples = [
        "[TOP OF MAIN]SENS_ACC: 6384, SENS_GYR: 131",
        "0.00001;0.00002;0.00003;0.00004;0.00005;0.00006;0.00007;0.00008",
        "0.00001,0.00002,0.00003,0.00004,0.00005,12.5,1,0",
        "0\t0.125\t12.500\t0.500",
        "MODEL_FILE: /example/subject_model/model.pkl",
        # The following records illustrate proposed stdout exporters.
        "RAW,0.00001,0.00002,0.00003,0.00004,0.00005,0.00006,0.00007,0.00008",
        "IMU: 0.1 0.2 0.3 0.4 0.5 0.6",
        "CAL,1,2,3,4,5,6,7,8,0.1,0.2,0.3,12.5,1,0",
        "TEST,1,2,3,4,5,6,7,8,0.1,0.2,0.3,12.5,1,1,0",
        "CAL," + ",".join(["9999"] * 14),
    ]
    for line in examples:
        display(line)


def positive_float(value):
    result = float(value)
    if not math.isfinite(result) or result <= 0:
        raise argparse.ArgumentTypeError("must be a finite number greater than zero")
    return result


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    modes = parser.add_subparsers(dest="mode", required=True)
    from ble_benchmark import add_modes, run as run_ble
    add_modes(modes)
    from neuroexo_trial import add_modes as add_trial_modes, run as run_trial
    add_trial_modes(modes)
    modes.add_parser("demo", help="show synthetic examples without hardware")
    modes.add_parser("stdin", help="read logs or telemetry from a pipe or redirected file")
    tcp = modes.add_parser("tcp", help="poll the documented robot TCP status interface")
    tcp.add_argument("host", help="robot IP address or hostname")
    tcp.add_argument("--port", type=int, default=11999)
    tcp.add_argument("--interval", type=positive_float, default=0.5, help="seconds between polls")
    tcp.add_argument("--timeout", type=positive_float, default=5.0, help="connect/read timeout in seconds")
    tcp.add_argument("--count", type=int, default=0, help="number of polls; 0 runs until Ctrl+C")
    bt = modes.add_parser("bluetooth", help="receive C++ telemetry as a Linux RFCOMM server")
    bt.add_argument("--bind", default="00:00:00:00:00:00", help="local adapter MAC; default: any")
    bt.add_argument("--channel", type=int, default=1, choices=range(1, 31))
    args = parser.parse_args()
    if args.mode.startswith("protocol-"):
        return run_trial(args)
    if args.mode in ("ble-scan", "ble-benchmark", "ble-preview"):
        return run_ble(args)
    if args.mode == "tcp" and (args.count < 0 or not 1 <= args.port <= 65535):
        parser.error("--count must be nonnegative and --port must be between 1 and 65535")

    try:
        if args.mode == "demo":
            demo()
        elif args.mode == "stdin":
            for line in sys.stdin:
                display(line)
        elif args.mode == "tcp":
            poll_arm(args)
        else:
            listen_bluetooth(args)
    except KeyboardInterrupt:
        print("\nReader stopped.", file=sys.stderr)
    except (OSError, ValueError, RuntimeError) as error:
        print(f"Reader error: {error}", file=sys.stderr)
        return 1
    return 0


if __name__ == "__main__":
    sys.exit(main())
