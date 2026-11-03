#!/usr/bin/env python3
"""Package the working-tree Python sequence sender, including local fixes."""
import argparse
import hashlib
import io
from pathlib import Path
import subprocess
import tarfile


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", default="/tmp/neuroexo-bbb-sequence.tar.gz")
    args = parser.parse_args()
    root = Path(__file__).resolve().parents[1]
    files = ["H_Robotics_Files/" + name for name in (
        "read_outputs.py", "neuroexo_motor.py", "neuroexo_protocol.py",
        "protocol_demo_sequence.py", "ble_benchmark.py", "requirements-ble.txt",
        "SEQUENCE_TEST_BBB.md")]
    # Read all inputs before creating the archive; never produce a partial bundle
    # because a required source was missing.
    entries = {name: (root / name).read_bytes() for name in files}
    commit = subprocess.check_output(["git", "rev-parse", "HEAD"], cwd=root, text=True).strip()
    entries["SOURCE_VERSION.txt"] = (
        "Base Git commit: " + commit + "\n"
        "Packaged working-tree files, including local sequence-protocol fixes.\n"
        "SHA256SUMS identifies the exact files in this bundle.\n"
        "Requires Nano BLE_PACKET_TEST=1 and current 12-byte Teensy telemetry.\n"
    ).encode()
    entries["SHA256SUMS"] = "".join(
        hashlib.sha256(data).hexdigest() + "  " + name + "\n"
        for name, data in entries.items()).encode()
    destination = Path(args.output).resolve()
    with tarfile.open(str(destination), "w:gz") as archive:
        for name, data in entries.items():
            info = tarfile.TarInfo(name)
            info.size = len(data)
            info.mode = 0o644
            archive.addfile(info, io.BytesIO(data))
    print("Created " + str(destination))
    print("Base commit: " + commit)


if __name__ == "__main__":
    main()
