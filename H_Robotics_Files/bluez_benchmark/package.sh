#!/usr/bin/env bash
set -euo pipefail
trial_repo=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/../.." && pwd)
trial_archive=${1:-/tmp/neuroexo-cpp-sequence.tar.gz}
trial_archive=$(realpath -m -- "$trial_archive")
trial_stage=$(mktemp -d /tmp/neuroexo-cpp-manifest.XXXXXX)
trap 'rm -f -- "$trial_stage/SOURCE_VERSION.txt" "$trial_stage/SHA256SUMS"; rmdir -- "$trial_stage"' EXIT
cd "$trial_repo"
trial_files=(
  H_Robotics_Files/bluez_benchmark/CMakeLists.txt
  H_Robotics_Files/bluez_benchmark/BluezLink.hpp
  H_Robotics_Files/bluez_benchmark/Trial.hpp
  H_Robotics_Files/bluez_benchmark/main.cpp
  H_Robotics_Files/bluez_benchmark/test_trial.cpp
  H_Robotics_Files/bluez_benchmark/test_link.cpp
  H_Robotics_Files/bluez_benchmark/test_receiver.cpp
  H_Robotics_Files/bluez_benchmark/receiver_stubs/Arduino.h
  H_Robotics_Files/bluez_benchmark/receiver_stubs/ArduinoBLE.h
  H_Robotics_Files/bluez_benchmark/receiver_stubs/Wire.h
  H_Robotics_Files/tests/nano_stubs/Arduino.h
  H_Robotics_Files/bluez_benchmark/README.md
  H_Robotics_Files/bluez_benchmark/TRANSMIT_ONLY.md
  H_Robotics_Files/bluez_benchmark/package.sh
  NeuroExoFirmware/platformio.ini
  NeuroExoFirmware/src/Nano33BLEFirmware.ino
  NeuroExoFirmware/lib/commProtocol/src/commProtocol.h
  NeuroExoFirmware/lib/commProtocol/src/commProtocol.cpp
)
printf 'ProtocolDemo C++ benchmark v3 with transmit-only support\nBase commit: %s\nIncludes local working-tree benchmark and RX-only Nano changes. See SHA256SUMS for exact source files.\n' \
  "$(git rev-parse HEAD)" > "$trial_stage/SOURCE_VERSION.txt"
sha256sum "${trial_files[@]}" > "$trial_stage/SHA256SUMS"
tar -czf "$trial_archive" "${trial_files[@]}" -C "$trial_stage" SOURCE_VERSION.txt SHA256SUMS
printf 'Created %s\n' "$trial_archive"
