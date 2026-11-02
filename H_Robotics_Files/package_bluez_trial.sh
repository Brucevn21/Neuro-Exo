#!/usr/bin/env bash
# Bundle matching C++ sources only; no Python, virtualenv, or generated binaries.
set -euo pipefail
repo_dir=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)
archive_path=${1:-/tmp/neuroexo-bluez-trial.tar.gz}
tar -czf "$archive_path" -C "$repo_dir" \
  H_Robotics_Files/CMakeLists.txt \
  H_Robotics_Files/BluezGatt.cpp \
  H_Robotics_Files/BleMotorClient.cpp \
  H_Robotics_Files/BleMotorClient.hpp \
  H_Robotics_Files/PacketGatt.hpp \
  H_Robotics_Files/PacketTrial.hpp \
  H_Robotics_Files/examples/motor_monitor.cpp \
  H_Robotics_Files/examples/packet_benchmark.cpp \
  H_Robotics_Files/BLUEZ_TRIAL.md \
  NeuroExoFirmware/src/commProtocol.h \
  NeuroExoFirmware/src/commProtocol.cpp \
  NeuroExoFirmware/src/packetDemoProtocol.h
printf 'Created %s\n' "$archive_path"
