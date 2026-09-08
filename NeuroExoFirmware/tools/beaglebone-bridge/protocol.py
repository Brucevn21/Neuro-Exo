"""
Python port of lib/commProtocol/src/commProtocol.cpp's 7-byte JointPacket.
Must stay byte-for-byte in sync with that file (and tools/patient-app/protocol.js).

Byte layout (56 bits total):
    Byte 0     : Mode (bits 1:0) + Speed (bits 3:2)
    Bytes 1-2  : Current Angle [degrees], int16, MSB first
    Bytes 3-4  : Target Angle [degrees], int16, MSB first
    Bytes 5-6  : Current [mA], int16, MSB first
"""

import struct

PACKET_SIZE = 7

# Matches Nano33BLEFirmware.ino's BLE UUIDs (16-bit aliases under the standard
# Bluetooth base UUID) - the app-facing side of this bridge reuses the same
# ones so tools/patient-app doesn't need to change anything.
SERVICE_UUID = "0000180c-0000-1000-8000-00805f9b34fb"
COMMAND_CHAR_UUID = "00002a56-0000-1000-8000-00805f9b34fb"
TELEMETRY_CHAR_UUID = "00002a57-0000-1000-8000-00805f9b34fb"
STOP_CHAR_UUID = "00002a58-0000-1000-8000-00805f9b34fb"


class Mode:
    RESISTIVE = 0
    ASSISTIVE = 1
    NEUTRAL = 2


class Speed:
    SLOW = 0
    MEDIUM = 1
    HIGH = 2


def encode_joint_packet(mode, speed, current_angle_deg=0, target_angle_deg=0, current_milliamps=0):
    byte0 = ((speed & 0x03) << 2) | (mode & 0x03)
    return struct.pack(">Bhhh", byte0, current_angle_deg, target_angle_deg, current_milliamps)


def decode_joint_packet(data):
    byte0, current_angle_deg, target_angle_deg, current_milliamps = struct.unpack(">Bhhh", bytes(data))
    return {
        "mode": byte0 & 0x03,
        "speed": (byte0 >> 2) & 0x03,
        "current_angle_deg": current_angle_deg,
        "target_angle_deg": target_angle_deg,
        "current_milliamps": current_milliamps,
    }
