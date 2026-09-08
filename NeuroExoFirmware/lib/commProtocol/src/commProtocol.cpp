#include "commProtocol.h"

namespace NeuroExoProtocol {

void encodeJointPacket(const JointPacket &packet, uint8_t buf[PACKET_SIZE]) {
    const uint8_t modeBits = static_cast<uint8_t>(packet.mode) & 0x03;
    const uint8_t speedBits = static_cast<uint8_t>(packet.speed) & 0x03;
    buf[0] = static_cast<uint8_t>((speedBits << 2) | modeBits);

    buf[1] = static_cast<uint8_t>((packet.currentAngleDeg >> 8) & 0xFF);
    buf[2] = static_cast<uint8_t>(packet.currentAngleDeg & 0xFF);

    buf[3] = static_cast<uint8_t>((packet.targetAngleDeg >> 8) & 0xFF);
    buf[4] = static_cast<uint8_t>(packet.targetAngleDeg & 0xFF);

    buf[5] = static_cast<uint8_t>((packet.currentMilliAmps >> 8) & 0xFF);
    buf[6] = static_cast<uint8_t>(packet.currentMilliAmps & 0xFF);
}

void decodeJointPacket(const uint8_t buf[PACKET_SIZE], JointPacket &packet) {
    packet.mode = static_cast<Mode>(buf[0] & 0x03);
    packet.speed = static_cast<Speed>((buf[0] >> 2) & 0x03);

    packet.currentAngleDeg = static_cast<int16_t>((buf[1] << 8) | buf[2]);
    packet.targetAngleDeg = static_cast<int16_t>((buf[3] << 8) | buf[4]);
    packet.currentMilliAmps = static_cast<int16_t>((buf[5] << 8) | buf[6]);
}

} // namespace NeuroExoProtocol
