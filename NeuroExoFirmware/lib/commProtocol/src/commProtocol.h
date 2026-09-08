/*
 * commProtocol.h
 *
 * Shared 7-byte joint data packet used between the BeagleBone/App,
 * the Arduino Nano 33 BLE (BLE <-> I2C bridge), and the Teensy 4.1
 * (motor controller), per the NeuroExo Protocol List.
 *
 * Byte layout (56 bits total):
 *   Byte 0     : Mode (bits 1:0) + Speed (bits 3:2)
 *   Bytes 1-2  : Current Angle [degrees], int16, MSB first
 *   Bytes 3-4  : Target Angle [degrees], int16, MSB first
 *   Bytes 5-6  : Current [mA], int16, MSB first
 */

#ifndef __COMM_PROTOCOL_H__
#define __COMM_PROTOCOL_H__

#include <stdint.h>

namespace NeuroExoProtocol {

constexpr uint8_t PACKET_SIZE = 7;

// I2C command byte prefixing a full joint packet sent Nano -> Teensy.
constexpr uint8_t I2C_CMD_JOINT_PACKET = 0x02;

// I2C command (no payload) requesting an immediate halt of any in-flight motion.
constexpr uint8_t I2C_CMD_STOP = 0x03;

enum class Mode : uint8_t {
    Resistive = 0,
    Assistive = 1,
    Neutral = 2
};

enum class Speed : uint8_t {
    Slow = 0,
    Medium = 1,
    High = 2
};

struct JointPacket {
    Mode mode = Mode::Neutral;
    Speed speed = Speed::Medium;
    int16_t currentAngleDeg = 0;
    int16_t targetAngleDeg = 0;
    int16_t currentMilliAmps = 0;
};

void encodeJointPacket(const JointPacket &packet, uint8_t buf[PACKET_SIZE]);
void decodeJointPacket(const uint8_t buf[PACKET_SIZE], JointPacket &packet);

// Double buffer that lets one 7-byte packet be filled with fresh data
// while the previously completed packet is safely transmitted over BLE.
class JointPacketPingPongBuffer {
public:
    JointPacketPingPongBuffer() : activeIndex_(0) {
        for (uint8_t i = 0; i < 2; ++i) {
            for (uint8_t j = 0; j < PACKET_SIZE; ++j) {
                buffers_[i][j] = 0;
            }
        }
    }

    // Buffer currently safe to fill with new data.
    uint8_t *writeBuffer() { return buffers_[1 - activeIndex_]; }

    // Most recently completed packet, safe to read/transmit.
    const uint8_t *readBuffer() const { return buffers_[activeIndex_]; }

    // Publish the write buffer as the new read buffer.
    void swap() { activeIndex_ = 1 - activeIndex_; }

private:
    uint8_t buffers_[2][PACKET_SIZE];
    volatile uint8_t activeIndex_;
};

} // namespace NeuroExoProtocol

#endif // __COMM_PROTOCOL_H__
