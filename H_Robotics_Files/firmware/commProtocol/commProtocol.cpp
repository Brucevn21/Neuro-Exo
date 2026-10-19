#include "commProtocol.h"

namespace NeuroExoProtocol {

uint8_t crc8(const uint8_t *data, uint8_t length) {
    uint8_t crc = 0;
    while (length-- != 0) {
        crc ^= *data++;
        for (uint8_t bit = 0; bit < 8; ++bit) {
            crc = (crc & 0x80) ? static_cast<uint8_t>((crc << 1) ^ 0x07) : static_cast<uint8_t>(crc << 1);
        }
    }
    return crc;
}

static uint8_t encodeFrame(uint8_t type, const uint8_t *payload, uint8_t length, uint8_t *frame) {
    frame[0] = START_BYTE;
    frame[1] = type;
    frame[2] = length;
    for (uint8_t i = 0; i < length; ++i) {
        frame[3 + i] = payload[i];
    }
    frame[3 + length] = crc8(&frame[1], static_cast<uint8_t>(2 + length));
    frame[4 + length] = STOP_BYTE;
    return static_cast<uint8_t>(FRAME_OVERHEAD + length);
}

uint8_t encodeControlFrame(const ControlPacket &packet, uint8_t *frame) {
    const uint8_t payload[CONTROL_PAYLOAD_SIZE] = {
        static_cast<uint8_t>(packet.mode), static_cast<uint8_t>(packet.speed),
        static_cast<uint8_t>(packet.targetAngleDeg >> 8), static_cast<uint8_t>(packet.targetAngleDeg)
    };
    return encodeFrame(MESSAGE_CONTROL, payload, CONTROL_PAYLOAD_SIZE, frame);
}

uint8_t encodeTelemetryFrame(const TelemetryPacket &packet, uint8_t *frame) {
    const uint8_t payload[TELEMETRY_PAYLOAD_SIZE] = {
        static_cast<uint8_t>(packet.currentAngleDeg >> 8), static_cast<uint8_t>(packet.currentAngleDeg),
        static_cast<uint8_t>(packet.currentMilliAmps >> 8), static_cast<uint8_t>(packet.currentMilliAmps), packet.status
    };
    return encodeFrame(MESSAGE_TELEMETRY, payload, TELEMETRY_PAYLOAD_SIZE, frame);
}

bool decodeControlFrame(const Frame &frame, ControlPacket &packet) {
    if (frame.type != MESSAGE_CONTROL || frame.length != CONTROL_PAYLOAD_SIZE ||
        frame.payload[0] > static_cast<uint8_t>(Mode::Neutral) ||
        frame.payload[1] > static_cast<uint8_t>(Speed::High)) {
        return false;
    }
    packet.mode = static_cast<Mode>(frame.payload[0]);
    packet.speed = static_cast<Speed>(frame.payload[1]);
    packet.targetAngleDeg = static_cast<int16_t>((static_cast<uint16_t>(frame.payload[2]) << 8) | frame.payload[3]);
    return true;
}

bool decodeTelemetryFrame(const Frame &frame, TelemetryPacket &packet) {
    if (frame.type != MESSAGE_TELEMETRY || frame.length != TELEMETRY_PAYLOAD_SIZE) {
        return false;
    }
    packet.currentAngleDeg = static_cast<int16_t>((static_cast<uint16_t>(frame.payload[0]) << 8) | frame.payload[1]);
    packet.currentMilliAmps = static_cast<int16_t>((static_cast<uint16_t>(frame.payload[2]) << 8) | frame.payload[3]);
    packet.status = frame.payload[4];
    return true;
}

FrameParser::FrameParser() { reset(); }

void FrameParser::reset() {
    state_ = State::WaitingForStart;
    frame_ = Frame{};
    payloadIndex_ = 0;
    checksum_ = 0;
    frameReady_ = false;
}

void FrameParser::push(uint8_t value) {
    switch (state_) {
        case State::WaitingForStart:
            if (value == START_BYTE) {
                state_ = State::Type;
                frameReady_ = false;
                payloadIndex_ = 0;
            }
            break;
        case State::Type:
            if (value == START_BYTE) {
                payloadIndex_ = 0;
            } else {
                frame_.type = value;
                state_ = State::Length;
            }
            break;
        case State::Length:
            if (value == START_BYTE) { payloadIndex_ = 0; }
            else if (value > MAX_PAYLOAD_SIZE) { reset(); }
            else { frame_.length = value; payloadIndex_ = 0; state_ = value == 0 ? State::Checksum : State::Payload; }
            break;
        case State::Payload:
            frame_.payload[payloadIndex_++] = value;
            if (payloadIndex_ >= frame_.length) { state_ = State::Checksum; }
            break;
        case State::Checksum:
            checksum_ = crc8(&frame_.type, static_cast<uint8_t>(2 + frame_.length));
            state_ = value == checksum_ ? State::Stop : State::WaitingForStart;
            break;
        case State::Stop:
            if (value == STOP_BYTE) {
                frameReady_ = true;
                state_ = State::WaitingForStart;
            } else if (value == START_BYTE) {
                frameReady_ = false;
                payloadIndex_ = 0;
                state_ = State::Type;
            } else {
                state_ = State::WaitingForStart;
            }
            break;
    }
}

bool FrameParser::takeFrame(Frame &frame) {
    if (!frameReady_) { return false; }
    frame = frame_;
    frameReady_ = false;
    return true;
}

} // namespace NeuroExoProtocol
