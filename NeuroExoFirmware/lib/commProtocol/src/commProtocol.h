#ifndef __COMM_PROTOCOL_H__
#define __COMM_PROTOCOL_H__

#include <stdint.h>

namespace NeuroExoProtocol {

constexpr uint8_t START_BYTE = 0x02;
constexpr uint8_t STOP_BYTE = 0x03;
constexpr uint8_t MESSAGE_CONTROL = 0x10;
constexpr uint8_t MESSAGE_TELEMETRY = 0x11;
constexpr uint8_t CONTROL_PAYLOAD_SIZE = 4;
constexpr uint8_t TELEMETRY_PAYLOAD_SIZE = 5;
constexpr uint8_t FRAME_OVERHEAD = 5;
constexpr uint8_t MAX_PAYLOAD_SIZE = TELEMETRY_PAYLOAD_SIZE;
constexpr uint8_t MAX_FRAME_SIZE = FRAME_OVERHEAD + MAX_PAYLOAD_SIZE;
constexpr uint32_t FRAME_TIMEOUT_MS = 25;
constexpr uint32_t COMMAND_TIMEOUT_MS = 250;

enum class TelemetryStatus : uint8_t {
    None = 0,
    MotionActive = 1 << 0,
    CommandTimeout = 1 << 1,
    InvalidCommand = 1 << 2
};

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

struct __attribute__((packed)) ControlPacket {
    Mode mode = Mode::Neutral;
    Speed speed = Speed::Medium;
    int16_t targetAngleDeg = 0;
};

struct __attribute__((packed)) TelemetryPacket {
    int16_t currentAngleDeg = 0;
    int16_t currentMilliAmps = 0;
    uint8_t status = 0;
};

struct Frame {
    uint8_t type = 0;
    uint8_t length = 0;
    uint8_t payload[MAX_PAYLOAD_SIZE] = {};
};

uint8_t crc8(const uint8_t *data, uint8_t length);
uint8_t encodeControlFrame(const ControlPacket &packet, uint8_t *frame);
uint8_t encodeTelemetryFrame(const TelemetryPacket &packet, uint8_t *frame);
bool decodeControlFrame(const Frame &frame, ControlPacket &packet);
bool decodeTelemetryFrame(const Frame &frame, TelemetryPacket &packet);

class FrameParser {
public:
    FrameParser();
    void reset();
    void push(uint8_t value);
    bool hasFrame() const { return frameReady_; }
    bool takeFrame(Frame &frame);

private:
    enum class State : uint8_t { WaitingForStart, Type, Length, Payload, Checksum, Stop };
    State state_;
    Frame frame_;
    uint8_t payloadIndex_;
    uint8_t checksum_;
    bool frameReady_;
};

static_assert(sizeof(ControlPacket) == CONTROL_PAYLOAD_SIZE, "Control payload layout changed");
static_assert(sizeof(TelemetryPacket) == TELEMETRY_PAYLOAD_SIZE, "Telemetry payload layout changed");

} // namespace NeuroExoProtocol

#endif // __COMM_PROTOCOL_H__
