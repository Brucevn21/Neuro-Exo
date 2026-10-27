// Build the actual firmware codec on the host to check Python interoperability.
#include "commProtocol.h"
#include <cstdio>

using namespace NeuroExoProtocol;

int main() {
    uint8_t bytes[MAX_FRAME_SIZE];
    const int angles[] = {-150, -123, -40, 0, 60, 80};
    for (int mode = 0; mode <= 5; ++mode) {
        const int low = mode <= 1 ? 1 : 0;
        const int high = mode <= 1 ? 9 : mode == 3 ? 0 : 2;
        for (int setting = low; setting <= high; ++setting) {
            for (int angle : angles) {
                ControlPacket command;
                command.mode = static_cast<Mode>(mode);
                command.setting = setting;
                command.targetAngleDeg = mode >= 4 ? 0 : angle;
                int size = encodeControlFrame(command, bytes);
                std::printf("%d %d %d ", mode, setting, angle);
                for (int i = 0; i < size; ++i) std::printf("%02x", bytes[i]);
                std::puts("");
            }
        }
    }
}
