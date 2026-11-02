#pragma once
#include <cstddef>
#include <cstdint>
#include <functional>
#include <memory>
#include <string>
#include <vector>

namespace neuroexo {
// Raw diagnostic service, separate from the motor client's latest-value mailbox.
// The callback sees every notification on the transport thread; it must not
// call transport methods. disconnect() joins that thread before returning.
class PacketGatt {
 public:
  using Notification = std::function<void(const uint8_t*, size_t)>;
  virtual ~PacketGatt() = default;
  virtual void connect(const std::string&, const std::string&) = 0;
  virtual bool connected() const = 0;
  virtual void disconnect() noexcept = 0;
  virtual void writePacket(const std::vector<uint8_t>&, bool control) = 0;
  virtual std::vector<uint8_t> readStatistics() = 0;
};
std::unique_ptr<PacketGatt> makePacketGatt(PacketGatt::Notification callback,
                                         const std::string& testBus = {});
}
