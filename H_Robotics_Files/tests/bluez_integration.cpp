// Exercises the actual BlueZ D-Bus client against an isolated fake org.bluez
// whose "Nano" relays to a simple Teensy model: 50 Hz telemetry, keepalive
// frames, TargetReached, the 250 ms watchdog, and a latched SafetyFault.
// No Bluetooth adapter is used.
#include "../ble/BleJointClient.hpp"
#include <gio/gio.h>
#include <atomic>
#include <condition_variable>
#include <cstring>
#include <iostream>
#include <mutex>
#include <stdexcept>
#include <string>
#include <thread>
#include <vector>

namespace neuroexo {
std::unique_ptr<GattTransport> makeBluezGattForTests(const std::string&);
}
namespace {
namespace proto = NeuroExoProtocol;
using namespace std::chrono_literals;
const char* adapter = "/org/bluez/hci0";
const char* device = "/org/bluez/hci0/dev_AA_BB_CC_DD_EE_FF";
const char* service = "/org/bluez/hci0/dev_AA_BB_CC_DD_EE_FF/service0001";
const char* command = "/org/bluez/hci0/dev_AA_BB_CC_DD_EE_FF/service0001/char0001";
const char* telemetry = "/org/bluez/hci0/dev_AA_BB_CC_DD_EE_FF/service0001/char0002";
const char* xml = R"XML(
<node>
<interface name="org.freedesktop.DBus.ObjectManager">
 <method name="GetManagedObjects"><arg type="a{oa{sa{sv}}}" direction="out"/></method>
</interface>
<interface name="org.bluez.Adapter1">
 <method name="SetDiscoveryFilter"><arg type="a{sv}" direction="in"/></method>
 <method name="StartDiscovery"/><method name="StopDiscovery"/>
 <property name="Powered" type="b" access="read"/>
</interface>
<interface name="org.bluez.Device1">
 <method name="Connect"/><method name="Disconnect"/>
 <property name="Address" type="s" access="read"/>
 <property name="Adapter" type="o" access="read"/>
 <property name="Connected" type="b" access="read"/>
 <property name="ServicesResolved" type="b" access="read"/>
</interface>
<interface name="org.bluez.GattService1">
 <property name="UUID" type="s" access="read"/>
 <property name="Device" type="o" access="read"/>
</interface>
<interface name="org.bluez.GattCharacteristic1">
 <method name="WriteValue"><arg type="ay" direction="in"/><arg type="a{sv}" direction="in"/></method>
 <method name="ReadValue"><arg type="a{sv}" direction="in"/><arg type="ay" direction="out"/></method>
 <method name="StartNotify"/><method name="StopNotify"/>
 <property name="UUID" type="s" access="read"/>
 <property name="Service" type="o" access="read"/>
 <property name="Flags" type="as" access="read"/>
</interface>
</node>)XML";
void check(bool value, const char* message) { if (!value) throw std::runtime_error(message); }
int64_t nowMs() { return g_get_monotonic_time() / 1000; }

class FakeBluez {
 public:
  // Fault injection, set from the test thread.
  std::atomic<bool> badFlags{false}, disconnectOnWrite{false}, silent{false}, corrupt{false},
                    faultOnMotion{false}, rejectCommands{false}, neverReach{false};
  std::atomic<unsigned> writes{0}, validWrites{0}, trajectories{0};
  std::atomic<int> lastTarget{0}, lastMode{-1}, lastSpeed{-1};
  std::atomic<bool> lastWriteWithResponse{false};
  FakeBluez() {
    daemon_ = g_test_dbus_new(G_TEST_DBUS_NONE);
    g_test_dbus_up(daemon_);
    address = g_test_dbus_get_bus_address(daemon_);
    worker_ = std::thread([this] { serve(); });
    std::unique_lock<std::mutex> lock(mutex_);
    cv_.wait(lock, [this] { return ready_; });
    check(error_.empty(), error_.c_str());
  }
  ~FakeBluez() {
    if (loop_) {
      GSource* stop = g_idle_source_new();
      g_source_set_callback(stop, [](gpointer p) -> gboolean {
        g_main_loop_quit(static_cast<GMainLoop*>(p)); return G_SOURCE_REMOVE;
      }, loop_, nullptr);
      g_source_attach(stop, context_);
      g_source_unref(stop);
    }
    if (worker_.joinable()) worker_.join();
    g_test_dbus_down(daemon_);
    g_object_unref(daemon_);
  }
  void reset() {
    badFlags = disconnectOnWrite = silent = corrupt = faultOnMotion = rejectCommands = neverReach = false;
    writes = validWrites = trajectories = 0;
  }
  std::string address;
 private:
  GVariant* property(const std::string& path, const char* name) {
    if (!std::strcmp(name, "Powered")) return g_variant_new_boolean(TRUE);
    if (!std::strcmp(name, "Address")) return g_variant_new_string("AA:BB:CC:DD:EE:FF");
    if (!std::strcmp(name, "Adapter")) return g_variant_new_object_path(adapter);
    if (!std::strcmp(name, "Connected") || !std::strcmp(name, "ServicesResolved"))
      return g_variant_new_boolean(connected_);
    if (!std::strcmp(name, "Device")) return g_variant_new_object_path(device);
    if (!std::strcmp(name, "Service")) return g_variant_new_object_path(service);
    if (!std::strcmp(name, "UUID")) {
      // BlueZ reports full lowercase 128-bit UUIDs for 16-bit SIG values.
      const char* uuid = path == service ? "0000180c-0000-1000-8000-00805f9b34fb" :
                         path == command ? "00002a56-0000-1000-8000-00805f9b34fb" :
                                           "00002a57-0000-1000-8000-00805f9b34fb";
      return g_variant_new_string(uuid);
    }
    if (!std::strcmp(name, "Flags")) {
      if (path == command) {
        const char* flags[] = {"write", badFlags ? nullptr : "write-without-response", nullptr};
        return g_variant_new_strv(flags, -1);
      }
      const char* flags[] = {"read", "notify", nullptr};
      return g_variant_new_strv(flags, -1);
    }
    return nullptr;
  }
  GVariant* properties(const std::string& path) {
    std::vector<const char*> names;
    if (path == adapter) names = {"Powered"};
    else if (path == device) names = {"Address", "Adapter", "Connected", "ServicesResolved"};
    else if (path == service) names = {"UUID", "Device"};
    else names = {"UUID", "Service", "Flags"};
    GVariantBuilder b;
    g_variant_builder_init(&b, G_VARIANT_TYPE_VARDICT);
    for (auto name : names) g_variant_builder_add(&b, "{sv}", name, property(path, name));
    return g_variant_builder_end(&b);
  }
  static const char* interfaceFor(const std::string& path) {
    return path == "/" ? "org.freedesktop.DBus.ObjectManager" :
           path == adapter ? "org.bluez.Adapter1" :
           path == device ? "org.bluez.Device1" :
           path == service ? "org.bluez.GattService1" : "org.bluez.GattCharacteristic1";
  }
  GVariant* managed() {
    GVariantBuilder objects;
    g_variant_builder_init(&objects, G_VARIANT_TYPE("a{oa{sa{sv}}}"));
    for (const auto path : {adapter, device, service, command, telemetry}) {
      GVariantBuilder interfaces;
      g_variant_builder_init(&interfaces, G_VARIANT_TYPE("a{sa{sv}}"));
      g_variant_builder_add(&interfaces, "{s@a{sv}}", interfaceFor(path), properties(path));
      g_variant_builder_add(&objects, "{o@a{sa{sv}}}", path, g_variant_builder_end(&interfaces));
    }
    return g_variant_new("(@a{oa{sa{sv}}})", g_variant_builder_end(&objects));
  }
  void emit(const char* path, const char* interface, const char* key, GVariant* value) {
    GVariantBuilder properties, invalidated;
    g_variant_builder_init(&properties, G_VARIANT_TYPE_VARDICT);
    g_variant_builder_init(&invalidated, G_VARIANT_TYPE("as"));
    g_variant_builder_add(&properties, "{sv}", key, value);
    GError* error = nullptr;
    check(g_dbus_connection_emit_signal(bus_, nullptr, path,
      "org.freedesktop.DBus.Properties", "PropertiesChanged",
      g_variant_new("(sa{sv}as)", interface, &properties, &invalidated), &error),
      "emit failed");
    if (error) g_error_free(error);
  }
  // Teensy model: a new command (or any command after a timeout) starts a
  // trajectory at 1 deg per tick; identical frames only refresh the watchdog.
  // At the target it holds and sets TargetReached after 5 settled ticks.
  // 250 ms without a valid frame disables motion and clears a fault, but
  // TargetReached stays latched until the next trajectory starts.
  void publish() {
    if (!connected_ || !notifying_ || silent) return;
    const int64_t now = nowMs();
    if (now - lastValid_ > proto::COMMAND_TIMEOUT_MS) {
      timedOut_ = true; motion_ = false; fault_ = false;
    }
    if (motion_ && faultOnMotion) { fault_ = true; motion_ = false; }
    const int goal = neverReach ? lastTarget - 3 : int(lastTarget);
    if (motion_) {
      angle_ += angle_ < goal ? 1 : angle_ > goal ? -1 : 0;
      settled_ = angle_ == lastTarget ? settled_ + 1 : 0;
      if (settled_ >= 5) reached_ = true;
    }
    proto::TelemetryPacket packet;
    packet.currentAngleDeg = int16_t(angle_);
    packet.currentMilliAmps = motion_ ? 800 : 0;
    packet.status = uint8_t((motion_ ? 1 : 0) | (timedOut_ ? 2 : 0) | (invalid_ ? 4 : 0) |
                            (fault_ ? 8 : 0) | (reached_ ? neuroexo::STATUS_TARGET_REACHED : 0));
    uint8_t frame[proto::MAX_FRAME_SIZE];
    const uint8_t length = proto::encodeTelemetryFrame(packet, frame);
    if (corrupt) {
      uint8_t bad[proto::MAX_FRAME_SIZE];
      std::memcpy(bad, frame, length);
      bad[4] ^= 0x40;  // CRC no longer matches
      emit(telemetry, "org.bluez.GattCharacteristic1", "Value",
           g_variant_new_fixed_array(G_VARIANT_TYPE_BYTE, bad, length, 1));
    }
    emit(telemetry, "org.bluez.GattCharacteristic1", "Value",
         g_variant_new_fixed_array(G_VARIANT_TYPE_BYTE, frame, length, 1));
  }
  void method(const std::string& path, const std::string& name,
              GVariant* params, GDBusMethodInvocation* invocation) {
    if (name == "GetManagedObjects") {
      g_dbus_method_invocation_return_value(invocation, managed()); return;
    }
    if (name == "Connect") { connected_ = true; }
    if (name == "Disconnect") {
      connected_ = false; notifying_ = false;
      emit(device, "org.bluez.Device1", "Connected", g_variant_new_boolean(FALSE));
    }
    if (name == "StartNotify") notifying_ = true;
    if (name == "StopNotify") notifying_ = false;
    if (name == "WriteValue") {
      check(path == command && notifying_, "write before notification subscription");
      GVariant* bytes = g_variant_get_child_value(params, 0);
      GVariant* options = g_variant_get_child_value(params, 1);
      const gchar* type = nullptr;
      check(g_variant_lookup(options, "type", "&s", &type), "write type missing");
      check(!std::strcmp(type, "request") || !std::strcmp(type, "command"), "unknown write type");
      lastWriteWithResponse = !std::strcmp(type, "request");
      g_variant_unref(options);
      gsize length = 0;
      const auto* raw = static_cast<const uint8_t*>(g_variant_get_fixed_array(bytes, &length, 1));
      // Same path as Nano33BLEFirmware: parse, decode, then forward over I2C.
      proto::FrameParser parser;
      for (gsize i = 0; i < length; ++i) parser.push(raw[i]);
      proto::Frame frame;
      proto::ControlPacket packet;
      const bool valid = length == 9 && parser.takeFrame(frame) && proto::decodeControlFrame(frame, packet);
      g_variant_unref(bytes);
      ++writes;
      check(valid, "malformed control frame at fake Nano");
      if (disconnectOnWrite) {
        connected_ = false;
        emit(device, "org.bluez.Device1", "Connected", g_variant_new_boolean(FALSE));
      } else if (rejectCommands) {
        invalid_ = true;  // the Teensy saw a bad I2C frame; lastValid_ is not refreshed
      } else {
        ++validWrites;
        const bool same = int(packet.mode) == lastMode && int(packet.speed) == lastSpeed &&
                          packet.targetAngleDeg == lastTarget;
        const bool restart = !fault_ && (timedOut_ || !same);
        invalid_ = false; timedOut_ = false; lastValid_ = nowMs();
        if (restart) {
          lastMode = int(packet.mode); lastSpeed = int(packet.speed); lastTarget = packet.targetAngleDeg;
          motion_ = true; reached_ = false; settled_ = 0; ++trajectories;
        }
      }
    }
    g_dbus_method_invocation_return_value(invocation, g_variant_new("()"));
  }
  static void onMethod(GDBusConnection*, const gchar*, const gchar* path, const gchar*,
                       const gchar* name, GVariant* params,
                       GDBusMethodInvocation* invocation, gpointer user) {
    try { static_cast<FakeBluez*>(user)->method(path, name, params, invocation); }
    catch (const std::exception& e) {
      g_dbus_method_invocation_return_dbus_error(invocation, "org.bluez.Error.Failed", e.what());
    }
  }
  static GVariant* onProperty(GDBusConnection*, const gchar*, const gchar* path,
                             const gchar*, const gchar* name, GError**, gpointer user) {
    return static_cast<FakeBluez*>(user)->property(path, name);
  }
  void serve() {
    context_ = g_main_context_new();
    g_main_context_push_thread_default(context_);
    loop_ = g_main_loop_new(context_, FALSE);
    GError* error = nullptr;
    bus_ = g_dbus_connection_new_for_address_sync(address.c_str(),
      GDBusConnectionFlags(G_DBUS_CONNECTION_FLAGS_AUTHENTICATION_CLIENT |
                          G_DBUS_CONNECTION_FLAGS_MESSAGE_BUS_CONNECTION),
      nullptr, nullptr, &error);
    if (!bus_) {
      std::lock_guard<std::mutex> lock(mutex_);
      error_ = error->message; g_error_free(error); ready_ = true; cv_.notify_all(); return;
    }
    GVariant* result = g_dbus_connection_call_sync(bus_, "org.freedesktop.DBus",
      "/org/freedesktop/DBus", "org.freedesktop.DBus", "RequestName",
      g_variant_new("(su)", "org.bluez", 0u), G_VARIANT_TYPE("(u)"),
      G_DBUS_CALL_FLAGS_NONE, 2000, nullptr, &error);
    check(result != nullptr, "RequestName failed");
    g_variant_unref(result);
    GDBusNodeInfo* node = g_dbus_node_info_new_for_xml(xml, &error);
    check(node != nullptr, "introspection XML failed");
    static const GDBusInterfaceVTable vtable = {onMethod, onProperty, nullptr, {nullptr}};
    std::vector<guint> registrations;
    for (const auto path : {"/", adapter, device, service, command, telemetry}) {
      const auto* interface = interfaceFor(path);
      const guint id = g_dbus_connection_register_object(bus_, path,
        g_dbus_node_info_lookup_interface(node, interface), &vtable, this, nullptr, &error);
      check(id != 0, "register object failed");
      registrations.push_back(id);
    }
    GSource* tick = g_timeout_source_new(20);  // the Nano polls the Teensy at 50 Hz
    g_source_set_callback(tick, [](gpointer self) -> gboolean {
      static_cast<FakeBluez*>(self)->publish(); return G_SOURCE_CONTINUE;
    }, this, nullptr);
    g_source_attach(tick, context_);
    {
      std::lock_guard<std::mutex> lock(mutex_);
      ready_ = true; cv_.notify_all();
    }
    g_main_loop_run(loop_);
    g_source_destroy(tick);
    g_source_unref(tick);
    for (auto id : registrations) g_dbus_connection_unregister_object(bus_, id);
    g_dbus_node_info_unref(node);
    g_dbus_connection_close_sync(bus_, nullptr, nullptr);
    g_object_unref(bus_);
    g_main_loop_unref(loop_);
    g_main_context_pop_thread_default(context_);
    g_main_context_unref(context_);
  }
  GTestDBus* daemon_ = nullptr;
  GDBusConnection* bus_ = nullptr;
  GMainContext* context_ = nullptr;
  GMainLoop* loop_ = nullptr;
  std::thread worker_;
  std::mutex mutex_;
  std::condition_variable cv_;
  bool ready_ = false, connected_ = false, notifying_ = false;
  bool motion_ = false, timedOut_ = true, invalid_ = false, fault_ = false, reached_ = false;
  int angle_ = 0, settled_ = 0;
  int64_t lastValid_ = 0;
  std::string error_;
};
template<class F> std::string rejects(F operation, const char* message) {
  try { operation(); } catch (const std::exception& error) { return error.what(); }
  throw std::runtime_error(message);
}
void contains(const std::string& text, const char* part, const char* message) {
  if (text.find(part) == std::string::npos) throw std::runtime_error(std::string(message) + ": " + text);
}
}  // namespace

int main() {
  try {
    FakeBluez server;
    const neuroexo::JointCommand command{proto::Mode::Assistive, proto::Speed::High, 10};
    const neuroexo::JointCommand far{proto::Mode::Assistive, proto::Speed::High, 100};
    {
      neuroexo::BleJointClient client(neuroexo::makeBluezGattForTests(server.address));
      client.connect("aa:bb:cc:dd:ee:ff");
      // Monitoring alone never writes.
      const auto first = client.waitForTelemetry(1000ms);
      check(first.commandTimeout() && !first.motionActive(), "idle telemetry should show CommandTimeout");
      check(server.writes == 0, "monitoring wrote a command");
      unsigned seen = 0;
      int maxAngle = 0;
      auto result = client.runTrial(command, 3000ms, 100ms,
        [&](const neuroexo::JointTelemetry& t) { ++seen; maxAngle = std::max<int>(maxAngle, t.angleDeg); });
      check(result.targetReached && result.stopConfirmed, "target or stop not confirmed");
      check(result.commandsSent >= 3 && result.commandsSent == server.validWrites, "keepalive count");
      check(server.trajectories == 1, "keepalive frames restarted the trajectory");
      check(result.elapsedMs < 1500, "trial did not end at TargetReached");
      check(result.telemetryFrames == seen && seen >= 10, "telemetry not delivered at ~50 Hz");
      check(server.lastTarget == 10 && server.lastMode == 1 && server.lastSpeed == 2, "command fields");
      check(server.lastWriteWithResponse, "default write must request an ATT response");
      check(maxAngle == 10, "angle did not reach the target");
      check(client.rejectedNotifications() == 0, "valid telemetry rejected");

      // Same target again: the idle telemetry still carries the old
      // TargetReached bit (with CommandTimeout); it must not end the trial.
      {
        neuroexo::JointTelemetry idle;
        check(client.latestTelemetry(idle) && idle.targetReached() && idle.commandTimeout(),
              "fake should latch TargetReached while idle");
      }
      result = client.runTrial(command, 3000ms);
      // The fake needs 5 ticks (100 ms) of settling after the new trajectory starts.
      check(result.targetReached && server.trajectories == 2 && result.elapsedMs >= 80,
            "stale TargetReached ended the trial early");

      // Never settles: the time limit ends the trial and keepalives stop.
      server.neverReach = true;
      contains(rejects([&] { client.runTrial(command, 600ms); }, "time limit ignored"),
               "target not reached", "wrong time-limit error");
      const unsigned writesAtLimit = server.writes;
      std::this_thread::sleep_for(300ms);
      check(server.writes == writesAtLimit, "commands continued after the time limit");
      server.neverReach = false;

      client.setWriteWithResponse(false);
      client.send(command);
      check(!server.lastWriteWithResponse, "write-without-response not used");
      client.setWriteWithResponse(true);
      std::this_thread::sleep_for(300ms);  // let the fake time out again

      server.corrupt = true;
      client.takeTelemetry();
      std::this_thread::sleep_for(200ms);
      const auto frames = client.takeTelemetry();
      check(!frames.empty() && client.rejectedNotifications() > 0, "corrupt frames not rejected");
      server.corrupt = false;

      // A latched SafetyFault ends the trial, and keepalives stop.
      server.faultOnMotion = true;
      contains(rejects([&] { client.runTrial(far, 2000ms); }, "SafetyFault ignored"),
               "safety fault", "wrong fault error");
      const unsigned writesAtFault = server.writes;
      std::this_thread::sleep_for(300ms);
      check(server.writes == writesAtFault, "commands continued after SafetyFault");
      server.faultOnMotion = false;

      // The Teensy rejects every frame: CommandTimeout persists while sending.
      server.rejectCommands = true;
      contains(rejects([&] { client.runTrial(far, 2000ms); }, "CommandTimeout ignored"),
               "CommandTimeout", "wrong timeout error");
      server.rejectCommands = false;

      // Telemetry stops mid-trial.
      unsigned frames2 = 0;
      contains(rejects([&] {
        client.runTrial(far, 3000ms, 100ms, [&](const neuroexo::JointTelemetry&) {
          if (++frames2 == 5) server.silent = true;
        });
      }, "silence ignored"), "telemetry stopped", "wrong silence error");
      server.silent = false;
      client.disconnect();
    }
    server.reset();
    {
      neuroexo::BleJointClient client(neuroexo::makeBluezGattForTests(server.address));
      client.connect("AA:BB:CC:DD:EE:FF");
      server.disconnectOnWrite = true;
      client.send(command);  // the link drop arrives as a signal after the write
      std::this_thread::sleep_for(100ms);
      check(!client.connected(), "disconnect not detected");
      rejects([&] { client.send(command); }, "write after disconnect accepted");
      server.disconnectOnWrite = false;
    }
    server.badFlags = true;
    {
      neuroexo::BleJointClient client(neuroexo::makeBluezGattForTests(server.address));
      rejects([&] { client.connect("AA:BB:CC:DD:EE:FF"); }, "bad characteristic flags accepted");
    }
    std::cout << "BlueZ integration: discovery, flags, run-to-TargetReached, keepalive without "
                 "restart, stale TargetReached, time limit, stop confirmation, write types, CRC "
                 "rejection, SafetyFault, CommandTimeout, silence, disconnect PASS\n";
    return 0;
  } catch (const std::exception& error) {
    std::cerr << "FAIL: " << error.what() << '\n';
    return 1;
  }
}
