// Exercises the actual BlueZ D-Bus client against an isolated fake org.bluez.
// No system Bluetooth adapter or external network is accessed.
#include "BluezLink.hpp"
#include "Trial.hpp"
#include <gio/gio.h>
#include <atomic>
#include <condition_variable>
#include <cstring>
#include <iostream>
#include <mutex>
#include <stdexcept>
#include <thread>
#include <vector>

namespace {
const char* adapter = "/org/bluez/hci0";
const char* device = "/org/bluez/hci0/dev_AA_BB_CC_DD_EE_FF";
const char* service = "/org/bluez/hci0/dev_AA_BB_CC_DD_EE_FF/service0001";
const char* command = "/org/bluez/hci0/dev_AA_BB_CC_DD_EE_FF/service0001/char0001";
const char* event = "/org/bluez/hci0/dev_AA_BB_CC_DD_EE_FF/service0001/char0002";
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

class FakeBluez {
 public:
  std::atomic<bool> drop{false}, corrupt{false}, disconnectOnWrite{false}, badFlags{false};
  std::atomic<unsigned> writes{0}, stops{0}, controls{0}, requests{0}, commands{0};
  std::atomic<bool> verifySequence{false}, motionAfterWrite{false}, failSequence{false};
  std::atomic<unsigned> nextSequence{0}, badSequences{0}, writeDelayMs{0};
  std::atomic<bool> txOnlyServer{false};
  std::atomic<unsigned> notifyStarts{0}, notifyStops{0};
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
      const char* uuid = path == service ? "0000180c-0000-1000-8000-00805f9b34fb" :
                         path == command ? "00002a56-0000-1000-8000-00805f9b34fb" :
                         path == event ? "00002a57-0000-1000-8000-00805f9b34fb" :
                                         "b7e20003-6a2b-4f10-9c31-8b674045a901";
      return g_variant_new_string(uuid);
    }
    if (!std::strcmp(name, "Flags")) {
      const char* flags[] = {path == command ? (badFlags ? "read" : "write") :
                            path == event ? "notify" : "read", nullptr};
      if (path == command && !badFlags) {
        const char* writable[] = {"write", "write-without-response", nullptr};
        return g_variant_new_strv(writable, -1);
      }
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
    for (const auto path : {adapter, device, service, command, event}) {
      if (txOnlyServer && std::strcmp(path, event) == 0) continue;
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
    if (name == "StartNotify") {
      ++notifyStarts;
      check(!txOnlyServer, "TX-only receiver has no notifications");
      notifying_ = true;
    }
    if (name == "StopNotify") { ++notifyStops; notifying_ = false; }
    if (name == "WriteValue") {
      check(path == command && (notifying_ || txOnlyServer), "write before notification subscription");
      GVariant* bytes = g_variant_get_child_value(params, 0);
      GVariant* options = g_variant_get_child_value(params, 1);
      const gchar* type = nullptr;
      const bool request = g_variant_lookup(options, "type", "&s", &type) &&
                           std::strcmp(type, "request") == 0;
      const bool commandMode = type && std::strcmp(type, "command") == 0;
      g_variant_unref(options);
      check(request || commandMode, "write mode missing");
      check(!txOnlyServer || commandMode, "TX-only requires write without response");
      if (request) ++requests; else ++commands;
      gsize length = 0;
      const auto* raw = static_cast<const uint8_t*>(g_variant_get_fixed_array(bytes, &length, 1));
      NeuroExoProtocol::ControlPacket control;
      NeuroExoProtocol::FrameParser parser;
      for (gsize i = 0; i < length; ++i) parser.push(raw[i]);
      NeuroExoProtocol::Frame frame;
      const bool framed = parser.takeFrame(frame);
      const bool stopped = framed && NeuroExoProtocol::isStopFrame(frame);
      const bool accepted = stopped || (framed && NeuroExoProtocol::decodeControlFrame(frame, control));
      if (!stopped && accepted && verifySequence) {
        if (uint16_t(control.targetAngleDeg) != nextSequence.fetch_add(1)) ++badSequences;
      }
      if (stopped) ++stops; else if (accepted) ++controls;
      g_variant_unref(bytes);
      check(accepted, "malformed command at fake Nano");
      ++writes;
      if (writeDelayMs) std::this_thread::sleep_for(std::chrono::milliseconds(writeDelayMs.load()));
      if (verifySequence && failSequence && nextSequence >= 3) {
        g_dbus_method_invocation_return_dbus_error(invocation, "org.bluez.Error.Failed", "injected write failure");
        return;
      }
      if (disconnectOnWrite) {
        connected_ = false;
        emit(device, "org.bluez.Device1", "Connected", g_variant_new_boolean(FALSE));
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
    for (const auto path : {"/", adapter, device, service, command, event}) {
      const auto* interface = interfaceFor(path);
      const guint id = g_dbus_connection_register_object(bus_, path,
        g_dbus_node_info_lookup_interface(node, interface), &vtable, this, nullptr, &error);
      check(id != 0, "register object failed");
      registrations.push_back(id);
    }
    {
      std::lock_guard<std::mutex> lock(mutex_);
      ready_ = true; cv_.notify_all();
    }
    GSource* telemetryTimer = g_timeout_source_new(20);
    g_source_set_callback(telemetryTimer, [](gpointer p) -> gboolean {
      auto* self = static_cast<FakeBluez*>(p);
      if (self->connected_ && self->notifying_ && !self->drop) {
        uint8_t raw[NeuroExoProtocol::MAX_FRAME_SIZE];
        NeuroExoProtocol::TelemetryPacket packet;
        packet.currentAngleDeg = 10; packet.currentMilliAmps = -1200;
        packet.patientEffortCentiNm = -123;
        if (self->motionAfterWrite && self->nextSequence)
          packet.status = uint8_t(NeuroExoProtocol::TelemetryStatus::MotionActive);
        NeuroExoProtocol::encodeTelemetryFrame(packet, raw);
        if (self->corrupt) raw[10] ^= 1;
        self->emit(event, "org.bluez.GattCharacteristic1", "Value",
                   g_variant_new_fixed_array(G_VARIANT_TYPE_BYTE, raw, sizeof(raw), 1));
      }
      return G_SOURCE_CONTINUE;
    }, this, nullptr);
    g_source_attach(telemetryTimer, context_);
    g_main_loop_run(loop_);
    g_source_destroy(telemetryTimer);
    g_source_unref(telemetryTimer);
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
  std::string error_;
};
template<class F> void rejects(F operation, const char* message) {
  try { operation(); } catch (const std::exception&) { return; }
  throw std::runtime_error(message);
}
}

int main(int argc, char** argv) {
  try {
    FakeBluez server;
    g_setenv("DBUS_SYSTEM_BUS_ADDRESS", server.address.c_str(), TRUE);
    trial::Receiver receiver;
    BluezLink client([&](const uint8_t* raw, size_t size) { receiver.notification(raw, size); });
    client.connect("aa:bb:cc:dd:ee:ff", "hci0", false);
    const auto deadline = trial::Clock::now() + std::chrono::seconds(2);
    while (!receiver.get().seen && trial::Clock::now() < deadline)
      std::this_thread::sleep_for(std::chrono::milliseconds(5));
    check(receiver.get().seen && receiver.get().latest.patientEffortCentiNm == -123, "12-byte telemetry missing/wrong");
    check(server.writes == 0, "monitor sends commands");
    receiver.begin(trial::Clock::now(), 2);
    for (int i = 0; i < 100; ++i) client.write(trial::controlFrame(20, NeuroExoProtocol::Speed::Slow), false);
    client.write(trial::stopFrame(), true);
    check(server.controls == 100 && server.stops == 1 && server.commands == 100 && server.requests == 1, "write type/packet mismatched");
    server.corrupt = true;
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    check(receiver.get().invalid > 0, "bad telemetry accepted");
    server.corrupt = false;
    client.disconnect();
    server.badFlags = true;
    rejects([&] { client.connect("AA:BB:CC:DD:EE:FF", "hci0", false); }, "bad flags accepted");
    client.disconnect();
    server.badFlags = false;
    if (argc > 1) {
      // The actual CLI uses the private mock system bus, never real hardware.
      gchar* output = nullptr; gchar* errors = nullptr; gint status = 0; GError* error = nullptr;
      gchar* command[] = {argv[1], const_cast<gchar*>("--address"), const_cast<gchar*>("AA:BB:CC:DD:EE:FF"),
        const_cast<gchar*>("--angle"), const_cast<gchar*>("20"), const_cast<gchar*>("--rate"), const_cast<gchar*>("50"),
        const_cast<gchar*>("--duration"), const_cast<gchar*>("1"), nullptr};
      check(g_spawn_sync(nullptr, command, nullptr, G_SPAWN_DEFAULT, nullptr, nullptr,
          &output, &errors, &status, &error), "CLI failed to launch");
      std::cout << output;
      const bool valid = std::strstr(output, "TX RATE ") && std::strstr(output, "UNAVAILABLE") &&
          !std::strstr(output, "INCOMPLETE") && server.stops == 2 && server.controls > 100;
      if (!valid) std::cerr << errors;
      g_free(output); g_free(errors);
      check(valid, "full CLI trial or final stop failed");
      // A slow host skips scheduling slots but must send contiguous sequence IDs.
      server.verifySequence = true;
      server.writeDelayMs = 12;
      const unsigned stopsBefore = server.stops;
      gchar* sequenceArgs[] = {argv[1], const_cast<gchar*>("--address"), const_cast<gchar*>("AA:BB:CC:DD:EE:FF"),
        const_cast<gchar*>("--sequence-test"), const_cast<gchar*>("--rate"), const_cast<gchar*>("250"),
        const_cast<gchar*>("--duration"), const_cast<gchar*>("1"), nullptr};
      check(g_spawn_sync(nullptr, sequenceArgs, nullptr, G_SPAWN_DEFAULT, nullptr, nullptr,
          &output, &errors, &status, &error), "sequence CLI failed to launch");
      std::cout << output;
      const bool sequenceValid = std::strstr(output, "TX RATE NOT MET") &&
          std::strstr(output, "Attempted sequence IDs: 0..") &&
          std::strstr(output, "Compare Nano serial") &&
          server.nextSequence > 1 && server.nextSequence < 250 && server.badSequences == 0 && server.stops == stopsBefore;
      if (!sequenceValid) std::cerr << errors;
      g_free(output); g_free(errors);
      check(sequenceValid, "sequence IDs, missed-slot accounting or no-stop behavior incorrect");
      // Write failures must not become definitive packet-loss percentages.
      server.nextSequence = 0;
      server.failSequence = true;
      check(g_spawn_sync(nullptr, sequenceArgs, nullptr, G_SPAWN_DEFAULT, nullptr, nullptr,
          &output, &errors, &status, &error), "failure CLI failed to launch");
      check(std::strstr(output, "INCOMPLETE") && !std::strstr(output, "Compare Nano serial") &&
          server.nextSequence == 3 && server.stops == stopsBefore, "failed sequence write was retried or treated as loss");
      g_free(output); g_free(errors);
      server.failSequence = false;
      // If unexpected motion is observed, abort and attempt the native stop.
      server.nextSequence = 0;
      server.motionAfterWrite = true;
      check(g_spawn_sync(nullptr, sequenceArgs, nullptr, G_SPAWN_DEFAULT, nullptr, nullptr,
          &output, &errors, &status, &error), "motion CLI failed to launch");
      check(std::strstr(output, "INCOMPLETE") && server.stops == stopsBefore + 1,
            "sequence test did not stop on unexpected motion");
      g_free(output); g_free(errors);
      // The TX-only CLI must work with NO telemetry characteristic or updates.
      server.motionAfterWrite = false;
      server.txOnlyServer = true;
      server.nextSequence = 0;
      const auto subscriptionsBefore = server.notifyStarts.load();
      const auto unsubscribesBefore = server.notifyStops.load();
      const auto requestsBefore = server.requests.load();
      gchar* txOnlyArgs[] = {argv[1], const_cast<gchar*>("--address"), const_cast<gchar*>("AA:BB:CC:DD:EE:FF"),
        const_cast<gchar*>("--sequence-test"), const_cast<gchar*>("--tx-only"),
        const_cast<gchar*>("--rate"), const_cast<gchar*>("250"),
        const_cast<gchar*>("--duration"), const_cast<gchar*>("1"), nullptr};
      check(g_spawn_sync(nullptr, txOnlyArgs, nullptr, G_SPAWN_DEFAULT, nullptr, nullptr,
          &output, &errors, &status, &error), "TX-only CLI failed to launch");
      std::cout << output;
      const bool txOnlyValid = std::strstr(output, "Telemetry: DISABLED") &&
          std::strstr(output, "TX RATE NOT MET") && std::strstr(output, "Compare Nano serial") &&
          server.nextSequence > 1 && server.badSequences == 0 &&
          server.notifyStarts == subscriptionsBefore && server.notifyStops == unsubscribesBefore &&
          server.requests == requestsBefore && server.stops == stopsBefore + 1;
      if (!txOnlyValid) std::cerr << errors;
      g_free(output); g_free(errors);
      check(txOnlyValid, "TX-only subscribed, waited for telemetry, or sent response/stop writes");
      server.nextSequence = 0;
      server.disconnectOnWrite = true;
      check(g_spawn_sync(nullptr, txOnlyArgs, nullptr, G_SPAWN_DEFAULT, nullptr, nullptr,
          &output, &errors, &status, &error), "TX-only disconnect CLI failed to launch");
      check(std::strstr(output, "INCOMPLETE") && !std::strstr(output, "Compare Nano serial"),
            "TX-only disconnect was treated as a completed test");
      g_free(output); g_free(errors);
    }
    std::cout << "BlueZ transport/control/stop/telemetry/flags and real CLI PASS\n";
    return 0;
  } catch (const std::exception& error) { std::cerr << error.what() << '\n'; return 1; }
}
