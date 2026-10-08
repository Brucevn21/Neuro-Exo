// Exercises the actual BlueZ D-Bus client against an isolated fake org.bluez.
// No system Bluetooth adapter or external network is accessed.
#include "../BleTrialClient.hpp"
#include <gio/gio.h>
#include <atomic>
#include <condition_variable>
#include <cstring>
#include <iostream>
#include <mutex>
#include <stdexcept>
#include <thread>
#include <vector>

namespace neuroexo {
std::unique_ptr<GattTransport> makeBluezGattForTests(const std::string&);
}
namespace {
const char* adapter = "/org/bluez/hci0";
const char* device = "/org/bluez/hci0/dev_AA_BB_CC_DD_EE_FF";
const char* service = "/org/bluez/hci0/dev_AA_BB_CC_DD_EE_FF/service0001";
const char* command = "/org/bluez/hci0/dev_AA_BB_CC_DD_EE_FF/service0001/char0001";
const char* event = "/org/bluez/hci0/dev_AA_BB_CC_DD_EE_FF/service0001/char0002";
const char* info = "/org/bluez/hci0/dev_AA_BB_CC_DD_EE_FF/service0001/char0003";
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
  std::atomic<bool> drop{false}, badSequence{false}, disconnectOnWrite{false}, badFlags{false};
  std::atomic<unsigned> writes{0};
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
      const char* uuid = path == service ? "b7e20000-6a2b-4f10-9c31-8b674045a901" :
                         path == command ? "b7e20001-6a2b-4f10-9c31-8b674045a901" :
                         path == event ? "b7e20002-6a2b-4f10-9c31-8b674045a901" :
                                         "b7e20003-6a2b-4f10-9c31-8b674045a901";
      return g_variant_new_string(uuid);
    }
    if (!std::strcmp(name, "Flags")) {
      const char* flags[] = {path == command ? (badFlags ? "read" : "write") :
                            path == event ? "notify" : "read", nullptr};
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
    for (const auto path : {adapter, device, service, command, event, info}) {
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
    if (name == "Connect") { connected_ = true; nano_.reset(); }
    if (name == "Disconnect") {
      connected_ = false; notifying_ = false; nano_.reset();
      emit(device, "org.bluez.Device1", "Connected", g_variant_new_boolean(FALSE));
    }
    if (name == "StartNotify") notifying_ = true;
    if (name == "StopNotify") notifying_ = false;
    if (name == "ReadValue") {
      neuroexo::Packet reply{};
      nano_.info(reply.data());
      g_dbus_method_invocation_return_value(invocation,
        g_variant_new("(@ay)", g_variant_new_fixed_array(G_VARIANT_TYPE_BYTE,
                                                       reply.data(), reply.size(), 1)));
      return;
    }
    if (name == "WriteValue") {
      check(path == command && notifying_, "write before notification subscription");
      GVariant* bytes = g_variant_get_child_value(params, 0);
      GVariant* options = g_variant_get_child_value(params, 1);
      const gchar* type = nullptr;
      const bool request = g_variant_lookup(options, "type", "&s", &type) &&
                           std::strcmp(type, "request") == 0;
      g_variant_unref(options);
      check(request, "write must explicitly request ATT response");
      gsize length = 0;
      const auto* raw = static_cast<const uint8_t*>(g_variant_get_fixed_array(bytes, &length, 1));
      neuroexo::Packet reply{};
      const uint32_t now = uint32_t(g_get_monotonic_time() / 1000);
      const bool accepted = nano_.process(raw, length, now, reply.data());
      g_variant_unref(bytes);
      check(accepted, "malformed command at fake Nano");
      ++writes;
      if (disconnectOnWrite) {
        connected_ = false;
        emit(device, "org.bluez.Device1", "Connected", g_variant_new_boolean(FALSE));
      } else if (!drop) {
        if (badSequence) ++reply[4];
        // Emit before ATT WriteValue completes to check the early-reply race.
        emit(event, "org.bluez.GattCharacteristic1", "Value",
             g_variant_new_fixed_array(G_VARIANT_TYPE_BYTE, reply.data(), reply.size(), 1));
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
    for (const auto path : {"/", adapter, device, service, command, event, info}) {
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
    g_main_loop_run(loop_);
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
  neuroexo::Simulator nano_;
};
template<class F> void rejects(F operation, const char* message) {
  try { operation(); } catch (const std::exception&) { return; }
  throw std::runtime_error(message);
}
}
int main() {
  try {
    FakeBluez server;
    {
      neuroexo::BleTrialClient client(neuroexo::makeBluezGattForTests(server.address));
      client.connect("aa:bb:cc:dd:ee:ff");
      check(client.simulated(), "simulation flag missing");
      client.calibrate();
      client.setMaximumPosition(90);
      const auto timing = client.runTrial({1, 60, 0, 30}, std::chrono::milliseconds(130));
      check(timing.positionReplies >= 2, "position feedback missing");
      check(client.position(1).state == neuroexo::ENDED, "End not delivered");
      rejects([&] { client.configure({2, 100, 0, 30}); }, "out-of-range configuration accepted");
      client.disconnect();
    }
    for (int failure = 0; failure < 3; ++failure) {
      neuroexo::BleTrialClient client(neuroexo::makeBluezGattForTests(server.address));
      client.connect("AA:BB:CC:DD:EE:FF");
      client.setTimeout(100);
      server.drop = failure == 0;
      server.badSequence = failure == 1;
      server.disconnectOnWrite = failure == 2;
      rejects([&] { client.calibrate(); }, "bad/disconnected/missing reply accepted");
      check(!client.connected(), "failed transport remained usable");
      server.drop = server.badSequence = server.disconnectOnWrite = false;
    }
    server.badFlags = true;
    {
      neuroexo::BleTrialClient client(neuroexo::makeBluezGattForTests(server.address));
      rejects([&] { client.connect("AA:BB:CC:DD:EE:FF"); }, "bad characteristic flags accepted");
    }
    std::cout << "BlueZ integration: discovery, flags, early replies, full trial, rejection, "
                 "timeout, bad sequence, disconnect, cleanup PASS\n";
    return 0;
  } catch (const std::exception& error) {
    std::cerr << "FAIL: " << error.what() << '\n';
    return 1;
  }
}
