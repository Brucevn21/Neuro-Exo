#pragma once
// The same BlueZ/GIO API used by the older BeagleBone client, scoped to
// ProtocolDemo's service. Every notification reaches the supplied callback.
#include <gio/gio.h>
#include <algorithm>
#include <chrono>
#include <cctype>
#include <cstring>
#include <functional>
#include <memory>
#include <mutex>
#include <stdexcept>
#include <string>
#include <thread>
#include <vector>

class BluezLink {
 public:
  using Clock = std::chrono::steady_clock;
  using Callback = std::function<void(const uint8_t*, size_t)>;
  explicit BluezLink(Callback callback) : callback_(std::move(callback)) {}
  ~BluezLink() { disconnect(); }
  BluezLink(const BluezLink&) = delete;
  BluezLink& operator=(const BluezLink&) = delete;
  void connect(std::string address, const std::string& adapter, bool response, bool telemetry = true) {
    disconnect();
    adapter_.clear(); device_.clear(); tx_.clear(); rx_.clear();
    { std::lock_guard<std::mutex> lock(mutex_); failure_.clear(); }
    if (address.size() != 17) throw std::runtime_error("use Nano address AA:BB:CC:DD:EE:FF");
    for (size_t i = 0; i < address.size(); ++i) {
      if (i % 3 == 2 ? address[i] != ':' : !std::isxdigit(static_cast<unsigned char>(address[i])))
        throw std::runtime_error("invalid Nano Bluetooth address");
      address[i] = char(std::toupper(static_cast<unsigned char>(address[i])));
    }
    if (adapter.size() < 4 || adapter.substr(0, 3) != "hci" ||
        !std::all_of(adapter.begin() + 3, adapter.end(), [](char c) { return c >= '0' && c <= '9'; }))
      throw std::runtime_error("adapter must be hci0, hci1, etc.");
    GError* error = nullptr;
    bus_ = g_bus_get_sync(G_BUS_TYPE_SYSTEM, nullptr, &error);
    if (!bus_) fail("open system D-Bus", error);
    adapter_ = "/org/bluez/" + adapter;
    if (!boolean(adapter_, "org.bluez.Adapter1", "Powered"))
      throw std::runtime_error("Bluetooth is off; run bluetoothctl power on");
    GVariantBuilder filter; g_variant_builder_init(&filter, G_VARIANT_TYPE_VARDICT);
    g_variant_builder_add(&filter, "{sv}", "Transport", g_variant_new_string("le"));
    call(adapter_, "org.bluez.Adapter1", "SetDiscoveryFilter", g_variant_new("(a{sv})", &filter));
    call(adapter_, "org.bluez.Adapter1", "StartDiscovery", nullptr); discovering_ = true;
    auto deadline = Clock::now() + std::chrono::seconds(15);
    while (device_.empty() && Clock::now() < deadline) {
      objects("org.bluez.Device1", [&](const std::string& path, GVariant* p) {
        if (property(p, "Address") == address && property(p, "Adapter") == adapter_) device_ = path;
      });
      if (device_.empty()) std::this_thread::sleep_for(std::chrono::milliseconds(50));
    }
    call(adapter_, "org.bluez.Adapter1", "StopDiscovery", nullptr); discovering_ = false;
    if (device_.empty()) throw std::runtime_error("Nano not found; check power and Bluetooth address");
    if (boolean(device_, "org.bluez.Device1", "Connected"))
      throw std::runtime_error("Nano is already connected; close the other BLE client first");
    owned_ = true;
    call(device_, "org.bluez.Device1", "Connect", nullptr, 15000);
    deadline = Clock::now() + std::chrono::seconds(10);
    while (!boolean(device_, "org.bluez.Device1", "ServicesResolved")) {
      if (Clock::now() >= deadline) throw std::runtime_error("service discovery timed out");
      std::this_thread::sleep_for(std::chrono::milliseconds(50));
    }
    std::string service;
    objects("org.bluez.GattService1", [&](const std::string& path, GVariant* p) {
      if (property(p, "Device") == device_ && property(p, "UUID") == "0000180c-0000-1000-8000-00805f9b34fb") {
        if (!service.empty()) throw std::runtime_error("multiple ProtocolDemo services");
        service = path;
      }
    });
    if (service.empty()) throw std::runtime_error("ProtocolDemo service 180C not found; check Nano firmware");
    objects("org.bluez.GattCharacteristic1", [&](const std::string& path, GVariant* p) {
      if (property(p, "Service") != service) return;
      const auto uuid = property(p, "UUID");
      if (uuid == "00002a56-0000-1000-8000-00805f9b34fb") {
        if (!tx_.empty() || !flag(p, "write") || !flag(p, response ? "write" : "write-without-response"))
          throw std::runtime_error("command characteristic flags/uniqueness mismatch");
        tx_ = path;
      }
      if (telemetry && uuid == "00002a57-0000-1000-8000-00805f9b34fb") {
        if (!rx_.empty() || !flag(p, "notify")) throw std::runtime_error("telemetry characteristic mismatch");
        rx_ = path;
      }
    });
    if (tx_.empty() || (telemetry && rx_.empty())) throw std::runtime_error("ProtocolDemo characteristics missing");
    context_ = g_main_context_new(); loop_ = g_main_loop_new(context_, FALSE);
    g_main_context_push_thread_default(context_);
    subscription_ = g_dbus_connection_signal_subscribe(bus_, "org.bluez", "org.freedesktop.DBus.Properties",
        "PropertiesChanged", nullptr, nullptr, G_DBUS_SIGNAL_FLAGS_NONE, changed, this, nullptr);
    ownerSubscription_ = g_dbus_connection_signal_subscribe(bus_, "org.freedesktop.DBus", "org.freedesktop.DBus",
        "NameOwnerChanged", "/org/freedesktop/DBus", "org.bluez", G_DBUS_SIGNAL_FLAGS_NONE,
        [](GDBusConnection*, const gchar*, const gchar*, const gchar*, const gchar*, GVariant*, gpointer p) {
          static_cast<BluezLink*>(p)->failed("BlueZ service changed or stopped");
        }, this, nullptr);
    g_main_context_pop_thread_default(context_);
    worker_ = std::thread([this] { g_main_loop_run(loop_); });
    if (telemetry) {
      call(rx_, "org.bluez.GattCharacteristic1", "StartNotify", nullptr); notifying_ = true;
    }
  }
  void check() {
    std::lock_guard<std::mutex> lock(mutex_);
    if (!failure_.empty()) throw std::runtime_error(failure_);
    if (!bus_ || g_dbus_connection_is_closed(bus_)) throw std::runtime_error("D-Bus connection closed");
  }
  void write(const std::vector<uint8_t>& raw, bool response) {
    check();
    GVariantBuilder options; g_variant_builder_init(&options, G_VARIANT_TYPE_VARDICT);
    g_variant_builder_add(&options, "{sv}", "type", g_variant_new_string(response ? "request" : "command"));
    call(tx_, "org.bluez.GattCharacteristic1", "WriteValue", g_variant_new("(@aya{sv})",
         g_variant_new_fixed_array(G_VARIANT_TYPE_BYTE, raw.data(), raw.size(), 1), &options), 1000);
  }
  void disconnect() noexcept {
    if (discovering_) bestEffort(adapter_, "org.bluez.Adapter1", "StopDiscovery");
    if (notifying_) bestEffort(rx_, "org.bluez.GattCharacteristic1", "StopNotify");
    if (owned_) bestEffort(device_, "org.bluez.Device1", "Disconnect");
    discovering_ = notifying_ = owned_ = false;
    if (worker_.joinable()) {
      GSource* stop = g_idle_source_new();
      g_source_set_callback(stop, [](gpointer p) -> gboolean { g_main_loop_quit(static_cast<GMainLoop*>(p)); return G_SOURCE_REMOVE; }, loop_, nullptr);
      g_source_attach(stop, context_); g_source_unref(stop); worker_.join();
    }
    if (bus_) {
      if (subscription_) g_dbus_connection_signal_unsubscribe(bus_, subscription_);
      if (ownerSubscription_) g_dbus_connection_signal_unsubscribe(bus_, ownerSubscription_);
      g_object_unref(bus_);
    }
    if (loop_) g_main_loop_unref(loop_);
    if (context_) g_main_context_unref(context_);
    bus_ = nullptr; loop_ = nullptr; context_ = nullptr; subscription_ = ownerSubscription_ = 0;
  }
 private:
  using Variant = std::unique_ptr<GVariant, decltype(&g_variant_unref)>;
  static Variant own(GVariant* value) { return Variant(value, g_variant_unref); }
  [[noreturn]] static void fail(const std::string& action, GError* error) {
    const std::string message = action + ": " + (error ? error->message : "unknown error");
    if (error) g_error_free(error);
    throw std::runtime_error(message);
  }
  Variant call(const std::string& path, const char* interface, const char* method, GVariant* parameters, int timeout = 2000) {
    GError* error = nullptr;
    auto* result = g_dbus_connection_call_sync(bus_, "org.bluez", path.c_str(), interface, method,
        parameters, nullptr, G_DBUS_CALL_FLAGS_NONE, timeout, nullptr, &error);
    if (!result) fail(method, error);
    return own(result);
  }
  void bestEffort(const std::string& path, const char* interface, const char* method) noexcept {
    try { if (bus_ && !path.empty()) call(path, interface, method, nullptr, 1000); } catch (...) {}
  }
  bool boolean(const std::string& path, const char* interface, const char* key) {
    auto reply = call(path, "org.freedesktop.DBus.Properties", "Get", g_variant_new("(ss)", interface, key));
    auto wrapped = own(g_variant_get_child_value(reply.get(), 0)); auto value = own(g_variant_get_variant(wrapped.get()));
    return g_variant_is_of_type(value.get(), G_VARIANT_TYPE_BOOLEAN) && g_variant_get_boolean(value.get());
  }
  static std::string property(GVariant* props, const char* key) {
    const gchar* value = nullptr;
    if (!g_variant_lookup(props, key, "&s", &value) && !g_variant_lookup(props, key, "&o", &value)) return {};
    return value;
  }
  static bool flag(GVariant* props, const char* key) {
    auto value = own(g_variant_lookup_value(props, "Flags", G_VARIANT_TYPE("as")));
    if (!value) return false;
    GVariantIter iter; g_variant_iter_init(&iter, value.get()); const gchar* name;
    while (g_variant_iter_next(&iter, "&s", &name)) if (!std::strcmp(name, key)) return true;
    return false;
  }
  template<class F> void objects(const char* interface, F callback) {
    auto reply = call("/", "org.freedesktop.DBus.ObjectManager", "GetManagedObjects", nullptr);
    auto map = own(g_variant_get_child_value(reply.get(), 0));
    GVariantIter iter; g_variant_iter_init(&iter, map.get()); const gchar* path; GVariant* interfaces;
    while (g_variant_iter_next(&iter, "{&o@a{sa{sv}}}", &path, &interfaces)) {
      auto holder = own(interfaces); auto props = own(g_variant_lookup_value(interfaces, interface, G_VARIANT_TYPE_VARDICT));
      if (props) callback(std::string(path), props.get());
    }
  }
  void failed(const std::string& why) { std::lock_guard<std::mutex> lock(mutex_); failure_ = why; }
  static void changed(GDBusConnection*, const gchar*, const gchar* path, const gchar*, const gchar*, GVariant* params, gpointer data) {
    auto* self = static_cast<BluezLink*>(data);
    try {
      const gchar* interface; GVariant* p; GVariant* invalid;
      g_variant_get(params, "(&s@a{sv}@as)", &interface, &p, &invalid);
      auto props = own(p); auto invalidated = own(invalid);
      if (self->device_ == path && !std::strcmp(interface, "org.bluez.Device1")) {
        gboolean value;
        if ((g_variant_lookup(p, "Connected", "b", &value) && !value) ||
            (g_variant_lookup(p, "ServicesResolved", "b", &value) && !value)) self->failed("Nano disconnected/services invalidated");
      }
      if (self->rx_ != path || std::strcmp(interface, "org.bluez.GattCharacteristic1")) return;
      auto value = own(g_variant_lookup_value(p, "Value", G_VARIANT_TYPE("ay")));
      if (!value) return;
      gsize size = 0; auto* raw = static_cast<const uint8_t*>(g_variant_get_fixed_array(value.get(), &size, 1));
      self->callback_(raw, size);
    } catch (const std::exception& error) { self->failed(error.what()); }
  }
  Callback callback_;
  GDBusConnection* bus_ = nullptr;
  GMainContext* context_ = nullptr;
  GMainLoop* loop_ = nullptr;
  std::thread worker_;
  std::string adapter_, device_, tx_, rx_, failure_;
  std::mutex mutex_;
  guint subscription_ = 0, ownerSubscription_ = 0;
  bool discovering_ = false, notifying_ = false, owned_ = false;
};
