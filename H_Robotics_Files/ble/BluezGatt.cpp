#include "BleJointClient.hpp"
#include <gio/gio.h>
#include <algorithm>
#include <cctype>
#include <cstring>
#include <mutex>
#include <stdexcept>
#include <thread>

namespace neuroexo {
namespace {
using Variant = std::unique_ptr<GVariant, decltype(&g_variant_unref)>;
Variant own(GVariant* value) { return Variant(value, g_variant_unref); }

std::string stringProperty(GVariant* properties, const char* name) {
  const gchar* value = nullptr;
  if (!g_variant_lookup(properties, name, "&s", &value)) {
    if (!g_variant_lookup(properties, name, "&o", &value)) return {};
  }
  return value;
}
bool flag(GVariant* properties, const char* needed) {
  auto flags = own(g_variant_lookup_value(properties, "Flags", G_VARIANT_TYPE("as")));
  if (!flags) return false;
  GVariantIter iter;
  g_variant_iter_init(&iter, flags.get());
  const gchar* value;
  while (g_variant_iter_next(&iter, "&s", &value))
    if (std::strcmp(value, needed) == 0) return true;
  return false;
}
std::string normalizeAddress(std::string address) {
  if (address.size() != 17) throw std::invalid_argument("use a Bluetooth address AA:BB:CC:DD:EE:FF");
  for (size_t i = 0; i < address.size(); ++i) {
    if (i % 3 == 2) {
      if (address[i] != ':') throw std::invalid_argument("invalid Bluetooth address");
    } else {
      if (!std::isxdigit(static_cast<unsigned char>(address[i])))
        throw std::invalid_argument("invalid Bluetooth address");
      address[i] = char(std::toupper(static_cast<unsigned char>(address[i])));
    }
  }
  return address;
}
bool sameUuid(const std::string& a, const char* b) {
  return a.size() == std::strlen(b) &&
         std::equal(a.begin(), a.end(), b, [](char x, char y) {
           return std::tolower(static_cast<unsigned char>(x)) == y;
         });
}

class BluezGatt final : public GattTransport {
 public:
  explicit BluezGatt(std::string testBus = {}) : testBus_(std::move(testBus)) {}
  ~BluezGatt() override { disconnect(); }

  void connect(const std::string& rawAddress, const std::string& adapter,
               Notification onTelemetry) override {
    disconnect();
    onTelemetry_ = std::move(onTelemetry);
    const std::string address = normalizeAddress(rawAddress);
    if (adapter.size() < 4 || adapter.substr(0, 3) != "hci" ||
        !std::all_of(adapter.begin() + 3, adapter.end(),
                     [](char c) { return std::isdigit(static_cast<unsigned char>(c)); }))
      throw std::invalid_argument("adapter must be hci0, hci1, etc.");
    adapter_ = "/org/bluez/" + adapter;
    GError* error = nullptr;
    if (testBus_.empty()) bus_ = g_bus_get_sync(G_BUS_TYPE_SYSTEM, nullptr, &error);
    else bus_ = g_dbus_connection_new_for_address_sync(testBus_.c_str(),
      GDBusConnectionFlags(G_DBUS_CONNECTION_FLAGS_AUTHENTICATION_CLIENT |
                           G_DBUS_CONNECTION_FLAGS_MESSAGE_BUS_CONNECTION),
      nullptr, nullptr, &error);
    if (!bus_) failGError("open D-Bus", error);
    bool discovering = false;
    try {
      if (!booleanProperty(adapter_, "org.bluez.Adapter1", "Powered"))
        throw std::runtime_error("Bluetooth adapter is off; use bluetoothctl power on");
      GVariantBuilder filter;
      g_variant_builder_init(&filter, G_VARIANT_TYPE_VARDICT);
      g_variant_builder_add(&filter, "{sv}", "Transport", g_variant_new_string("le"));
      call(adapter_, "org.bluez.Adapter1", "SetDiscoveryFilter",
           g_variant_new("(a{sv})", &filter));
      call(adapter_, "org.bluez.Adapter1", "StartDiscovery", nullptr);
      discovering = true;
      const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(15);
      do {
        findDevice(address);
        if (!device_.empty()) break;
        std::this_thread::sleep_for(std::chrono::milliseconds(50));
      } while (std::chrono::steady_clock::now() < deadline);
      call(adapter_, "org.bluez.Adapter1", "StopDiscovery", nullptr);
      discovering = false;
      if (device_.empty()) throw std::runtime_error("Nano not discovered; verify power and Nano33BLEFirmware (Nano33BLE_Master)");
      if (booleanProperty(device_, "org.bluez.Device1", "Connected"))
        throw std::runtime_error("Nano already connected; disconnect the patient app, Python reader, or other BLE client");
      // Mark ownership before Connect: a timed-out Connect may have succeeded.
      ownsDevice_ = true;
      call(device_, "org.bluez.Device1", "Connect", nullptr, 15000);
      const auto resolvedBy = std::chrono::steady_clock::now() + std::chrono::seconds(10);
      while (!booleanProperty(device_, "org.bluez.Device1", "ServicesResolved")) {
        if (std::chrono::steady_clock::now() >= resolvedBy)
          throw std::runtime_error("timed out resolving Nano GATT services");
        std::this_thread::sleep_for(std::chrono::milliseconds(50));
      }
      discoverCharacteristics();
      context_ = g_main_context_new();
      loop_ = g_main_loop_new(context_, FALSE);
      g_main_context_push_thread_default(context_);
      eventSubscription_ = g_dbus_connection_signal_subscribe(bus_, "org.bluez",
        "org.freedesktop.DBus.Properties", "PropertiesChanged", telemetry_.c_str(),
        "org.bluez.GattCharacteristic1", G_DBUS_SIGNAL_FLAGS_NONE, changed, this, nullptr);
      deviceSubscription_ = g_dbus_connection_signal_subscribe(bus_, "org.bluez",
        "org.freedesktop.DBus.Properties", "PropertiesChanged", device_.c_str(),
        "org.bluez.Device1", G_DBUS_SIGNAL_FLAGS_NONE, changed, this, nullptr);
      ownerSubscription_ = g_dbus_connection_signal_subscribe(bus_, "org.freedesktop.DBus",
        "org.freedesktop.DBus", "NameOwnerChanged", "/org/freedesktop/DBus",
        "org.bluez", G_DBUS_SIGNAL_FLAGS_NONE, ownerChanged, this, nullptr);
      g_main_context_pop_thread_default(context_);
      closedHandler_ = g_signal_connect(bus_, "closed", G_CALLBACK(busClosed), this);
      {
        std::lock_guard<std::mutex> guard(stateMutex_);
        failure_.clear();
      }
      worker_ = std::thread([this] {
        g_main_context_push_thread_default(context_);
        g_main_loop_run(loop_);
        g_main_context_pop_thread_default(context_);
      });
      call(telemetry_, "org.bluez.GattCharacteristic1", "StartNotify", nullptr);
      notifying_ = true;
    } catch (...) {
      if (discovering) bestEffort(adapter_, "org.bluez.Adapter1", "StopDiscovery");
      disconnect();
      throw;
    }
  }

  void write(const uint8_t* data, size_t length, bool withResponse, int timeoutMs) override {
    std::lock_guard<std::mutex> operation(operationMutex_);
    {
      std::lock_guard<std::mutex> guard(stateMutex_);
      if (!failure_.empty()) throw std::runtime_error(failure_);
    }
    if (!bus_ || !notifying_) throw std::runtime_error("BLE is not connected");
    GVariantBuilder options;
    g_variant_builder_init(&options, G_VARIANT_TYPE_VARDICT);
    // "request" waits for the ATT write response; "command" is write-without-response.
    g_variant_builder_add(&options, "{sv}", "type",
                          g_variant_new_string(withResponse ? "request" : "command"));
    auto* bytes = g_variant_new_fixed_array(G_VARIANT_TYPE_BYTE, data, length, 1);
    call(command_, "org.bluez.GattCharacteristic1", "WriteValue",
         g_variant_new("(@aya{sv})", bytes, &options), timeoutMs);
  }

  std::string failure() const override {
    std::lock_guard<std::mutex> guard(stateMutex_);
    return failure_;
  }

  void disconnect() noexcept override {
    std::lock_guard<std::mutex> operation(operationMutex_);
    if (notifying_) bestEffort(telemetry_, "org.bluez.GattCharacteristic1", "StopNotify");
    notifying_ = false;
    if (ownsDevice_) bestEffort(device_, "org.bluez.Device1", "Disconnect");
    ownsDevice_ = false;
    if (worker_.joinable()) {
      GSource* stop = g_idle_source_new();
      g_source_set_callback(stop, [](gpointer p) -> gboolean {
        g_main_loop_quit(static_cast<GMainLoop*>(p)); return G_SOURCE_REMOVE;
      }, loop_, nullptr);
      g_source_attach(stop, context_);
      g_source_unref(stop);
    }
    if (worker_.joinable()) worker_.join();
    if (bus_) {
      for (guint id : {eventSubscription_, deviceSubscription_, ownerSubscription_})
        if (id) g_dbus_connection_signal_unsubscribe(bus_, id);
      if (closedHandler_) g_signal_handler_disconnect(bus_, closedHandler_);
      g_object_unref(bus_);
    }
    if (loop_) g_main_loop_unref(loop_);
    if (context_) g_main_context_unref(context_);
    bus_ = nullptr; loop_ = nullptr; context_ = nullptr; closedHandler_ = 0;
    eventSubscription_ = deviceSubscription_ = ownerSubscription_ = 0;
    adapter_.clear(); device_.clear(); command_.clear(); telemetry_.clear();
    onTelemetry_ = nullptr;
  }

 private:
  [[noreturn]] static void failGError(const std::string& action, GError* error) {
    const std::string message = action + ": " + (error ? error->message : "unknown D-Bus error");
    if (error) g_error_free(error);
    throw std::runtime_error(message);
  }
  Variant call(const std::string& path, const char* interface, const char* method,
               GVariant* parameters, int timeout = 2000) {
    GError* error = nullptr;
    GVariant* result = g_dbus_connection_call_sync(bus_, "org.bluez", path.c_str(),
      interface, method, parameters, nullptr, G_DBUS_CALL_FLAGS_NONE, timeout, nullptr, &error);
    if (!result) failGError(method, error);
    return own(result);
  }
  void bestEffort(const std::string& path, const char* interface, const char* method) noexcept {
    if (!bus_ || path.empty()) return;
    try { call(path, interface, method, nullptr, 1000); } catch (...) {}
  }
  bool booleanProperty(const std::string& path, const char* interface, const char* property) {
    auto result = call(path, "org.freedesktop.DBus.Properties", "Get",
                       g_variant_new("(ss)", interface, property));
    auto wrapped = own(g_variant_get_child_value(result.get(), 0));
    auto value = own(g_variant_get_variant(wrapped.get()));
    if (!g_variant_is_of_type(value.get(), G_VARIANT_TYPE_BOOLEAN))
      throw std::runtime_error("unexpected BlueZ property type");
    return g_variant_get_boolean(value.get());
  }
  template<class Callback> void objects(const char* interface, Callback callback) {
    auto result = call("/", "org.freedesktop.DBus.ObjectManager", "GetManagedObjects", nullptr);
    auto map = own(g_variant_get_child_value(result.get(), 0));
    GVariantIter iter;
    g_variant_iter_init(&iter, map.get());
    const gchar* path;
    GVariant* interfaces;
    while (g_variant_iter_next(&iter, "{&o@a{sa{sv}}}", &path, &interfaces)) {
      auto holder = own(interfaces);
      auto properties = own(g_variant_lookup_value(interfaces, interface, G_VARIANT_TYPE_VARDICT));
      if (properties) callback(std::string(path), properties.get());
    }
  }
  void findDevice(const std::string& address) {
    objects("org.bluez.Device1", [&](const std::string& path, GVariant* properties) {
      if (stringProperty(properties, "Adapter") == adapter_ &&
          stringProperty(properties, "Address") == address) device_ = path;
    });
  }
  void discoverCharacteristics() {
    std::string service;
    objects("org.bluez.GattService1", [&](const std::string& path, GVariant* p) {
      if (stringProperty(p, "Device") == device_ && sameUuid(stringProperty(p, "UUID"), SERVICE_UUID)) {
        if (!service.empty()) throw std::runtime_error("ambiguous Nano joint service 180C");
        service = path;
      }
    });
    if (service.empty()) throw std::runtime_error("Nano joint service 180C missing; load Nano33BLEFirmware");
    objects("org.bluez.GattCharacteristic1", [&](const std::string& path, GVariant* p) {
      if (stringProperty(p, "Service") != service) return;
      const std::string uuid = stringProperty(p, "UUID");
      std::string* destination = nullptr;
      bool usable = false;
      if (sameUuid(uuid, COMMAND_UUID)) {
        destination = &command_;
        usable = flag(p, "write") && flag(p, "write-without-response");
      }
      if (sameUuid(uuid, TELEMETRY_UUID)) { destination = &telemetry_; usable = flag(p, "notify"); }
      if (destination) {
        if (!destination->empty() || !usable)
          throw std::runtime_error("Nano characteristic flags or uniqueness mismatch");
        *destination = path;
      }
    });
    if (command_.empty() || telemetry_.empty())
      throw std::runtime_error("Nano command 2A56 or telemetry 2A57 characteristic missing");
  }
  void failed(const std::string& reason) {
    std::lock_guard<std::mutex> guard(stateMutex_);
    if (failure_.empty()) failure_ = reason;
  }
  static void busClosed(GDBusConnection*, gboolean, GError*, gpointer data) {
    static_cast<BluezGatt*>(data)->failed("D-Bus connection closed");
  }
  static void ownerChanged(GDBusConnection*, const gchar*, const gchar*, const gchar*,
                           const gchar*, GVariant*, gpointer data) {
    static_cast<BluezGatt*>(data)->failed("BlueZ service owner changed");
  }
  static void changed(GDBusConnection*, const gchar*, const gchar*, const gchar*,
                      const gchar*, GVariant* parameters, gpointer data) {
    auto* self = static_cast<BluezGatt*>(data);
    try {
      const gchar* interface;
      GVariant* properties;
      GVariant* invalidated;
      g_variant_get(parameters, "(&s@a{sv}@as)", &interface, &properties, &invalidated);
      auto propertyHolder = own(properties);
      auto invalidatedHolder = own(invalidated);
      if (std::strcmp(interface, "org.bluez.Device1") == 0) {
        gboolean value;
        if ((g_variant_lookup(properties, "Connected", "b", &value) && !value) ||
            (g_variant_lookup(properties, "ServicesResolved", "b", &value) && !value))
          self->failed("Nano disconnected or services invalidated");
        return;
      }
      auto value = own(g_variant_lookup_value(properties, "Value", G_VARIANT_TYPE("ay")));
      if (!value) return;
      gsize length = 0;
      const auto* bytes = static_cast<const uint8_t*>(g_variant_get_fixed_array(value.get(), &length, 1));
      // The client validates and copies the bytes; nothing here blocks.
      if (self->onTelemetry_) self->onTelemetry_(bytes, length);
    } catch (const std::exception& error) { self->failed(error.what()); }
  }

  std::string testBus_, adapter_, device_, command_, telemetry_, failure_;
  Notification onTelemetry_;
  GDBusConnection* bus_ = nullptr;
  GMainContext* context_ = nullptr;
  GMainLoop* loop_ = nullptr;
  std::thread worker_;
  guint eventSubscription_ = 0, deviceSubscription_ = 0, ownerSubscription_ = 0;
  gulong closedHandler_ = 0;
  bool ownsDevice_ = false, notifying_ = false;
  std::mutex operationMutex_;
  mutable std::mutex stateMutex_;
};
}  // namespace
std::unique_ptr<GattTransport> makeBluezGatt() {
  return std::unique_ptr<GattTransport>(new BluezGatt());
}
// Test helper uses a private Unix D-Bus daemon, never the machine's Bluetooth.
std::unique_ptr<GattTransport> makeBluezGattForTests(const std::string& busAddress) {
  return std::unique_ptr<GattTransport>(new BluezGatt(busAddress));
}
}  // namespace neuroexo
