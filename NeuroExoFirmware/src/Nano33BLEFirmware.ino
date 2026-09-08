#include <ArduinoBLE.h>
#include <Wire.h>
#include "commProtocol.h"

using namespace NeuroExoProtocol;

// This Nano 33 BLE plays two roles:
//  1. Mediator: BLE peripheral toward the BeagleBone Black (BLE central) <-> I2C
//     master toward the Teensy 4.1 motor controller.
//  2. PMS: under-voltage protection for the shared power rail (relay cutoff).
const int SLAVE_ADDR = 0x08;
const unsigned long TELEMETRY_INTERVAL_MS = 20; // 20 Hz telemetry to the BeagleBone Black

// Command characteristic: BeagleBone Black -> Nano, 7-byte joint packet (mode/speed/target angle).
BLEService jointService("180C");
BLECharacteristic commandChar("2A56", BLEWrite | BLEWriteWithoutResponse, PACKET_SIZE);
// Telemetry characteristic: Nano -> BeagleBone Black, 7-byte joint packet (current angle/current).
BLECharacteristic telemetryChar("2A57", BLERead | BLENotify, PACKET_SIZE);

// Ping-pong buffer: one half is filled with fresh telemetry from the Teensy
// while the other half (already complete) is safely sent out over BLE.
JointPacketPingPongBuffer telemetryBuffer;
unsigned long lastTelemetryTime = 0;

// --- Power Management System (under-voltage protection) ---
// Ported from PMSFirmware/NeuroExo_PMS_Code.ino so a single Nano 33 BLE can
// both mediate joint traffic and guard the shared power rail.
const int PMS_ANALOG_PIN = A3;
const int PMS_RELAY_PIN = 5;

// Resistor values for the voltage divider
const float PMS_R1 = 101000.0; // 100k Ohms *Nominal value shown
const float PMS_R2 = 9900.0;   // 10k Ohms *Nominal value shown

// Voltage settings
const float PMS_THRESHOLD_VOLTAGE = 22.0; // Lower Voltage limit
const float PMS_HYSTERESIS = 0.5;         // Prevents relay chatter (re-engages at 22.5V)
const unsigned long PMS_CHECK_INTERVAL_MS = 500; // Sample rate for the voltage divider

unsigned long lastPMSCheckTime = 0;

// Samples the power rail through the voltage divider and drives the relay
// with hysteresis so it doesn't chatter near the threshold. Non-blocking so
// it never stalls BLE/I2C mediation, and runs every loop() iteration
// regardless of BLE connection state since it's a safety function.
void checkPowerSupply() {
  if (millis() - lastPMSCheckTime < PMS_CHECK_INTERVAL_MS) {
    return;
  }
  lastPMSCheckTime = millis();

  // Read ADC (0 to 1023)
  int rawValue = analogRead(PMS_ANALOG_PIN);

  // Convert ADC value to voltage at the pin (3.3V Logic)
  float vOut = (rawValue * 3.3) / 1023.0;

  // Calculate original input voltage based on the divider ratio
  // Formula: Vin = Vout * (R1 + R2) / R2
  float vIn = vOut * ((PMS_R1 + PMS_R2) / PMS_R2);

  Serial.print("PMS Input Voltage: ");
  Serial.print(vIn);
  Serial.println(" V");

  if (vIn < PMS_THRESHOLD_VOLTAGE) {
    // Voltage too low! Disconnect the load.
    digitalWrite(PMS_RELAY_PIN, LOW);
  } else if (vIn > (PMS_THRESHOLD_VOLTAGE + PMS_HYSTERESIS)) {
    // Voltage is safe and above recovery threshold. Connect load.
    digitalWrite(PMS_RELAY_PIN, HIGH);
  }
  // Otherwise, hold the current relay state (hysteresis dead zone).
}

void setup() {
  // Relay defaults OFF for safety until the rail voltage is verified.
  pinMode(PMS_RELAY_PIN, OUTPUT);
  digitalWrite(PMS_RELAY_PIN, LOW);

  Serial.begin(115200);
  Wire.begin();

  if (!BLE.begin()) {
    Serial.println("BLE failed to start!");
    while (1);
  }

  BLE.setLocalName("Nano33BLE_Master");
  BLE.setAdvertisedService(jointService);
  jointService.addCharacteristic(commandChar);
  jointService.addCharacteristic(telemetryChar);
  BLE.addService(jointService);
  BLE.advertise();

  Serial.println("System Ready.");
}

// Forwards a joint packet received over BLE straight through to the Teensy over I2C.
void forwardCommandToTeensy(const uint8_t packet[PACKET_SIZE]) {
  Wire.beginTransmission(SLAVE_ADDR);
  Wire.write(I2C_CMD_JOINT_PACKET);
  Wire.write(packet, PACKET_SIZE);
  byte error = Wire.endTransmission();

  if (error != 0) {
    Serial.print("I2C Error: ");
    Serial.println(error);
  }
}

// Pulls the latest joint packet from the Teensy and republishes it over BLE
// using the ping-pong buffer so the notify payload is never torn mid-fill.
void pollTeensyTelemetry() {
  int received = Wire.requestFrom(SLAVE_ADDR, (int)PACKET_SIZE);
  if (received != PACKET_SIZE) {
    return;
  }

  uint8_t *writeBuf = telemetryBuffer.writeBuffer();
  for (uint8_t i = 0; i < PACKET_SIZE; ++i) {
    writeBuf[i] = Wire.read();
  }

  telemetryBuffer.swap();
  telemetryChar.writeValue(telemetryBuffer.readBuffer(), PACKET_SIZE);
}

void loop() {
  // Runs every iteration, connected or not - power protection can't wait on BLE.
  checkPowerSupply();

  BLEDevice central = BLE.central();

  if (central) {
    Serial.print("Central connected: ");
    Serial.println(central.address());

    while (central.connected()) {
      checkPowerSupply();

      if (commandChar.written()) {
        uint8_t packet[PACKET_SIZE];
        int len = commandChar.readValue(packet, PACKET_SIZE);
        if (len == PACKET_SIZE) {
          forwardCommandToTeensy(packet);
        }
      }

      if (millis() - lastTelemetryTime >= TELEMETRY_INTERVAL_MS) {
        lastTelemetryTime = millis();
        pollTeensyTelemetry();
      }
    }

    Serial.println("Central disconnected");
  }
}
