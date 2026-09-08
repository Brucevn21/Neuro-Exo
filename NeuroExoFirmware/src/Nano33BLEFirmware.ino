#include <ArduinoBLE.h>
#include <Wire.h>
#include "commProtocol.h"

using namespace NeuroExoProtocol;

const int SLAVE_ADDR = 0x08;
const unsigned long TELEMETRY_INTERVAL_MS = 20; // 20 Hz telemetry to the app

// Command characteristic: app -> Nano, 7-byte joint packet (mode/speed/target angle).
BLEService jointService("180C");
BLECharacteristic commandChar("2A56", BLEWrite | BLEWriteWithoutResponse, PACKET_SIZE);
// Telemetry characteristic: Nano -> app, 7-byte joint packet (current angle/current).
BLECharacteristic telemetryChar("2A57", BLERead | BLENotify, PACKET_SIZE);
// Stop characteristic: app -> Nano, any write triggers an immediate motion halt.
BLECharacteristic stopChar("2A58", BLEWrite | BLEWriteWithoutResponse, 1);

// Ping-pong buffer: one half is filled with fresh telemetry from the Teensy
// while the other half (already complete) is safely sent out over BLE.
JointPacketPingPongBuffer telemetryBuffer;
unsigned long lastTelemetryTime = 0;

void setup() {
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
  jointService.addCharacteristic(stopChar);
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

// Forwards a mid-motion stop request straight through to the Teensy over I2C.
void forwardStopToTeensy() {
  Wire.beginTransmission(SLAVE_ADDR);
  Wire.write(I2C_CMD_STOP);
  byte error = Wire.endTransmission();

  if (error != 0) {
    Serial.print("I2C Error (stop): ");
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
  BLEDevice central = BLE.central();

  if (central) {
    Serial.print("Central connected: ");
    Serial.println(central.address());

    while (central.connected()) {
      if (commandChar.written()) {
        uint8_t packet[PACKET_SIZE];
        int len = commandChar.readValue(packet, PACKET_SIZE);
        if (len == PACKET_SIZE) {
          forwardCommandToTeensy(packet);
        }
      }

      if (stopChar.written()) {
        forwardStopToTeensy();
      }

      if (millis() - lastTelemetryTime >= TELEMETRY_INTERVAL_MS) {
        lastTelemetryTime = millis();
        pollTeensyTelemetry();
      }
    }

    Serial.println("Central disconnected");
  }
}
