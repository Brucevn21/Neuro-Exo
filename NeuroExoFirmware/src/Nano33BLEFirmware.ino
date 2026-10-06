#include <ArduinoBLE.h>
#include <Wire.h>
#include "commProtocol.h"

using namespace NeuroExoProtocol;

// This Nano 33 BLE plays two roles:
//  1. Mediator: BLE peripheral toward the BeagleBone Black (BLE central) <-> I2C
//     master toward the Teensy 4.1 motor controller.
//  2. PMS: under-voltage protection for the shared power rail (relay cutoff).
const int SLAVE_ADDR = 0x08;
const unsigned long TELEMETRY_INTERVAL_MS = 20; // 50 Hz telemetry to the BeagleBone Black

// Command characteristic: BeagleBone Black -> Nano, framed control packet.
BLEService jointService("180C");
BLECharacteristic commandChar("2A56", BLEWrite | BLEWriteWithoutResponse, MAX_FRAME_SIZE);
// Telemetry characteristic: Nano -> BeagleBone Black, framed telemetry packet.
BLECharacteristic telemetryChar("2A57", BLERead | BLENotify, MAX_FRAME_SIZE);

unsigned long lastTelemetryTime = 0;
unsigned long lastCommandByteTime = 0;
FrameParser commandParser;

// --- Power Management System (under-voltage protection) ---
// Ported from PMSFirmware/NeuroExo_PMS_Code.ino so a single Nano 33 BLE can
// both mediate joint traffic and guard the shared power rail.
const int PMS_ANALOG_PIN = A3;
const int PMS_RELAY_PIN = 5;

// Resistor values for the voltage divider
const float PMS_R1 = 101000.0; // 100k Ohms *Nominal value shown
const float PMS_R2 = 9900.0;   // 10k Ohms *Nominal value shown

// Voltage settings
const float PMS_THRESHOLD_VOLTAGE = 22.0;   // Lower Voltage limit
const float PMS_OVERVOLTAGE_CUTOFF = 30.0;  // Upper Voltage limit
const float PMS_HYSTERESIS = 0.5;           // Prevents relay chatter (re-engages between 22.5V and 32.0V)
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

  if (vIn < PMS_THRESHOLD_VOLTAGE) {
    // Voltage too low! Disconnect the load.
    digitalWrite(PMS_RELAY_PIN, LOW);
  } else if (vIn > PMS_OVERVOLTAGE_CUTOFF) {
    // Voltage too high! Disconnect the load.
    digitalWrite(PMS_RELAY_PIN, LOW);
  } else if (vIn > (PMS_THRESHOLD_VOLTAGE + PMS_HYSTERESIS) &&
             vIn < (PMS_OVERVOLTAGE_CUTOFF - PMS_HYSTERESIS)) {
    // Voltage is safe and inside both recovery thresholds. Connect load.
    digitalWrite(PMS_RELAY_PIN, HIGH);
  }
  // Otherwise, hold the current relay state (hysteresis dead zone).
}

const char *modeName(Mode mode) {
  switch (mode) {
    case Mode::Resistive: return "Resistive";
    case Mode::Assistive: return "Assistive";
    case Mode::Neutral: return "Neutral";
    default: return "Unknown";
  }
}

const char *speedName(Speed speed) {
  switch (speed) {
    case Speed::Slow: return "Slow";
    case Speed::Medium: return "Medium";
    case Speed::High: return "High";
    default: return "Unknown";
  }
}

void printReceivedCommand(const uint8_t *frame, uint8_t length, const ControlPacket &command) {
  Serial.println("[Bridge] Packet received from host");
  Serial.print("RAW: [");
  for (uint8_t i = 0; i < length; ++i) {
    if (i > 0) {
      Serial.print(", ");
    }
    Serial.print("0x");
    if (frame[i] < 0x10) {
      Serial.print('0');
    }
    Serial.print(frame[i], HEX);
  }
  Serial.println("]");
  Serial.print("Target Angle: ");
  Serial.print(command.targetAngleDeg);
  Serial.println(" deg");
  Serial.print("Mode: ");
  Serial.println(modeName(command.mode));
  Serial.print("Speed: ");
  Serial.println(speedName(command.speed));
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

void forwardCommandToTeensy(const uint8_t *frame, uint8_t length) {
  Wire.beginTransmission(SLAVE_ADDR);
  Wire.write(frame, length);
  byte error = Wire.endTransmission();

  if (error != 0) {
    Serial.print("I2C Error: ");
    Serial.println(error);
  }
}

void pollTeensyTelemetry() {
  int received = Wire.requestFrom(SLAVE_ADDR, (int)MAX_FRAME_SIZE);
  if (received <= 0 || received > MAX_FRAME_SIZE) {
    return;
  }

  FrameParser parser;
  for (int i = 0; i < received; ++i) {
    parser.push((uint8_t)Wire.read());
  }
  Frame frame;
  TelemetryPacket telemetry;
  if (!parser.takeFrame(frame) || !decodeTelemetryFrame(frame, telemetry)) {
    return;
  }

  uint8_t encoded[MAX_FRAME_SIZE];
  uint8_t length = encodeTelemetryFrame(telemetry, encoded);
  telemetryChar.writeValue(encoded, length);
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
        uint8_t bytes[MAX_FRAME_SIZE];
        int len = commandChar.readValue(bytes, MAX_FRAME_SIZE);
        unsigned long now = millis();
        if (now - lastCommandByteTime > FRAME_TIMEOUT_MS) {
          commandParser.reset();
        }
        lastCommandByteTime = now;
        if (len > 0 && len <= MAX_FRAME_SIZE) {
          for (int i = 0; i < len; ++i) {
            commandParser.push(bytes[i]);
          }
          Frame frame;
          ControlPacket command;
          if (commandParser.takeFrame(frame) && decodeControlFrame(frame, command)) {
            uint8_t encoded[MAX_FRAME_SIZE];
            uint8_t frameLength = encodeControlFrame(command, encoded);
            printReceivedCommand(encoded, frameLength, command);
            forwardCommandToTeensy(encoded, frameLength);
          }
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
