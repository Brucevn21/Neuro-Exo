#include <ArduinoBLE.h>
#include "trial_protocol.h"

// Learning prototype: every reported position is simulated.
// No Wire/I2C, motor pins, or encoder inputs are accessed.
BLEService trialService("b7e20000-6a2b-4f10-9c31-8b674045a901");
BLECharacteristic commandChar("b7e20001-6a2b-4f10-9c31-8b674045a901", BLEWrite, 20);
BLECharacteristic eventChar("b7e20002-6a2b-4f10-9c31-8b674045a901", BLENotify, 20, true);
BLECharacteristic infoChar("b7e20003-6a2b-4f10-9c31-8b674045a901", BLERead, 20, true);
neuroexo::Simulator arm;

void publishInfo() {
  uint8_t value[20];
  arm.info(value);
  infoChar.writeValue(value, sizeof(value));
}

void onCommand(BLEDevice central, BLECharacteristic characteristic) {
  (void)central;
  uint8_t reply[20];
  if (arm.process(characteristic.value(), characteristic.valueLength(), millis(), reply))
    eventChar.writeValue(reply, sizeof(reply));
}

void onConnect(BLEDevice central) {
  (void)central;
  arm.reset();
  publishInfo();
}

void onDisconnect(BLEDevice central) {
  (void)central;
  arm.reset();  // Discard the simulated trial; never resume automatically.
  publishInfo();
  BLE.advertise();
}

void setup() {
  Serial.begin(115200);
  if (!BLE.begin()) {
    if (Serial) Serial.println("BLE initialization failed");
    while (true) delay(1000);
  }
  BLE.setLocalName("NanoNeuroExo");
  BLE.setAdvertisedService(trialService);
  trialService.addCharacteristic(commandChar);
  trialService.addCharacteristic(eventChar);
  trialService.addCharacteristic(infoChar);
  BLE.addService(trialService);
  commandChar.setEventHandler(BLEWritten, onCommand);
  BLE.setEventHandler(BLEConnected, onConnect);
  BLE.setEventHandler(BLEDisconnected, onDisconnect);
  publishInfo();
  BLE.advertise();
  if (Serial) Serial.println("NanoNeuroExo ready: SIMULATED arm, no motor or I2C output");
}

void loop() {
  arm.tick(millis());
  BLE.poll();
}
