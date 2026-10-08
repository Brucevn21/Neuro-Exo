#include <ArduinoBLE.h>
#include "receiver_protocol.h"

BLEService benchmarkService("b7e10000-6a2b-4f10-9c31-8b674045a901");
BLECharacteristic controlChar("b7e10001-6a2b-4f10-9c31-8b674045a901", BLEWrite, 20);
BLECharacteristic dataChar("b7e10002-6a2b-4f10-9c31-8b674045a901",
                           BLEWrite | BLEWriteWithoutResponse, bench::MAX_WRITE);
BLECharacteristic statusChar("b7e10003-6a2b-4f10-9c31-8b674045a901", BLERead, 20, true);
bench::Receiver receiver;
uint32_t previousPollUs = 0;

void publishStatus(uint8_t page) {
  uint8_t value[20];
  receiver.status(page, millis(), value);
  statusChar.writeValue(value, sizeof(value));
}

void onControl(BLEDevice central, BLECharacteristic characteristic) {
  (void)central;
  const uint8_t* value = characteristic.value();
  const int length = characteristic.valueLength();
  if (length < 6 || value[0] != bench::VERSION) {
    receiver.state = bench::ERROR; publishStatus(0); return;
  }
  const uint8_t operation = value[1];
  const uint32_t run = bench::read32(value + 2);
  if (operation == 1 && length == 12) {
    receiver.start(run, bench::read16(value + 6), bench::read32(value + 8), millis());
    previousPollUs = micros();
    publishStatus(0);
  } else if (operation == 2 && length == 7 && run == receiver.run) {
    publishStatus(value[6]);
  } else if (operation == 3 && length == 6 && run == receiver.run) {
    receiver.stop(millis());
    publishStatus(0);
  } else {
    receiver.state = bench::ERROR;
    publishStatus(0);
  }
}

void onData(BLEDevice central, BLECharacteristic characteristic) {
  (void)central;
  // Handle each write in its callback. Polling characteristic.written() in
  // loop() could overlook earlier values if several writes arrive per poll.
  receiver.receive(characteristic.value(), characteristic.valueLength(), millis());
}

void onDisconnect(BLEDevice central) {
  (void)central;
  receiver.stop(millis());
  publishStatus(0);
  BLE.advertise();
}

void setup() {
  Serial.begin(115200); // Never wait for Serial: the Nano can run from USB power.
  if (!BLE.begin()) {
    if (Serial) Serial.println("BLE initialization failed");
    while (true) delay(1000);
  }
  BLE.setLocalName("NanoBLEBench");
  BLE.setAdvertisedService(benchmarkService);
  benchmarkService.addCharacteristic(controlChar);
  benchmarkService.addCharacteristic(dataChar);
  benchmarkService.addCharacteristic(statusChar);
  BLE.addService(benchmarkService);
  controlChar.setEventHandler(BLEWritten, onControl);
  dataChar.setEventHandler(BLEWritten, onData);
  BLE.setEventHandler(BLEDisconnected, onDisconnect);
  publishStatus(0);
  BLE.advertise();
  previousPollUs = micros();
  if (Serial) Serial.println("NanoBLEBench ready; waiting for Python sender");
}

void loop() {
  const uint32_t now = micros();
  const uint32_t gap = now - previousPollUs; // Correct across micros() wrap.
  previousPollUs = now;
  if (receiver.state == bench::RUNNING && gap > receiver.maxPollGapUs)
    receiver.maxPollGapUs = gap;
  BLE.poll();
}
