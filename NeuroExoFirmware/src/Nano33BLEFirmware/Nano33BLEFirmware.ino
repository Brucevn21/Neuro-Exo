#include <ArduinoBLE.h>
#include <Wire.h>

const int SLAVE_ADDR = 0x08;

BLEService commandService("180C");
BLEStringCharacteristic commandChar("2A56", BLEWrite | BLEWriteWithoutResponse, 20);

String messageBuffer = "";
unsigned long lastWriteTime = 0;
const unsigned long BUFFER_TIMEOUT = 200;

void setup() {
  Serial.begin(115200);
  Wire.begin();

  if (!BLE.begin()) {
    Serial.println("BLE failed to start!");
    while (1);
  }

  BLE.setLocalName("Nano33BLE_Master");
  BLE.setAdvertisedService(commandService);
  commandService.addCharacteristic(commandChar);
  BLE.addService(commandService);
  BLE.advertise();

  Serial.println("System Ready.");
}

void sendI2CStartCommand(uint16_t angle) {
  Serial.print("Sending Angle: ");
  Serial.println(angle);

  uint8_t msb = (angle >> 8) & 0xFF;
  uint8_t lsb = angle & 0xFF;

  Wire.beginTransmission(SLAVE_ADDR);
  Wire.write(0x01);
  Wire.write(msb);
  Wire.write(lsb);
  byte error = Wire.endTransmission();

  if (error == 0) {
    Serial.println("I2C Transmission Successful");
  } else {
    Serial.print("I2C Error: ");
    Serial.println(error);
  }
}

void loop() {
  BLEDevice central = BLE.central();

  if (central) {
    Serial.print("Central connected: ");
    Serial.println(central.address());

    while (central.connected()) {
      if (commandChar.written()) {
        String newData = commandChar.value();
        messageBuffer += newData;
        lastWriteTime = millis();

        Serial.print("Buffering: ");
        Serial.println(newData);
      }

      if (messageBuffer.length() > 0 && (millis() - lastWriteTime) > BUFFER_TIMEOUT) {
        messageBuffer.trim();

        Serial.print("Complete message: ");
        Serial.print(messageBuffer);
        Serial.print(" | Length: ");
        Serial.println(messageBuffer.length());

        uint16_t angle = messageBuffer.toInt();

        Serial.print("Parsed Angle: ");
        Serial.println(angle);

        sendI2CStartCommand(angle);

        messageBuffer = "";
      }
    }

    Serial.println("Central disconnected");
  }
}
