#include <ArduinoBLE.h>
#include <Wire.h>

// --- Configuration ---
const int SLAVE_ADDR = 0x08;

BLEService commandService("180C");
BLEStringCharacteristic commandChar("2A56", BLEWrite | BLEWriteWithoutResponse, 20); 

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

// --- Send I2C packet ---
void sendI2CStartCommand(uint16_t angle) {
  Serial.print("Sending Angle: ");
  Serial.println(angle);

  uint8_t lsb = angle & 0xFF;
  uint8_t msb = (angle >> 8) & 0xFF;

  Wire.beginTransmission(SLAVE_ADDR);
  Wire.write(0x01);   // Command byte
  Wire.write(lsb);    // LSB
  Wire.write(msb);    // MSB
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
        // Get the BLE characteristic value as a String
        String incoming = commandChar.value();
        incoming.trim();  // remove \n, spaces

        Serial.print("Received string: ");
        Serial.print(incoming);
        Serial.print(" | Length: ");
        Serial.println(incoming.length());

        // Convert directly to integer
        uint16_t angle = incoming.toInt();

        Serial.print("Parsed Angle: ");
        Serial.println(angle);

        // Send over I2C
        sendI2CStartCommand(angle);
      }
    }

    Serial.println("Central disconnected");
  }
}