/*
 * simplified_motor_trigger.ino
 * Receives a single byte from Nano 33 BLE and moves motor for 1 second.
 */
// Bluetooth Test Code for Teensy 4.1 - SPI Slave Example
#include <SPI.h>
#include <SPISlave_T4.h>
#include "motorDriver.h"
#include "pinMap4.1.h" // Assumes Teensy 4.1 based on previous code

// --- Motor Setup ---
motorWiring_t motorWiring;
motorLimit_t  motorLimit;
dir_t         direction;
motorDriver motor(1, (char*)"Simple Motor", 3.3f, 8);

// --- Timing Variables ---
unsigned long motorStartTime = 0;
bool motorRunning = false;
const float MOVE_VOLTAGE = 1.5f; // Adjust this (0.0 to 3.3) for speed

// --- SPI Slave Setup ---
SPISlave_T4<&SPI, SPI_8_BITS> mySlave;

void setup() {
    Serial.begin(115200);

    // 1. Motor Pin Wiring (from your pinMap)
    motorWiring.enablePin = ENABLE_PIN;
    motorWiring.dirPin    = DIR_PIN;
    motorWiring.pwmPin    = PWM_PIN;
    
    // 2. Basic Motor Limits
    motorLimit.forwardLimit = 150.0f;
    motorLimit.backwardLimit = -150.0f;
    
    // 3. Direction Logic
    direction.FORWARD  = LOW;
    direction.BACKWARD = HIGH;

    motor.init(motorWiring, motorLimit);
    
    // 4. Start SPI Slave
    mySlave.begin();
    Serial.println("Teensy Ready. Waiting for 0x01 from Nano...");
}

void loop() {
    // 1. Check if there is data in the SPI receive hardware
    if (mySlave.available()) {
        
        // 2. 'popr' pulls the byte out of the hardware register
        uint8_t received = mySlave.popr(); 
        
        Serial.print("Received from Nano: 0x");
        Serial.println(received, HEX);

        if (received == 0x01) {
            Serial.println("Triggering motor for 1 second...");
            motorStartTime = millis();
            motorRunning = true;
        }
    }

    // --- Motor Timing Logic ---
    if (motorRunning == true) {
        if (millis() - motorStartTime < 1000) {
            motor.rotate(MOVE_VOLTAGE, direction);
        } else {
            motor.stop();
            motor.disable();
            motorRunning = false;
            Serial.println("Done.");
        }
    }
}