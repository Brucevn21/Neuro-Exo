/*
 * simplified_motor_trigger_i2c.ino
 * Receives I2C command from Nano 33 BLE and moves motor for 1 second.
 */
#include <Wire.h>
#include "motorDriver.h"
#include "pinMap4.1.h"

void receiveEvent(int howMany); // Forward declaration

// --- I2C CONFIGURATION ---
const int SLAVE_ADDR = 0x08; // Must match the Master's target address
volatile bool triggerMotor = false; 

// --- Motor Setup ---
motorWiring_t motorWiring;
motorLimit_t  motorLimit;
dir_t         direction;
motorDriver motor(1, (char*)"Simple Motor", 3.3f, 8);

// --- Timing Variables ---
unsigned long motorStartTime = 0;
bool motorRunning = false;
const float MOVE_VOLTAGE = -1.0f;

void setup() {
    Serial.begin(115200);

    pinMode(LED,OUTPUT);
    digitalWrite(LED, HIGH);
    // Motor hardware setup
    motorWiring.enablePin = ENABLE_PIN;
    motorWiring.dirPin    = DIR_PIN;
    motorWiring.pwmPin    = PWM_PIN;
    motorWiring.BWSwitchPin = 0;
	motorWiring.FWSwitchPin = 0;
	digitalWrite(motorWiring.FWSwitchPin, LOW);  //Set to HIGH if pins are set 0 or normal - T3.2
 	digitalWrite(motorWiring.BWSwitchPin, LOW); 
    motorLimit.forwardLimit = 150.0f;
    motorLimit.backwardLimit = -150.0f;
    direction.FORWARD  = LOW;
    direction.BACKWARD = HIGH;
    motor.init(motorWiring, motorLimit);

    // Initialize I2C as Slave
    Wire.begin(SLAVE_ADDR);
    // Register the function to call when data is received from Master
    Wire.onReceive(receiveEvent);

    Serial.println("I2C Slave Ready. Waiting for Master command...");
}

void loop() {
    // --- Trigger Logic ---
    if (triggerMotor && !motorRunning) {
        Serial.println("I2C Trigger! Starting 3s move...");
        
        // Command the motor ONCE
        motor.rotate(MOVE_VOLTAGE, direction); 
        
        motorStartTime = millis();
        motorRunning = true;
        triggerMotor = false; 
    }

    // --- Motor Timing Logic (Non-blocking) ---
    if (motorRunning) {
        // Check if time is up
        if (millis() - motorStartTime >= 1000) {
            motor.stop(); 
            motor.disable();    
            motorRunning = false;
            Serial.println("Sequence Complete.");
        }
        // Notice: We don't call motor.rotate() here anymore. 
        // The PWM hardware on the Teensy will keep the signal 
        // steady at 2.5V automatically.
    }
}

// --- I2C INTERRUPT HANDLER ---
// This runs in the background whenever the Master sends data
void receiveEvent(int howMany) {
    while (Wire.available()) {
        uint8_t received = Wire.read();
        if (received == 0x01) {
            triggerMotor = true; 
        }
    }
}