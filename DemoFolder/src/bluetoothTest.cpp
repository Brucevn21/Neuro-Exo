/*
 * Combined Bluetooth + PID Motor Control
 * 
 * Combines working I2C Bluetooth control (from bluetoothTest.cpp) with 
 * PID motor control to 45 degrees (from NeuroExoFirmware main.cpp)
 * 
 * Features:
 * - I2C Slave: Receives 0x01 packet to trigger motor motion
 * - PID Control: Smooth interpolated motion to 45 degrees
 * - Encoder Feedback: AS5045 encoder for position control
 * - Interrupt-based loop: 2ms (500 Hz) motor control ISR
 * - Safety: Position error detection and emergency stop
 */

#include <Wire.h>
#include <IntervalTimer.h>
#include "motorDriver.h"
#include "pinMap4.1.h"
#include "AS5045.h"
#include "encoder_utils.h"
#include "luu_utils.h"

// --- I2C CONFIGURATION ---
const int SLAVE_ADDR = 0x08; // Must match the Master's target address
volatile bool triggerMotor = false;

// --- Motor Setup (from working bluetoothTest.cpp) ---
motorWiring_t motorWiring;
motorLimit_t motorLimit;
dir_t direction;
motorDriver motor(1, (char*)"Combined Motor", 3.3f, 8);

// --- Encoder Setup (from main.cpp) ---
AS5045 myAS5045(AS5045_CS_PIN, AS5045_CLK_PIN, AS5045_DATA_PIN, 0xFF, 3);
unsigned int encBinary;
float encRaw;
volatile float encDeg = 0.0f;
float encRange[2] = {180, -180};
float encOffset = 0.0f;
int nbits = 16;

// --- PID Control Setup (from main.cpp) ---
PID_t jointPID;
volatile float interpolateBegin = 0.0f;
volatile float interpolateEnd = 45.0f; // Target 45 degrees
volatile float setPointInterpolated = 0.0f;
volatile int interpCounter = 0;
volatile bool interpInitialized = false;
volatile int interpCycles = 4000; // Number of cycles for interpolation
volatile float interpIncrement = 0.0f;
float Vc = 0.0f; // Motor voltage command
IntervalTimer motorControlTimer;

// --- Timing Variables ---
volatile bool motorMotionActive = false;
float DerivativeFilterAlpha = 0.2f;
unsigned long lastDebugTime = 0;

// --- Forward Declarations ---
void receiveEvent(int howMany);
void motorControlISR();

/**
 * Motor Control ISR - Runs every 2ms (500 Hz)
 * Performs trajectory interpolation and PID control
 */
void motorControlISR() {
    // Initialize interpolation only once at the beginning
    if (!interpInitialized && motorMotionActive) {
        interpolateBegin = encDeg; // Set start position only once
        interpCounter = 0;
        setPointInterpolated = interpolateBegin;
        interpIncrement = (interpolateEnd - interpolateBegin) / (float)interpCycles;
        interpInitialized = true;
    }

    // Interpolate trajectory: smooth motion from start to end position
    if (interpCounter < interpCycles && motorMotionActive) {
        setPointInterpolated += interpIncrement;
        interpCounter++;
    } else if (motorMotionActive) {
        setPointInterpolated = interpolateEnd;
        motorMotionActive = false; // Motion complete
        interpInitialized = false;
    }

    // PID control: calculate motor voltage command
    bool safetyStop = false;

    // Safety check: Large position error detection (>10° difference)
    float positionError = abs(setPointInterpolated - encDeg);
    if (positionError > 10.0f) {
        safetyStop = true;
    }

    if (safetyStop || !motorMotionActive) {
        Vc = 0.0f;
        motor.disable();
        jointPID.integral = 0.0f;
        jointPID.lastError = 0.0f;
        jointPID.filteredDerivative = 0.0f;
    } else {
        float motorControl = motor.computePID(setPointInterpolated, encDeg, jointPID);
        motor.enable();
        Vc = motorControl;
        motor.rotate(Vc, direction);
    }
}

/**
 * I2C Interrupt Handler
 * Called when Master sends data
 */
void receiveEvent(int howMany) {
    while (Wire.available()) {
        uint8_t received = Wire.read();
        if (received == 0x01) {
            // Trigger motor motion to 45 degrees
            if (!motorMotionActive) {
                motorMotionActive = true;
                interpolateEnd = 45.0f;
                interpInitialized = false;
                Serial.println("I2C Trigger! Starting PID motion to 45 degrees...");
            }
        }
    }
}

void setup() {
    Serial.begin(115200);
    delay(500);

    // LED setup
    pinMode(LED, OUTPUT);
    digitalWrite(LED, HIGH);

    // Encoder setup
    if (!myAS5045.begin()) {
        Serial.println("Error setting up AS5045");
        delay(1000);
    }

    // Motor hardware setup (from working bluetoothTest.cpp)
    motorWiring.enablePin = ENABLE_PIN;
    motorWiring.dirPin = DIR_PIN;
    motorWiring.pwmPin = PWM_PIN;
    motorWiring.BWSwitchPin = 0;
    motorWiring.FWSwitchPin = 0;
    digitalWrite(motorWiring.FWSwitchPin, LOW);
    digitalWrite(motorWiring.BWSwitchPin, LOW);

    // Motor limits
    motorLimit.forwardLimit = 35.0f;
    motorLimit.backwardLimit = -150.0f;

    // Direction setup
    direction.FORWARD = LOW;
    direction.BACKWARD = HIGH;

    // Initialize motor with working setup from bluetoothTest.cpp
    motor.init(motorWiring, motorLimit);

    // PID parameters (from main.cpp)
    jointPID = {0.2f, 0.0f, 0.002f, 0.002f, 0.0f, 0.0f, 0.0f, 0.3f, DerivativeFilterAlpha};

    // Setup motor control interrupt (2ms = 500 Hz)
    motorControlTimer.begin(motorControlISR, 2000);
    motorControlTimer.priority(128);

    // Initialize I2C as Slave
    Wire.begin(SLAVE_ADDR);
    Wire.onReceive(receiveEvent);

    // Print startup info
    Serial.println("========================================");
    Serial.println("Combined Bluetooth + PID Motor Control");
    Serial.println("========================================");
    Serial.println("I2C Slave Ready. Waiting for 0x01 command...");
    Serial.print("PID Gains - Kp: ");
    Serial.print(jointPID.Kp);
    Serial.print(", Kd: ");
    Serial.print(jointPID.Kd);
    Serial.print(", Deadband: ");
    Serial.println(jointPID.deadband);
    Serial.println();
}

void loop() {
    unsigned long currentTime = millis();

    // Read encoder value
    encBinary = myAS5045.read();
    encRaw = EncDeg(encBinary);
    encDeg = EncCalib(encRange, encOffset, encRaw);

    // Debug output every 1 second
    if (currentTime - lastDebugTime >= 1000) {
        lastDebugTime = currentTime;
        Serial.print("Encoder: ");
        Serial.print(encDeg, 2);
        Serial.print("° | Target: ");
        Serial.print(interpolateEnd, 2);
        Serial.print("° | Interpolated: ");
        Serial.print(setPointInterpolated, 2);
        Serial.print("° | Voltage: ");
        Serial.print(Vc, 2);
        Serial.print("V | Motion: ");
        Serial.println(motorMotionActive ? "ACTIVE" : "IDLE");
    }
}
