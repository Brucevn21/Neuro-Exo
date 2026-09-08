/*
 * NeuroExoFirmware - Combined I2C + PID Motor Controller
 *
 * This firmware is now the primary project firmware for the Neuro-Exo system.
 * It combines the working behavior from the DemoFolder:
 * - I2C slave communication from the Nano 33 BLE master
 * - Trajectory interpolation and PID motor control
 * - Encoder-based position feedback and safety checks
 */

#include <Wire.h>
#include <IntervalTimer.h>
#include "motorDriver.h"
#include "pinMap4.1.h"
#include "AS5045.h"
#include "encoder_utils.h"
#include "luu_utils.h"
#include "ControlAlgorithm.h"
#include "commProtocol.h"

const int SLAVE_ADDR = 0x08;
volatile bool triggerMotor = false;

volatile NeuroExoProtocol::Mode lastCommandedMode = NeuroExoProtocol::Mode::Neutral;
volatile NeuroExoProtocol::Speed lastCommandedSpeed = NeuroExoProtocol::Speed::Medium;

motorWiring_t motorWiring;
motorLimit_t motorLimit;
dir_t direction;
motorDriver motor(1, (char*)"NeuroExo Joint", 3.3f, 8);

AS5045 myAS5045(AS5045_CS_PIN, AS5045_CLK_PIN, AS5045_DATA_PIN, 0xFF, 3);
unsigned int encBinary;
float encRaw;
volatile float encDeg = 0.0f;
float encRange[2] = {180, -180};
float encOffset = 65.21f;
int nbits = 16;

PID_t jointPID;
volatile float interpolateBegin = 0.0f;
volatile float interpolateEnd = 45.0f;
volatile float setPointInterpolated = 0.0f;
volatile int interpCounter = 0;
volatile bool interpInitialized = false;
volatile int interpCycles = 3500;
// Trial duration (motorControlISR runs every 2ms) selected by the commanded Speed.
const int INTERP_CYCLES_SLOW = 5250;   // ~10.5s move
const int INTERP_CYCLES_MEDIUM = 3500; // ~7.0s move
const int INTERP_CYCLES_FAST = 1750;   // ~3.5s move
volatile float interpIncrement = 0.0f;
float Vc = 0.0f;
IntervalTimer motorControlTimer;

volatile bool motorMotionActive = false;
float DerivativeFilterAlpha = 0.2f;
unsigned long lastDebugTime = 0;

volatile float measuredVelocityDegPerSec = 0.0f;
volatile float measuredAccelDegPerSec2 = 0.0f;
volatile float measuredMotorCurrentA = 0.0f;

float lastEncDegForDeriv = 0.0f;
float lastVelDegPerSec = 0.0f;
unsigned long lastKinematicMicros = 0;
unsigned long lastStreamTime = 0;
const unsigned long STREAM_INTERVAL_MS = 20; // 50 Hz feed for the live visualizer

const float CURRENT_SENSOR_ZERO_V = 0.0f;
const float CURRENT_SENSOR_A_PER_V = 1.0f;

void receiveEvent(int howMany);
void requestEvent();
void motorControlISR();

void motorControlISR() {
    if (!interpInitialized && motorMotionActive) {
        interpolateBegin = encDeg;
        interpCounter = 0;
        setPointInterpolated = interpolateBegin;
        interpIncrement = (interpolateEnd - interpolateBegin) / (float)interpCycles;
        interpInitialized = true;
    }

    if (interpCounter < interpCycles && motorMotionActive) {
        setPointInterpolated += interpIncrement;
        interpCounter++;
    } else if (motorMotionActive) {
        setPointInterpolated = interpolateEnd;
        motorMotionActive = false;
        interpInitialized = false;
    }

    bool safetyStop = false;
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
        ControlAlgorithm_Reset();
    } else {
        float motorControl = motor.computePID(setPointInterpolated, encDeg, jointPID);

        bool forwardDirection = (motorControl >= 0.0f);
        float assistControl = ControlAlgorithm_UpdateSignedAssist(
            measuredVelocityDegPerSec,
            measuredAccelDegPerSec2,
            measuredMotorCurrentA,
            jointPID.dt,
            forwardDirection
        );

        float combinedControl = motorControl + assistControl;
        motor.enable();
        Vc = combinedControl;
        motor.rotate(Vc, direction);
    }
}

void receiveEvent(int howMany) {
    if (howMany < 1) {
        return;
    }

    uint8_t command = Wire.read();

    if (command == NeuroExoProtocol::I2C_CMD_JOINT_PACKET &&
        Wire.available() >= (int)NeuroExoProtocol::PACKET_SIZE) {
        uint8_t buf[NeuroExoProtocol::PACKET_SIZE];
        for (uint8_t i = 0; i < NeuroExoProtocol::PACKET_SIZE; ++i) {
            buf[i] = Wire.read();
        }

        NeuroExoProtocol::JointPacket packet;
        NeuroExoProtocol::decodeJointPacket(buf, packet);

        lastCommandedMode = packet.mode;
        lastCommandedSpeed = packet.speed;

        if (!motorMotionActive) {
            switch (packet.speed) {
                case NeuroExoProtocol::Speed::Slow:   interpCycles = INTERP_CYCLES_SLOW; break;
                case NeuroExoProtocol::Speed::Medium: interpCycles = INTERP_CYCLES_MEDIUM; break;
                case NeuroExoProtocol::Speed::High:   interpCycles = INTERP_CYCLES_FAST; break;
            }
            motorMotionActive = true;
            interpolateEnd = (float)packet.targetAngleDeg;
            interpInitialized = false;
            Serial.print("BLE/I2C Joint Packet! Starting PID motion to ");
            Serial.print(packet.targetAngleDeg);
            Serial.println(" degrees...");
        }
    } else if (command == NeuroExoProtocol::I2C_CMD_STOP) {
        motorMotionActive = false;
        interpInitialized = false;
        interpolateEnd = encDeg;
        setPointInterpolated = encDeg;
        Vc = 0.0f;
        motor.disable();
        jointPID.integral = 0.0f;
        jointPID.lastError = 0.0f;
        jointPID.filteredDerivative = 0.0f;
        ControlAlgorithm_Reset();
        Serial.println("STOP command received - halting motion.");
    }

    // Discard any unread bytes so a malformed packet doesn't desync the bus.
    while (Wire.available()) {
        Wire.read();
    }
}

// Reports current joint state back to the Nano 33 BLE for its telemetry ping-pong buffer.
void requestEvent() {
    NeuroExoProtocol::JointPacket packet;
    packet.mode = lastCommandedMode;
    packet.speed = lastCommandedSpeed;
    packet.currentAngleDeg = (int16_t)lroundf(encDeg);
    packet.targetAngleDeg = (int16_t)lroundf(interpolateEnd);
    packet.currentMilliAmps = (int16_t)lroundf(measuredMotorCurrentA * 1000.0f);

    uint8_t buf[NeuroExoProtocol::PACKET_SIZE];
    NeuroExoProtocol::encodeJointPacket(packet, buf);
    Wire.write(buf, NeuroExoProtocol::PACKET_SIZE);
}

void setup() {
    Serial.begin(115200);
    delay(500);

    pinMode(LED, OUTPUT);
    digitalWrite(LED, HIGH);

    if (!myAS5045.begin()) {
        Serial.println("Error setting up AS5045");
        delay(1000);
    }

    motorWiring.enablePin = ENABLE_PIN;
    motorWiring.dirPin = DIR_PIN;
    motorWiring.pwmPin = PWM_PIN;
    motorWiring.BWSwitchPin = 0;
    motorWiring.FWSwitchPin = 0;
    digitalWrite(motorWiring.FWSwitchPin, LOW);
    digitalWrite(motorWiring.BWSwitchPin, LOW);

    motorLimit.forwardLimit = 80.0f;
    motorLimit.backwardLimit = -150.0f;

    direction.FORWARD = LOW;
    direction.BACKWARD = HIGH;

    motor.init(motorWiring, motorLimit);

    jointPID = {0.2f, 0.0f, 0.0002f, 0.002f, 0.0f, 0.0f, 0.0f, 0.3f, DerivativeFilterAlpha};

    lastKinematicMicros = micros();
    lastEncDegForDeriv = encDeg;
    lastVelDegPerSec = 0.0f;

    motorControlTimer.begin(motorControlISR, 2000);
    motorControlTimer.priority(128);

    Wire.begin(SLAVE_ADDR);
    Wire.onReceive(receiveEvent);
    Wire.onRequest(requestEvent);

    Serial.println("========================================");
    Serial.println("NeuroExoFirmware - Main Controller");
    Serial.println("========================================");
    Serial.println("I2C Slave Ready. Waiting for joint packets...");
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

    encBinary = myAS5045.read();
    encRaw = EncDeg(encBinary);
    encDeg = EncCalib(encRange, encOffset, encRaw);

    unsigned long nowMicros = micros();
    float dtSec = (nowMicros - lastKinematicMicros) * 1.0e-6f;
    if (dtSec > 0.0f) {
        float vel = (encDeg - lastEncDegForDeriv) / dtSec;
        float acc = (vel - lastVelDegPerSec) / dtSec;

        measuredVelocityDegPerSec = vel;
        measuredAccelDegPerSec2 = acc;

        lastVelDegPerSec = vel;
        lastEncDegForDeriv = encDeg;
        lastKinematicMicros = nowMicros;
    }

    int adcCurrent = analogRead(ESCON_AN1);
    float currentVoltage = (3.3f * (float)adcCurrent) / 1023.0f;
    measuredMotorCurrentA = (currentVoltage - CURRENT_SENSOR_ZERO_V) * CURRENT_SENSOR_A_PER_V;

    // Compact CSV line consumed by the Python live joint visualizer.
    if (currentTime - lastStreamTime >= STREAM_INTERVAL_MS) {
        lastStreamTime = currentTime;
        Serial.print("JOINT,");
        Serial.print(encDeg, 2);
        Serial.print(',');
        Serial.print(setPointInterpolated, 2);
        Serial.print(',');
        Serial.print(interpolateEnd, 2);
        Serial.print(',');
        Serial.print(Vc, 2);
        Serial.print(',');
        Serial.print(measuredVelocityDegPerSec, 2);
        Serial.print(',');
        Serial.println(motorMotionActive ? 1 : 0);
    }

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
        Serial.print("V | Vel: ");
        Serial.print(measuredVelocityDegPerSec, 2);
        Serial.print(" deg/s | Acc: ");
        Serial.print(measuredAccelDegPerSec2, 2);
        Serial.print(" deg/s^2 | I: ");
        Serial.print(measuredMotorCurrentA, 2);
        Serial.print(" A | Motion: ");
        Serial.println(motorMotionActive ? "ACTIVE" : "IDLE");
    }
}
