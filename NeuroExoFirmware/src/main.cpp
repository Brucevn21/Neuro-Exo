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
volatile uint8_t i2cReceiveBuffer[32];
volatile uint8_t i2cReceiveLength = 0;
volatile bool i2cReceiveReady = false;
uint8_t telemetryBuffers[2][NeuroExoProtocol::MAX_FRAME_SIZE] = {};
volatile uint8_t activeTelemetryBuffer = 0;
volatile uint8_t telemetryFrameLength = 0;
unsigned long lastValidCommandTime = 0;
bool commandTimedOut = true;
bool invalidCommand = false;

motorWiring_t motorWiring;
motorLimit_t motorLimit;
dir_t direction;
const float MOTOR_VCC_V = 3.3f; // must match motor's Vcc constructor arg below
motorDriver motor(1, (char*)"NeuroExo Joint", MOTOR_VCC_V, 8);

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
volatile float interpIncrement = 0.0f;
float Vc = 0.0f;
IntervalTimer motorControlTimer;

volatile bool motorMotionActive = false;
float DerivativeFilterAlpha = 0.2f;
unsigned long lastDebugTime = 0;
const unsigned long DEBUG_INTERVAL_MS = 500;

// Speed setting controls both trajectory duration (interpCycles) and the
// maximum effort (voltage) the PID/assist output is allowed to command.
int interpCyclesForSpeed(NeuroExoProtocol::Speed speed) {
    switch (speed) {
        case NeuroExoProtocol::Speed::Slow:   return 60000;
        case NeuroExoProtocol::Speed::High:   return 1750;
        case NeuroExoProtocol::Speed::Medium:
        default:                              return 9000;
    }
}

float maxEffortScaleForSpeed(NeuroExoProtocol::Speed speed) {
    switch (speed) {
        case NeuroExoProtocol::Speed::Slow:   return 0.35f;
        case NeuroExoProtocol::Speed::High:   return 1.0f;
        case NeuroExoProtocol::Speed::Medium:
        default:                              return 0.6f;
    }
}

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
void processI2CCommand();
void updateTelemetryFrame();

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

        // Mode dictates physical rotation direction: Assistive always drives
        // forward, Resistive always drives backward (counterclockwise).
        // Neutral keeps the PID's natural error-correcting direction.
        NeuroExoProtocol::Mode mode = lastCommandedMode;
        float directedControl = combinedControl;
        if (mode == NeuroExoProtocol::Mode::Assistive) {
            directedControl = fabsf(combinedControl);
        } else if (mode == NeuroExoProtocol::Mode::Resistive) {
            directedControl = -fabsf(combinedControl);
        }

        const float maxEffort = MOTOR_VCC_V * maxEffortScaleForSpeed(lastCommandedSpeed);
        directedControl = constrain(directedControl, -maxEffort, maxEffort);

        motor.enable();
        Vc = directedControl;
        motor.rotate(Vc, direction);
    }
}

void receiveEvent(int howMany) {
    if (howMany < 1 || howMany > (int)sizeof(i2cReceiveBuffer) || i2cReceiveReady) {
        while (Wire.available()) {
            Wire.read();
        }
        return;
    }

    for (uint8_t i = 0; i < (uint8_t)howMany && Wire.available(); ++i) {
        i2cReceiveBuffer[i] = Wire.read();
    }
    while (Wire.available()) {
        Wire.read();
    }
    i2cReceiveLength = (uint8_t)howMany;
    i2cReceiveReady = true;
}

void requestEvent() {
    const uint8_t bufferIndex = activeTelemetryBuffer;
    Wire.write((const uint8_t *)telemetryBuffers[bufferIndex], telemetryFrameLength);
}

void processI2CCommand() {
    uint8_t localBuffer[32];
    uint8_t localLength = 0;
    noInterrupts();
    if (i2cReceiveReady) {
        localLength = i2cReceiveLength;
        for (uint8_t i = 0; i < localLength; ++i) {
            localBuffer[i] = i2cReceiveBuffer[i];
        }
        i2cReceiveReady = false;
    }
    interrupts();
    if (localLength == 0) {
        return;
    }

    NeuroExoProtocol::FrameParser parser;
    for (uint8_t i = 0; i < localLength; ++i) {
        parser.push(localBuffer[i]);
    }
    NeuroExoProtocol::Frame frame;
    NeuroExoProtocol::ControlPacket packet;
    if (!parser.takeFrame(frame) || !NeuroExoProtocol::decodeControlFrame(frame, packet)) {
        invalidCommand = true;
        return;
    }

    invalidCommand = false;
    lastValidCommandTime = millis();
    commandTimedOut = false;
    lastCommandedMode = packet.mode;
    lastCommandedSpeed = packet.speed;
    motorMotionActive = true;
    interpolateEnd = (float)packet.targetAngleDeg;
    interpCycles = interpCyclesForSpeed(packet.speed);
    interpInitialized = false;
}

void updateTelemetryFrame() {
    NeuroExoProtocol::TelemetryPacket packet;
    packet.currentAngleDeg = (int16_t)lroundf(encDeg);
    packet.currentMilliAmps = (int16_t)lroundf(measuredMotorCurrentA * 1000.0f);
    packet.status = static_cast<uint8_t>(NeuroExoProtocol::TelemetryStatus::None);
    if (motorMotionActive) {
        packet.status |= static_cast<uint8_t>(NeuroExoProtocol::TelemetryStatus::MotionActive);
    }
    if (commandTimedOut) {
        packet.status |= static_cast<uint8_t>(NeuroExoProtocol::TelemetryStatus::CommandTimeout);
    }
    if (invalidCommand) {
        packet.status |= static_cast<uint8_t>(NeuroExoProtocol::TelemetryStatus::InvalidCommand);
    }

    const uint8_t nextBuffer = 1 - activeTelemetryBuffer;
    const uint8_t length = NeuroExoProtocol::encodeTelemetryFrame(packet, telemetryBuffers[nextBuffer]);
    noInterrupts();
    telemetryFrameLength = length;
    activeTelemetryBuffer = nextBuffer;
    interrupts();
}

void setup() {
    Serial.begin(115200);

    pinMode(LED, OUTPUT);
    digitalWrite(LED, HIGH);

    if (!myAS5045.begin()) {
        Serial.println("Error setting up AS5045");
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
    lastValidCommandTime = millis();
    updateTelemetryFrame();

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

    processI2CCommand();
    if (currentTime - lastValidCommandTime > NeuroExoProtocol::COMMAND_TIMEOUT_MS) {
        commandTimedOut = true;
        motorMotionActive = false;
    }

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
    updateTelemetryFrame();

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

    if (currentTime - lastDebugTime >= DEBUG_INTERVAL_MS) {
        lastDebugTime = currentTime;
        Serial.print("[Teensy Telemetry] Target: ");
        Serial.print(interpolateEnd, 2);
        Serial.print(" deg | Current: ");
        Serial.print(encDeg, 2);
        Serial.print(" deg | Current Draw: ");
        Serial.print(measuredMotorCurrentA * 1000.0f, 2);
        Serial.println(" mA");
    }
}
