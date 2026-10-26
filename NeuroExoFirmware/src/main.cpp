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
volatile uint8_t lastCommandedSetting = 0;      // raw setting byte, for logging
volatile float effortLevel = 0.5f;              // Assistive/Resistive level, 0.1..0.9
volatile float patientEffortNm = 0.0f;          // latest effort estimate (effort modes)
volatile float trialDir = 1.0f;                 // +1 / -1 travel direction of the trial
volatile uint8_t i2cReceiveBuffer[32];
volatile uint8_t i2cReceiveLength = 0;
volatile bool i2cReceiveReady = false;
uint8_t telemetryBuffers[2][NeuroExoProtocol::MAX_FRAME_SIZE] = {};
volatile uint8_t activeTelemetryBuffer = 0;
volatile uint8_t telemetryFrameLength = 0;
bool invalidCommand = false;

// ---- Trial state ----
// A single control packet starts a trial; it ends when the joint reaches the
// target, a stop packet arrives, a safety fault trips, an Assistive patient
// resists, or the time limit expires (move time at the slowest speed the mode
// can run + margin). A Calibrate trial ends when its sweep is done.
// motorMotionActive is "trial active".
const float TRIAL_TARGET_TOLERANCE_DEG = 2.0f;
const unsigned long TRIAL_TIMEOUT_MARGIN_MS = 5000;
const float CALIBRATION_MIN_RANGE_DEG = 10.0f;
const unsigned long CALIBRATION_PASS_MARGIN_MS = 3000;  // accelerating / settling per pass
unsigned long trialStartMs = 0;
unsigned long trialTimeLimitMs = 0;
volatile bool trialTargetReached = false;
bool trialTimedOut = false;
volatile bool trialPatientResisting = false;   // last Assistive trial aborted on resistance
// One-shot events from the control ISR, reported by loop().
volatile bool calibrationFinishedEvent = false;
volatile bool resistanceAbortEvent = false;

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
unsigned long lastDebugTime = 0;
const unsigned long DEBUG_INTERVAL_MS = 2000;

// Neutral speed setting controls both trajectory duration (interpCycles) and
// the fraction of the controller's max current that may be commanded.
int interpCyclesForSpeed(NeuroExoProtocol::Speed speed) {
    switch (speed) {
        case NeuroExoProtocol::Speed::Slow:   return 10000;  // 20 s at 2 ms per cycle
        case NeuroExoProtocol::Speed::High:   return 3000;   // 6 s
        case NeuroExoProtocol::Speed::Medium: return 6000;   // 12 s
        default:                              return 6000;
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
// Differentiate at a fixed rate (not every loop pass) so one encoder count
// (~0.088 deg) doesn't become a huge velocity spike, then low-pass with an
// EMA. At 2 ms: alpha 0.2 ~ 20 Hz cutoff, alpha 0.1 ~ 9 Hz cutoff.
const unsigned long KINEMATIC_SAMPLE_US = 2000;
const float VELOCITY_FILTER_ALPHA = 0.2f;
const float ACCEL_FILTER_ALPHA = 0.1f;
unsigned long lastStreamTime = 0;
const unsigned long STREAM_INTERVAL_MS = 20; // 50 Hz feed for the live visualizer
const bool STREAM_JOINT_CSV = true;           // true to feed the live visualizer

// ESCON analog out 1: 0 V = -7 A, 1.65 V = 0 A, 3.3 V = +7 A.
const float CURRENT_SENSOR_ZERO_V = 1.65f;
const float CURRENT_SENSOR_A_PER_V = 7.0f / 1.65f;

// ---- Controller tuning (see ControlAlgorithm.cpp) ----
// Conservative starting values from the control-algorithm PDF; tune on the
// bench with no one attached first.
const float CONTROL_DT_S = 0.002f;            // motorControlTimer period
const float RAD_PER_DEG = 0.01745329f;
// ESCON Studio current set-value at 90 % PWM duty. MUST be set to match the
// ESCON configuration; while 0 the controller stays off and the motor is never enabled.
const float ESCON_FULL_SCALE_CURRENT_A = 3.29f;
const float NEUTRAL_ASSISTANCE = 1.0f;        // Neutral mode: robot does all the work (passive motion)
const float NEUTRAL_MAX_TRACKING_ERROR_DEG = 10.0f;  // Neutral only

NeuroExoControl::ControlConfig makeControlConfig() {
    NeuroExoControl::ControlConfig cfg;
    cfg.esconFullScaleCurrentA = ESCON_FULL_SCALE_CURRENT_A;
    cfg.kpNmPerRad = 10.0f;
    cfg.kdNmSPerRad = 1.0f;
    cfg.massKg = 0.0f;                        // Set limb + brace mass to enable gravity compensation
    cfg.centerOfMassM = 0.0f;
    cfg.softVelocityLimitRadPerSec = 90.0f * RAD_PER_DEG;
    cfg.overspeedDampingNmSPerRad = 3.0f;
    cfg.hardVelocityLimitRadPerSec = 300.0f * RAD_PER_DEG;
    cfg.maxTorqueNm = 10.0f;
    cfg.maxCurrentA = 2.0f;                   // ~11 N*m at the joint
    cfg.overcurrentTripA = 3.0f;
    cfg.overcurrentTicks = 25;                // 50 ms at 2 ms per tick

    // Patient-driven modes (Assistive / Resistive / Calibrate). Starting values;
    // tune on the bench with no one attached first.
    cfg.trackKpNmPerRad = 60.0f;              // 1 deg behind the reference -> ~1 N*m
    cfg.trackKiNmPerRadS = 150.0f;
    cfg.trackKdNmSPerRad = 2.0f;
    cfg.trackLeashRad = 3.0f * RAD_PER_DEG;
    cfg.refAccelRadPerSec2 = 60.0f * RAD_PER_DEG;
    cfg.approachGainPerSec = 2.0f;            // slows to 20 deg/s at 10 deg from the target
    cfg.effortFilterHz = 3.0f;
    cfg.effortDeadbandNm = 0.5f;
    cfg.creepMinRadPerSec = 1.0f * RAD_PER_DEG;   // no effort at 10 % assistance
    cfg.creepMaxRadPerSec = 5.0f * RAD_PER_DEG;   // no effort at 90 % assistance
    cfg.virtualDampingNmSPerRad = 28.6f;      // 5 N*m of help at 50 % -> +20 deg/s
    cfg.maxAssistSpeedRadPerSec = 45.0f * RAD_PER_DEG;
    cfg.resistAbortNm = 2.0f;
    cfg.resistAbortSec = 0.3f;
    cfg.resistSpeedRadPerSec = 10.0f * RAD_PER_DEG;
    cfg.resistMaxTorqueNm = 10.0f;            // level 9 -> up to 9 N*m against the patient
    cfg.calSlowRadPerSec = 5.0f * RAD_PER_DEG;
    cfg.calFastRadPerSec = 20.0f * RAD_PER_DEG;
    return cfg;
}

NeuroExoControl::AssistMode effortModeFor(NeuroExoProtocol::Mode mode) {
    switch (mode) {
        case NeuroExoProtocol::Mode::Assistive: return NeuroExoControl::AssistMode::PatientAssist;
        case NeuroExoProtocol::Mode::Resistive: return NeuroExoControl::AssistMode::PatientResist;
        case NeuroExoProtocol::Mode::Calibrate: return NeuroExoControl::AssistMode::Calibrate;
        default:                                return NeuroExoControl::AssistMode::Track;
    }
}

// Set when the controller or the Neutral tracking check faults. Control
// packets are ignored until a stop packet clears it.
volatile bool safetyEstopLatched = false;
volatile NeuroExoControl::EstopReason lastEstopReason = NeuroExoControl::EstopReason::None;
volatile float commandedCurrentA = 0.0f;

void receiveEvent(int howMany);
void requestEvent();
void motorControlISR();
void processI2CCommand();
void updateTelemetryFrame();

void stopMotor() {
    Vc = 0.0f;
    commandedCurrentA = 0.0f;
    motor.disable();
}

void motorControlISR() {
    const NeuroExoProtocol::Mode mode = lastCommandedMode;
    const bool timedMove = mode == NeuroExoProtocol::Mode::Neutral;

    if (timedMove && motorMotionActive) {
        if (!interpInitialized) {
            interpolateBegin = encDeg;
            interpCounter = 0;
            setPointInterpolated = interpolateBegin;
            interpIncrement = (interpolateEnd - interpolateBegin) / (float)interpCycles;
            interpInitialized = true;
        }
        if (interpCounter < interpCycles) {
            setPointInterpolated += interpIncrement;
            interpCounter++;
        } else {
            // Trajectory finished: hold the setpoint on the target until the joint arrives.
            setPointInterpolated = interpolateEnd;
        }
    }

    // Neutral must settle within tolerance; patient-driven modes can lurch past
    // the target, so reaching or passing it counts.
    const bool atTarget = timedMove ? fabsf(encDeg - interpolateEnd) <= TRIAL_TARGET_TOLERANCE_DEG
                                    : trialDir * (encDeg - interpolateEnd) >= -TRIAL_TARGET_TOLERANCE_DEG;
    if (motorMotionActive && mode != NeuroExoProtocol::Mode::Calibrate && atTarget) {
        trialTargetReached = true;
        motorMotionActive = false;
        interpInitialized = false;
    }

    if (!motorMotionActive) {
        stopMotor();
        patientEffortNm = 0.0f;
        ControlAlgorithm_Reset();
        return;
    }

    NeuroExoControl::ControlInput in;
    in.angleRad = encDeg * RAD_PER_DEG;
    in.setpointRad = setPointInterpolated * RAD_PER_DEG;
    in.targetRad = interpolateEnd * RAD_PER_DEG;
    in.velocityRadPerSec = measuredVelocityDegPerSec * RAD_PER_DEG;
    in.measuredCurrentA = measuredMotorCurrentA;
    in.dtSec = CONTROL_DT_S;
    in.mode = effortModeFor(mode);
    in.atForwardLimit = encDeg >= motorLimit.forwardLimit;
    in.atBackwardLimit = encDeg <= motorLimit.backwardLimit;

    NeuroExoControl::ControlOutput out;
    float currentA;
    bool trackingFault = false;
    if (timedMove) {
        trackingFault = fabsf(setPointInterpolated - encDeg) > NEUTRAL_MAX_TRACKING_ERROR_DEG;
        in.assistance = NEUTRAL_ASSISTANCE;
        in.currentLimitScale = maxEffortScaleForSpeed(lastCommandedSpeed);
        currentA = ControlAlgorithm_Update(in, out);
    } else {
        in.assistance = effortLevel;
        in.currentLimitScale = 1.0f;
        currentA = ControlAlgorithm_UpdateEffort(in, out);
        setPointInterpolated = out.referenceRad / RAD_PER_DEG;
        patientEffortNm = out.patientEffortNm;
        if (out.calibrationDone || out.patientResisting) {
            calibrationFinishedEvent = out.calibrationDone;
            trialPatientResisting = out.patientResisting;
            resistanceAbortEvent = out.patientResisting;
            stopMotor();
            motorMotionActive = false;
            return;
        }
    }
    const NeuroExoControl::SafetyState state = ControlAlgorithm_GetSafetyState();

    if (trackingFault || state != NeuroExoControl::SafetyState::ACTIVE) {
        if (trackingFault || state == NeuroExoControl::SafetyState::ESTOP_FAULT) {
            safetyEstopLatched = true;
            lastEstopReason = ControlAlgorithm_GetEstopReason();
        }
        stopMotor();
        motorMotionActive = false;
        interpInitialized = false;
        return;
    }

    // ESCON in current mode: |current| maps onto the 10-90 % PWM window via
    // the driver's 0..MOTOR_VCC_V input range; sign selects the DIR pin.
    commandedCurrentA = currentA;
    Vc = (currentA / ESCON_FULL_SCALE_CURRENT_A) * MOTOR_VCC_V;
    motor.enable();
    motor.rotate(Vc, direction);
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
    if (!parser.takeFrame(frame)) {
        invalidCommand = true;
        Serial.print("[Teensy] Invalid command frame (");
        Serial.print(localLength);
        Serial.println(" bytes)");
        return;
    }

    if (NeuroExoProtocol::isStopFrame(frame)) {
        invalidCommand = false;
        // Stop ends any trial and acknowledges a latched E-stop.
        noInterrupts();
        motorMotionActive = false;
        interpInitialized = false;
        safetyEstopLatched = false;
        interrupts();
        Serial.println("[Teensy] Stop received - trial ended");
        return;
    }

    if (!NeuroExoProtocol::decodeControlFrame(frame, packet)) {
        invalidCommand = true;
        Serial.print("[Teensy] Invalid command frame (");
        Serial.print(localLength);
        Serial.println(" bytes)");
        return;
    }

    Serial.print("[Teensy] Command received - Target: ");
    Serial.print(packet.targetAngleDeg);
    Serial.print(" deg | Mode: ");
    Serial.print((int)packet.mode);
    Serial.print(" | Setting: ");
    Serial.println((int)packet.setting);

    invalidCommand = false;
    if (safetyEstopLatched) {
        Serial.println("[Teensy] Ignored: safety E-stop latched (send stop to clear)");
        return;
    }
    if (motorMotionActive) {
        Serial.println("[Teensy] Ignored: trial in progress (send stop first)");
        return;
    }

    const float startDeg = encDeg;
    const float targetDeg = (float)packet.targetAngleDeg;
    const float distanceRad = fabsf(targetDeg - startDeg) * RAD_PER_DEG;
    const NeuroExoControl::AssistMode effortMode = effortModeFor(packet.mode);
    const float level = packet.setting / 10.0f;
    int cycles = interpCycles;
    unsigned long moveMs = 0;

    if (packet.mode == NeuroExoProtocol::Mode::Neutral) {
        cycles = interpCyclesForSpeed(static_cast<NeuroExoProtocol::Speed>(packet.setting));
        moveMs = (unsigned long)(cycles * CONTROL_DT_S * 1000.0f);
    } else if (packet.mode == NeuroExoProtocol::Mode::Calibrate) {
        if (distanceRad < CALIBRATION_MIN_RANGE_DEG * RAD_PER_DEG) {
            Serial.println("[Teensy] Ignored: calibration range under 10 deg");
            return;
        }
        if (targetDeg > motorLimit.forwardLimit || targetDeg < motorLimit.backwardLimit) {
            Serial.println("[Teensy] Ignored: calibration target outside the joint limits");
            return;
        }
        const NeuroExoControl::ControlConfig &cfg = ControlAlgorithm_GetConfig();
        const float sweepSec = 2.0f * distanceRad / cfg.calSlowRadPerSec + 2.0f * distanceRad / cfg.calFastRadPerSec;
        moveMs = (unsigned long)(sweepSec * 1000.0f) + 4 * CALIBRATION_PASS_MARGIN_MS;
    } else {
        if (!ControlAlgorithm_BaselineCovers(startDeg * RAD_PER_DEG, targetDeg * RAD_PER_DEG)) {
            Serial.println("[Teensy] Ignored: no calibration covering this range (run Calibrate first)");
            return;
        }
        moveMs = (unsigned long)(1000.0f * distanceRad / ControlAlgorithm_MinTrialSpeedRadPerSec(effortMode, level));
    }

    noInterrupts();
    lastCommandedMode = packet.mode;
    lastCommandedSetting = packet.setting;
    if (packet.mode == NeuroExoProtocol::Mode::Neutral) {
        lastCommandedSpeed = static_cast<NeuroExoProtocol::Speed>(packet.setting);
    }
    effortLevel = level;
    trialDir = targetDeg >= startDeg ? 1.0f : -1.0f;
    interpolateEnd = targetDeg;
    interpCycles = cycles;
    interpInitialized = false;
    trialTargetReached = false;
    trialTimedOut = false;
    trialPatientResisting = false;
    patientEffortNm = 0.0f;
    if (packet.mode != NeuroExoProtocol::Mode::Neutral) {
        ControlAlgorithm_BeginEffortTrial(effortMode, startDeg * RAD_PER_DEG, targetDeg * RAD_PER_DEG, level);
        setPointInterpolated = startDeg;
    }
    trialStartMs = millis();
    trialTimeLimitMs = moveMs + TRIAL_TIMEOUT_MARGIN_MS;
    motorMotionActive = true;
    interrupts();
    Serial.println(packet.mode == NeuroExoProtocol::Mode::Calibrate
                       ? "[Teensy] Calibration started - keep the arm relaxed"
                       : "[Teensy] Trial started");
}

void updateTelemetryFrame() {
    NeuroExoProtocol::TelemetryPacket packet;
    packet.currentAngleDeg = (int16_t)lroundf(encDeg);
    packet.currentMilliAmps = (int16_t)lroundf(measuredMotorCurrentA * 1000.0f);
    const float effortCentiNm = patientEffortNm * 100.0f;
    packet.patientEffortCentiNm = (int16_t)lroundf(fmaxf(-32767.0f, fminf(32767.0f, effortCentiNm)));
    packet.status = static_cast<uint8_t>(NeuroExoProtocol::TelemetryStatus::None);
    if (motorMotionActive) {
        packet.status |= static_cast<uint8_t>(NeuroExoProtocol::TelemetryStatus::MotionActive);
    }
    if (trialTargetReached) {
        packet.status |= static_cast<uint8_t>(NeuroExoProtocol::TelemetryStatus::TargetReached);
    }
    if (trialTimedOut) {
        packet.status |= static_cast<uint8_t>(NeuroExoProtocol::TelemetryStatus::TrialTimeout);
    }
    if (ControlAlgorithm_BaselineValid()) {
        packet.status |= static_cast<uint8_t>(NeuroExoProtocol::TelemetryStatus::Calibrated);
    }
    if (trialPatientResisting) {
        packet.status |= static_cast<uint8_t>(NeuroExoProtocol::TelemetryStatus::PatientResisting);
    }
    if (safetyEstopLatched) {
        packet.status |= static_cast<uint8_t>(NeuroExoProtocol::TelemetryStatus::SafetyFault);
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

    ControlAlgorithm_SetConfig(makeControlConfig());

    lastKinematicMicros = micros();
    lastEncDegForDeriv = encDeg;
    lastVelDegPerSec = 0.0f;

    motorControlTimer.begin(motorControlISR, 2000);
    motorControlTimer.priority(128);

    Wire.begin(SLAVE_ADDR);
    Wire.onReceive(receiveEvent);
    Wire.onRequest(requestEvent);
    updateTelemetryFrame();

    Serial.println("========================================");
    Serial.println("NeuroExoFirmware - Main Controller");
    Serial.println("========================================");
    Serial.println("I2C Slave Ready. Waiting for joint packets...");
    if (!ControlAlgorithm_IsConfigured()) {
        Serial.println("WARNING: controller config incomplete (set ESCON_FULL_SCALE_CURRENT_A). Motor will stay disabled.");
    }
    const NeuroExoControl::ControlConfig &cfg = ControlAlgorithm_GetConfig();
    Serial.print("Controller - Kp: ");
    Serial.print(cfg.kpNmPerRad);
    Serial.print(" N*m/rad, Kd: ");
    Serial.print(cfg.kdNmSPerRad);
    Serial.print(" N*m*s/rad, max current: ");
    Serial.print(cfg.maxCurrentA);
    Serial.println(" A");
    Serial.println();
}

void loop() {
    unsigned long currentTime = millis();

    processI2CCommand();
    noInterrupts();
    const bool timeLimitHit = motorMotionActive && millis() - trialStartMs > trialTimeLimitMs;
    if (timeLimitHit) {
        motorMotionActive = false;
        trialTimedOut = true;
    }
    interrupts();
    if (timeLimitHit) {
        Serial.println("[Teensy] Trial aborted - time limit exceeded");
    }
    if (calibrationFinishedEvent) {
        calibrationFinishedEvent = false;
        Serial.println(ControlAlgorithm_BaselineValid()
                           ? "[Teensy] Calibration complete"
                           : "[Teensy] Calibration failed - not enough steady motion, run it again");
        ControlAlgorithm_PrintBaselineSummary();
    }
    if (resistanceAbortEvent) {
        resistanceAbortEvent = false;
        Serial.println("[Teensy] Trial aborted - patient pushing against the motion");
    }

    encBinary = myAS5045.read();
    encRaw = EncDeg(encBinary);
    encDeg = EncCalib(encRange, encOffset, encRaw);

    unsigned long nowMicros = micros();
    if (nowMicros - lastKinematicMicros >= KINEMATIC_SAMPLE_US) {
        float dtSec = (nowMicros - lastKinematicMicros) * 1.0e-6f;
        float rawVel = (encDeg - lastEncDegForDeriv) / dtSec;
        float vel = VELOCITY_FILTER_ALPHA * rawVel + (1.0f - VELOCITY_FILTER_ALPHA) * lastVelDegPerSec;
        float rawAcc = (vel - lastVelDegPerSec) / dtSec;
        float acc = ACCEL_FILTER_ALPHA * rawAcc + (1.0f - ACCEL_FILTER_ALPHA) * measuredAccelDegPerSec2;

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
    if (STREAM_JOINT_CSV && currentTime - lastStreamTime >= STREAM_INTERVAL_MS) {
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
        Serial.print(motorMotionActive ? 1 : 0);
        Serial.print(',');
        Serial.print(measuredAccelDegPerSec2, 2);
        Serial.print(',');
        Serial.print(measuredMotorCurrentA, 4);
        Serial.print(',');
        Serial.print(currentTime);
        Serial.print(',');
        Serial.print(patientEffortNm, 2);
        Serial.print(',');
        Serial.print((int)lastCommandedMode);
        Serial.print(',');
        Serial.println((int)lastCommandedSetting);
    }

    if (currentTime - lastDebugTime >= DEBUG_INTERVAL_MS) {
        lastDebugTime = currentTime;
        Serial.print("[Teensy Telemetry] Target: ");
        Serial.print(interpolateEnd, 2);
        Serial.print(" deg | Current: ");
        Serial.print(encDeg, 2);
        Serial.print(" deg | Current Draw: ");
        Serial.print(measuredMotorCurrentA * 1000.0f, 2);
        Serial.print(" mA | Cmd: ");
        Serial.print(commandedCurrentA * 1000.0f, 2);
        Serial.print(" mA | Effort: ");
        Serial.print(patientEffortNm, 2);
        Serial.print(" N*m");
        if (safetyEstopLatched) {
            Serial.print(" | E-STOP reason ");
            Serial.print((int)lastEstopReason);
            Serial.print(" (0=tracking, 1=invalid input, 2=overspeed, 3=overcurrent)");
        }
        Serial.println();
    }
}
