/*
 * bluetoothDemo.c — Neuro-Exo BLE→SPI Bluetooth Demo
 *
 * Receives framed SPI packets from the Arduino Nano 33 BLE bridge and drives
 * the elbow joint motor to a commanded position using closed-loop PID control.
 * This demo extends bluetoothTest.c with full frame parsing, XOR checksum
 * validation, and position feedback via the AS5045 encoder.
 *
 * ─── SPI Frame (from Nano bridge) ────────────────────────────────────────────
 *   [0xAA | CMD | LEN | payload[0]…payload[LEN-1] | XOR_CHECKSUM]
 *
 *   XOR_CHECKSUM covers all bytes from 0xAA through the last payload byte.
 *
 * ─── Supported BLE Commands ──────────────────────────────────────────────────
 *   0x01  MOVE_TO_POS   2-byte payload: int16_t target × 10 (degrees), MSB first
 *                       Example: 45.0° → 450 → [0x01, 0xC2]
 *                                -30.0° → -300 → [0xFE, 0xD4]
 *   0x02  STOP          No payload — stop motor and disable PID
 *
 * ─── SPI Wiring (Nano 33 BLE → Teensy) ──────────────────────────────────────
 *   Nano D11 (MOSI) → Teensy pin 11
 *   Nano D12 (MISO) → Teensy pin 12
 *   Nano D13 (SCK)  → Teensy pin 13  ⚠ shares LED pin — LED inactive during SPI
 *   Nano D10 (CS)   → Teensy pin 10  (BLE_SPI_CS_PIN, separate from encoder CS)
 *
 * ─── SPI Slave Implementation ────────────────────────────────────────────────
 *   Teensy 4.1 (IMXRT1062): Uses SPISlave_T4 library (Teensyduino built-in).
 *   Teensy 3.2 (MK20DX256):  Uses AVR hardware SPI registers (SPCR / SPDR).
 *   Both variants call the same processByte() assembler function.
 *
 * ─── Control Loop ────────────────────────────────────────────────────────────
 *   IntervalTimer fires motorControlISR() every 2 ms (500 Hz), matching main.cpp.
 *   PD gains and deadband are taken directly from main.cpp tuning values.
 *   Encoder is read in loop() to keep the ISR short.
 *
 * Hardware:
 *   MCU     : Teensy 4.1 / 3.2
 *   Motor   : Maxon DC with ESCON controller (±3.3 V PWM command)
 *   Encoder : AS5045 absolute magnetic encoder (bit-banged SPI, no conflict)
 *
 * Author : [your name]
 * Date   : [date]
 */

// ─── Board-conditional pin map and SPI slave include ─────────────────────────
#if defined(__MK20DX256__)
    #include "pinMap.h"
    #define BOARD_NAME "Teensy 3.2"

#elif defined(__IMXRT1062__)
    #include "pinMap4.1.h"
    #define BOARD_NAME "Teensy 4.1"
    #include <SPISlave_T4.h>    // Teensyduino built-in library for T4.x SPI slave

#else
    #error "Unsupported board. Compile for Teensy 3.2 or Teensy 4.1."
#endif

#include <SPI.h>
#include <IntervalTimer.h>
#include <stdint.h>
#include <stdbool.h>

#include "AS5045.h"
#include "motorDriver.h"
#include "encoder_utils.h"

// ─── SPI frame constants ──────────────────────────────────────────────────────
#define FRAME_START      0xAA
#define MAX_FRAME_LEN    24        // maximum total bytes per frame (START+CMD+LEN+18+CHK)

// Command IDs
#define CMD_MOVE_POS     0x01      // move to absolute angle position
#define CMD_STOP         0x02      // stop motor and disable PID

// SPI slave chip-select driven by the Nano bridge (Nano D10 → Teensy pin 10).
// On Teensy 4.1 this is the SPI0 hardware CS0 pin, managed by SPISlave_T4.
// On Teensy 3.2 this is the SPI0 hardware CS0 pin, managed by SPCR/SPDR.
// Keep distinct from encoder CS (AS5045_CS_PIN = pin 2 on Teensy 4.1).
#define BLE_SPI_CS_PIN   10

// ─── Motor objects ────────────────────────────────────────────────────────────
static motorWiring_t  motorWiring;
static motorLimit_t   motorLimit;
static dir_t          direction;
static PID_t          jointPID;

// Motor ID 1, label, Vcc = 3.3 V, 8-bit PWM resolution — matches bluetoothTest.c
motorDriver motor(1, (char*)"BLE Demo Motor", 3.3f, 8);

// ─── Encoder ─────────────────────────────────────────────────────────────────
// AS5045 uses bit-banged SPI via its own pins — no conflict with hardware SPI slave.
// Pin assignments come from the board's pinMap (pinMap4.1.h for Teensy 4.1).
static const unsigned int ENC_CLK_DELAY = 3;    // µs delay per clock cycle
AS5045 myEncoder(AS5045_CS_PIN, AS5045_CLK_PIN, AS5045_DATA_PIN, 0xFF, ENC_CLK_DELAY);

// Encoder calibration — adjust encOffset to set the zero-degree reference.
// encRange maps the calibrated output to physical joint range (degrees).
static float   encRange[2] = {-180.0f, 180.0f};
static float   encOffset   = 0.0f;

volatile float encDeg = 0.0f;    // calibrated encoder position, updated in loop()

// ─── PID / motor control state ────────────────────────────────────────────────
volatile float targetDeg  = 0.0f;   // commanded position (degrees)
volatile bool  pidActive  = false;  // true: PID loop drives motor
volatile float Vc         = 0.0f;   // last voltage command (for logging)

IntervalTimer  motorTimer;
#define CONTROL_PERIOD_US  2000     // 2 ms → 500 Hz control loop, matching main.cpp

// ─── SPI frame reception buffers ─────────────────────────────────────────────
// Frame assembler state — written from ISR, consumed in loop().
// Access to frameReady is implicitly protected: ISR only sets it, loop() clears it.
static volatile uint8_t frameBuf[MAX_FRAME_LEN];  // assembled frame bytes
static volatile uint8_t frameBufIdx      = 0;     // next write position
static volatile uint8_t frameExpectedLen = 0;     // total bytes expected for current frame
static volatile bool    frameReady       = false; // true when a validated frame is waiting

// ─── Teensy 4.1 SPI slave object ─────────────────────────────────────────────
#if defined(__IMXRT1062__)
SPISlave_T4<&SPI, SPI_8_BITS> SPISlave;
#endif

// ─── Forward declarations ─────────────────────────────────────────────────────
static void processByte(uint8_t b);
static void dispatchFrame(void);
static void motorControlISR(void);

#if defined(__IMXRT1062__)
static void onSPIReceive(void);
#endif

// ──────────────────────────────────────────────────────────────────────────────
//  processByte() — byte-level state machine for SPI frame assembly
//
//  Called from the SPI receive ISR on every arriving byte.
//  Builds a complete, checksum-validated frame in frameBuf[].
//  Sets frameReady = true when a good frame is available.
// ──────────────────────────────────────────────────────────────────────────────
static void processByte(uint8_t b) {
    if (frameReady) return;  // stall: previous frame not yet consumed by loop()

    if (frameBufIdx == 0) {
        // Byte 0: must be the start marker; ignore everything else
        if (b != FRAME_START) return;
        frameBuf[0] = b;
        frameBufIdx = 1;

    } else if (frameBufIdx == 1) {
        // Byte 1: CMD ID
        frameBuf[1] = b;
        frameBufIdx = 2;

    } else if (frameBufIdx == 2) {
        // Byte 2: payload length N
        // Total frame size = START(1) + CMD(1) + LEN(1) + payload(N) + CHECKSUM(1) = N + 4
        frameBuf[2]      = b;
        frameExpectedLen = (uint8_t)(4u + b);
        if (frameExpectedLen > MAX_FRAME_LEN) {
            // Reject malformed frame with oversized payload
            frameBufIdx = 0;
            return;
        }
        frameBufIdx = 3;

    } else {
        // Bytes 3…(N+2): payload; byte (N+3): XOR checksum
        frameBuf[frameBufIdx++] = b;

        if (frameBufIdx >= frameExpectedLen) {
            // All bytes received — validate XOR checksum
            // Checksum covers frameBuf[0] through frameBuf[frameExpectedLen-2]
            uint8_t chk = 0;
            for (uint8_t i = 0; i < frameExpectedLen - 1u; i++) {
                chk ^= frameBuf[i];
            }

            if (chk == frameBuf[frameExpectedLen - 1]) {
                frameReady = true;
            } else {
                Serial.println("SPI RX: bad checksum — frame discarded");
            }
            frameBufIdx = 0;  // reset for the next incoming frame
        }
    }
}

// ──────────────────────────────────────────────────────────────────────────────
//  Teensy 4.1 — SPISlave_T4 receive callback
//  Called by the library when one or more bytes arrive over SPI.
// ──────────────────────────────────────────────────────────────────────────────
#if defined(__IMXRT1062__)
static void onSPIReceive() {
    while (SPISlave.available()) {
        processByte((uint8_t)SPISlave.read());
    }
}
#endif

// ──────────────────────────────────────────────────────────────────────────────
//  Teensy 3.2 — AVR hardware SPI byte-received interrupt
// ──────────────────────────────────────────────────────────────────────────────
#if defined(__MK20DX256__)
ISR(SPI_STC_vect) {
    processByte(SPDR);
}
#endif

// ──────────────────────────────────────────────────────────────────────────────
//  dispatchFrame() — parse a validated frame and execute the command
//
//  Called from loop() when frameReady == true.
// ──────────────────────────────────────────────────────────────────────────────
static void dispatchFrame() {
    uint8_t cmd = frameBuf[1];
    uint8_t len = frameBuf[2];

    Serial.print("BLE CMD: 0x"); Serial.print(cmd, HEX);
    Serial.print("  len: ");     Serial.println(len);

    switch (cmd) {

        // ── CMD 0x01 : MOVE_TO_POS ────────────────────────────────────────────
        // Payload: 2 bytes, int16_t, MSB first, value = target_degrees × 10
        // Range  : −3276.7° to +3276.7° before clamping to joint limits
        case CMD_MOVE_POS: {
            if (len < 2) {
                Serial.println("  ERROR: MOVE_TO_POS needs 2 payload bytes");
                break;
            }
            int16_t raw    = (int16_t)(((uint16_t)frameBuf[3] << 8) | frameBuf[4]);
            float   newTgt = (float)raw / 10.0f;  // ×10 encoding → degrees

            // Clamp to configured joint limits
            if (newTgt > motor.jointLimit.forwardLimit)  newTgt = motor.jointLimit.forwardLimit;
            if (newTgt < motor.jointLimit.backwardLimit) newTgt = motor.jointLimit.backwardLimit;

            // Apply atomically — prevent ISR from seeing partial update
            noInterrupts();
            targetDeg = newTgt;
            // Reset PID transient state so integrator and derivative do not carry
            // over error history from the previous setpoint
            jointPID.integral           = 0.0f;
            jointPID.lastError          = 0.0f;
            jointPID.filteredDerivative = 0.0f;
            pidActive = true;
            interrupts();

            Serial.print("  → target: "); Serial.print(newTgt, 1); Serial.println(" deg");
            break;
        }

        // ── CMD 0x02 : STOP ───────────────────────────────────────────────────
        case CMD_STOP:
            noInterrupts();
            pidActive = false;
            targetDeg = encDeg;  // hold position so next MOVE_POS starts from here
            interrupts();
            motor.stop();
            motor.disable();
            Serial.println("  → motor stopped");
            break;

        default:
            Serial.print("  WARN: unknown CMD 0x"); Serial.println(cmd, HEX);
            break;
    }
}

// ──────────────────────────────────────────────────────────────────────────────
//  motorControlISR() — 500 Hz PID position control loop
//
//  Runs every 2 ms via IntervalTimer, matching the control rate in main.cpp.
//  Reads encDeg (updated by loop()) and writes the motor voltage command.
//  Kept short: no Serial calls, no encoder reads.
// ──────────────────────────────────────────────────────────────────────────────
static void motorControlISR() {
    if (!pidActive) return;

    // computePID returns voltage in ±Vcc, saturated to ±3.3 V
    float pidOut = motor.computePID(targetDeg, encDeg, jointPID);
    Vc = pidOut;

    // rotate() selects forward/backward and calls enable() if needed
    motor.rotate(Vc, direction);
}

// ──────────────────────────────────────────────────────────────────────────────
//  setup()
// ──────────────────────────────────────────────────────────────────────────────
void setup() {
    Serial.begin(115200);
    while (!Serial && millis() < 3000);

    Serial.print("bluetoothDemo | board: "); Serial.println(BOARD_NAME);

    // ── Motor pin wiring ───────────────────────────────────────────────────────
    // Uses pinMap4.1.h values: ENABLE_PIN=8, DIR_PIN=9, PWM_PIN=7
    motorWiring.enablePin   = ENABLE_PIN;
    motorWiring.dirPin      = DIR_PIN;
    motorWiring.pwmPin      = PWM_PIN;
    motorWiring.FWSwitchPin = 0;   // limit switches not used in this demo
    motorWiring.BWSwitchPin = 0;

    // Joint limits — adjust to match the physical range of your exoskeleton joint
    motorLimit.forwardLimit  =  150.0f;
    motorLimit.backwardLimit = -150.0f;
    motorLimit.maxSpeed      =    3.3f;
    motorLimit.minSpeed      =    0.0f;
    motorLimit.maxAcc        =    0.0f;
    motorLimit.maxTorque     =    0.0f;

    // LOW = FORWARD for right-side elbow (matches main.cpp right-leg convention)
    direction.FORWARD  = LOW;
    direction.BACKWARD = HIGH;

    motor.init(motorWiring, motorLimit);
    motor.getInfo();

    // ── PID parameters ─────────────────────────────────────────────────────────
    // Taken directly from main.cpp runtime tuning for right-leg joints:
    //   Kp=0.2, Ki=0 (disabled), Kd=0.002, dt=0.002 s  (PD control at 500 Hz)
    //   deadband=0.4°  — suppresses ±0.35° encoder quantization noise
    //   alpha=0.2      — EMA coefficient ≈ 5-sample derivative smoothing
    //   { Kp,  Ki,   Kd,    dt,    integral, lastError, filtDeriv, deadband, alpha }
    jointPID = {0.2f, 0.0f, 0.002f, 0.002f, 0.0f,     0.0f,      0.0f,    0.4f,    0.2f};

    // ── Encoder setup ──────────────────────────────────────────────────────────
    // AS5045 uses its own bit-banged SPI (CLK=1, CS=2, DATA=0 on Teensy 4.1),
    // completely independent from the hardware SPI0 slave used for the BLE bridge.
    if (!myEncoder.begin()) {
        Serial.println("ERROR: AS5045 encoder init failed — check wiring and clock delay");
    } else {
        Serial.println("Encoder OK");
    }

    // ── SPI slave setup ────────────────────────────────────────────────────────
    // Teensy 4.1: SPISlave_T4 manages pin 10 (CS0), 11 (MOSI), 12 (MISO), 13 (SCK).
    //             Note: pin 13 doubles as the onboard LED — LED will not work while
    //             SPI slave is active. This is a hardware limitation of Teensy 4.1.
    //
    // Teensy 3.2: SPCR-based hardware SPI slave; same physical pins.
#if defined(__IMXRT1062__)
    SPISlave.begin();
    SPISlave.onReceive(onSPIReceive);
    Serial.println("SPI slave ready (SPISlave_T4)");

#elif defined(__MK20DX256__)
    pinMode(MISO, OUTPUT);
    SPCR |= _BV(SPE);       // enable SPI hardware in slave mode
    SPI.attachInterrupt();   // enable SPI byte-received interrupt
    Serial.println("SPI slave ready (AVR SPCR)");
#endif

    // ── 500 Hz motor control timer ─────────────────────────────────────────────
    motorTimer.begin(motorControlISR, CONTROL_PERIOD_US);
    motorTimer.priority(128);  // same priority as main.cpp

    Serial.println("Ready — send BLE commands from the Nano bridge");
}

// ──────────────────────────────────────────────────────────────────────────────
//  loop()
// ──────────────────────────────────────────────────────────────────────────────
void loop() {
    // ── Read encoder and update calibrated angle ───────────────────────────────
    // Done here (not in ISR) to keep motorControlISR() short.
    // EncCalib() maps the raw AS5045 reading → calibrated degrees in encRange.
    unsigned int encBinary = myEncoder.read();
    float        encRawDeg = EncDeg(encBinary);             // counts → degrees (0–360)
    encDeg = EncCalib(encRange, encOffset, encRawDeg);      // apply offset + range mapping
    motor.encoder = encDeg;

    // ── Process any complete SPI frame ─────────────────────────────────────────
    if (frameReady) {
        dispatchFrame();
        frameReady = false;
    }

    // ── Serial status output every 1 s ────────────────────────────────────────
    static unsigned long lastPrint = 0;
    if (millis() - lastPrint >= 1000UL) {
        lastPrint = millis();
        Serial.print("Enc: ");    Serial.print(encDeg, 1);   Serial.print(" deg");
        Serial.print(" | Tgt: "); Serial.print(targetDeg, 1); Serial.print(" deg");
        Serial.print(" | Err: "); Serial.print(targetDeg - encDeg, 2); Serial.print(" deg");
        Serial.print(" | Vc: ");  Serial.print(Vc, 3);       Serial.print(" V");
        Serial.print(" | PID: "); Serial.println(pidActive ? "ON" : "OFF");
    }
}