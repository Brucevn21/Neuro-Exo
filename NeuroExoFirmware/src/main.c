/*
    Neuro-Exo - Single Elbow Joint Firmware

Description:
Dynamically controls the movement of the Neuro-Exo joint, receiving commands via the connection
of the BeagleBone Black device and associated app, outputting it to the motor controller for appropriate response

Key Features:


Hardware:
- MCU: Teensy 4.1
- Joint Motor: MAXON DC with ESCON Controller (±3.3V PWM command)
- Encoder: AS5045 absolute position sensor

Control Parameters:


Last Updated: 4/10/2026

*/

#include <Arduino.h>
#include <Arduino_FreeRTOS.h>

#include "AS5045.h"
#include "motorDriver.h"
#include "encoder_utils.h"
#include "luu_utils.h"

// Include related libraries
//Conditional libraries depending on the board
#if defined(__IMXRT1062__) //Teensy 4.1 conditional compilation
	#include "pinMap4.1.h" //Pin map for Teensy 4.1
	#define BOARD_NAME "Teensy 4.1"
#else
	#error "Unsupported board. Please compile for Teensy 4.1."
#endif

// CONFIG
#define CONTROL_PERIOD_MS 2   // 500 Hz
#define DEBUG_PERIOD_MS   1000

#define interpDuration 40
#define interpCycles (interpDuration / CONTROL_PERIOD_MS)

// GLOBAL VARIABLES

// Encoder
AS5045 encoder(AS5045_CS_PIN, AS5045_CLK_PIN, AS5045_DATA_PIN, 0xFF, clockDelay);

volatile float encDeg = 0;
volatile float encOffset = 0;
float encRange[2] = {180, -180};

// Motor
motorDriver motor(motorID, "Joint", Vcc, PWM_resolution);
PID_t jointPID = {0.2, 0, 0.002, 0.002, 0, 0, 0, 0.4, 0.2};

dir_t direction;

// Interpolation
volatile float interpolateBegin = 0;
volatile float interpolateEnd = 0;
volatile float setPointInterpolated = 0;
volatile float interpIncrement = 0;
volatile int interpCounter = 0;

// Command (NO CAN → local variable)
volatile float desiredPosition = 0;

// ESCON + Temp
float ESCON_current = 0;
float ESCON_speed = 0;

// ------------------ TASKS ------------------

// Encoder Task 
void EncoderTask(void *pvParameters) {
    while (1) {
        uint16_t encBinary = encoder.read();
        float encRaw = EncDeg(encBinary);
        encDeg = EncCalib(encRange, encOffset, encRaw);

        // ESCON reads
        int adc_current = analogRead(ESCON_AN1);
        int adc_speed   = analogRead(ESCON_AN2);

        ESCON_current = byte2float(adc_current, (float[2]){-6.5, 6.5}, 12);
        ESCON_speed   = byte2float(adc_speed, (float[2]){-3800, 3800}, 12);

        vTaskDelay(pdMS_TO_TICKS(1));
    }
}

//  Motor Control Task 
void MotorControlTask(void *pvParameters) {
    while (1) {

        // Interpolation update
        if (interpCounter < interpCycles) {
            setPointInterpolated += interpIncrement;
            interpCounter++;
        } else {
            setPointInterpolated = interpolateEnd;
        }

        // Simple safety
        bool safetyStop = false;

        if (abs(setPointInterpolated - encDeg) > 5.0) {
            safetyStop = true;
        }

        float Vc = 0;

        if (safetyStop) {
            motor.disable();
            motor.rotate(0, direction);
        } else {
            Vc = motor.computePID(setPointInterpolated, encDeg, jointPID);
            motor.enable();
            motor.rotate(Vc, direction);
        }

        vTaskDelay(pdMS_TO_TICKS(CONTROL_PERIOD_MS));
    }
}

//  Debug Task 
void DebugTask(void *pvParameters) {
    while (1) {
        Serial.print("Enc: ");
        Serial.print(encDeg);
        Serial.print(" | Target: ");
        Serial.print(interpolateEnd);
        Serial.print(" | Cmd: ");
        Serial.print(setPointInterpolated);
        Serial.println();

        vTaskDelay(pdMS_TO_TICKS(DEBUG_PERIOD_MS));
    }
}

// Command Setpoint Update 
void updateSetpoint(float newTarget) {
    interpolateBegin = setPointInterpolated;
    interpolateEnd = newTarget;

    interpCounter = 0;
    interpIncrement = (interpolateEnd - interpolateBegin) / interpCycles;
}

//  ------------------ SETUP ------------------
void setup() {
    Serial.begin(115200);

    encoder.begin();

    direction.FORWARD = LOW;
    direction.BACKWARD = HIGH;

    motor.init({DIR_PIN, ENABLE_PIN, PWM_PIN, 0, 0},
               {-200, 200});

    // Initial setpoint
    desiredPosition = 0;
    updateSetpoint(desiredPosition);

    // Create RTOS Tasks
    xTaskCreate(EncoderTask, "Encoder", 2048, NULL, 2, NULL);
    xTaskCreate(MotorControlTask, "MotorControl", 2048, NULL, 3, NULL);
    xTaskCreate(DebugTask, "Debug", 2048, NULL, 1, NULL);

    vTaskStartScheduler();
}

void loop() {
    // Not used in FreeRTOS
}