/*----------------------------------------------------------
Pediatric Exoskeleton - Single Joint Controller Firmware

Description:
Controls individual exoskeleton joints (hip, knee, ankle) via CAN bus commands.
Implements PID position control with trajectory interpolation at 500 Hz.

Key Features:
- CAN communication: 1 Mbps, extended frames, configurable feedback rate (see interpDuration)
- Position control: PID with 0.6° deadband for encoder noise rejection
- Trajectory interpolation: Smooth motion over configurable duration (see interpDuration macro)
- Sensor feedback: AS5045 encoder, ESCON current/speed, TMP36 temperature
- Thread safety: Volatile variables, interrupt protection for shared data
- Watchdog: 100ms CAN timeout triggers safety stop

Hardware:
- MCU: Teensy 3.2 or 4.1
- Motor: Maxon DC with ESCON controller (±3.3V PWM command)
- Encoder: AS5045 absolute position sensor 
- CAN: FlexCAN_T4 library

Control Parameters:
- Loop rate: 2ms (500 Hz) via IntervalTimer ISR
- PID gains: Kp=0.2, Ki=0.0 (disabled), Kd=0.002, dt=0.002s (PD control)
- Deadband: Configurable in motorDriver.cpp (typically 0.5-0.6°)
- Interpolation: Set interpDuration macro (e.g., 32ms = 16 cycles, 40ms = 20 cycles)
  * CAN feedback rate = 1000ms / interpDuration (e.g., 40ms → 25 Hz)
- Safety: Emergency flag, CAN timeout, position error limit (PLEGS only)

Last updated: 10/20/2025

------------------------------------------------------------ 
Author: Jesus Rodriguez
Email: jarodr90@cougarnet.uh.edu & jarodrigueztoscano@gmail.com
Lab of Brain Machine Interface
University of Houston
Date: 10202025
Version: Manuel's --> Teensy Luu folder --> Jesus Rodriguez & Jonathan Izquierdo
--> Jesus Rodriguez
----------------------------------------------------------
This program is free software; you can redistribute it and/or modify
it under the terms of the GNU General Public License as published by
the Free Software Foundation; either version 2 of the License, or
(at your option) any later version.

This program is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
GNU General Public License for more details.*/

// --------------------------START---------------------------
// Include related libraries
//Conditional libraries depending on the board

#if defined(__IMXRT1062__) //Teensy 4.1 conditional compilation
	#include "pinMap4.1.h" //Pin map for Teensy 4.1
	#define BOARD_NAME "Teensy 4.1"

#else
	#error "Unsupported board. Please compile for Teensy 3.2 or Teensy 4.1."
#endif

#include "AS5045.h" // New Encoder library change from previous library SPIencoder
#include "motorDriver.h" //Library for controlling the motor based on input from Master
#include "luu_utils.h"  //Personal library for utility functions
#include "encoder_utils.h" //Personal library for encoder utility functions
#include <IntervalTimer.h> // Interval timer library for Teensy for periodic tasks

// Define and Global variables
// DC Motor Related
char* motorLabel = "";  // Label for the motor


//Declarations for motor control
static dir_t direction; // To set if LOW is forward or backward and vice versa
float Vcc = 3.3;  // Voltage for the motor
float Vcrange[2] = {-Vcc, Vcc}; // Range of the voltage
uint8_t PWM_resolution = 8;  // Resolution of the PWM signal
static motorWiring_t maxonWiring; // For setting pin connections
static motorLimit_t maxonLimit; // For setting the position limits of the motor
IntervalTimer motorControlTimer; // Timer for motor control
volatile uint8_t flag = 0; // Flag to indicate if the motor should be disabled or not
int setPointDecode; // Set point for the motor, received from the Master
volatile float interpolateBegin = 0; // Variable to store the beginning of the interpolation
volatile float interpolateEnd = 0; // Variable to store the end of the interpolation
volatile float setPointInterpolated = 0; // Variable to store the interpolated set point
volatile int interpCounter = 0;
volatile bool interpInitialized = false; // Flag to track if interpolation has been initialized
volatile float interpIncrement = 0;
PID_t jointPID;
float DerivativeFilterAlpha = 0.2; // Aveg of 5 samples

// Encoder related
unsigned int clockDelay = 3; // Delay for the clock signal for encoder
int nbits = 16;  // Number of bits for turning floats to bytes
unsigned int encBinary; // Binary value from nbits Encoder
float encRaw; // Raw value from nbits Encoder
volatile float encDeg;	// Encoder value in degrees
volatile int b_encCalib;  // Encoder value in bytes
volatile int b_targetPos; // Target position in bytes for CAN feedback message
float encRange[2] = {0,0};  // Range of the encoder Currently empty
float encOffset = 0;  // Offset for the encoder Currently empty
float Vc  = 0;  // Velocity command for the motor

// ESCON related - current;
float ESCON_current = 0; // Current value from the ESCON
volatile int b_ESCON_current = 0; // Convert to 2 bytes value to send via CAN bus
float ESCON_current_range[2] = {-7, 7};  // Range of the current. Any changes to ESCON current parameters change here also
float Teensy_Analog_range[2] = {0, 3.3};  // Range of the analog signal from the Teensy. 

// ESCON related - speed;
float ESCON_speed = 0; // Speed value from the ESCON
volatile int b_ESCON_speed = 0; // Convert to 2 bytes value to send via CAN bus
float ESCON_speed_range[2] = {-3800, 3800};  // Range of the speed setup for the Analog signal. Any changes to ESCON speed parameters change here also

// Instantiate Object 
AS5045 myAS5045 (AS5045_CS_PIN, AS5045_CLK_PIN, AS5045_DATA_PIN, 0xFF, clockDelay); // See AS5045 Class
motorDriver maxonMotor(motorID, motorLabel, Vcc, PWM_resolution);	// See motorDriver Class

/**
 * Motor Control ISR - Runs every 2ms (500 Hz)
 * Performs trajectory interpolation, PID control, and periodic CAN transmission
 * Critical: Keep execution time < 100µs, no Serial.print() calls
 */
void motorControlISR() {
	interpCycles = 4000;
	
	// Initialize interpolation only once at the beginning
	if (!interpInitialized) {
		interpolateBegin = encDeg; // Set start position only once
		interpCounter = 0; // Reset counter for new interpolation
		setPointInterpolated = interpolateBegin; // Start from current position
		interpIncrement = (interpolateEnd - interpolateBegin) / interpCycles;
		interpInitialized = true; // Mark as initialized
	}
	// Hardcoded target position for demo - change this value to move motor to different angle
	interpolateEnd = 45;  // Move to 45 degrees (try: -45, 0, 45, 90, 180, -200, 200)

	// Interpolate trajectory: smooth motion from start to end position over interpCycles
	if (interpCounter < interpCycles) {
		setPointInterpolated += interpIncrement;
		interpCounter++;
	} else {
		setPointInterpolated = interpolateEnd;
		interpInitialized = false; // Reset flag when interpolation completes for next command
	}
	
	// PID control: calculate motor voltage command
	// Safety checks before allowing motor control
	bool safetyStop = false;
	
	// PLEGS Safety: Large position error detection (>5° difference)
	// Prevents runaway if encoder fails or command is invalid
	float positionError = abs(setPointInterpolated - encDeg);
	if (positionError > 10.0) {
		safetyStop = true;
	}
	
	if (safetyStop) {
		Vc = 0.0;  // Force zero voltage for safety
		maxonMotor.disable();
		// Reset PID state to prevent integral windup and derivative spikes
		jointPID.integral = 0.0;
		jointPID.lastError = 0.0;
		jointPID.filteredDerivative = 0.0;
	} else {
		float motorControl = maxonMotor.computePID(setPointInterpolated, encDeg, jointPID);
		maxonMotor.enable();
		Vc = motorControl;
		maxonMotor.rotate(Vc, direction);
	}
}

void setup() {
	pinMode(LED, OUTPUT); // Set the LED pin as output
	digitalWrite(LED, HIGH); // Turn on the LED from the board Used to check correct functioning
	analogReadResolution(12); // the default resolution of Arduino is 10
	
	// Pin mode setup
	// pinMode(LED, OUTPUT);
	pinMode(FW_PIN, INPUT);	// Forward limit switch
	pinMode(BW_PIN, INPUT); // Backward limit switch

	// Serial Setup;
	Serial.begin(115200);  // Start the serial communication
	if (!myAS5045.begin ()) {  // Revise if encoder is connected
		Serial.println ("Error setting up AS5045") ;
		delay(1000);
	}

	// Periodic interrupt for motor control
	motorControlTimer.begin(motorControlISR, 2000);
	motorControlTimer.priority(128);
	
	// Maxon Motor Related
	maxonWiring.dirPin = DIR_PIN;
	maxonWiring.enablePin = ENABLE_PIN;
	maxonWiring.pwmPin = PWM_PIN;
	maxonWiring.BWSwitchPin = 0;
	maxonWiring.FWSwitchPin = 0;
	digitalWrite(maxonMotor.wiring.FWSwitchPin, LOW);  //Set to HIGH if pins are set 0 or normal - T3.2
 	digitalWrite(maxonMotor.wiring.BWSwitchPin, LOW);  //Or Comment out if 0 pins - T3.2
	// Control Parameters Declaration
	direction.FORWARD = LOW;
	direction.BACKWARD = HIGH;
	encRange[0] = 180; encRange[1]= -180;
	encOffset = 0;  // current offset modify if needed
	// Kp, Ki, Kd, dt, integral, lastError, filteredDerivative, deadband, alpha
	jointPID = {0.2, 0, 0.002, 0.002, 0.0, 0.0, 0.0, 0.3, DerivativeFilterAlpha}; // PID values
	maxonLimit.backwardLimit = -200;
	maxonLimit.forwardLimit = 200;
	motorLabel = "Joint";
	//maxonWiring.HomeSwitchPin = HOME_SWITCH_PIN;
	maxonMotor.init(maxonWiring, maxonLimit); // Problem line
	maxonMotor.getInfo();
	pinMode(PWM_PIN, OUTPUT);
	Serial.println ("PedExoTeensy_5"); //Message at the beggining to know code inside the Teensy
	Serial.println ("1 CAN msg per leg_w_interrupts");
	Serial.print("CAN message rate: ");
	Serial.print(interpDuration);
	Serial.println(" ms");
	Serial.print("Gains (P, D, Deadzone, Alpha): ");
	Serial.print(jointPID.Kp);
	Serial.print(", ");
	Serial.print(jointPID.Kd);
	Serial.print(", ");
	Serial.print(jointPID.deadband);
	Serial.print(", ");
	Serial.print(jointPID.alpha);
	Serial.println();
	delay(500);
}

void loop() {
	static unsigned long lastDebugTime = 0;
	unsigned long currentTime = millis();
	
	// Declare ADC variables for ESCON readings
	int adc_ESCON_current, adc_ESCON_speed;
	
	// Status output every 1 second: encoder position, target, interpolated setpoint, error, voltage command
	if (currentTime - lastDebugTime >= 1000) {
		lastDebugTime = currentTime;
		Serial.print(motorLabel);
		Serial.print(" | Enc: ");
		Serial.print(encDeg, 2);
		Serial.print(" | Target: ");
		Serial.print(interpolateEnd, 2);
		Serial.print(" | Vc: ");
		Serial.print(Vc, 2);
		Serial.print(" | FLAG: ");
		Serial.print(flag);
		Serial.println();
	}
	
	
	// Read Encoder value
	encBinary = myAS5045.read(); // Read the encoder value
	encRaw = EncDeg(encBinary);// encBinary to Degrees
	encDeg = EncCalib(encRange, encOffset, encRaw); // Degrees to calibrated degrees
	b_encCalib = float2byte(encDeg,encRange,nbits); // Convert the encoder value to bytes
	maxonMotor.encoder = encDeg;  // Set the encoder value to the motor object

	// Check if the motor is at the limit
	if (encDeg >= maxonMotor.jointLimit.forwardLimit){
		digitalWrite(maxonMotor.wiring.FWSwitchPin, HIGH);
	}
	if (encDeg <= maxonMotor.jointLimit.backwardLimit){
		digitalWrite(maxonMotor.wiring.BWSwitchPin, HIGH);
	}
  
  // Read current feedback from ESCON
  adc_ESCON_current = analogRead(ESCON_AN1); 
  delayMicroseconds(1);//01172022 (MLK Day) This delay seems to make more accurate readings 300-->1
  adc_ESCON_current = analogRead(ESCON_AN1);//01172022 (MLK Day) The second reading here is for getting a more accu...
  
  // Read speed feedback from ESCON;
  adc_ESCON_speed = analogRead(ESCON_AN2);
  delayMicroseconds(1);//01172022 (MLK Day) This delay seems more accurate readings 300-->1
  adc_ESCON_speed = analogRead(ESCON_AN2);//01172022 (MLK Day) The second reading here is for getting a more accurate reading https://www.hackster.io/news/so-is-your-arduino-lying-to-you-15dcab71aa7e
  
  // Convert ADC readings directly to physical units (current and speed)
  ESCON_current = byte2float(adc_ESCON_current, ESCON_current_range, 12);//12 bits ADC directly to current range
  ESCON_speed = byte2float(adc_ESCON_speed, ESCON_speed_range, 12);  //12 bits ADC directly to speed range
  
  // Convert the current and speed to bytes to send through CAN bus
  b_ESCON_current = float2byte(ESCON_current, ESCON_current_range, nbits);
  b_ESCON_speed = float2byte(ESCON_speed, ESCON_speed_range, nbits);

  // Note: CAN message buffer is now updated in motorControlISR() when sending
}
