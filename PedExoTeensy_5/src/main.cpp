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
#if defined(__MK20DX256__) //Teensy 3.2 conditional compilation
	#include "pinMap.h" //Pin map for Teensy 3.2
	#define BOARD_NAME "Teensy 3.2"

#elif defined(__IMXRT1062__) //Teensy 4.1 conditional compilation
	#include "pinMap4.1.h" //Pin map for Teensy 4.1
	#define BOARD_NAME "Teensy 4.1"

#else
	#error "Unsupported board. Please compile for Teensy 3.2 or Teensy 4.1."
#endif

#include "AS5045.h" // New Encoder library change from previous library SPIencoder
#include "motorDriver.h" //Library for controlling the motor based on input from Master
#include "FlexCAN_T4.h" // New Can library for T4.1
#include "luu_utils.h"  //Personal library for utility functions
#include "encoder_utils.h" //Personal library for encoder utility functions
#include "CAN_utils.h" //Personal library for CAN utility functions
#include <IntervalTimer.h> // Interval timer library for Teensy for periodic tasks

#define IS_PLEGS 0 // value 1 means firmware for PLEGS, 0 means firmware for testbench
#define aref_voltage 3.3 // Voltage reference for Teensy
#define RIGHT_LEG  // Change this line from LEFT or RIGHT depending on leg to be uploaded to
#define ANKLE_JOINT  // Change this line from HIP or KNEE or ANKLE depending on joint to be uploaded to
#define NumSamplesUsedForDerivative 5 // Number of samples used for derivative low-pass filter
#define DerivativeFilterAlpha (1.0 / NumSamplesUsedForDerivative) // Alpha value for derivative low-pass filter
#define interpDuration 40 // Milliseconds per trajectory segment (set to 40ms = 20 cycles for 25 Hz)
// Don't forget to change platformio.ini accordingly

// Interpolation timing configuration - CHANGE THIS VALUE to adjust CAN message rate
// interpDuration: Duration for each trajectory segment in milliseconds
// Examples: 32ms → 31.25 Hz, 40ms → 25 Hz, 50ms → 20 Hz
#if (interpDuration % 2) != 0
	#error "interpDuration must be an even number (divisible by 2)."
#endif
#define interpCycles (interpDuration / 2) // Number of 2ms control cycles per segment
#if interpCycles <= 0 
	#error "Set interpCycles to a value greater than 0."
#endif
#define countMissedCAN_limit (interpCycles * 3 + 2) // Number of missed CAN messages before disabling motor
// Define and Global variables
// DC Motor Related
char* motorLabel = "";  // Label for the motor

// Define motor ID dynamically using preprocessor concatenation
#if defined(RIGHT_LEG) // RIGHT LEG is 0x101 from Master
    #define MOTOR_ID_BASE 2 // Right leg base index (RIGHT HIP)
	#define MSG_ID 257 // Decimal value for CAN message with ID 0x101
#elif defined(LEFT_LEG) // LEFT LEG is 0x101 from Master
    #define MOTOR_ID_BASE 5 // Left leg base index (LEFT HIP)
	#define MSG_ID 258 // Decimal value for CAN message with ID 0x102
#else
    #error "Unsupported leg. Please define RIGHT_LEG or LEFT_LEG."
#endif

#if defined(KNEE_JOINT) // Right Knee ID: 0x301, Left Knee ID: 0x601
    #define MOTOR_ID (MOTOR_ID_BASE + 1) // Operation to obtain Motor ID
	#define FlagBit 2 // Error detecting flag for corresponding joint - bit number
#elif defined(ANKLE_JOINT) // Right Ankle ID: 0x401, Left Knee ID: 0x701
    #define MOTOR_ID (MOTOR_ID_BASE + 2) // Operation to obtain Motor ID
	#define FlagBit 3 // Error detecting flag for corresponding joint - bit number
#elif defined(HIP_JOINT) // Right hip ID: 0x201, Left hip ID: 0x501
    #define MOTOR_ID MOTOR_ID_BASE // Operation to obtain Motor ID
	#define FlagBit 1 // Error detecting flag for corresponding joint - bit number
#else
    #error "Unsupported joint. Please define HIP_JOINT, KNEE_JOINT, or ANKLE_JOINT."
#endif

#define SetPointByte1 (FlagBit * 2) // High byte number for velocity command
#define SetPointByte2 (SetPointByte1 + 1) // Low byte number for velocity command
uint8_t motorID = MOTOR_ID; 

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
volatile float interpIncrement = 0;
volatile bool canMsgSent = false; // Flag to track if CAN message has been sent for current interpolation
volatile bool firstCANmsg = true; // Flag to track if this is the first CAN message received
PID_t jointPID;

// CAN bus related
static CAN_message_t writemsgCAN; // Message to be written to the CAN bus
static CAN_message_t readmsgCAN;  // Message to be read from the CAN bus
uint32_t msgID; // Message ID
uint16_t reducedMsgID; //Variable used to separate the node ID and message type
uint8_t nodeID;  // Node ID
uint8_t msgType; // Message type
volatile unsigned long countMissedCAN; // If number of missed CAN messages is too high, deactivate
int CANdataID = 1;  // Data ID for the CAN message

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
float ESCON_current_range[2] = {-6.5, 6.5};  // Range of the current. Any changes to ESCON current parameters change here also
float Teensy_Analog_range[2] = {0, 3.3};  // Range of the analog signal from the Teensy. 

// ESCON related - speed;
float ESCON_speed = 0; // Speed value from the ESCON
volatile int b_ESCON_speed = 0; // Convert to 2 bytes value to send via CAN bus
float ESCON_speed_range[2] = {-3800, 3800};  // Range of the speed setup for the Analog signal. Any changes to ESCON speed parameters change here also

//TMP36 Pin Variables
int tempPin = A11; 
int tempReading;    // The analog reading from the sensor
volatile int b_tempReading = 0;
float tempReading_range[2] = {-40, 125};//https://www.analog.com/media/en/technical-documentation/data-sheets/TMP35_36_37.pdf

// Instantiate Object 
AS5045 myAS5045 (AS5045_CS_PIN, AS5045_CLK_PIN, AS5045_DATA_PIN, 0xFF, clockDelay); // See AS5045 Class
motorDriver maxonMotor(motorID, motorLabel, Vcc, PWM_resolution);	// See motorDriver Class
// CAN bus object depending on the board
#if defined(__MK20DX256__) //Teensy 3.2 conditional compilation
	FlexCAN_T4<CAN0, RX_SIZE_128, TX_SIZE_128> CAN;  // CAN bus object for Teensy 3.2. Pins are internally defined in library
#elif defined(__IMXRT1062__) //Teensy 4.1 conditional compilation
	FlexCAN_T4<CAN3, RX_SIZE_128, TX_SIZE_128> CAN;  // CAN bus object for Teensy 4.1. Pins are internally defined in library
#endif

volatile unsigned long loopCounter = 0; // Loop counter for CAN message timeout detection

/**
 * CAN Receive Interrupt Handler
 * Triggered when a CAN message with matching ID is received
 * Updates target position and resets interpolation for smooth trajectory tracking
 * Note: Keep this ISR fast - no Serial.print() calls to avoid buffer overflow
 */
void ISR_canVcUpdate(const CAN_message_t &msg) {
	noInterrupts(); // Prevent race conditions with motor control ISR
	loopCounter = 0;
	countMissedCAN = 0;
	
	// Start interpolation: use last target for smooth chaining, except on first message
	if (firstCANmsg) {
		interpolateBegin = encDeg;  // First message: start from current encoder position
		firstCANmsg = false;
	} else {
		interpolateBegin = interpolateEnd;  // Subsequent messages: chain from last target
	}
	
	// Decode flag byte for emergency stop
	uint8_t index0[] = {1};
	int flag_byte = decodeBuffer(index0, sizeof(index0), msg);
	flag = extractBit(flag_byte, FlagBit);
	
	// Decode target position from CAN message
	// For KNEE: reads bytes 4 and 5 (SetPointByte1=4, SetPointByte2=5)
	uint8_t index1[] = {SetPointByte1, SetPointByte2};
	setPointDecode = decodeBuffer(index1, sizeof(index1), msg);
	
	// Skip processing if both position bytes are zero (invalid/uninitialized message)
	// This prevents spurious movements on startup before Master sends real data
	if (msg.buf[SetPointByte1] == 0 && msg.buf[SetPointByte2] == 0) {
		interrupts();
		return;  // Ignore this message
	}
	
	interpolateEnd = byte2float(setPointDecode, encRange, nbits);
	
	// Initialize interpolation: smooth trajectory over interpCycles
	setPointInterpolated = interpolateBegin;
	interpCounter = 0;
	interpIncrement = (interpolateEnd - interpolateBegin) / interpCycles;
	canMsgSent = false;
	
	interrupts();
}

/**
 * Motor Control ISR - Runs every 2ms (500 Hz)
 * Performs trajectory interpolation, PID control, and periodic CAN transmission
 * Critical: Keep execution time < 100µs, no Serial.print() calls
 */
void motorControlISR() {
	// Interpolate trajectory: smooth motion from start to end position over interpCycles
	if (interpCounter < interpCycles) {
		setPointInterpolated += interpIncrement;
		interpCounter++;
	} else {
		setPointInterpolated = interpolateEnd;
	}
	
	// PID control: calculate motor voltage command
	// Safety checks before allowing motor control
	bool safetyStop = false;
	
	// Safety: Emergency flag or CAN timeout
	if (flag == 1 || countMissedCAN >= countMissedCAN_limit) {
		safetyStop = true;
	}
	
	#if IS_PLEGS == 1
	// PLEGS Safety: Large position error detection (>5° difference)
	// Prevents runaway if encoder fails or command is invalid
	float positionError = abs(setPointInterpolated - encDeg);
	if (positionError > 5.0) {
		safetyStop = true;
	}
	#endif
	
	if (safetyStop) {
		Vc = 0.0;  // Force zero voltage for safety
		maxonMotor.disable();
		maxonMotor.rotate(0.0, direction);  // Ensure PWM is zero
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
	
	// Send CAN feedback message at the end of interpolation period
	// Send at interpCycles-1 to avoid race condition with CAN receive ISR
	if (interpCounter == (interpCycles - 1) && !canMsgSent) {
		// Convert target position to bytes for transmission
		b_targetPos = float2byte(interpolateEnd, encRange, nbits);
		
		// Pack sensor data into CAN message buffer
		writemsgCAN.buf[0] = highByte(b_encCalib);
		writemsgCAN.buf[1] = lowByte(b_encCalib);
		writemsgCAN.buf[2] = highByte(b_ESCON_current);
		writemsgCAN.buf[3] = lowByte(b_ESCON_current);
		writemsgCAN.buf[4] = highByte(b_ESCON_speed);
		writemsgCAN.buf[5] = lowByte(b_ESCON_speed);
		writemsgCAN.buf[6] = highByte(b_targetPos);
		writemsgCAN.buf[7] = lowByte(b_targetPos);
		
		CAN.write(writemsgCAN);
		canMsgSent = true;
	}
	
	loopCounter++;
	countMissedCAN++; // Timeout counter for CAN communication watchdog
}

/**
 * CAN Bus Setup
 * Configures FlexCAN for 1 Mbps, extended frames, with message filtering
 */
void setupCAN() {
	const uint32_t CANbaud = 1000000; // CANbus bitrate, Max 1Mbps
    pinMode(33, OUTPUT);   //Investigate later
    digitalWrite(33, LOW); 		

    CAN.begin(); // Start the CAN bus
	CAN.setBaudRate(CANbaud); // Set the baud rate of the CAN bus
	CAN.setMaxMB(16); // Set the maximum number of mailboxes
	CAN.enableFIFO(); // Enable FIFO buffer

	// Enable FIFO interrupts 
    CAN.enableFIFOInterrupt();

	// Filter: only accept messages with matching MSG_ID (0x101 for right leg, 0x102 for left leg)
	CAN.setFIFOFilter(REJECT_ALL);
	CAN.setFIFOFilter(0, MSG_ID, EXT);
	CAN.onReceive(ISR_canVcUpdate); // When a message is received through filter, call the ISR

	// Configure outgoing message format
	writemsgCAN.len = 8;
	writemsgCAN.flags.extended = 1;
	writemsgCAN.id = motorID * 256 + CANdataID; // Format: motorID*256 + dataID
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
	if (!myAS5045.begin ())  // Revise if encoder is connected
	Serial.println ("Error setting up AS5045") ;

	// CAN bus setup;
	setupCAN();
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
	#if MOTOR_ID == 2 // Right Hip
		direction.FORWARD = LOW; // Set the direction of the motor
		direction.BACKWARD = HIGH;
		encRange[0] = 180; encRange[1]= -180;  // Set the range of the encoder
		#if IS_PLEGS == 1 // Encoder offset value from PLEGS configuration
			encOffset = -63.19;  // current offset modify if needed
		#elif IS_PLEGS == 0  // Encoder offset value from Testbench configuration
			encOffset = 47.29;  // current offset modify if needed
		#endif
		// Kp, Ki, Kd, dt, integral, lastError, filteredDerivative, deadband, alpha
		jointPID = {0.2, 0, 0.002, 0.002, 0.0, 0.0, 0.0, 0.4, DerivativeFilterAlpha}; // Hip PID values
   		maxonLimit.backwardLimit = -200; // Set the limits of the motor, currently not used. Errors handled inside Master
    	maxonLimit.forwardLimit = 200;
		motorLabel = "Right Hip";  // Label for the motor
	#elif MOTOR_ID == 3 // Right Knee
		direction.FORWARD = LOW;
		direction.BACKWARD = HIGH;
		encRange[0] = 180; encRange[1]= -180;
		#if IS_PLEGS == 1 // Encoder offset value from PLEGS configuration
			encOffset = 42.89;  // current offset modify if needed
		#elif IS_PLEGS == 0  // Encoder offset value from Testbench configuration
			encOffset = -107.75;  // current offset modify if needed
		#endif
		// Kp, Ki, Kd, dt, integral, lastError, filteredDerivative, deadband, alpha
		jointPID = {0.2, 0, 0.002, 0.002, 0.0, 0.0, 0.0, 0.4, DerivativeFilterAlpha}; // Knee PID values
		maxonLimit.backwardLimit = -200;
		maxonLimit.forwardLimit = 200;	
		motorLabel = "Right Knee";
	#elif MOTOR_ID == 4  // Right Ankle
		direction.FORWARD = LOW;
		direction.BACKWARD = HIGH;
		encRange[0] = 180; encRange[1]= -180;
		#if IS_PLEGS == 1 // Encoder offset value from PLEGS configuration
			encOffset = 150.12;  // current offset modify if needed
		#elif IS_PLEGS == 0  // Encoder offset value from Testbench configuration
			encOffset = -10.02;  // current offset modify if needed
		#endif
		// Kp, Ki, Kd, dt, integral, lastError, filteredDerivative, deadband, alpha
		jointPID = {0.2, 0, 0.002, 0.002, 0.0, 0.0, 0.0, 0.4, DerivativeFilterAlpha}; // PID values - REDUCED for stability
		maxonLimit.backwardLimit = -200;
		maxonLimit.forwardLimit = 200; 
		motorLabel = "Right Ankle";
	#elif MOTOR_ID == 5  // Left Hip
		direction.FORWARD = HIGH;
		direction.BACKWARD = LOW;
		encRange[0] = -180; encRange[1]= 180;
		#if IS_PLEGS == 1 // Encoder offset value from PLEGS configuration
			encOffset = -103.01;  // current offset modify if needed
		#elif IS_PLEGS == 0  // Encoder offset value from Testbench configuration
			encOffset = 68.55;  // current offset modify if needed
		#endif
		// Kp, Ki, Kd, dt, integral, lastError, filteredDerivative, deadband, alpha
		jointPID = {0.2, 0, 0.002, 0.002, 0.0, 0.0, 0.0, 0.4, DerivativeFilterAlpha}; // PID values
		maxonLimit.backwardLimit = -200;
		maxonLimit.forwardLimit = 200;
		motorLabel = "Left Hip";
	#elif MOTOR_ID == 6  // Left Knee
		direction.FORWARD = HIGH;
		direction.BACKWARD = LOW;
		encRange[0] = -180; encRange[1]= 180;
		#if IS_PLEGS == 1 // Encoder offset value from PLEGS configuration
			encOffset =-148;  // current offset modify if needed
		#elif IS_PLEGS == 0  // Encoder offset value from Testbench configuration
			encOffset = -43.77;  // current offset modify if needed
		#endif
		// Kp, Ki, Kd, dt, integral, lastError, filteredDerivative, deadband, alpha
		jointPID = {0.2, 0, 0.002, 0.002, 0.0, 0.0, 0.0, 0.4, DerivativeFilterAlpha}; // PID values
		maxonLimit.backwardLimit = -200;
		maxonLimit.forwardLimit = 200;
		motorLabel = "Left Knee";
	#elif MOTOR_ID == 7  // Left Ankle
		direction.FORWARD = HIGH;
		direction.BACKWARD = LOW;
		encRange[0] = -180; encRange[1]= 180;
		#if IS_PLEGS == 1 // Encoder offset value from PLEGS configuration
			encOffset =-167.61; // current offset modify if needed
		#elif IS_PLEGS == 0  // Encoder offset value from Testbench configuration
			encOffset = 153.72;  // current offset modify if needed
		#endif
		// Kp, Ki, Kd, dt, integral, lastError, filteredDerivative, deadband, alpha
		jointPID = {0.2, 0, 0.002, 0.002, 0.0, 0.0, 0.0, 0.4, DerivativeFilterAlpha}; // PID values
		maxonLimit.backwardLimit = -200;
		maxonLimit.forwardLimit = 200;
		motorLabel = "Left Ankle";
	#endif
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
	
	
	// delayMicroseconds(1000);   // Delay for 1 ms to match the sampling rate of Master
	CAN.events(); // Used to handle queue events when the system does not have interrupts enabled
	
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

  // Read temp feedback from PCB;
  tempReading = analogRead(tempPin);
  delayMicroseconds(1);//01172022 (MLK Day) This delay seems more accurate readings 300-->1
  tempReading = analogRead(tempPin);//01172022 (MLK Day) The second reading here is for getting a more accurate reading https://www.hackster.io/news/so-is-your-arduino-lying-to-you-15dcab71aa7e
  // converting that reading to voltage, which is based off the reference voltage
  float voltage = tempReading * aref_voltage;
  voltage /= 4096.0; //resolution = 2^12
  float temperatureC = voltage  * 555.56-50 ;
  
  // Convert ADC readings directly to physical units (current and speed)
  ESCON_current = byte2float(adc_ESCON_current, ESCON_current_range, 12);//12 bits ADC directly to current range
  ESCON_speed = byte2float(adc_ESCON_speed, ESCON_speed_range, 12);  //12 bits ADC directly to speed range
  
  // Convert the current and speed to bytes to send through CAN bus
  b_ESCON_current = float2byte(ESCON_current, ESCON_current_range, nbits);
  b_ESCON_speed = float2byte(ESCON_speed, ESCON_speed_range, nbits);
  b_tempReading = float2byte(temperatureC, tempReading_range, nbits);

  // Note: CAN message buffer is now updated in motorControlISR() when sending
	
	// CAN Watchdog: Timeout after 3 missed messages (automatically scales with interpDuration)
	if (countMissedCAN >= countMissedCAN_limit){ // countMissedCAN_limit = interpCycles*3+2 (e.g., 62 cycles * 2ms = 124ms for 40ms messages)
		//Serial.println("CAN Not Available!"); // Print to Serial Monitor
		interpolateEnd = encDeg; // Set the target to the current position to avoid sudden jumps
		interpCounter = interpCycles; // Skip interpolation
		interpolateBegin = encDeg; // Set the beginning of the interpolation to current position
		canMsgSent = false; // Allow CAN message to be sent again
		setPointInterpolated = encDeg; // Set the interpolated point to current position
		interpIncrement = 0; // No increment
		firstCANmsg = true; // Reset first CAN message flag
	} 
}
