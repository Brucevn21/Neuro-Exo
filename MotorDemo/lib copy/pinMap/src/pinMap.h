/* PIN Definition.
 Project: Pediatric Exoskeleton
 Board: Teensey 3.2. https://www.pjrc.com/store/teensy32.html
 Node: Joint Node.
 Authors: Trieu Phat Luu, David Eguren
 Email: tpluu2207@gmail.com
 Lab of Brain Machine Interface
 University of Houston
 Date: 180223
----------------------------------------------------------*/
// CAN bus
#define CAN_TX_PIN 3		// CAN transmit
#define CAN_RX_PIN 4		// CAN receive
// Motor control
#define BW_PIN 16			// Rotate Backward.
#define HOME_SWITCH_PIN 1	// Home switch. Not used in the PCB
#define FW_PIN 15			// Rotate Forward.
#define CONTROL_MODE_PIN 17	// 1. Manual (open loop); 0: Closed loop
#define PWM_PIN 6		// To motor, PWM pin
#define DIR_PIN 27			// To motor, direction pin
#define ENABLE_PIN 5		// To motor, enable pin
#define FW_SWITCH_PIN 23	// Forward Limit switch //23 //If set to 23 set HIGH on pins
#define BW_SWITCH_PIN 22	// Backward Limit switch //22
// ESCON Feedback.
#define ESCON_AN1 31		// Configured Analog feedback from ESCON
#define ESCON_AN2 21		//
#define ESCON_BREAK 28 
// TEMPERATURE Feedback
#define FEEDBACK_TEMP 18		//
// led pin
#define LED 13				// Onboard LED teensey 
// AS4050 Encoder, SPI comm protocol
#define  AS5045_CLK_PIN 14	// Clock PIN
#define  AS5045_CS_PIN 20   // Chip Select
#define  AS5045_DATA_PIN 12	// Data In
// SPI pin map for Straingauge torque
#define  TORQUE_CLK_PIN 13	// Clock PIN
#define  TORQUE_CS_PIN 9   // Chip Select
#define  TORQUE_DATA_PIN 11	// Data In
