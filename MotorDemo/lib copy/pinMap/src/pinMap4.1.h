/* PIN Definition.
 Project: Pediatric Exoskeleton
 Board: Teensey 4.1. https://www.pjrc.com/store/teensy41.html
 Node: Joint Node.
 Authors: Trieu Phat Luu, David Eguren, Jesus Rodriguez
 Email: jarodrigueztoscano@gmail.com
 Lab of Brain Machine Interface
 University of Houston
 Date: 02212025
----------------------------------------------------------*/
// CAN bus
//CAN bus is defined in library FlexCAN_T4.h
// Motor control //Currently these connections are not used, but they are defined for future use
#define BW_PIN 16			// Rotate Backward.
#define HOME_SWITCH_PIN 1	// Home switch. Not used in the PCB
#define FW_PIN 15			// Rotate Forward.
#define CONTROL_MODE_PIN 17	// 1. Manual (open loop); 0: Closed loop
#define PWM_PIN 7			// To motor, PWM pin
#define DIR_PIN 9			// To motor, direction pin
#define ENABLE_PIN 8		// To motor, enable pin
#define FW_SWITCH_PIN 23	// Forward Limit switch
#define BW_SWITCH_PIN 22	// Backward Limit switch
// ESCON Feedback.
#define ESCON_AN1 21		// Current - Configured Analog feedback from ESCON
#define ESCON_AN2 20		// Velocity
// led pin
#define LED 13				// Onboard LED teensey 
// AS4050 Encoder, SPI comm protocol
#define AS5045_CLK_PIN 1 // Clock PIN
#define AS5045_CS_PIN 2	   // Chip Select
#define AS5045_DATA_PIN 0	// Data In
