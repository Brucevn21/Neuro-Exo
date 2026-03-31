/*----------------------------------------------------------
 Description
 -Input:
 -Output: 
 -Example:
------------------------------------------------------------ 
 Author: Trieu Phat Luu
 Email: tpluu2207@gmail.com
 Lab of Brain Machine Interface
 University of Houston
 Date: 
 Version:
------------------------------------------------------------
 This program is free software; you can redistribute it and/or modify
 it under the terms of the GNU General Public License as published by
 the Free Software Foundation; either version 2 of the License, or
 (at your option) any later version.

 This program is distributed in the hope that it will be useful,
 but WITHOUT ANY WARRANTY; without even the implied warranty of
 MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 GNU General Public License for more details.*/

// --------------------------START---------------------------

#ifndef __MOTOR_DRIVER_H__
#define __MOTOR_DRIVER_H__

#include "Arduino.h"

typedef struct motorWiring_t{
	uint8_t enablePin;
	uint8_t dirPin;
	uint8_t pwmPin;
	// Limit and Home Switch
	uint8_t FWSwitchPin;
	uint8_t BWSwitchPin;
	// uint8_t HomeSwitchPin;
	// SPI Encoder
}motorWiring_t;

typedef struct dir_t{
	bool FORWARD;
	bool BACKWARD;
}dir_t;

//typedef enum {
//	FORWARD = LOW,
//	BACKWARD = HIGH
//}dir_t; // Motor orientation of Exo will affect this definition

typedef struct motorStatus_t {
	volatile bool isEnable;			// 1: Enable; 0: Disable
	volatile dir_t direction;		// FORWARD; BACKWARD
	volatile bool isAccelerate;		// 1: Accelerating; 0: Decelerate
	volatile bool isHome;			// 1: At Home position;
	volatile int switchLimit;		// 1: Forward limit; 0: Home; -1: Backward limit
	volatile bool isMaxTorque;		// 1: Motor at maximum torque.
	volatile bool isMaxSpeed;		// 1: Motor at maximum speed.
	volatile float pos;				// Current position.
	volatile float speed;			// Current Speed.
	volatile float acc;				// Current Acceleration
} motorStatus_t;

typedef struct motorLimit_t {
	float forwardLimit;
	float backwardLimit;
	float maxSpeed;
	float minSpeed;
	float maxAcc;
	float maxTorque;
} motorLimit_t;

typedef struct PID_t {
	float Kp;
	float Ki;
	float Kd;
	float dt; // smapling time
	float integral;
	float lastError;
	float filteredDerivative; // EMA-filtered derivative for noise reduction
	float deadband; // Position deadband in degrees (e.g., 0.4°)
	float alpha; // EMA filter coefficient (e.g., 0.2 for ~5 sample average)
} PID_t;

class motorDriver {	
	private:					
		char* label;
		float Vcc;
		uint8_t ID;	
		float Vmax;		
		uint8_t nbitsPWM; //nbits resolution of PWM	
		static void ISR_FWSwitch();
		static void ISR_BWSwitch();
	//
	public:	
		// Constructor
		motorDriver(uint8_t motorID, char* motorlabel, float VccIn, uint8_t PWMres);	
		void init(const motorWiring_t &wiringInput, const motorLimit_t &limitInput);
		// Get properties
		void getInfo();
		uint8_t getID();
		motorLimit_t jointLimit;
		motorWiring_t wiring;	
		float encoder;
		// Set properties
		void setVmax(float);
		void setPWM(float);
		void setDir(bool); // Set Motor Direction	
		void enable();
		uint8_t isEnable();
		void disable();	
		void rotateForward(float Vin, dir_t direction);
		void rotateBackward(float Vin, dir_t direction);
		void rotate(float, dir_t direction); //
		void stop(); //
		//void goHome();
	float computePWM(float Vin);
	float ESCON_mapping(float input, float pwm_limit);
	float computePID(float setPoint, float currPos, PID_t &PIDstruct);  // Changed to pass by reference
	static void threshold(float &valIn, float* range);
		static float toVc(float inputVal, float* Vrange, uint8_t nbits); // Convert nbits int to float, control signal
};

#endif //__MOTOR_DRIVER_H__