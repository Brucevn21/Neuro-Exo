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
 Date: 190207
 Version: v2.0
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
#include "motorDriver.h"

static motorDriver *instance;	// Define static instance to attach interrupt

motorDriver::motorDriver(uint8_t motorID, char* motorlabel = "Maxon DC Motor", float VccIn = 3.3, uint8_t PWMres = 8){
	ID = motorID;		// Motor ID, also used as CAN node ID
	label = motorlabel;	// Motor's name.
	Vcc = VccIn;		// Vcc. Default: 3.3V, Teensey 4.1	
	Vmax = 0.9*Vcc;		// setVmax to limit motor speed.
	nbitsPWM = PWMres;	// PWM resolution. Default: 8 bits		
	encoder = 0;
	instance = this;	// Static Instance to attach Interrupt
}

void motorDriver::init(const motorWiring_t &motorWiring, const motorLimit_t &motorLimit){
	wiring.enablePin = motorWiring.enablePin;
	wiring.dirPin = motorWiring.dirPin;
	wiring.pwmPin = motorWiring.pwmPin; 
	wiring.BWSwitchPin = motorWiring.BWSwitchPin;
	wiring.FWSwitchPin = motorWiring.FWSwitchPin; 
	jointLimit.backwardLimit = motorLimit.backwardLimit;
	jointLimit.forwardLimit = motorLimit.forwardLimit;
	jointLimit.maxSpeed = motorLimit.maxSpeed;
	jointLimit.minSpeed = motorLimit.minSpeed;
	jointLimit.maxTorque = motorLimit.maxTorque;
	jointLimit.maxAcc = motorLimit.maxAcc; 
	// Pin Mode Input/Output Setting;
	pinMode(wiring.enablePin, OUTPUT); 
	pinMode(wiring.dirPin, OUTPUT);	
	pinMode(wiring.pwmPin, OUTPUT); 
	// If Switches are connected.
	if (wiring.BWSwitchPin !=0){
		pinMode(wiring.BWSwitchPin, INPUT);
		attachInterrupt(digitalPinToInterrupt(wiring.BWSwitchPin), instance->ISR_BWSwitch,HIGH);
	}	
	if (wiring.FWSwitchPin !=0){
		pinMode(wiring.FWSwitchPin, INPUT);
		attachInterrupt(digitalPinToInterrupt(wiring.FWSwitchPin), instance->ISR_FWSwitch,HIGH);
	}	
}

void motorDriver::ISR_FWSwitch(){
	Serial.println((String)"ISR-->Forward Limit:" + instance->jointLimit.forwardLimit + \
					". Encoder:" + instance->encoder);
	if(instance->isEnable())
		instance->disable();

}

void motorDriver::ISR_BWSwitch(){
	Serial.println((String)"ISR-->Backward Limit:" + instance->jointLimit.backwardLimit + \
					". Encoder:" + instance->encoder);
	if(instance->isEnable())
		instance->disable();
}

uint8_t motorDriver::isEnable(){
	return digitalRead(wiring.enablePin);
}

void motorDriver::getInfo(){
	Serial.println((String)"ID: " + ID);
	Serial.println((String)"Motor Label: " + label);
	Serial.println((String)"Vcc: " + Vcc);
	Serial.println((String)"Vmax: " + Vmax);
	Serial.println((String)"PWM nbits Resolution: " + nbitsPWM);
	// Motor Wiring.
	Serial.println("Motor Wiring: ");
	Serial.println((String)"Enable Pin-- " + wiring.enablePin);
	Serial.println((String)"Direction Pin-- " + wiring.dirPin);
	Serial.println((String)"PWM Pin-- " + wiring.pwmPin);
	Serial.println((String)"Forward Switch Pin-- " + wiring.FWSwitchPin);
	Serial.println((String)"Backward Switch Pin-- " + wiring.BWSwitchPin);
	Serial.println((String)"Forward Angle Limit " + jointLimit.forwardLimit);
	Serial.println((String)"Backward Angle Limit: " + jointLimit.backwardLimit);
	
}

uint8_t motorDriver::getID(){
	return ID;
}

void motorDriver::disable(){
	// Set Enable Pin to LOW
	digitalWrite(wiring.enablePin, LOW);
}

void motorDriver::enable(){
	// Set Enable Pin to HIGH
	digitalWrite(wiring.enablePin, HIGH);
}

void motorDriver::stop(){
	this->setPWM(0);
}

void motorDriver::rotateForward(float Vin, dir_t direction){
	if (!isEnable()) this->enable();	
	//if (isEnable()) this->disable();
	this->setDir(direction.FORWARD);
	this->setPWM(Vin);
}

void motorDriver::rotateBackward(float Vin, dir_t direction){
	if (!isEnable()) this->enable();	
	//if (isEnable()) this->disable()
	this->setDir(direction.BACKWARD);
	this->setPWM(Vin);
}

void motorDriver::rotate(float Vin, dir_t direction){
	if (Vin >=0) this->rotateForward(Vin, direction);		
	else this->rotateBackward(Vin, direction);	
}

void motorDriver::setVmax(float Vin = 3){
	// Set Maximum Voltage to control Max speed. Default 3 V.
	Vmax = Vin;
}

void motorDriver::setDir(bool direction){
	digitalWrite(wiring.dirPin, direction);
}

void motorDriver::setPWM(float Vin){
	analogWrite(wiring.pwmPin, this->computePWM(Vin));
}

float motorDriver::computePWM(float Vin){
    // Compute PWM value from control Voltage Input
    float absVin = abs(Vin);  // Only care about magnitude for speed
    return ESCON_mapping(absVin, 0.9);  // Limit duty cycle to 90% max
}

void motorDriver::threshold(float &valIn, float* range){
	if (valIn < range[0]) valIn = range[0];
	else if (valIn > range[1]) valIn = range[1];
	else;
}	


float motorDriver::toVc(float inputVal, float* Vrange, uint8_t nbits){
	int res = (1<<nbits) - 1; // nbits resolution;
	float Vc = inputVal * (Vrange[1] - Vrange[0])/res + Vrange[0] ;
	return Vc;
}

float motorDriver::ESCON_mapping(float inputVal, float percent_limit = 0.9){
    // Define the 8-bit PWM range (0-255) with 10%-90% duty cycle
    float minPWM = 0.1 * 255;  // 10% of 255 = ~25
    float maxPWM = percent_limit * 255;  // 90% of 255 = ~230

    // Map inputVal (0 to 3.3V) to the PWM range (minPWM to maxPWM)
    float mappedPWM = map(inputVal, 0.0, 3.3, minPWM, maxPWM);

    // Ensure mappedPWM stays within bounds
    if (mappedPWM < minPWM) mappedPWM = minPWM;
    if (mappedPWM > maxPWM) mappedPWM = maxPWM;

    return mappedPWM;
}

// PID controller
/**
 * PID Controller with Deadband for Encoder Noise Rejection
 * 
 * Features:
 * - 0.6° deadband to eliminate oscillation from encoder quantization (0.35° resolution)
 * - Smooth error transition: errors shifted by deadband amount to prevent derivative spikes
 * - Integral anti-windup: limits Iout to ±2V to prevent saturation
 * - lastError reset in deadband: prevents derivative spikes when entering deadband
 * 
 * @param setPoint Target position in degrees
 * @param currPos Current encoder position in degrees
 * @param PIDstruct PID parameters (passed by reference to persist integral/lastError)
 * @return Voltage command (±Vcc), saturated to ±3.3V
 */
float motorDriver::computePID(float setPoint, float currPos, PID_t &PIDstruct) {
	float Kp = PIDstruct.Kp;
	float Ki = PIDstruct.Ki; 
	float Kd = PIDstruct.Kd;
	float dt = PIDstruct.dt;
	float lastErr = PIDstruct.lastError;
	float deadband = PIDstruct.deadband;
	float alpha = PIDstruct.alpha;
	float err = setPoint - currPos;

	// Deadband: ignore small errors to prevent oscillation from encoder noise
	if (abs(err) < deadband) {
		// Inside deadband: zero error and reset states to prevent spikes
		err = 0.0;
		PIDstruct.integral = 0.0;  // Clear integral windup
		PIDstruct.lastError = 0.0;  // Reset lastError to prevent derivative spike
		lastErr = 0.0;  // Update local copy too
	} else {
		// Outside deadband: shift error by deadband for smooth transition
		// Example: err=1.0°, deadband=0.4° → err=0.6° (prevents jump from 1.0→0 at deadband edge)
		if (err > deadband) {
			err = err - deadband;
		} else if (err < -deadband) {
			err = err + deadband;
		}
	}
	
	// Proportional term: P = Kp × error
	float Pout = Kp * err;
	
	// Integral term with anti-windup: I = Ki × Σ(error × dt)
	PIDstruct.integral += dt * err;
	//float maxIntegral = 2.0 / Ki; // Limit so Iout ≤ ±2V
	//if (PIDstruct.integral > maxIntegral) PIDstruct.integral = maxIntegral;
	//if (PIDstruct.integral < -maxIntegral) PIDstruct.integral = -maxIntegral;
	float Iout = Ki * PIDstruct.integral;
	
	// Derivative term with EMA low-pass filter for encoder noise rejection
	// Raw derivative calculation
	float derivative = (err - lastErr) / dt;
	
	// EMA filter: smooths LSB flickering from encoder quantization
	// alpha from struct (e.g., 0.2 = ~5 sample average, 0.1 = ~10 sample average)
	PIDstruct.filteredDerivative = alpha * derivative + (1.0 - alpha) * PIDstruct.filteredDerivative;
	
	float Dout = Kd * PIDstruct.filteredDerivative;
	
	// Total PID output
	float Vout = Pout + Iout + Dout;
	
	// Update lastError for next cycle (persists because passed by reference)
	PIDstruct.lastError = err;
	
	// Saturate output to motor voltage limits
	float Vc_threshold[2] = {(-1) * this->Vcc, this->Vcc};
	threshold(Vout, Vc_threshold);
	
	return Vout;
}




