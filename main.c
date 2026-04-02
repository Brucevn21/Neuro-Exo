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


Last Updated: 4/2/2026

*/

// Related libraries
#include <Arduino.h>    // Core Teensy 4.1 / Teensyduino framework
#include <SPI.h>        // SPI bus library
#include <AS5045.h>     // AS5045 magnetic encoder library
#include <stdint.h>     // Fixed-width integer types
#include <stdbool.h>    // Boolean type and true/false constants
#include <string.h>     // String manipulation functions
#include <stdio.h>      // Standard I/O functions

// AS5045 chip select pin
#define CS_PIN 10

AS5045 encoder(CS_PIN);

// Start of main loop (SHORT)
int main(void) {

}

// Setup and initilization calls
void setup() {
    
}
