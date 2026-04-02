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
#if defined(__MK20DX256__) //Teensy 3.2 conditional compilation
    #include "pinMap.h"         // Pin map for Teensy 3.2
    #define BOARD_NAME "Teensy 3.2"

#elif defined(__IMXRT1062__) //Teensy 4.1 conditional compilation
    #include "pinMap4.1.h"      // Pin map for Teensy 4.1
    #define BOARD_NAME "Teensy 4.1"

#else
    #error "Unsupported board. Please compile for Teensy 3.2 or Teensy 4.1."
#endif

#include <stdint.h>             // Fixed-width types used by drivers below
#include <stdbool.h>            // Boolean support for C
#include <string.h>             // String manipulation functions (memcpy, strlen, strcmp, etc.)
#include <stdio.h>              // Standard I/O functions (printf, scanf, fprintf, etc.)

#include "AS5045.h"             // Use only if library exposes a C interface; otherwise wrap in extern
#include "motorDriver.h"        // Use only if library exposes a C interface; otherwise wrap in extern
#include "luu_utils.h"          // Personal utility library — ensure functions are declared as C linkage
#include "encoder_utils.h"      // Personal encoder utility library — ensure functions are declared as C linkage

// AS5045 chip select pin
#define CS_PIN 10

AS5045 encoder(CS_PIN);

// Start of main loop (SHORT)
int main(void) {

}

// Setup and initilization calls
void setup() {
    
}
