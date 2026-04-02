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

int main(void) {

}

void setup() {
    
}
