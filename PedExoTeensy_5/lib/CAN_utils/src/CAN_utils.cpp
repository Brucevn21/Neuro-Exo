/*----------------------------------------------------------
Description
------------------------------------------------------------ 
Author: Jesus A Rodriguez Toscano
Email: jarodrigueztoscano@gmail.com
Lab of Brain Machine Interface
University of Houston
Date: 20250221
Version: 0.0.1
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
// Include related libraries
#include "CAN_utils.h" // Include header file

// Function to decode a buffer by reducing an 8-byte buffer to 1 byte 
// and combining it with 1 byte times the size of the buffer index
int decodeBuffer (uint8_t *buffIndex, int size, CAN_message_t msg){
	int valOut = 0; // Initializing output value
  
	for (int i=0; i < size; i++){ 
	   // Grabbing the byte from the message buffer at the index specified by buffIndex[i]
	   // and casting it to uint8_t to ensure it's treated as a single byte (removing unnecessary higher bits)
	   int  castedBuffer = (int)(msg.buf[buffIndex[i]] & 0xFF);
	   // Combining the extracted byte with the current value in valOut.
	   // We shift the current value of valOut 8 bits to the left (essentially making space for the next byte)
	   // and then use a bitwise OR to add the extracted byte at the least significant byte position.
	   valOut = ( valOut<<8 ) | castedBuffer ; // Combining the extracted byte with the current value in valOut.
	}
   return valOut; 
  }

uint8_t extractBit(int value, uint8_t position) {
    return (value >> (position - 1)) & 1;
}