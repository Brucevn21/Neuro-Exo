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
#ifndef __CAN_UTILS_H__
#define __CAN_UTILS_H__
// Include related libraries
#include <stdint.h>
#include <FlexCAN_T4.h>  // Include the original library for CAN_message_t
// Functions
int decodeBuffer (uint8_t *buffIndex, int size, CAN_message_t msg);
uint8_t extractBit(int value, uint8_t position);
#endif