/*----------------------------------------------------------
Description
------------------------------------------------------------ 
Author: Trieu Phat Luu
Email: tpluu2207@gmail.com
Lab of Brain Machine Interface
University of Houston
Date: 20190206
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
#ifndef __LUU_UTILS_H__
#define __LUU_UTILS_H__
// Include related libraries
#include <stdint.h>
//#include "Arduino.h" // Include header file
// Convert array or char to float values
// Functions
float byte2float(int charVal, float *range, int nbits);
int float2byte(float inputVal, float *range, int nbits);
float calibrate_Encoder(float encRawDeg, float *range, float offset);
#endif