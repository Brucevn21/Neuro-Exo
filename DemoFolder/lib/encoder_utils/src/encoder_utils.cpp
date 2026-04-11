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
#include "encoder_utils.h" // Include header file

float EncDeg(unsigned int read){ //a
	// Return encoder value in degrees from binary of unsigned int
	return (read*360.0f)/(1<<12); 
}

float EncCalib(float *range, float offset, float degCalib){
	// Calibrate and return encoder value.
	// Offset
	degCalib += offset + 180;
	if (degCalib > 360) degCalib -= 360;
	if (degCalib < 0) degCalib += 360;
	// Calibrate to input range.
	degCalib = (range[1]-range[0])*degCalib/360 + range[0];
	return degCalib;
}