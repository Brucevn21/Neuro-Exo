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
// Include related libraries
#include "luu_utils.h" // Include header file

char debugmsg[500];
// Custom Struct type
// Convert array or char to float values
float byte2float(int charVal, float *range, int nbits = 16){	
	//Convert byte value into float
	// Input: nbits resolution and range of data
	int res = (1<<nbits) -1 ;
	float outputVal;
	outputVal = charVal*(range[1] - range[0])/res + range[0];	
	return outputVal;
}

int float2byte(float inputVal, float *range, int nbits = 16){
	// Convert float into value in n bytes
	int res = (1<<nbits) - 1;
	int outputVal = res*(inputVal-range[0])/(range[1]-range[0]);
	return outputVal;
}

float calibrate_Encoder(float encRawDeg, float *range, float offset){
	// Calibrate and return encoder value.
	float degCalib = encRawDeg;		// 0-360 degs
	// Offset
	degCalib += offset + 180;
	if (degCalib > 360) degCalib -= 360;
	if (degCalib < 0) degCalib += 360;
	// Calibrate to input range.
	degCalib = (range[1]-range[0])*degCalib/360 + range[0];
	return degCalib;
}