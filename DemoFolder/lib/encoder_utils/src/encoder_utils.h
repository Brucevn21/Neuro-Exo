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
#ifndef __ENCODER_UTILS_H__
#define __ENCODER_UTILS_H__
// Include related libraries
#include <stdint.h>
//#include "Arduino.h" // Include header file
// Functions
float EncDeg(unsigned int read);
float EncCalib(float *range, float offset, float degCalib);
#endif