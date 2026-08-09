#ifndef __CONTROL_ALGORITHM_H__
#define __CONTROL_ALGORITHM_H__

#include <stdbool.h>

float ControlAlgorithm_UpdateAssist(float velocityDegPerSec,
                                    float accelerationDegPerSec2,
                                    float motorCurrentA,
                                    float dtSec);

float ControlAlgorithm_UpdateSignedAssist(float velocityDegPerSec,
                                          float accelerationDegPerSec2,
                                          float motorCurrentA,
                                          float dtSec,
                                          bool forwardDirection);

void ControlAlgorithm_Reset();

#endif // __CONTROL_ALGORITHM_H__