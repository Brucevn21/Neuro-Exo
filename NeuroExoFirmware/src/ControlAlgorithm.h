#ifndef __CONTROL_ALGORITHM_H__
#define __CONTROL_ALGORITHM_H__

#include <stdbool.h>

namespace NeuroExoControl {

enum class SafetyState {
    UNINITIALIZED,
    STANDBY,
    ACTIVE,
    DEGRADED,
    ESTOP_FAULT
};

struct ControlConfig {
    // Known hardware constants.
    float torqueConstantNmPerA = 0.0404f;
    float gearRatio = 160.0f;
    float transmissionEfficiency = 0.849f;
    float continuousCurrentLimitA = 3.29f;

    // Runtime calibration values. Set to 0.0 until measured on the device.
    float kpNmPerRad = 0.0f;
    float kdNmSPerRad = 0.0f;
    float massKg = 0.0f;
    float centerOfMassM = 0.0f;
    float maxVelocityRadPerSec = 0.0f;
    float emergencyTorqueNm = 0.0f;
    float commandVoltagePerAmp = 0.0f;
    float commandVoltageOffset = 0.0f;
    float assistance = 0.0f;
    float velocityFilterAlpha = 0.0f;
    float maxCurrentA = 0.0f;
};
} // namespace NeuroExoControl

float ControlAlgorithm_UpdateAssist(float velocityDegPerSec,
                                    float accelerationDegPerSec2,
                                    float motorCurrentA,
                                    float dtSec);

float ControlAlgorithm_UpdateSignedAssist(float velocityDegPerSec,
                                          float accelerationDegPerSec2,
                                          float motorCurrentA,
                                          float dtSec,
                                          bool forwardDirection);

float ControlAlgorithm_UpdatePositionAssist(float currentAngleRad,
                                          float targetAngleRad,
                                          float angularVelocityRadPerSec,
                                          float dtSec);

void ControlAlgorithm_SetConfig(const NeuroExoControl::ControlConfig &cfg);
const NeuroExoControl::ControlConfig &ControlAlgorithm_GetConfig();
NeuroExoControl::SafetyState ControlAlgorithm_GetSafetyState();
void ControlAlgorithm_Reset();

#endif // __CONTROL_ALGORITHM_H__