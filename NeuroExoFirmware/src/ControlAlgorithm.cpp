/*
 * ControlAlgorithm.cpp
 *
 * Position-based assist controller for the NeuroExo motor arm.
 *
 * This implementation is deliberately safe-by-default:
 * - Known motor hardware values are populated.
 * - Calibration parameters are initialized to zero and remain zero until
 *   measured/validated on the hardware.
 * - The controller outputs zero while configuration is incomplete.
 */

#include <Arduino.h>
#include <math.h>
#include "ControlAlgorithm.h"

namespace NeuroExoControl {

namespace {

constexpr float GRAVITY_M_PER_S2 = 9.81f;

static inline float clampf(float value, float minValue, float maxValue) {
    if (value < minValue) return minValue;
    if (value > maxValue) return maxValue;
    return value;
}

static inline bool isValidFloat(float value) {
    return isfinite(value);
}

static inline bool configReady(const ControlConfig &cfg) {
    return cfg.kpNmPerRad > 0.0f &&
           cfg.kdNmSPerRad > 0.0f &&
           cfg.massKg > 0.0f &&
           cfg.centerOfMassM > 0.0f &&
           cfg.maxVelocityRadPerSec > 0.0f &&
           cfg.emergencyTorqueNm > 0.0f &&
           cfg.assistance > 0.0f &&
           cfg.velocityFilterAlpha > 0.0f &&
           cfg.velocityFilterAlpha <= 1.0f &&
           cfg.commandVoltagePerAmp > 0.0f;
}

} // namespace

struct ControlState {
    SafetyState safetyState = SafetyState::UNINITIALIZED;
    float filteredVelocityRadPerSec = 0.0f;
    float previousAngleRad = 0.0f;
    float commandedCurrentA = 0.0f;
    float commandedVoltageV = 0.0f;
};

class ArmAssistController {
public:
    ArmAssistController() {
        cfg_ = ControlConfig{};
        state_ = ControlState{};
    }

    void setConfig(const ControlConfig &cfg) {
        cfg_ = cfg;
        if (!configReady(cfg_)) {
            state_.safetyState = SafetyState::UNINITIALIZED;
            state_.commandedCurrentA = 0.0f;
            state_.commandedVoltageV = 0.0f;
            return;
        }
        if (cfg_.maxCurrentA <= 0.0f) {
            cfg_.maxCurrentA = cfg_.continuousCurrentLimitA;
        }
        if (cfg_.commandVoltagePerAmp <= 0.0f) {
            cfg_.commandVoltagePerAmp = 1.0f;
            cfg_.commandVoltageOffset = 0.0f;
        }
        state_.safetyState = SafetyState::STANDBY;
    }

    const ControlConfig &config() const {
        return cfg_;
    }

    SafetyState safetyState() const {
        return state_.safetyState;
    }

    void reset() {
        state_.safetyState = SafetyState::UNINITIALIZED;
        state_.filteredVelocityRadPerSec = 0.0f;
        state_.previousAngleRad = 0.0f;
        state_.commandedCurrentA = 0.0f;
        state_.commandedVoltageV = 0.0f;
    }

    float updateLegacy(float velocityDegPerSec,
                       float accelerationDegPerSec2,
                       float motorCurrentA,
                       float dtSec) {
        if (!isValidFloat(velocityDegPerSec) || !isValidFloat(accelerationDegPerSec2) ||
            !isValidFloat(motorCurrentA) || !isValidFloat(dtSec) || dtSec <= 0.0f) {
            return 0.0f;
        }

        if (!configReady(cfg_)) {
            return 0.0f;
        }

        // Legacy API is intentionally kept for compatibility. It is not used for
        // the PDF position controller while calibration values remain zero.
        (void)accelerationDegPerSec2;
        (void)motorCurrentA;
        (void)velocityDegPerSec;
        return state_.commandedVoltageV;
    }

    float updateSignedLegacy(float velocityDegPerSec,
                             float accelerationDegPerSec2,
                             float motorCurrentA,
                             float dtSec,
                             bool forwardDirection) {
        const float signedAssist = updateLegacy(velocityDegPerSec,
                                              accelerationDegPerSec2,
                                              motorCurrentA,
                                              dtSec);
        return forwardDirection ? signedAssist : -signedAssist;
    }

    float updatePosition(float currentAngleRad,
                         float targetAngleRad,
                         float angularVelocityRadPerSec,
                         float dtSec) {
        if (!isValidFloat(currentAngleRad) || !isValidFloat(targetAngleRad) ||
            !isValidFloat(angularVelocityRadPerSec) || !isValidFloat(dtSec) || dtSec <= 0.0f) {
            state_.safetyState = SafetyState::DEGRADED;
            state_.commandedCurrentA = 0.0f;
            state_.commandedVoltageV = 0.0f;
            return 0.0f;
        }

        if (!configReady(cfg_)) {
            state_.safetyState = SafetyState::UNINITIALIZED;
            state_.commandedCurrentA = 0.0f;
            state_.commandedVoltageV = 0.0f;
            return 0.0f;
        }

        const float positionErrorRad = targetAngleRad - currentAngleRad;
        state_.filteredVelocityRadPerSec =
            cfg_.velocityFilterAlpha * angularVelocityRadPerSec +
            (1.0f - cfg_.velocityFilterAlpha) * state_.filteredVelocityRadPerSec;

        const float gravityTorqueNm =
            cfg_.massKg * GRAVITY_M_PER_S2 * cfg_.centerOfMassM * sinf(currentAngleRad);
        const float requiredTorqueNm =
            cfg_.kpNmPerRad * positionErrorRad -
            cfg_.kdNmSPerRad * state_.filteredVelocityRadPerSec +
            gravityTorqueNm;

        const float jointTorquePerAmp =
            cfg_.torqueConstantNmPerA * cfg_.gearRatio * cfg_.transmissionEfficiency;
        const float commandedTorqueNm = cfg_.assistance * requiredTorqueNm;
        const float limitedTorqueNm = clampf(commandedTorqueNm,
                                           -cfg_.emergencyTorqueNm,
                                           cfg_.emergencyTorqueNm);

        const float currentLimitA = cfg_.maxCurrentA > 0.0f ? cfg_.maxCurrentA : cfg_.continuousCurrentLimitA;
        const float commandedCurrentA = clampf(limitedTorqueNm / jointTorquePerAmp,
                                              -currentLimitA,
                                              currentLimitA);

        if (fabsf(state_.filteredVelocityRadPerSec) > cfg_.maxVelocityRadPerSec) {
            state_.safetyState = SafetyState::ESTOP_FAULT;
            state_.commandedCurrentA = 0.0f;
            state_.commandedVoltageV = 0.0f;
            return 0.0f;
        }

        if (fabsf(limitedTorqueNm) > cfg_.emergencyTorqueNm) {
            state_.safetyState = SafetyState::ESTOP_FAULT;
            state_.commandedCurrentA = 0.0f;
            state_.commandedVoltageV = 0.0f;
            return 0.0f;
        }

        state_.commandedCurrentA = commandedCurrentA;
        state_.commandedVoltageV = cfg_.commandVoltageOffset +
                                 state_.commandedCurrentA * cfg_.commandVoltagePerAmp;
        state_.safetyState = SafetyState::ACTIVE;
        return state_.commandedVoltageV;
    }

private:
    ControlConfig cfg_;
    ControlState state_;
};

} // namespace NeuroExoControl

static NeuroExoControl::ArmAssistController gArmAssistController;

float ControlAlgorithm_UpdateAssist(float velocityDegPerSec,
                                    float accelerationDegPerSec2,
                                    float motorCurrentA,
                                    float dtSec) {
    return gArmAssistController.updateLegacy(velocityDegPerSec,
                                           accelerationDegPerSec2,
                                           motorCurrentA,
                                           dtSec);
}

float ControlAlgorithm_UpdateSignedAssist(float velocityDegPerSec,
                                          float accelerationDegPerSec2,
                                          float motorCurrentA,
                                          float dtSec,
                                          bool forwardDirection) {
    return gArmAssistController.updateSignedLegacy(velocityDegPerSec,
                                                 accelerationDegPerSec2,
                                                 motorCurrentA,
                                                 dtSec,
                                                 forwardDirection);
}

float ControlAlgorithm_UpdatePositionAssist(float currentAngleRad,
                                          float targetAngleRad,
                                          float angularVelocityRadPerSec,
                                          float dtSec) {
    return gArmAssistController.updatePosition(currentAngleRad,
                                             targetAngleRad,
                                             angularVelocityRadPerSec,
                                             dtSec);
}

void ControlAlgorithm_Reset() {
    gArmAssistController.reset();
}

void ControlAlgorithm_SetConfig(const NeuroExoControl::ControlConfig &cfg) {
    gArmAssistController.setConfig(cfg);
}

const NeuroExoControl::ControlConfig &ControlAlgorithm_GetConfig() {
    return gArmAssistController.config();
}

NeuroExoControl::SafetyState ControlAlgorithm_GetSafetyState() {
    return gArmAssistController.safetyState();
}
