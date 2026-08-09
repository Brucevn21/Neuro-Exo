/*
 * ControlAlgorithm.cpp
 *
 * Dynamic assist controller for the NeuroExo motor arm.
 *
 * Goal:
 * - Increase commanded assist when user motion is faster or more forceful.
 * - Use velocity, acceleration, and motor current to estimate demand.
 * - Keep output smooth with low-pass filtering and slew-rate limiting.
 *
 * Notes:
 * - This source file is written to be easy to tune on embedded targets.
 * - All values are SI-like units in degrees-based kinematics for encoder workflows.
 */

#include <Arduino.h>
#include <math.h>
#include "ControlAlgorithm.h"

namespace NeuroExoControl {

struct ControlConfig {
    float minAssistVoltage;      // Minimum absolute assist command in volts.
    float maxAssistVoltage;      // Maximum absolute assist command in volts.

    float velocityRefDegPerSec;  // Velocity that maps to full normalized demand.
    float accelRefDegPerSec2;    // Acceleration that maps to full normalized demand.

    float currentOffsetA;        // Baseline current offset (sensor bias/friction).
    float currentRefA;           // Current magnitude that maps to full normalized demand.

    float weightVelocity;        // Weight of speed contribution.
    float weightAcceleration;    // Weight of acceleration contribution.
    float weightCurrent;         // Weight of user effort contribution.
    float weightInteraction;     // Weight of speed x effort interaction.

    float filterAlpha;           // EMA smoothing factor, range (0..1].
    float maxSlewRateVPerSec;    // Max command change rate in volts/second.
};

struct ControlState {
    float filteredDemand;        // Smoothed demand in [0..1].
    float commandedAssistV;      // Last commanded absolute assist voltage.
};

class ArmAssistController {
public:
    ArmAssistController()
        : cfg_{
            0.15f,   // minAssistVoltage
            2.80f,   // maxAssistVoltage
            180.0f,  // velocityRefDegPerSec
            2000.0f, // accelRefDegPerSec2
            0.10f,   // currentOffsetA
            5.00f,   // currentRefA
            0.35f,   // weightVelocity
            0.20f,   // weightAcceleration
            0.35f,   // weightCurrent
            0.10f,   // weightInteraction
            0.20f,   // filterAlpha
            8.00f    // maxSlewRateVPerSec
        },
          st_{0.0f, 0.0f} {}

    void setConfig(const ControlConfig &cfg) {
        cfg_ = cfg;
        sanitizeConfig();
    }

    const ControlConfig &config() const {
        return cfg_;
    }

    void reset() {
        st_.filteredDemand = 0.0f;
        st_.commandedAssistV = 0.0f;
    }

    float update(float velocityDegPerSec,
                 float accelerationDegPerSec2,
                 float motorCurrentA,
                 float dtSec) {
        sanitizeConfig();

        if (dtSec <= 0.0f) {
            return st_.commandedAssistV;
        }

        const float speedNorm = normalizeMagnitude(velocityDegPerSec, cfg_.velocityRefDegPerSec);
        const float accelNorm = normalizeMagnitude(accelerationDegPerSec2, cfg_.accelRefDegPerSec2);
        const float effortNorm = normalizeCurrent(motorCurrentA);

        float demand =
            cfg_.weightVelocity * speedNorm +
            cfg_.weightAcceleration * accelNorm +
            cfg_.weightCurrent * effortNorm +
            cfg_.weightInteraction * (speedNorm * effortNorm);

        demand = constrain(demand, 0.0f, 1.0f);

        // Smooth normalized demand to avoid abrupt support changes.
        st_.filteredDemand += cfg_.filterAlpha * (demand - st_.filteredDemand);
        st_.filteredDemand = constrain(st_.filteredDemand, 0.0f, 1.0f);

        float targetAssistV = cfg_.minAssistVoltage +
                              st_.filteredDemand * (cfg_.maxAssistVoltage - cfg_.minAssistVoltage);

        // Slew-rate limit the output command so the motor drive changes smoothly.
        const float maxDelta = cfg_.maxSlewRateVPerSec * dtSec;
        float delta = targetAssistV - st_.commandedAssistV;
        delta = constrain(delta, -maxDelta, maxDelta);

        st_.commandedAssistV += delta;
        st_.commandedAssistV = constrain(st_.commandedAssistV,
                                         cfg_.minAssistVoltage,
                                         cfg_.maxAssistVoltage);

        return st_.commandedAssistV;
    }

    // Returns signed voltage command that can be passed to motor.rotate(...).
    // Positive direction uses +assist, negative uses -assist.
    float updateSigned(float velocityDegPerSec,
                       float accelerationDegPerSec2,
                       float motorCurrentA,
                       float dtSec,
                       bool forwardDirection) {
        const float absAssist = update(velocityDegPerSec,
                                       accelerationDegPerSec2,
                                       motorCurrentA,
                                       dtSec);
        return forwardDirection ? absAssist : -absAssist;
    }

private:
    ControlConfig cfg_;
    ControlState st_;

    static float normalizeMagnitude(float value, float reference) {
        if (reference <= 0.0f) {
            return 0.0f;
        }
        return constrain(fabsf(value) / reference, 0.0f, 1.0f);
    }

    float normalizeCurrent(float currentA) const {
        const float magnitude = fabsf(currentA);
        const float shifted = magnitude - cfg_.currentOffsetA;
        if (shifted <= 0.0f || cfg_.currentRefA <= cfg_.currentOffsetA) {
            return 0.0f;
        }

        const float span = cfg_.currentRefA - cfg_.currentOffsetA;
        return constrain(shifted / span, 0.0f, 1.0f);
    }

    void sanitizeConfig() {
        if (cfg_.minAssistVoltage < 0.0f) cfg_.minAssistVoltage = 0.0f;
        if (cfg_.maxAssistVoltage < cfg_.minAssistVoltage) cfg_.maxAssistVoltage = cfg_.minAssistVoltage;
        if (cfg_.velocityRefDegPerSec <= 0.0f) cfg_.velocityRefDegPerSec = 1.0f;
        if (cfg_.accelRefDegPerSec2 <= 0.0f) cfg_.accelRefDegPerSec2 = 1.0f;
        if (cfg_.currentRefA <= cfg_.currentOffsetA) cfg_.currentRefA = cfg_.currentOffsetA + 0.1f;
        cfg_.filterAlpha = constrain(cfg_.filterAlpha, 0.01f, 1.0f);
        if (cfg_.maxSlewRateVPerSec <= 0.0f) cfg_.maxSlewRateVPerSec = 0.1f;
    }
};

} // namespace NeuroExoControl

/*
 * Optional C-style wrapper API.
 * You can call these from main.cpp without exposing class internals.
 */
static NeuroExoControl::ArmAssistController gArmAssistController;

float ControlAlgorithm_UpdateAssist(float velocityDegPerSec,
                                    float accelerationDegPerSec2,
                                    float motorCurrentA,
                                    float dtSec) {
    return gArmAssistController.update(velocityDegPerSec,
                                       accelerationDegPerSec2,
                                       motorCurrentA,
                                       dtSec);
}

float ControlAlgorithm_UpdateSignedAssist(float velocityDegPerSec,
                                          float accelerationDegPerSec2,
                                          float motorCurrentA,
                                          float dtSec,
                                          bool forwardDirection) {
    return gArmAssistController.updateSigned(velocityDegPerSec,
                                             accelerationDegPerSec2,
                                             motorCurrentA,
                                             dtSec,
                                             forwardDirection);
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
