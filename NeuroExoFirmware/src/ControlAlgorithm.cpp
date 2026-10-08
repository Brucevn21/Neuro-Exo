/*
 * ControlAlgorithm.cpp
 *
 * Torque-based assist/resist controller for the NeuroExo joint, following
 * "Overview on Control Algorithm Robot Arm" (Scenario A: current-based, no
 * torque sensor, Option A assistance). The ESCON runs in current mode, so the
 * output is a motor current and torque = Kt * I * N * eta at the joint.
 *
 * Assist (Assistive / Neutral modes):
 *   tau_required = Kp*(setpoint - angle) + Kd*(setpointVel - vel) + tau_gravity
 *   tau_robot    = assistance * tau_required
 *   The tracking part never pushes away from the final target (PDF Part 6).
 *
 * Resist:
 *   tau_robot = -resistDamping * vel + tau_gravity
 *   Opposes motion in either direction and holds the limb's weight, so the
 *   patient works against pure damping. (The PDF's "-50% x tau_required"
 *   would drive the joint away from the target on its own, so it isn't used.)
 *
 * Both modes:
 *   Above softVelocityLimit, extra damping proportional to the excess speed
 *   slows the patient down smoothly. Above hardVelocityLimit, or with
 *   sustained overcurrent, the controller latches ESTOP_FAULT and outputs 0.
 *
 * Measured current can't isolate patient effort here: in current mode the
 * ESCON makes measured current track the command. It is used for overcurrent
 * protection and reported as measuredTorqueNm for logging.
 *
 * Safe by default: calibration values start at 0 and the controller outputs
 * 0 (UNINITIALIZED) until a complete config is set.
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

static inline float signf(float value) {
    return (value > 0.0f) ? 1.0f : ((value < 0.0f) ? -1.0f : 0.0f);
}

static inline bool configReady(const ControlConfig &cfg) {
    return cfg.torqueConstantNmPerA > 0.0f &&
           cfg.gearRatio > 0.0f &&
           cfg.transmissionEfficiency > 0.0f &&
           cfg.esconFullScaleCurrentA > 0.0f &&
           cfg.kpNmPerRad > 0.0f &&
           cfg.kdNmSPerRad >= 0.0f &&
           cfg.massKg >= 0.0f &&
           cfg.centerOfMassM >= 0.0f &&
           cfg.resistDampingNmSPerRad >= 0.0f &&
           cfg.softVelocityLimitRadPerSec > 0.0f &&
           cfg.overspeedDampingNmSPerRad >= 0.0f &&
           cfg.hardVelocityLimitRadPerSec > cfg.softVelocityLimitRadPerSec &&
           cfg.maxTorqueNm > 0.0f &&
           cfg.maxCurrentA > 0.0f &&
           cfg.maxCurrentA <= cfg.continuousCurrentLimitA &&
           cfg.overcurrentTripA > 0.0f &&
           cfg.overcurrentTicks > 0;
}

} // namespace

class ArmAssistController {
public:
    void setConfig(const ControlConfig &cfg) {
        cfg_ = cfg;
        configured_ = configReady(cfg_);
        reset();
    }

    const ControlConfig &config() const { return cfg_; }
    bool configured() const { return configured_; }
    SafetyState safetyState() const { return safetyState_; }
    EstopReason estopReason() const { return estopReason_; }

    void reset() {
        safetyState_ = configured_ ? SafetyState::STANDBY : SafetyState::UNINITIALIZED;
        estopReason_ = EstopReason::None;
        hasLastSetpoint_ = false;
        lastSetpointRad_ = 0.0f;
        overcurrentCount_ = 0;
    }

    float update(const ControlInput &in, ControlOutput &out) {
        out = ControlOutput{};
        const float jointTorquePerAmp =
            cfg_.torqueConstantNmPerA * cfg_.gearRatio * cfg_.transmissionEfficiency;
        out.measuredTorqueNm = in.measuredCurrentA * jointTorquePerAmp;

        if (!configured_) {
            safetyState_ = SafetyState::UNINITIALIZED;
            return 0.0f;
        }
        if (safetyState_ == SafetyState::ESTOP_FAULT) {
            return 0.0f;
        }

        if (!isfinite(in.angleRad) || !isfinite(in.setpointRad) || !isfinite(in.targetRad) ||
            !isfinite(in.velocityRadPerSec) || !isfinite(in.measuredCurrentA) ||
            !isfinite(in.dtSec) || in.dtSec <= 0.0f) {
            return trip(EstopReason::InvalidInput);
        }

        const float vel = in.velocityRadPerSec;
        if (fabsf(vel) > cfg_.hardVelocityLimitRadPerSec) {
            return trip(EstopReason::Overspeed);
        }

        if (fabsf(in.measuredCurrentA) > cfg_.overcurrentTripA) {
            if (++overcurrentCount_ >= cfg_.overcurrentTicks) {
                return trip(EstopReason::Overcurrent);
            }
        } else {
            overcurrentCount_ = 0;
        }

        // Gravity torque on the joint in the +angle direction is m*g*L*sin(angle)
        // (0 = up), so compensation is the negative of that.
        const float gravityCompNm =
            -cfg_.massKg * GRAVITY_M_PER_S2 * cfg_.centerOfMassM * sinf(in.angleRad);

        float torqueNm = 0.0f;
        if (in.mode == AssistMode::Assist) {
            const float assistance = clampf(in.assistance, 0.0f, 1.0f);
            if (!hasLastSetpoint_) {
                lastSetpointRad_ = in.setpointRad;
                hasLastSetpoint_ = true;
            }
            const float setpointVel = (in.setpointRad - lastSetpointRad_) / in.dtSec;
            lastSetpointRad_ = in.setpointRad;

            float trackingNm = assistance * (cfg_.kpNmPerRad * (in.setpointRad - in.angleRad) +
                                             cfg_.kdNmSPerRad * (setpointVel - vel));
            // Assist only toward the final target, never away from it.
            const float targetDir = signf(in.targetRad - in.angleRad);
            if (trackingNm * targetDir < 0.0f) {
                trackingNm = 0.0f;
            }
            torqueNm = trackingNm + assistance * gravityCompNm;
        } else {
            torqueNm = -cfg_.resistDampingNmSPerRad * vel + gravityCompNm;
        }

        const float excessVel = fabsf(vel) - cfg_.softVelocityLimitRadPerSec;
        if (excessVel > 0.0f) {
            torqueNm -= cfg_.overspeedDampingNmSPerRad * excessVel * signf(vel);
        }

        if ((in.atForwardLimit && torqueNm > 0.0f) || (in.atBackwardLimit && torqueNm < 0.0f)) {
            torqueNm = 0.0f;
        }

        torqueNm = clampf(torqueNm, -cfg_.maxTorqueNm, cfg_.maxTorqueNm);

        const float currentLimitA = fminf(cfg_.maxCurrentA * clampf(in.currentLimitScale, 0.0f, 1.0f),
                                          cfg_.esconFullScaleCurrentA);
        const float currentA = clampf(torqueNm / jointTorquePerAmp, -currentLimitA, currentLimitA);

        out.commandedCurrentA = currentA;
        out.commandedTorqueNm = currentA * jointTorquePerAmp;
        safetyState_ = SafetyState::ACTIVE;
        return currentA;
    }

private:
    float trip(EstopReason reason) {
        safetyState_ = SafetyState::ESTOP_FAULT;
        estopReason_ = reason;
        return 0.0f;
    }

    ControlConfig cfg_{};
    bool configured_ = false;
    SafetyState safetyState_ = SafetyState::UNINITIALIZED;
    EstopReason estopReason_ = EstopReason::None;
    bool hasLastSetpoint_ = false;
    float lastSetpointRad_ = 0.0f;
    int overcurrentCount_ = 0;
};

} // namespace NeuroExoControl

static NeuroExoControl::ArmAssistController gArmAssistController;

float ControlAlgorithm_Update(const NeuroExoControl::ControlInput &in,
                              NeuroExoControl::ControlOutput &out) {
    return gArmAssistController.update(in, out);
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

bool ControlAlgorithm_IsConfigured() {
    return gArmAssistController.configured();
}

NeuroExoControl::SafetyState ControlAlgorithm_GetSafetyState() {
    return gArmAssistController.safetyState();
}

NeuroExoControl::EstopReason ControlAlgorithm_GetEstopReason() {
    return gArmAssistController.estopReason();
}
