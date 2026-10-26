/*
 * ControlAlgorithm.cpp
 *
 * Torque-based assist/resist controller for the NeuroExo joint, following
 * "Overview on Control Algorithm Robot Arm" (Scenario A: current-based, no
 * torque sensor, Option A assistance). The ESCON runs in current mode, so the
 * output is a motor current and torque = Kt * I * N * eta at the joint.
 *
 * Track (Neutral mode):
 *   tau_required = Kp*(setpoint - angle) + Kd*(setpointVel - vel) + tau_gravity
 *   tau_robot    = assistance * tau_required
 *   The tracking part never pushes away from the final target (PDF Part 6).
 *   Above softVelocityLimit, extra damping proportional to the excess speed
 *   slows the joint down smoothly.
 *
 * Patient-driven modes (Assistive / Resistive) and Calibrate:
 *   The joint isn't back-drivable, so the patient can't move it and their push
 *   never shows up in the encoder at rest. The robot always does the moving:
 *   a PID holds the joint on a moving reference (plus a feedforward of the
 *   calibrated baseline), and the mode picks the reference speed. Patient
 *   effort is the PDF's Option B estimate made from the robot's own command:
 *     effort = tau_baseline(angle, direction, speed) - tau_commanded
 *   i.e. how much less torque the robot needs than with the limb relaxed.
 *   Assistive:  speed = creep(level) + effort / ((1 - level) * B), capped and
 *               slowed near the target; sustained resistance aborts the trial.
 *   Resistive:  fixed speed; the robot adds at most level * resistMaxTorque
 *               beyond the baseline, so resisting harder than that stalls it.
 *   Calibrate:  slow and fast passes to the target and back with the patient
 *               relaxed, recording the commanded torque as the baseline.
 *
 * All modes: above hardVelocityLimit, or with sustained overcurrent, the
 * controller latches ESTOP_FAULT and outputs 0.
 *
 * Measured current can't isolate patient effort: in current mode the ESCON
 * makes measured current track the command. It is used for overcurrent
 * protection and reported as measuredTorqueNm for logging.
 *
 * Safe by default: calibration values start at 0 and the controller outputs
 * 0 (UNINITIALIZED) until a complete config is set.
 */

#include <Arduino.h>
#include <math.h>
#include "ControlAlgorithm.h"
#include "BaselineCalibration.h"

namespace NeuroExoControl {

namespace {

constexpr float GRAVITY_M_PER_S2 = 9.81f;
constexpr float DEG_PER_RAD = 57.2957795f;
constexpr float TWO_PI_F = 6.2831853f;
constexpr float MIN_LEVEL = 0.1f;
constexpr float MAX_LEVEL = 0.9f;
// Effort modes: the joint counts as stuck once the reference has pulled this
// share of the leash ahead of it (speed-independent, unlike the encoder-derived
// velocity at creep speeds).
constexpr float STALL_LEAD_FRACTION = 0.8f;
// Calibration samples are taken only while the reference runs at full pass
// speed, the joint keeps within this share of the leash, and it has travelled
// at least SETTLE_TRAVEL_DEG from the pass start (not still breaking free).
constexpr float STEADY_LEAD_FRACTION = 0.5f;
constexpr float SETTLE_TRAVEL_DEG = 2.0f;
// A settled pass that jams (gets stuck again) more than this many times in the
// whole sweep is stick-slip: the recorded baseline can't be trusted.
constexpr int MAX_STICK_SLIP_EVENTS = 2;
// Breakaway torque is the largest torque before the joint has moved this far
// (two encoder counts): later torque includes the overshoot while it gets going.
constexpr float BREAKAWAY_TRAVEL_DEG = 0.18f;
// The joint running this share of the leash ahead of the reference (e.g. a
// stick-slip breakaway) drains the integral quickly so it doesn't fling the arm.
// Judged from the lead, not the velocity: at creep speeds the encoder-derived
// velocity spikes on every count.
constexpr float OVERRUN_LEAD_FRACTION = 0.3f;
constexpr float OVERRUN_BLEED_SEC = 0.02f;
// When the joint breaks free after being stuck, the reference is pulled back
// to this share of the leash ahead of it, so the joint doesn't surge to make up
// the lag the reference built while it was stuck.
constexpr float BREAKFREE_LEAD_FRACTION = 0.3f;

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
           cfg.softVelocityLimitRadPerSec > 0.0f &&
           cfg.overspeedDampingNmSPerRad >= 0.0f &&
           cfg.hardVelocityLimitRadPerSec > cfg.softVelocityLimitRadPerSec &&
           cfg.maxTorqueNm > 0.0f &&
           cfg.maxCurrentA > 0.0f &&
           cfg.maxCurrentA <= cfg.continuousCurrentLimitA &&
           cfg.overcurrentTripA > 0.0f &&
           cfg.overcurrentTicks > 0 &&
           cfg.trackKpNmPerRad > 0.0f &&
           cfg.trackKiNmPerRadS >= 0.0f &&
           cfg.trackKdNmSPerRad >= 0.0f &&
           cfg.trackLeashRad > 0.0f &&
           cfg.refAccelRadPerSec2 > 0.0f &&
           cfg.approachGainPerSec > 0.0f &&
           cfg.effortFilterHz > 0.0f &&
           cfg.effortDeadbandNm >= 0.0f &&
           cfg.creepMinRadPerSec > 0.0f &&
           cfg.creepMaxRadPerSec >= cfg.creepMinRadPerSec &&
           cfg.virtualDampingNmSPerRad > 0.0f &&
           cfg.maxAssistSpeedRadPerSec > cfg.creepMaxRadPerSec &&
           cfg.maxAssistSpeedRadPerSec < cfg.softVelocityLimitRadPerSec &&
           cfg.resistAbortNm > 0.0f &&
           cfg.resistAbortSec > 0.0f &&
           cfg.resistSpeedRadPerSec > 0.0f &&
           cfg.resistSpeedRadPerSec < cfg.softVelocityLimitRadPerSec &&
           cfg.resistMaxTorqueNm > 0.0f &&
           cfg.calSlowRadPerSec > 0.0f &&
           cfg.calFastRadPerSec > cfg.calSlowRadPerSec &&
           cfg.calFastRadPerSec < cfg.softVelocityLimitRadPerSec;
}

BaselineTable gBaseline;
CalibrationSweep gSweep;
int gStickSlipEvents = 0;   // jams during the last calibration sweep

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
        effortActive_ = false;
    }

    float update(const ControlInput &in, ControlOutput &out) {
        out = ControlOutput{};
        out.measuredTorqueNm = in.measuredCurrentA * jointTorquePerAmp();

        if (!safetyChecksPass(in)) {
            return 0.0f;
        }
        if (!isfinite(in.setpointRad) || !isfinite(in.targetRad)) {
            return trip(EstopReason::InvalidInput);
        }
        const float vel = in.velocityRadPerSec;

        // Gravity torque on the joint in the +angle direction is m*g*L*sin(angle)
        // (0 = up), so compensation is the negative of that.
        const float gravityCompNm =
            -cfg_.massKg * GRAVITY_M_PER_S2 * cfg_.centerOfMassM * sinf(in.angleRad);

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
        float torqueNm = trackingNm + assistance * gravityCompNm;

        const float excessVel = fabsf(vel) - cfg_.softVelocityLimitRadPerSec;
        if (excessVel > 0.0f) {
            torqueNm -= cfg_.overspeedDampingNmSPerRad * excessVel * signf(vel);
        }

        const float currentA = limitedCurrent(torqueNm, in);
        out.commandedCurrentA = currentA;
        out.commandedTorqueNm = currentA * jointTorquePerAmp();
        safetyState_ = SafetyState::ACTIVE;
        return currentA;
    }

    void beginEffort(AssistMode mode, float startRad, float targetRad, float level) {
        reset();
        effortActive_ = mode != AssistMode::Track;
        effortMode_ = mode;
        level_ = clampf(level, MIN_LEVEL, MAX_LEVEL);
        targetRad_ = targetRad;
        travelDir_ = targetRad >= startRad ? 1.0f : -1.0f;
        refRad_ = startRad;
        refSpeed_ = 0.0f;
        trackIntegralNm_ = 0.0f;
        effortNm_ = 0.0f;
        resistTicks_ = 0;
        lastPass_ = 0;
        passStartDeg_ = startRad * DEG_PER_RAD;
        breakawayPeakNm_ = 0.0f;
        breakawayRecorded_ = false;
        wasStuck_ = false;
        stuckLatched_ = false;
        if (mode == AssistMode::Calibrate) {
            gStickSlipEvents = 0;
            gBaseline.begin(cfg_.calSlowRadPerSec, cfg_.calFastRadPerSec);
            gSweep.begin(startRad * DEG_PER_RAD, targetRad * DEG_PER_RAD,
                         cfg_.calSlowRadPerSec * DEG_PER_RAD, cfg_.calFastRadPerSec * DEG_PER_RAD);
        }
    }

    float updateEffort(const ControlInput &in, ControlOutput &out) {
        out = ControlOutput{};
        out.measuredTorqueNm = in.measuredCurrentA * jointTorquePerAmp();
        if (!effortActive_ || !safetyChecksPass(in)) {
            return 0.0f;
        }
        const float dt = in.dtSec;
        const float angle = in.angleRad;
        const float vel = in.velocityRadPerSec;
        const float angleDeg = angle * DEG_PER_RAD;
        const bool calibrating = effortMode_ == AssistMode::Calibrate;

        // Where this motion is heading and how fast the reference wants to go.
        float endRad = targetRad_;
        float passSpeed = 0.0f;
        if (calibrating) {
            if (!gSweep.update(angleDeg, refRad_ * DEG_PER_RAD)) {
                gBaseline.finalize(gSweep.loDeg(), gSweep.hiDeg());
                if (gStickSlipEvents > MAX_STICK_SLIP_EVENTS) {
                    gBaseline.reject();
                }
                effortActive_ = false;
                out.calibrationDone = true;
                safetyState_ = SafetyState::STANDBY;
                return 0.0f;
            }
            if (gSweep.pass() != lastPass_) {
                // Reversing for the next pass: restart the reference from rest.
                lastPass_ = gSweep.pass();
                refRad_ = angle;
                refSpeed_ = 0.0f;
                trackIntegralNm_ = 0.0f;
                passStartDeg_ = angleDeg;
                breakawayPeakNm_ = 0.0f;
                breakawayRecorded_ = false;
                stuckLatched_ = false;
            }
            travelDir_ = (float)gSweep.passDir();
            endRad = gSweep.passEndDeg() / DEG_PER_RAD;
            passSpeed = gSweep.passSpeedDegPerSec() / DEG_PER_RAD;
        }
        const float dir = travelDir_;
        const int dirSign = dir > 0.0f ? 1 : -1;

        // Advance the reference, never past the end or too far ahead of the joint.
        refRad_ += dir * refSpeed_ * dt;
        if (dir * (refRad_ - endRad) > 0.0f) {
            refRad_ = endRad;
        }
        refRad_ = clampf(refRad_, angle - cfg_.trackLeashRad, angle + cfg_.trackLeashRad);
        bool stuck = dir * (refRad_ - angle) >= STALL_LEAD_FRACTION * cfg_.trackLeashRad;
        if (stuckLatched_ && !stuck) {
            // Just broke free: drop the lag so the joint doesn't race to catch up.
            const float snapRad = angle + dir * BREAKFREE_LEAD_FRACTION * cfg_.trackLeashRad;
            if (dir * (refRad_ - snapRad) > 0.0f) {
                refRad_ = snapRad;
            }
        }
        // Latched until the lead falls well below the stall threshold, so the
        // break-free moment is detected once, not on every encoder count.
        if (stuck) {
            stuckLatched_ = true;
        } else if (dir * (refRad_ - angle) <= BREAKFREE_LEAD_FRACTION * cfg_.trackLeashRad) {
            stuckLatched_ = false;
        }
        stuck = stuck || stuckLatched_;
        const float leadRad = dir * (refRad_ - angle);

        // What the relaxed limb needs right now: the moving baseline, or the
        // breakaway torque while the joint is stuck.
        const float feedforwardNm = calibrating ? 0.0f : gBaseline.lookup(angleDeg, dirSign, refSpeed_);
        const float relaxedNeedNm = calibrating ? 0.0f
                                    : (stuck ? gBaseline.staticNeed(angleDeg, dirSign) : feedforwardNm);

        // Hold the joint on the reference: baseline feedforward + PID.
        const float errorRad = refRad_ - angle;
        const float integralStep = cfg_.trackKiNmPerRadS * errorRad * dt;
        trackIntegralNm_ = clampf(trackIntegralNm_ + integralStep, -cfg_.maxTorqueNm, cfg_.maxTorqueNm);
        if (leadRad < -OVERRUN_LEAD_FRACTION * cfg_.trackLeashRad) {
            trackIntegralNm_ *= 1.0f - dt / OVERRUN_BLEED_SEC;
        }
        float torqueNm = dir * feedforwardNm + cfg_.trackKpNmPerRad * errorRad +
                         cfg_.trackKdNmSPerRad * (dir * refSpeed_ - vel) + trackIntegralNm_;

        // Resistive: push at most level x resistMaxTorque beyond what the relaxed limb needs.
        if (effortMode_ == AssistMode::PatientResist) {
            const float capNm = relaxedNeedNm + level_ * cfg_.resistMaxTorqueNm;
            if (dir * torqueNm > capNm) {
                torqueNm = dir * capNm;
                trackIntegralNm_ -= integralStep;  // don't wind up while stalled
            }
        }

        const float currentA = limitedCurrent(torqueNm, in);
        const float commandedNm = currentA * jointTorquePerAmp();
        const float travelTorqueNm = dir * commandedNm;

        // Patient effort: what the relaxed limb needs minus what the robot is
        // supplying (+ = patient helping toward the target).
        if (!calibrating) {
            const float rawEffortNm = relaxedNeedNm - travelTorqueNm;
            const float rc = 1.0f / (TWO_PI_F * cfg_.effortFilterHz);
            effortNm_ += (dt / (rc + dt)) * (rawEffortNm - effortNm_);
        }

        // Pick the reference speed for the next tick.
        const float remainingRad = fmaxf(0.0f, dir * (endRad - angle));
        const float creep = creepFor(level_);
        float desiredSpeed;
        if (effortMode_ == AssistMode::PatientAssist) {
            const float help = fmaxf(0.0f, effortNm_ - cfg_.effortDeadbandNm);
            desiredSpeed = creep + help / ((1.0f - level_) * cfg_.virtualDampingNmSPerRad);
            desiredSpeed = fminf(desiredSpeed, cfg_.maxAssistSpeedRadPerSec);
            desiredSpeed = fminf(desiredSpeed, fmaxf(creep, cfg_.approachGainPerSec * remainingRad));

            if (effortNm_ < -cfg_.resistAbortNm) {
                if (++resistTicks_ * dt >= cfg_.resistAbortSec) {
                    out.patientResisting = true;
                }
            } else {
                resistTicks_ = 0;
            }
        } else {
            const float travelSpeed = calibrating ? passSpeed : cfg_.resistSpeedRadPerSec;
            desiredSpeed = fminf(travelSpeed, fmaxf(cfg_.creepMinRadPerSec, cfg_.approachGainPerSec * remainingRad));
        }
        if (stuck) {
            // Don't speed up until the joint is actually moving again.
            desiredSpeed = fminf(desiredSpeed, refSpeed_);
        }
        const float maxStep = cfg_.refAccelRadPerSec2 * dt;
        refSpeed_ += clampf(desiredSpeed - refSpeed_, -maxStep, maxStep);

        if (calibrating) {
            const bool settled = dir * (angleDeg - passStartDeg_) >= SETTLE_TRAVEL_DEG;
            if (settled && stuck && !wasStuck_) {
                gStickSlipEvents++;
            }
            wasStuck_ = stuck;
            // Record the torque needed only while keeping pace at full pass speed.
            if (settled && refSpeed_ >= 0.95f * passSpeed &&
                fabsf(leadRad) < STEADY_LEAD_FRACTION * cfg_.trackLeashRad) {
                gBaseline.addSample(dirSign, gSweep.passSpeedIdx(), angleDeg, travelTorqueNm);
            }
            // Slow passes: the peak torque before the joint gets going is the breakaway.
            if (gSweep.passSpeedIdx() == 0 && !breakawayRecorded_) {
                breakawayPeakNm_ = fmaxf(breakawayPeakNm_, travelTorqueNm);
                if (dir * (angleDeg - passStartDeg_) >= BREAKAWAY_TRAVEL_DEG) {
                    gBaseline.addBreakaway(dirSign, passStartDeg_, breakawayPeakNm_);
                    breakawayRecorded_ = true;
                }
            }
        }

        out.commandedCurrentA = currentA;
        out.commandedTorqueNm = commandedNm;
        out.patientEffortNm = calibrating ? 0.0f : effortNm_;
        out.referenceRad = refRad_;
        safetyState_ = SafetyState::ACTIVE;
        return currentA;
    }

    float creepFor(float level) const {
        const float t = (clampf(level, MIN_LEVEL, MAX_LEVEL) - MIN_LEVEL) / (MAX_LEVEL - MIN_LEVEL);
        return cfg_.creepMinRadPerSec + t * (cfg_.creepMaxRadPerSec - cfg_.creepMinRadPerSec);
    }

private:
    float jointTorquePerAmp() const {
        return cfg_.torqueConstantNmPerA * cfg_.gearRatio * cfg_.transmissionEfficiency;
    }

    // Config, e-stop, input-validity, over-speed and over-current checks shared
    // by every mode. Returns false when the caller must output 0.
    bool safetyChecksPass(const ControlInput &in) {
        if (!configured_) {
            safetyState_ = SafetyState::UNINITIALIZED;
            return false;
        }
        if (safetyState_ == SafetyState::ESTOP_FAULT) {
            return false;
        }
        if (!isfinite(in.angleRad) || !isfinite(in.velocityRadPerSec) ||
            !isfinite(in.measuredCurrentA) || !isfinite(in.dtSec) || in.dtSec <= 0.0f) {
            trip(EstopReason::InvalidInput);
            return false;
        }
        if (fabsf(in.velocityRadPerSec) > cfg_.hardVelocityLimitRadPerSec) {
            trip(EstopReason::Overspeed);
            return false;
        }
        if (fabsf(in.measuredCurrentA) > cfg_.overcurrentTripA) {
            if (++overcurrentCount_ >= cfg_.overcurrentTicks) {
                trip(EstopReason::Overcurrent);
                return false;
            }
        } else {
            overcurrentCount_ = 0;
        }
        return true;
    }

    // Joint-limit, torque and current limits shared by every mode.
    float limitedCurrent(float torqueNm, const ControlInput &in) const {
        if ((in.atForwardLimit && torqueNm > 0.0f) || (in.atBackwardLimit && torqueNm < 0.0f)) {
            torqueNm = 0.0f;
        }
        torqueNm = clampf(torqueNm, -cfg_.maxTorqueNm, cfg_.maxTorqueNm);
        const float currentLimitA = fminf(cfg_.maxCurrentA * clampf(in.currentLimitScale, 0.0f, 1.0f),
                                          cfg_.esconFullScaleCurrentA);
        return clampf(torqueNm / jointTorquePerAmp(), -currentLimitA, currentLimitA);
    }

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

    // Effort-mode state, set by beginEffort().
    bool effortActive_ = false;
    AssistMode effortMode_ = AssistMode::Track;
    float level_ = MIN_LEVEL;
    float targetRad_ = 0.0f;
    float travelDir_ = 1.0f;
    float refRad_ = 0.0f;
    float refSpeed_ = 0.0f;           // magnitude, toward travel
    float trackIntegralNm_ = 0.0f;
    float effortNm_ = 0.0f;
    int resistTicks_ = 0;
    int lastPass_ = 0;
    float passStartDeg_ = 0.0f;
    float breakawayPeakNm_ = 0.0f;
    bool breakawayRecorded_ = false;
    bool wasStuck_ = false;
    bool stuckLatched_ = false;
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

void ControlAlgorithm_BeginEffortTrial(NeuroExoControl::AssistMode mode, float startRad,
                                       float targetRad, float level) {
    gArmAssistController.beginEffort(mode, startRad, targetRad, level);
}

float ControlAlgorithm_UpdateEffort(const NeuroExoControl::ControlInput &in,
                                    NeuroExoControl::ControlOutput &out) {
    return gArmAssistController.updateEffort(in, out);
}

float ControlAlgorithm_MinTrialSpeedRadPerSec(NeuroExoControl::AssistMode mode, float level) {
    const NeuroExoControl::ControlConfig &cfg = gArmAssistController.config();
    switch (mode) {
        case NeuroExoControl::AssistMode::PatientAssist: return gArmAssistController.creepFor(level);
        case NeuroExoControl::AssistMode::PatientResist: return cfg.resistSpeedRadPerSec;
        case NeuroExoControl::AssistMode::Calibrate:     return cfg.calSlowRadPerSec;
        default:                                         return cfg.creepMinRadPerSec;
    }
}

bool ControlAlgorithm_BaselineValid() {
    return NeuroExoControl::gBaseline.valid();
}

bool ControlAlgorithm_BaselineCovers(float aRad, float bRad) {
    return NeuroExoControl::gBaseline.covers(aRad * NeuroExoControl::DEG_PER_RAD,
                                             bRad * NeuroExoControl::DEG_PER_RAD);
}

void ControlAlgorithm_PrintBaselineSummary() {
    const NeuroExoControl::BaselineTable &b = NeuroExoControl::gBaseline;
    Serial.print("  range ");
    Serial.print(b.loDeg(), 1);
    Serial.print(" to ");
    Serial.print(b.hiDeg(), 1);
    Serial.print(" deg | measured 2-deg bins +slow/+fast/-slow/-fast: ");
    Serial.print(b.measuredBins(1, 0));
    Serial.print('/');
    Serial.print(b.measuredBins(1, 1));
    Serial.print('/');
    Serial.print(b.measuredBins(-1, 0));
    Serial.print('/');
    Serial.println(b.measuredBins(-1, 1));
    Serial.print("  breakaway excess over moving torque: +dir ");
    Serial.print(b.stictionExcessNm(1), 2);
    Serial.print(" N*m, -dir ");
    Serial.print(b.stictionExcessNm(-1), 2);
    Serial.println(" N*m (large values mean stick-slip lurches)");
    Serial.print("  jams mid-pass: ");
    Serial.print(NeuroExoControl::gStickSlipEvents);
    Serial.print(NeuroExoControl::gStickSlipEvents > NeuroExoControl::MAX_STICK_SLIP_EVENTS
                     ? " - too jerky, calibration rejected (stick-slip)" : "");
    Serial.println();
}
