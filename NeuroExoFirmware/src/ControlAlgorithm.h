#ifndef __CONTROL_ALGORITHM_H__
#define __CONTROL_ALGORITHM_H__

#include <stdbool.h>

namespace NeuroExoControl {

enum class SafetyState {
    UNINITIALIZED,  // Config incomplete; output forced to 0.
    STANDBY,        // Configured, not yet driving.
    ACTIVE,         // Driving normally.
    ESTOP_FAULT     // Latched until ControlAlgorithm_Reset().
};

enum class EstopReason {
    None,
    InvalidInput,
    Overspeed,
    Overcurrent
};

// Track (Neutral): robot supplies `assistance` x (PD + gravity) torque toward
// the timed A->B trajectory, never pushing away from the target.
//
// The joint isn't back-drivable, so in the patient-driven modes the robot
// always does the moving: it follows a moving reference whose speed the mode
// sets, and reads patient effort as the relaxed baseline torque (from the
// calibration sweep) minus the torque it is commanding.
//   PatientAssist (Assistive): speed = creep + effort / ((1 - level) * B), so
//     the same push moves the arm faster at higher assistance levels.
//   PatientResist (Resistive): robot moves at a fixed speed while the patient
//     pushes against it, adding at most level * resistMaxTorque beyond the
//     baseline; resisting harder than that stalls the arm.
//   Calibrate: constant-speed sweep that records the baseline table.
enum class AssistMode {
    Track,
    PatientAssist,
    PatientResist,
    Calibrate
};

struct ControlConfig {
    // Hardware constants (motor/gearbox datasheets).
    float torqueConstantNmPerA = 0.0404f;
    float gearRatio = 160.0f;
    float transmissionEfficiency = 0.849f;
    float continuousCurrentLimitA = 3.29f;

    // ESCON Studio: motor current commanded at 90 % PWM duty (10 % = 0 A).
    // Must be set from the ESCON configuration; 0 keeps the controller off.
    float esconFullScaleCurrentA = 0.0f;

    // Position tracking (PDF Part 3).
    float kpNmPerRad = 0.0f;
    float kdNmSPerRad = 0.0f;

    // Gravity compensation; leave mass at 0 to disable.
    float massKg = 0.0f;
    float centerOfMassM = 0.0f;

    // Over-speed: smooth damping above the soft limit, latched e-stop above
    // the hard limit.
    float softVelocityLimitRadPerSec = 0.0f;
    float overspeedDampingNmSPerRad = 0.0f;
    float hardVelocityLimitRadPerSec = 0.0f;

    // Output limits.
    float maxTorqueNm = 0.0f;
    float maxCurrentA = 0.0f;
    // Measured current above this for overcurrentTicks consecutive updates
    // latches an e-stop.
    float overcurrentTripA = 0.0f;
    int overcurrentTicks = 25;

    // ---- Patient-driven modes (Assistive / Resistive / Calibrate) ----
    // PID holding the joint on the moving reference.
    float trackKpNmPerRad = 0.0f;
    float trackKiNmPerRadS = 0.0f;
    float trackKdNmSPerRad = 0.0f;
    float trackLeashRad = 0.0f;              // max reference lead over the joint
    float refAccelRadPerSec2 = 0.0f;         // reference speed rate limit
    float approachGainPerSec = 0.0f;         // speed <= gain x remaining distance near the end
    float effortFilterHz = 0.0f;             // low-pass on the patient effort estimate
    float effortDeadbandNm = 0.0f;           // effort below this doesn't change speed
    // Assistive: speed = creep + effort / ((1 - level) * virtualDamping).
    float creepMinRadPerSec = 0.0f;          // creep at level 1 (10 %)
    float creepMaxRadPerSec = 0.0f;          // creep at level 9 (90 %)
    float virtualDampingNmSPerRad = 0.0f;
    float maxAssistSpeedRadPerSec = 0.0f;
    float resistAbortNm = 0.0f;              // effort below -resistAbortNm ...
    float resistAbortSec = 0.0f;             // ... for this long aborts an Assistive trial
    // Resistive: fixed speed, robot adds at most level x resistMaxTorque beyond the baseline.
    float resistSpeedRadPerSec = 0.0f;
    float resistMaxTorqueNm = 0.0f;
    // Calibration sweep speeds.
    float calSlowRadPerSec = 0.0f;
    float calFastRadPerSec = 0.0f;
};

struct ControlInput {
    float angleRad;           // AS5045, 0 = joint pointing up.
    float setpointRad;        // Interpolated A->B setpoint.
    float targetRad;          // Final target B (for the direction check).
    float velocityRadPerSec;  // Already low-pass filtered.
    float measuredCurrentA;   // ESCON analog monitor, signed.
    float dtSec;
    AssistMode mode;          // Track only; effort modes are set by BeginEffortTrial.
    float assistance;         // 0..1 fraction of required torque (Track only).
    float currentLimitScale;  // 0..1 extra scaling of maxCurrentA (e.g. speed setting).
    bool  atForwardLimit;     // Joint at/over the forward limit: no forward torque.
    bool  atBackwardLimit;    // Joint at/over the backward limit: no backward torque.
};

struct ControlOutput {
    float commandedCurrentA = 0.0f;  // Signed; + = forward (increasing angle).
    float commandedTorqueNm = 0.0f;  // Joint torque requested from the robot.
    float measuredTorqueNm = 0.0f;   // Joint torque estimated from measured current.
    // Effort modes only:
    float patientEffortNm = 0.0f;    // + helping toward the target, - resisting.
    float referenceRad = 0.0f;       // Moving reference the joint is held on.
    bool patientResisting = false;   // Assistive: sustained resistance, end the trial.
    bool calibrationDone = false;    // Calibrate: sweep finished (see BaselineValid).
};

} // namespace NeuroExoControl

// Runs one control step. Returns the signed motor current to command; 0 when
// unconfigured or faulted (check ControlAlgorithm_GetSafetyState()).
float ControlAlgorithm_Update(const NeuroExoControl::ControlInput &in,
                              NeuroExoControl::ControlOutput &out);

void ControlAlgorithm_SetConfig(const NeuroExoControl::ControlConfig &cfg);
const NeuroExoControl::ControlConfig &ControlAlgorithm_GetConfig();
bool ControlAlgorithm_IsConfigured();
NeuroExoControl::SafetyState ControlAlgorithm_GetSafetyState();
NeuroExoControl::EstopReason ControlAlgorithm_GetEstopReason();
// Clears state and any latched e-stop. Call between motions.
void ControlAlgorithm_Reset();

// Patient-driven trials and the calibration sweep. Begin resets the controller
// (Calibrate also clears the stored baseline); then call UpdateEffort every
// control tick instead of ControlAlgorithm_Update. level is 0.1..0.9.
void ControlAlgorithm_BeginEffortTrial(NeuroExoControl::AssistMode mode, float startRad,
                                       float targetRad, float level);
float ControlAlgorithm_UpdateEffort(const NeuroExoControl::ControlInput &in,
                                    NeuroExoControl::ControlOutput &out);
// Assistive/Resistive creep or travel speed for a level, for trial time limits.
float ControlAlgorithm_MinTrialSpeedRadPerSec(NeuroExoControl::AssistMode mode, float level);
bool ControlAlgorithm_BaselineValid();
bool ControlAlgorithm_BaselineCovers(float aRad, float bRad);
// Prints the stored baseline's range, coverage and stiction to Serial.
void ControlAlgorithm_PrintBaselineSummary();

#endif // __CONTROL_ALGORITHM_H__
