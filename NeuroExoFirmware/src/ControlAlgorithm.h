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

// Assistive/Neutral: robot supplies `assistance` x (PD + gravity) torque
// toward the A->B trajectory, never pushing away from the target.
// Resistive: robot opposes the patient's motion with viscous damping.
enum class AssistMode {
    Assist,
    Resist
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

    // Resistive mode: opposing torque per unit joint velocity.
    float resistDampingNmSPerRad = 0.0f;

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
};

struct ControlInput {
    float angleRad;           // AS5045, 0 = joint pointing up.
    float setpointRad;        // Interpolated A->B setpoint.
    float targetRad;          // Final target B (for the direction check).
    float velocityRadPerSec;  // Already low-pass filtered.
    float measuredCurrentA;   // ESCON analog monitor, signed.
    float dtSec;
    AssistMode mode;
    float assistance;         // 0..1 fraction of required torque (Assist only).
    float currentLimitScale;  // 0..1 extra scaling of maxCurrentA (e.g. speed setting).
    bool  atForwardLimit;     // Joint at/over the forward limit: no forward torque.
    bool  atBackwardLimit;    // Joint at/over the backward limit: no backward torque.
};

struct ControlOutput {
    float commandedCurrentA = 0.0f;  // Signed; + = forward (increasing angle).
    float commandedTorqueNm = 0.0f;  // Joint torque requested from the robot.
    float measuredTorqueNm = 0.0f;   // Joint torque estimated from measured current.
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

#endif // __CONTROL_ALGORITHM_H__
