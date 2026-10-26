#ifndef __BASELINE_CALIBRATION_H__
#define __BASELINE_CALIBRATION_H__

#include <stdint.h>

namespace NeuroExoControl {

// Joint torque the robot needs to move the relaxed limb + brace, recorded by
// the calibration sweep per angle bin, travel direction and speed. The joint
// isn't back-drivable, so patient effort is read as the gap between this
// baseline and the torque the robot is actually commanding.
//
// Torques are "toward travel": positive = pushing in the direction of motion.
class BaselineTable {
public:
    static constexpr float MIN_DEG = -150.0f;
    static constexpr float MAX_DEG = 80.0f;
    static constexpr float BIN_DEG = 2.0f;
    static constexpr int BINS = (int)((MAX_DEG - MIN_DEG) / BIN_DEG) + 1;
    static constexpr int SPEEDS = 2;          // slow and fast sweep passes
    static constexpr int MIN_BIN_SAMPLES = 3;
    static constexpr float MIN_COVERAGE = 0.6f;  // measured share of bins a row needs

    // Forget the stored baseline and start a new recording at these speeds.
    void begin(float slowRadPerSec, float fastRadPerSec);
    void addSample(int dirSign, int speedIdx, float angleDeg, float torqueNm);
    // Average the samples over [loDeg, hiDeg] and fill unmeasured bins.
    // Returns true (and marks the table valid) when every row has enough data.
    bool finalize(float loDeg, float hiDeg);

    bool valid() const { return valid_; }
    // Discard a finished recording (e.g. the sweep was too jerky to trust).
    void reject() { valid_ = false; }
    bool covers(float aDeg, float bDeg) const;
    float loDeg() const { return lo_; }
    float hiDeg() const { return hi_; }
    // Diagnostics: bins with enough samples in a row, and the stiction excess.
    int measuredBins(int dirSign, int speedIdx) const;
    float stictionExcessNm(int dirSign) const { return stictionExcessNm_[dirIndex(dirSign)]; }
    // Baseline torque toward travel at an angle, direction and travel speed.
    float lookup(float angleDeg, int dirSign, float speedRadPerSec) const;
    // Peak torque seen while breaking the relaxed joint free from rest (slow
    // passes); finalize() turns it into a per-direction excess over the baseline.
    void addBreakaway(int dirSign, float angleDeg, float peakTorqueNm);
    // Torque needed to start the relaxed joint moving from rest.
    float staticNeed(float angleDeg, int dirSign) const;

private:
    static int dirIndex(int dirSign) { return dirSign >= 0 ? 0 : 1; }
    static int binFor(float deg);
    float rowAt(int dir, int speedIdx, float angleDeg) const;

    float sum_[2][SPEEDS][BINS] = {};
    uint16_t count_[2][SPEEDS][BINS] = {};
    float avg_[2][SPEEDS][BINS] = {};
    float speedRadPerSec_[SPEEDS] = {};
    static constexpr int MAX_BREAKAWAYS = 4;
    float breakawayDeg_[2][MAX_BREAKAWAYS] = {};
    float breakawayNm_[2][MAX_BREAKAWAYS] = {};
    int breakawayCount_[2] = {};
    float stictionExcessNm_[2] = {};
    bool valid_ = false;
    float lo_ = 0.0f;
    float hi_ = 0.0f;
};

// Sequences the calibration passes between the start angle A and end angle B:
// A->B slow, B->A slow, A->B fast, B->A fast.
class CalibrationSweep {
public:
    static constexpr int PASSES = 4;
    static constexpr float PASS_END_TOLERANCE_DEG = 1.0f;

    void begin(float startDeg, float endDeg, float slowDegPerSec, float fastDegPerSec);
    // Advances to the next pass once the reference has reached the pass end and
    // the joint is within tolerance of it (or past it). Returns false when all
    // passes are done.
    bool update(float angleDeg, float referenceDeg);

    int pass() const { return pass_; }
    float passEndDeg() const { return (pass_ % 2 == 0) ? end_ : start_; }
    int passDir() const { return (passEndDeg() >= passStartDeg()) ? 1 : -1; }
    int passSpeedIdx() const { return pass_ < 2 ? 0 : 1; }
    float passSpeedDegPerSec() const { return speedDegPerSec_[passSpeedIdx()]; }
    float loDeg() const { return start_ < end_ ? start_ : end_; }
    float hiDeg() const { return start_ < end_ ? end_ : start_; }

private:
    float passStartDeg() const { return (pass_ % 2 == 0) ? start_ : end_; }

    float start_ = 0.0f;
    float end_ = 0.0f;
    float speedDegPerSec_[2] = {};
    int pass_ = PASSES;
};

} // namespace NeuroExoControl

#endif // __BASELINE_CALIBRATION_H__
