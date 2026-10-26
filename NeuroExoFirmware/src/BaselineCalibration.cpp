#include "BaselineCalibration.h"

#include <math.h>

namespace NeuroExoControl {

namespace {

inline float clampf(float value, float minValue, float maxValue) {
    if (value < minValue) return minValue;
    if (value > maxValue) return maxValue;
    return value;
}

} // namespace

void BaselineTable::begin(float slowRadPerSec, float fastRadPerSec) {
    for (int d = 0; d < 2; ++d) {
        for (int s = 0; s < SPEEDS; ++s) {
            for (int b = 0; b < BINS; ++b) {
                sum_[d][s][b] = 0.0f;
                count_[d][s][b] = 0;
                avg_[d][s][b] = 0.0f;
            }
        }
    }
    speedRadPerSec_[0] = slowRadPerSec;
    speedRadPerSec_[1] = fastRadPerSec;
    for (int d = 0; d < 2; ++d) {
        breakawayCount_[d] = 0;
        stictionExcessNm_[d] = 0.0f;
    }
    valid_ = false;
    lo_ = 0.0f;
    hi_ = 0.0f;
}

int BaselineTable::binFor(float deg) {
    const int bin = (int)lroundf((deg - MIN_DEG) / BIN_DEG);
    return bin < 0 ? 0 : (bin >= BINS ? BINS - 1 : bin);
}

void BaselineTable::addSample(int dirSign, int speedIdx, float angleDeg, float torqueNm) {
    if (speedIdx < 0 || speedIdx >= SPEEDS || !isfinite(torqueNm)) {
        return;
    }
    const int d = dirIndex(dirSign);
    const int b = binFor(angleDeg);
    if (count_[d][speedIdx][b] < UINT16_MAX) {
        sum_[d][speedIdx][b] += torqueNm;
        count_[d][speedIdx][b]++;
    }
}

bool BaselineTable::finalize(float loDeg, float hiDeg) {
    const int first = binFor(loDeg);
    const int last = binFor(hiDeg);
    bool ok = last > first;

    for (int d = 0; d < 2 && ok; ++d) {
        for (int s = 0; s < SPEEDS && ok; ++s) {
            bool measured[BINS] = {};
            int measuredCount = 0;
            for (int b = first; b <= last; ++b) {
                if (count_[d][s][b] >= MIN_BIN_SAMPLES) {
                    avg_[d][s][b] = sum_[d][s][b] / count_[d][s][b];
                    measured[b] = true;
                    measuredCount++;
                }
            }
            if (measuredCount < MIN_COVERAGE * (last - first + 1)) {
                ok = false;
                break;
            }
            // Fill gaps by linear interpolation; hold the nearest value at the ends.
            for (int b = first; b <= last; ++b) {
                if (measured[b]) continue;
                int lower = b - 1;
                while (lower >= first && !measured[lower]) lower--;
                int upper = b + 1;
                while (upper <= last && !measured[upper]) upper++;
                if (lower < first) {
                    avg_[d][s][b] = avg_[d][s][upper];
                } else if (upper > last) {
                    avg_[d][s][b] = avg_[d][s][lower];
                } else {
                    const float t = (float)(b - lower) / (float)(upper - lower);
                    avg_[d][s][b] = avg_[d][s][lower] + t * (avg_[d][s][upper] - avg_[d][s][lower]);
                }
            }
        }
    }

    lo_ = loDeg;
    hi_ = hiDeg;
    valid_ = ok;
    // Stiction: how much more than the slow-pass baseline it took to start moving.
    for (int d = 0; d < 2 && ok; ++d) {
        float excess = 0.0f;
        for (int i = 0; i < breakawayCount_[d]; ++i) {
            excess = fmaxf(excess, breakawayNm_[d][i] - rowAt(d, 0, breakawayDeg_[d][i]));
        }
        stictionExcessNm_[d] = excess;
    }
    return ok;
}

int BaselineTable::measuredBins(int dirSign, int speedIdx) const {
    if (speedIdx < 0 || speedIdx >= SPEEDS) {
        return 0;
    }
    int n = 0;
    for (int b = 0; b < BINS; ++b) {
        if (count_[dirIndex(dirSign)][speedIdx][b] >= MIN_BIN_SAMPLES) n++;
    }
    return n;
}

void BaselineTable::addBreakaway(int dirSign, float angleDeg, float peakTorqueNm) {
    const int d = dirIndex(dirSign);
    if (breakawayCount_[d] < MAX_BREAKAWAYS && isfinite(peakTorqueNm)) {
        breakawayDeg_[d][breakawayCount_[d]] = angleDeg;
        breakawayNm_[d][breakawayCount_[d]] = peakTorqueNm;
        breakawayCount_[d]++;
    }
}

float BaselineTable::staticNeed(float angleDeg, int dirSign) const {
    if (!valid_) {
        return 0.0f;
    }
    const int d = dirIndex(dirSign);
    return rowAt(d, 0, angleDeg) + stictionExcessNm_[d];
}

bool BaselineTable::covers(float aDeg, float bDeg) const {
    const float slack = BIN_DEG;
    const float lo = aDeg < bDeg ? aDeg : bDeg;
    const float hi = aDeg < bDeg ? bDeg : aDeg;
    return valid_ && lo >= lo_ - slack && hi <= hi_ + slack;
}

float BaselineTable::rowAt(int dir, int speedIdx, float angleDeg) const {
    const float pos = (clampf(angleDeg, lo_, hi_) - MIN_DEG) / BIN_DEG;
    const int b0 = (int)floorf(pos);
    const int b1 = b0 + 1 < binFor(hi_) ? b0 + 1 : binFor(hi_);
    const float t = clampf(pos - b0, 0.0f, 1.0f);
    const int lower = b0 < binFor(lo_) ? binFor(lo_) : b0;
    return avg_[dir][speedIdx][lower] + t * (avg_[dir][speedIdx][b1] - avg_[dir][speedIdx][lower]);
}

float BaselineTable::lookup(float angleDeg, int dirSign, float speedRadPerSec) const {
    if (!valid_) {
        return 0.0f;
    }
    const int d = dirIndex(dirSign);
    const float slow = rowAt(d, 0, angleDeg);
    const float fast = rowAt(d, 1, angleDeg);
    // Interpolate between the two sweep speeds; outside them hold the nearest
    // value (above the fast speed this underestimates the load, which makes
    // patient effort read low - the safe direction).
    const float span = speedRadPerSec_[1] - speedRadPerSec_[0];
    const float t = span > 0.0f ? clampf((speedRadPerSec - speedRadPerSec_[0]) / span, 0.0f, 1.0f) : 0.0f;
    return slow + t * (fast - slow);
}

void CalibrationSweep::begin(float startDeg, float endDeg, float slowDegPerSec, float fastDegPerSec) {
    start_ = startDeg;
    end_ = endDeg;
    speedDegPerSec_[0] = slowDegPerSec;
    speedDegPerSec_[1] = fastDegPerSec;
    pass_ = 0;
}

bool CalibrationSweep::update(float angleDeg, float referenceDeg) {
    if (pass_ >= PASSES) {
        return false;
    }
    const float end = passEndDeg();
    if (fabsf(referenceDeg - end) < 0.01f && passDir() * (angleDeg - end) >= -PASS_END_TOLERANCE_DEG) {
        pass_++;
    }
    return pass_ < PASSES;
}

} // namespace NeuroExoControl
