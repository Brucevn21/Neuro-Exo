"""Traffic model of the active debug path in Hardware_Interface.cpp.

Only output shape/timing are modeled. Values are synthetic; no ADC, IMU,
H-infinity filter, classifier, or electrode model is reproduced.
"""
import math

SOURCE_SLEEP_SECONDS = 0.004
NOMINAL_SAMPLE_RATE = 250.0


def eeg_values(sequence):
    """Deterministic changing values; amplitude is illustrative, not calibrated."""
    t = sequence / NOMINAL_SAMPLE_RATE
    return tuple(
        (channel + 1) * 1e-5 * math.sin(2 * math.pi * (8 + channel) * t + channel * .3)
        + 2e-6 * math.cos(2 * math.pi * 3 * t)
        for channel in range(8)
    )


def serialize_eeg(values):
    """Match default C++ ostream precision (6 significant digits), C locale."""
    if len(values) != 8 or not all(math.isfinite(value) for value in values):
        raise ValueError("one debug record must contain eight finite values")
    return (";".join(format(value, ".6g") for value in values) + "\n").encode("ascii")


def eeg_record(sequence):
    return serialize_eeg(eeg_values(sequence))
