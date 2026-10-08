"""
Rolling CSV logger for NeuroExo trials.

A trial starts on the first sample with motion_active=1 and ends once motion
has been inactive for `idle_gap_ms` (so back-to-back commands from the
BeagleBone count as one trial). Only active samples are written.

Completed trials are rotated through a fixed set of files so the newest is
always trial_run_1.csv and the oldest kept is trial_run_<keep>.csv:

    telemetry_data/trial_run_1.csv   <- most recent completed trial
    ...
    telemetry_data/trial_run_5.csv   <- oldest kept trial

A trial still in progress lives in .trial_in_progress.csv and is discarded if
the visualizer exits before the trial completes.
"""

import csv
import os

CSV_HEADER = [
    "elapsed_ms",
    "current_A",
    "velocity_deg_s",
    "acceleration_deg_s2",
    "current_position_deg",
    "target_position_deg",
    "setpoint_position_deg",
]


class TrialCsvLogger:
    def __init__(self, log_dir, keep=5, idle_gap_ms=300):
        self.log_dir = log_dir
        self.keep = keep
        self.idle_gap_ms = idle_gap_ms
        self.in_progress_path = os.path.join(log_dir, ".trial_in_progress.csv")
        os.makedirs(log_dir, exist_ok=True)

        self._file = None
        self._writer = None
        self._start_ms = None
        self._last_active_ms = None
        self.rows_in_trial = 0

    @property
    def trial_active(self):
        return self._file is not None

    def trial_path(self, n):
        return os.path.join(self.log_dir, f"trial_run_{n}.csv")

    def handle_sample(self, device_ms, motion_active, current_a, velocity, accel, position, target, setpoint):
        """Feed one parsed JOINT sample. Returns the path of a just-completed trial, else None."""
        completed = None
        if not motion_active:
            if self.trial_active and device_ms - self._last_active_ms >= self.idle_gap_ms:
                completed = self._finish_trial()
            return completed

        if self.trial_active and device_ms < self._last_active_ms:
            # Device clock went backwards (Teensy reset): close out the old trial.
            completed = self._finish_trial()
        if not self.trial_active:
            self._start_trial(device_ms)

        self._last_active_ms = device_ms
        self._writer.writerow([
            device_ms - self._start_ms,
            f"{current_a:.4f}",
            f"{velocity:.2f}",
            f"{accel:.2f}",
            f"{position:.2f}",
            f"{target:.2f}",
            f"{setpoint:.2f}",
        ])
        self.rows_in_trial += 1
        return completed

    def close(self):
        """Discard any incomplete trial."""
        if self._file is not None:
            self._file.close()
            self._file = None
            os.remove(self.in_progress_path)

    def _start_trial(self, device_ms):
        self._file = open(self.in_progress_path, "w", newline="")
        self._writer = csv.writer(self._file)
        self._writer.writerow(CSV_HEADER)
        self._start_ms = device_ms
        self._last_active_ms = device_ms
        self.rows_in_trial = 0

    def _finish_trial(self):
        self._file.close()
        self._file = None
        self._writer = None

        # Shift trial_run_1..keep-1 up by one, dropping the oldest.
        oldest = self.trial_path(self.keep)
        if os.path.exists(oldest):
            os.remove(oldest)
        for n in range(self.keep - 1, 0, -1):
            src = self.trial_path(n)
            if os.path.exists(src):
                os.replace(src, self.trial_path(n + 1))

        newest = self.trial_path(1)
        os.replace(self.in_progress_path, newest)
        return newest
