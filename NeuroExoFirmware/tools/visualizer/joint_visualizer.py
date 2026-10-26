#!/usr/bin/env python3
"""
Live serial visualizer for the NeuroExo joint.

Reads the "JOINT,<enc>,<setpoint>,<target>,<Vc>,<vel>,<active>,<acc>,<currentA>,<ms>"
CSV lines streamed by src/main.cpp (see STREAM_INTERVAL_MS) over a serial port
and renders the joint as a rotating single-link arm, plus plots of angle and
velocity. The arm and plots only update while a trial is running (motionActive=1);
each trial starts a fresh plot at t = 0 and the last trial stays on screen until
the next one begins. Active trials are logged to CSV (see trial_logger.py).

Usage:
    python joint_visualizer.py --port /dev/ttyACM0 --baud 115200

On Windows, --port would look like "COM5".
"""

import argparse
import os
import sys
import time

import matplotlib.pyplot as plt
import matplotlib.animation as animation
import numpy as np
import serial

from trial_logger import TrialCsvLogger

# Mechanical limits from main.cpp motorLimit (forwardLimit / backwardLimit).
DEFAULT_MIN_DEG = -150.0
DEFAULT_MAX_DEG = 80.0
MIN_PLOT_SECONDS = 5.0  # x-axis span before a trial grows past it
DEFAULT_LOG_DIR = os.path.join(os.path.dirname(os.path.abspath(__file__)), "telemetry_data")


def parse_args():
    parser = argparse.ArgumentParser(description="Live NeuroExo joint visualizer")
    parser.add_argument("--port", required=True, help="Serial port, e.g. /dev/ttyACM0 or COM5")
    parser.add_argument("--baud", type=int, default=115200, help="Serial baud rate")
    parser.add_argument("--min-deg", type=float, default=DEFAULT_MIN_DEG, help="Joint minimum angle (deg)")
    parser.add_argument("--max-deg", type=float, default=DEFAULT_MAX_DEG, help="Joint maximum angle (deg)")
    parser.add_argument("--link-length", type=float, default=1.0, help="Visual link length (arbitrary units)")
    parser.add_argument("--log-dir", default=DEFAULT_LOG_DIR, help="Directory for trial_run_N.csv files")
    parser.add_argument("--keep-trials", type=int, default=5, help="Number of completed trials to keep")
    parser.add_argument("--idle-gap-ms", type=int, default=300,
                        help="Inactive time (ms) that ends a trial; shorter gaps are treated as one trial")
    parser.add_argument("--no-log", action="store_true", help="Disable trial CSV logging")
    return parser.parse_args()


class JointDataStream:
    """Reads and parses JOINT,... lines from the serial port without blocking the UI."""

    def __init__(self, port, baud, logger=None):
        self.ser = serial.Serial(port, baud, timeout=0.05)
        self.logger = logger
        self.legacy_stream = False
        self.encoder_deg = 0.0
        self.setpoint_deg = 0.0
        self.target_deg = 0.0
        self.voltage = 0.0
        self.velocity_deg_s = 0.0
        self.motion_active = False
        self.accel_deg_s2 = 0.0
        self.current_a = 0.0
        self.last_update = None

    def poll(self):
        """Read all available lines, log each one, and return the new samples.

        Each sample is (time_s, encoder_deg, setpoint_deg, velocity_deg_s, motion_active),
        timed by the device clock when the firmware sends it, else the host clock.
        """
        samples = []
        while self.ser.in_waiting:
            raw = self.ser.readline().decode("utf-8", errors="ignore").strip()
            if not raw.startswith("JOINT,"):
                continue
            fields = raw.split(",")
            # 7 fields = older firmware without accel/current/timestamp (display only, no logging).
            if len(fields) not in (7, 10):
                continue
            try:
                encoder_deg = float(fields[1])
                setpoint_deg = float(fields[2])
                target_deg = float(fields[3])
                voltage = float(fields[4])
                velocity = float(fields[5])
                motion_active = fields[6] == "1"
                if len(fields) == 10:
                    accel = float(fields[7])
                    current_a = float(fields[8])
                    device_ms = int(fields[9])
            except ValueError:
                continue
            self.encoder_deg = encoder_deg
            self.setpoint_deg = setpoint_deg
            self.target_deg = target_deg
            self.voltage = voltage
            self.velocity_deg_s = velocity
            self.motion_active = motion_active
            self.legacy_stream = len(fields) == 7
            if not self.legacy_stream:
                self.accel_deg_s2 = accel
                self.current_a = current_a
                if self.logger is not None:
                    self._log_sample(device_ms)
            self.last_update = time.time()
            sample_s = self.last_update if self.legacy_stream else device_ms / 1000.0
            samples.append((sample_s, encoder_deg, setpoint_deg, velocity, motion_active))
        return samples

    def _log_sample(self, device_ms):
        try:
            completed = self.logger.handle_sample(
                device_ms, self.motion_active, self.current_a, self.velocity_deg_s,
                self.accel_deg_s2, self.encoder_deg, self.target_deg, self.setpoint_deg,
            )
        except OSError as exc:
            # e.g. trial_run_N.csv held open by Excel on Windows.
            print(f"Trial logging error: {exc}", file=sys.stderr)
            return
        if completed:
            print(f"Trial saved: {completed}")

    def close(self):
        self.ser.close()
        if self.logger is not None:
            self.logger.close()


class TrialTracker:
    """Keeps the samples of the current (or most recent) trial.

    The display only follows the joint while motion_active=1. A new trial clears
    the history and restarts time at 0; after it ends the last values are kept.
    Shared by the 2D and 3D visualizers.
    """

    def __init__(self, stream, logger):
        self.stream = stream
        self.logger = logger
        self.time_hist = []
        self.angle_hist = []
        self.setpoint_hist = []
        self.vel_hist = []
        self.running = False
        self.count = 0
        self.start_s = 0.0
        self.arm_deg = 0.0
        self.target_deg = 0.0
        self.voltage = 0.0
        self.current_a = 0.0

    def poll(self):
        """Read new samples; returns True if the trial display needs redrawing."""
        changed = False
        for sample in self.stream.poll():
            active = sample[4]
            if active and not self.running:
                for hist in (self.time_hist, self.angle_hist, self.setpoint_hist, self.vel_hist):
                    hist.clear()
                self.running = True
                self.start_s = sample[0]
                self.target_deg = self.stream.target_deg
                self.count += 1
            if active:
                self._record(sample)
                changed = True
            elif self.running:
                self._record(sample)  # keep the sample where the trial ended
                self.running = False
                changed = True
        return changed

    def _record(self, sample):
        t_s, encoder_deg, setpoint_deg, velocity, _active = sample
        self.time_hist.append(t_s - self.start_s)
        self.angle_hist.append(encoder_deg)
        self.setpoint_hist.append(setpoint_deg)
        self.vel_hist.append(velocity)
        self.arm_deg = encoder_deg
        self.voltage = self.stream.voltage
        self.current_a = self.stream.current_a

    def readout_text(self):
        if self.running:
            status = f"TRIAL {self.count} ({self.time_hist[-1]:.1f} s)"
        elif self.count:
            status = f"IDLE (trial {self.count}: {self.time_hist[-1]:.1f} s)"
        else:
            status = "IDLE (no trial yet)"
        stream = self.stream
        if stream.last_update is None or (time.time() - stream.last_update) > 1.0:
            status = "NO DATA"
        if self.logger is None:
            log_status = "off"
        elif stream.legacy_stream:
            log_status = "off (old firmware)"
        elif self.logger.trial_active:
            log_status = f"REC ({self.logger.rows_in_trial} rows)"
        else:
            log_status = "waiting"
        return (
            f"Encoder: {self.arm_deg:6.2f} deg\n"
            f"Target:  {self.target_deg:6.2f} deg\n"
            f"Voltage: {self.voltage:5.2f} V\n"
            f"Current: {self.current_a:6.3f} A\n"
            f"Status:  {status}\n"
            f"Log:     {log_status}"
        )


def open_stream(args):
    """Open the serial stream (and trial logger) from parsed args, or exit."""
    logger = None
    if not args.no_log:
        logger = TrialCsvLogger(args.log_dir, keep=args.keep_trials, idle_gap_ms=args.idle_gap_ms)
        print(f"Logging active trials to {os.path.abspath(args.log_dir)}")

    try:
        stream = JointDataStream(args.port, args.baud, logger)
    except serial.SerialException as exc:
        print(f"Could not open serial port {args.port}: {exc}", file=sys.stderr)
        sys.exit(1)
    return stream, logger


def main():
    args = parse_args()
    stream, logger = open_stream(args)
    trial = TrialTracker(stream, logger)

    fig, (ax_arm, ax_angle, ax_vel) = plt.subplots(1, 3, figsize=(13, 4.5))
    fig.suptitle(f"NeuroExo Joint Live Visualizer ({args.port} @ {args.baud})")

    # --- Arm view ---
    ax_arm.set_xlim(-1.3, 1.3)
    ax_arm.set_ylim(-1.3, 1.3)
    ax_arm.set_aspect("equal")
    ax_arm.set_title("Joint Angle")
    ax_arm.grid(True, linestyle=":", alpha=0.5)
    (base_dot,) = ax_arm.plot(0, 0, "ko", markersize=8)
    (link_line,) = ax_arm.plot([0, args.link_length], [0, 0], "b-", linewidth=4, solid_capstyle="round")
    (target_line,) = ax_arm.plot([], [], "r--", linewidth=1.5, alpha=0.7)
    angle_text = ax_arm.text(0.02, 0.95, "", transform=ax_arm.transAxes, fontsize=10, va="top")

    # Draw joint limit wedge for reference.
    limit_angles = np.linspace(np.radians(args.min_deg), np.radians(args.max_deg), 50)
    ax_arm.plot(
        np.cos(limit_angles) * args.link_length * 1.05,
        np.sin(limit_angles) * args.link_length * 1.05,
        color="gray",
        linewidth=1,
        alpha=0.4,
    )

    # --- Angle history ---
    ax_angle.set_title("Angle (deg) vs Trial Time")
    ax_angle.set_xlabel("Time (s)")
    ax_angle.set_ylabel("deg")
    ax_angle.grid(True, linestyle=":", alpha=0.5)
    (angle_line,) = ax_angle.plot([], [], "b-", label="Encoder")
    (setpoint_line,) = ax_angle.plot([], [], "r--", label="Setpoint")
    ax_angle.legend(loc="upper right", fontsize=8)

    # --- Velocity history ---
    ax_vel.set_title("Velocity (deg/s) vs Trial Time")
    ax_vel.set_xlabel("Time (s)")
    ax_vel.set_ylabel("deg/s")
    ax_vel.grid(True, linestyle=":", alpha=0.5)
    (vel_line,) = ax_vel.plot([], [], "g-")

    fig.tight_layout()

    def update(_frame):
        trial_changed = trial.poll()

        if trial_changed:
            angle_rad = np.radians(trial.arm_deg)
            link_line.set_data([0, args.link_length * np.cos(angle_rad)],
                               [0, args.link_length * np.sin(angle_rad)])
            target_rad = np.radians(trial.target_deg)
            target_line.set_data(
                [0, args.link_length * 1.15 * np.cos(target_rad)],
                [0, args.link_length * 1.15 * np.sin(target_rad)],
            )

        angle_text.set_text(trial.readout_text())

        if trial_changed and trial.time_hist:
            angle_line.set_data(trial.time_hist, trial.angle_hist)
            setpoint_line.set_data(trial.time_hist, trial.setpoint_hist)
            vel_line.set_data(trial.time_hist, trial.vel_hist)

            for ax in (ax_angle, ax_vel):
                ax.set_xlim(0, max(MIN_PLOT_SECONDS, trial.time_hist[-1]))
            ax_angle.set_ylim(args.min_deg - 10, args.max_deg + 10)
            if trial.vel_hist:
                vmin, vmax = min(trial.vel_hist), max(trial.vel_hist)
                pad = max(5.0, (vmax - vmin) * 0.1)
                ax_vel.set_ylim(vmin - pad, vmax + pad)

        return link_line, target_line, angle_text, angle_line, setpoint_line, vel_line

    ani = animation.FuncAnimation(fig, update, interval=40, blit=False, cache_frame_data=False)

    try:
        plt.show()
    finally:
        stream.close()


if __name__ == "__main__":
    main()
