#!/usr/bin/env python3
"""
Live serial visualizer for the NeuroExo joint.

Reads the "JOINT,<enc>,<setpoint>,<target>,<Vc>,<vel>,<active>,<acc>,<currentA>,<ms>"
CSV lines streamed by src/main.cpp (see STREAM_INTERVAL_MS) over a serial port
and renders the joint as a rotating single-link arm, plus live plots of angle
and velocity. Active trials are logged to CSV (see trial_logger.py).

Usage:
    python joint_visualizer.py --port /dev/ttyACM0 --baud 115200

On Windows, --port would look like "COM5".
"""

import argparse
import collections
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
HISTORY_SECONDS = 10.0
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
        """Read all available lines, log each one, and keep the most recent valid sample."""
        updated = False
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
            updated = True
        return updated

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


def main():
    args = parse_args()

    logger = None
    if not args.no_log:
        logger = TrialCsvLogger(args.log_dir, keep=args.keep_trials, idle_gap_ms=args.idle_gap_ms)
        print(f"Logging active trials to {os.path.abspath(args.log_dir)}")

    try:
        stream = JointDataStream(args.port, args.baud, logger)
    except serial.SerialException as exc:
        print(f"Could not open serial port {args.port}: {exc}", file=sys.stderr)
        sys.exit(1)

    max_samples = int(HISTORY_SECONDS * 50)  # ~50 Hz stream rate
    time_hist = collections.deque(maxlen=max_samples)
    angle_hist = collections.deque(maxlen=max_samples)
    target_hist = collections.deque(maxlen=max_samples)
    vel_hist = collections.deque(maxlen=max_samples)
    start_time = time.time()

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
    ax_angle.set_title("Angle (deg) vs Time")
    ax_angle.set_xlabel("Time (s)")
    ax_angle.set_ylabel("deg")
    ax_angle.grid(True, linestyle=":", alpha=0.5)
    (angle_line,) = ax_angle.plot([], [], "b-", label="Encoder")
    (setpoint_line,) = ax_angle.plot([], [], "r--", label="Setpoint")
    ax_angle.legend(loc="upper right", fontsize=8)

    # --- Velocity history ---
    ax_vel.set_title("Velocity (deg/s) vs Time")
    ax_vel.set_xlabel("Time (s)")
    ax_vel.set_ylabel("deg/s")
    ax_vel.grid(True, linestyle=":", alpha=0.5)
    (vel_line,) = ax_vel.plot([], [], "g-")

    fig.tight_layout()

    def update(_frame):
        stream.poll()

        now = time.time() - start_time
        time_hist.append(now)
        angle_hist.append(stream.encoder_deg)
        target_hist.append(stream.setpoint_deg)
        vel_hist.append(stream.velocity_deg_s)

        angle_rad = np.radians(stream.encoder_deg)
        link_line.set_data([0, args.link_length * np.cos(angle_rad)], [0, args.link_length * np.sin(angle_rad)])

        target_rad = np.radians(stream.target_deg)
        target_line.set_data(
            [0, args.link_length * 1.15 * np.cos(target_rad)],
            [0, args.link_length * 1.15 * np.sin(target_rad)],
        )

        status = "ACTIVE" if stream.motion_active else "IDLE"
        stale = stream.last_update is None or (time.time() - stream.last_update) > 1.0
        conn_status = "NO DATA" if stale else status
        if logger is None:
            log_status = "off"
        elif stream.legacy_stream:
            log_status = "off (old firmware)"
        elif logger.trial_active:
            log_status = f"REC ({logger.rows_in_trial} rows)"
        else:
            log_status = "waiting"
        angle_text.set_text(
            f"Encoder: {stream.encoder_deg:6.2f} deg\n"
            f"Target:  {stream.target_deg:6.2f} deg\n"
            f"Voltage: {stream.voltage:5.2f} V\n"
            f"Current: {stream.current_a:6.3f} A\n"
            f"Status:  {conn_status}\n"
            f"Log:     {log_status}"
        )

        if time_hist:
            t_arr = np.array(time_hist)
            angle_line.set_data(t_arr, angle_hist)
            setpoint_line.set_data(t_arr, target_hist)
            vel_line.set_data(t_arr, vel_hist)

            xmin = max(0, t_arr[-1] - HISTORY_SECONDS)
            for ax in (ax_angle, ax_vel):
                ax.set_xlim(xmin, max(xmin + HISTORY_SECONDS, t_arr[-1]))
            ax_angle.set_ylim(args.min_deg - 10, args.max_deg + 10)
            if vel_hist:
                vmin, vmax = min(vel_hist), max(vel_hist)
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
