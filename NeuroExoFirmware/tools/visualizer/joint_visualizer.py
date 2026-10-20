#!/usr/bin/env python3
"""
Live serial visualizer for the NeuroExo joint.

Reads the "JOINT,<enc>,<setpoint>,<target>,<Vc>,<vel>,<active>" CSV lines
streamed by src/main.cpp (see STREAM_INTERVAL_MS) over a serial port and
renders the joint as a rotating single-link arm, plus live plots of angle
and velocity.

Usage:
    python joint_visualizer.py --port /dev/ttyACM0 --baud 115200

On Windows, --port would look like "COM5".
"""

import argparse
import collections
import sys
import time

import matplotlib.pyplot as plt
import matplotlib.animation as animation
import numpy as np
import serial

# Mechanical limits from main.cpp motorLimit (forwardLimit / backwardLimit).
DEFAULT_MIN_DEG = -150.0
DEFAULT_MAX_DEG = 80.0
HISTORY_SECONDS = 10.0


def parse_args():
    parser = argparse.ArgumentParser(description="Live NeuroExo joint visualizer")
    parser.add_argument("--port", required=True, help="Serial port, e.g. /dev/ttyACM0 or COM5")
    parser.add_argument("--baud", type=int, default=115200, help="Serial baud rate")
    parser.add_argument("--min-deg", type=float, default=DEFAULT_MIN_DEG, help="Joint minimum angle (deg)")
    parser.add_argument("--max-deg", type=float, default=DEFAULT_MAX_DEG, help="Joint maximum angle (deg)")
    parser.add_argument("--link-length", type=float, default=1.0, help="Visual link length (arbitrary units)")
    return parser.parse_args()


class JointDataStream:
    """Reads and parses JOINT,... lines from the serial port without blocking the UI."""

    def __init__(self, port, baud):
        self.ser = serial.Serial(port, baud, timeout=0.05)
        self.encoder_deg = 0.0
        self.setpoint_deg = 0.0
        self.target_deg = 0.0
        self.voltage = 0.0
        self.velocity_deg_s = 0.0
        self.motion_active = False
        self.last_update = None

    def poll(self):
        """Read all available lines and update state with the most recent valid sample."""
        updated = False
        while self.ser.in_waiting:
            raw = self.ser.readline().decode("utf-8", errors="ignore").strip()
            if not raw.startswith("JOINT,"):
                continue
            fields = raw.split(",")
            if len(fields) != 7:
                continue
            try:
                self.encoder_deg = float(fields[1])
                self.setpoint_deg = float(fields[2])
                self.target_deg = float(fields[3])
                self.voltage = float(fields[4])
                self.velocity_deg_s = float(fields[5])
                self.motion_active = fields[6] == "1"
            except ValueError:
                continue
            self.last_update = time.time()
            updated = True
        return updated

    def close(self):
        self.ser.close()


def main():
    args = parse_args()

    try:
        stream = JointDataStream(args.port, args.baud)
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
        angle_text.set_text(
            f"Encoder: {stream.encoder_deg:6.2f} deg\n"
            f"Target:  {stream.target_deg:6.2f} deg\n"
            f"Voltage: {stream.voltage:5.2f} V\n"
            f"Status:  {conn_status}"
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
