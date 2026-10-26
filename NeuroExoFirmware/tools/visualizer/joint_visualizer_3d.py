#!/usr/bin/env python3
"""
3D live visualizer for the NeuroExo joint.

Same serial feed, trial behavior and CSV logging as joint_visualizer.py, but the
joint is drawn as a simple 3D elbow brace: a fixed upper arm, a motor hub on the
elbow axis, and a forearm that rotates about that axis. The red dashed line is
the trial target and the orange line is the moving setpoint. Drag with the
mouse to rotate the view, right-drag (or scroll) to zoom.

Usage:
    python joint_visualizer_3d.py --port /dev/ttyACM1 --baud 115200

Takes the same options as joint_visualizer.py.
"""

import matplotlib.pyplot as plt
import matplotlib.animation as animation
import numpy as np

from joint_visualizer import MIN_PLOT_SECONDS, TrialTracker, open_stream, parse_args

ELBOW_AXIS = np.array([0.0, 1.0, 0.0])  # the joint rotates about +y
UPPER_ARM_DEG = 180.0                   # fixed upper arm, outside the joint's travel


def direction(angle_deg):
    """Unit vector in the x-z plane for a joint angle (0 deg = +x, positive toward +z)."""
    a = np.radians(angle_deg)
    return np.array([np.cos(a), 0.0, np.sin(a)])


def cylinder(p0, p1, radius, segments=20):
    """Surface grid (X, Y, Z) for a cylinder from p0 to p1."""
    axis = p1 - p0
    unit = axis / np.linalg.norm(axis)
    helper = np.array([1.0, 0.0, 0.0]) if abs(unit[1]) > 0.9 else ELBOW_AXIS
    e1 = np.cross(unit, helper)
    e1 /= np.linalg.norm(e1)
    e2 = np.cross(unit, e1)
    theta = np.linspace(0, 2 * np.pi, segments)
    ring = radius * (np.outer(e1, np.cos(theta)) + np.outer(e2, np.sin(theta)))  # (3, segments)
    along = np.array([0.0, 1.0])
    grid = p0[:, None, None] + axis[:, None, None] * along[None, None, :] + ring[:, :, None]
    return grid[0], grid[1], grid[2]


def main():
    args = parse_args()
    stream, logger = open_stream(args)
    trial = TrialTracker(stream, logger)
    length = args.link_length
    origin = np.zeros(3)

    fig = plt.figure(figsize=(13, 6))
    fig.suptitle(f"NeuroExo Joint 3D Visualizer ({args.port} @ {args.baud})")
    grid = fig.add_gridspec(2, 2, width_ratios=[1.4, 1], height_ratios=[1, 1])
    ax3d = fig.add_subplot(grid[:, 0], projection="3d")
    ax_angle = fig.add_subplot(grid[0, 1])
    ax_text = fig.add_subplot(grid[1, 1])
    ax_text.axis("off")

    # --- 3D scene ---
    span = 1.3 * length
    ax3d.set_xlim(-span, span)
    ax3d.set_ylim(-span, span)
    ax3d.set_zlim(-span, span)
    ax3d.set_box_aspect((1, 1, 1))
    ax3d.set_xlabel("x")
    ax3d.set_ylabel("y (elbow axis)")
    ax3d.set_zlabel("z")
    ax3d.view_init(elev=20, azim=-65)

    hub_half = 0.15 * length
    ax3d.plot_surface(*cylinder(-hub_half * ELBOW_AXIS, hub_half * ELBOW_AXIS, 0.12 * length),
                      color="dimgray", alpha=0.9, linewidth=0, edgecolor="none")
    ax3d.plot_surface(*cylinder(origin, length * direction(UPPER_ARM_DEG), 0.08 * length),
                      color="silver", alpha=0.8, linewidth=0, edgecolor="none")

    limit = np.linspace(args.min_deg, args.max_deg, 60)
    arc = np.array([direction(d) for d in limit]) * length * 1.05
    ax3d.plot(arc[:, 0], arc[:, 1], arc[:, 2], color="gray", linewidth=1, alpha=0.5)

    (target_line,) = ax3d.plot([], [], [], "r--", linewidth=1.5, label="Target")
    (setpoint_line,) = ax3d.plot([], [], [], color="orange", linewidth=1.5, label="Setpoint")
    ax3d.legend(loc="upper left", fontsize=8)
    forearm = {"surface": None, "hand": None}

    def draw_ray(line, angle_deg, scale):
        end = direction(angle_deg) * length * scale
        line.set_data_3d([0, end[0]], [0, end[1]], [0, end[2]])

    def draw_forearm(angle_deg):
        for artist in forearm.values():
            if artist is not None:
                artist.remove()
        end = direction(angle_deg) * length
        forearm["surface"] = ax3d.plot_surface(*cylinder(origin, end, 0.07 * length),
                                               color="tab:blue", alpha=0.95, linewidth=0, edgecolor="none")
        forearm["hand"] = ax3d.scatter(*end, color="navy", s=80, depthshade=False)

    draw_forearm(0.0)

    # --- Angle history ---
    ax_angle.set_title("Angle (deg) vs Trial Time")
    ax_angle.set_xlabel("Time (s)")
    ax_angle.set_ylabel("deg")
    ax_angle.grid(True, linestyle=":", alpha=0.5)
    ax_angle.set_xlim(0, MIN_PLOT_SECONDS)
    ax_angle.set_ylim(args.min_deg - 10, args.max_deg + 10)
    (angle_line,) = ax_angle.plot([], [], "b-", label="Encoder")
    (angle_setpoint_line,) = ax_angle.plot([], [], color="orange", linestyle="--", label="Setpoint")
    ax_angle.legend(loc="upper right", fontsize=8)

    readout = ax_text.text(0.02, 0.95, "", transform=ax_text.transAxes, fontsize=11,
                           va="top", family="monospace")
    fig.tight_layout()

    def update(_frame):
        if trial.poll():
            draw_forearm(trial.arm_deg)
            draw_ray(target_line, trial.target_deg, 1.15)
            draw_ray(setpoint_line, trial.setpoint_hist[-1], 1.0)
            angle_line.set_data(trial.time_hist, trial.angle_hist)
            angle_setpoint_line.set_data(trial.time_hist, trial.setpoint_hist)
            ax_angle.set_xlim(0, max(MIN_PLOT_SECONDS, trial.time_hist[-1]))
        readout.set_text(trial.readout_text())

    ani = animation.FuncAnimation(fig, update, interval=40, blit=False, cache_frame_data=False)

    try:
        plt.show()
    finally:
        stream.close()


if __name__ == "__main__":
    main()
