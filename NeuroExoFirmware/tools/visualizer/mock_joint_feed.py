#!/usr/bin/env python3
"""
Fake Teensy for testing the visualizer and trial logger without hardware
(Linux/macOS only).

Opens a pseudo-terminal and streams JOINT lines at 50 Hz in the same format as
src/main.cpp, alternating idle periods with motion trials. Point the
visualizer at the printed port:

    python mock_joint_feed.py --trials 7
    python joint_visualizer.py --port /dev/pts/N
"""

import argparse
import math
import os
import pty
import time
import tty

STREAM_INTERVAL_S = 0.02


def main():
    parser = argparse.ArgumentParser(description="Mock NeuroExo JOINT serial feed")
    parser.add_argument("--trials", type=int, default=7, help="Number of trials to send before idling forever")
    parser.add_argument("--trial-s", type=float, default=2.0, help="Duration of each trial (s)")
    parser.add_argument("--idle-s", type=float, default=1.5, help="Idle time between trials (s)")
    args = parser.parse_args()

    master, slave = pty.openpty()
    tty.setraw(slave)  # no echo/line processing, like a real USB CDC port
    print(f"Mock feed on {os.ttyname(slave)}  (Ctrl+C to stop)", flush=True)

    start = time.monotonic()
    pos, vel = 0.0, 0.0
    trial = 0
    phase_start = 0.0
    active = False
    target = 0.0
    origin = 0.0

    while True:
        now = time.monotonic() - start
        ms = int(now * 1000)
        elapsed = now - phase_start

        if active and elapsed >= args.trial_s:
            active, phase_start = False, now
            print(f"trial {trial} done", flush=True)
        elif not active and elapsed >= args.idle_s and trial < args.trials:
            trial += 1
            active, phase_start = True, now
            origin = pos
            target = 60.0 if trial % 2 else -90.0
            print(f"trial {trial} start -> {target:.0f} deg", flush=True)

        if active:
            frac = min(1.0, (now - phase_start) / args.trial_s)
            setpoint = origin + (target - origin) * frac
            new_pos = setpoint - 1.5 * math.sin(math.pi * frac)  # small tracking lag
            current_a = 0.4 + 0.3 * math.sin(math.pi * frac)
        else:
            setpoint, new_pos, current_a = pos, pos, 0.0

        new_vel = (new_pos - pos) / STREAM_INTERVAL_S
        accel = (new_vel - vel) / STREAM_INTERVAL_S
        pos, vel = new_pos, new_vel
        voltage = 6.0 * current_a

        line = (f"JOINT,{pos:.2f},{setpoint:.2f},{target:.2f},{voltage:.2f},"
                f"{vel:.2f},{int(active)},{accel:.2f},{current_a:.4f},{ms}\r\n")
        os.write(master, line.encode())
        if ms % 1000 < 20:
            os.write(master, f"[Teensy Telemetry] Target: {target:.2f} deg\r\n".encode())
        time.sleep(STREAM_INTERVAL_S)


if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        pass
