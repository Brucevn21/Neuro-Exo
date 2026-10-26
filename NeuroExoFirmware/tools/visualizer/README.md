# NeuroExo Joint Live Visualizer

Live matplotlib visualization of the exoskeleton joint driven by serial data
streamed from `src/main.cpp`.

## Firmware side

`main.cpp` streams a compact CSV line at 50 Hz:

```
JOINT,<encoderDeg>,<setpointInterpolatedDeg>,<targetDeg>,<motorVoltage>,<velocityDegPerSec>,<motionActive 0|1>,<accelDegPerSec2>,<currentA>,<millis>,<patientEffortNm>,<mode>,<setting>
```

`setpointInterpolatedDeg` is the Neutral timed trajectory, or the moving
reference the joint is held on in the patient-driven modes. `patientEffortNm`
is positive when the patient helps toward the target and negative when they
resist. `mode` is 0 Resistive, 1 Assistive, 2 Neutral, 3 Calibrate; `setting`
is the Neutral speed (0-2) or the Assistive/Resistive level (1-9).

The Teensy env must be built with `-D USB_MIDI_SERIAL` (see `platformio.ini`)
so it enumerates as a real serial port (`/dev/ttyACM*` / `COMx`). With plain
`USB_MIDI`, `Serial` is HID-emulated and pyserial cannot open it.

Older firmware that sends only the first 7 fields still displays, but trial
logging is disabled. Firmware that sends 10 fields (no effort/mode/setting)
displays and logs with effort 0.

This is emitted alongside the existing 1 Hz human-readable debug print, so
both can be read from the same serial connection.

## Python visualizer

Install dependencies once:

```bash
pip install -r requirements.txt
```

Run it (find your port with `ls /dev/tty*` on Linux/macOS or Device Manager
on Windows):

```bash
python joint_visualizer.py --port /dev/ttyACM0 --baud 115200
```

Optional flags:

- `--min-deg` / `--max-deg`: joint travel limits for the plot range (defaults
  match `motorLimit.backwardLimit`/`forwardLimit` in `main.cpp`: -150 to 80).
- `--link-length`: visual length of the drawn arm link.
- `--log-dir`: where trial CSVs go (default `tools/visualizer/telemetry_data/`).
- `--keep-trials`: completed trials to keep (default 5).
- `--idle-gap-ms`: inactive time that ends a trial (default 300). Shorter gaps
  between commands are merged into one trial.
- `--no-log`: disable trial logging.

The window shows the rotating joint link (blue = current angle, red dashed =
commanded target), a readout (angle, target, current, patient effort, mode and
level), plus angle, velocity and patient-effort time-series plots.

The arm and plots only update while a trial is running (`motionActive=1`). Each
new trial clears the plots and starts at t = 0 (device time), and the plots show
the whole trial. When the trial ends the display freezes on it until the next
trial starts; the status line keeps updating so you can see the link is alive.

## 3D view

The left side of the window is a simple 3D elbow brace: a fixed upper arm, the
motor hub on the elbow axis, and a forearm that rotates about it. The red dashed
line is the trial target and the orange line is the moving setpoint. Drag to
rotate the view and right-drag or scroll to zoom. It follows the same trial
behavior as the 2D view.

## Trial CSV logging

While the visualizer runs, every sample with `motionActive=1` is recorded.
Idle samples are never written. A trial ends after `--idle-gap-ms` of
inactivity, then rotates into place:

```
telemetry_data/trial_run_1.csv   newest completed trial
...
telemetry_data/trial_run_5.csv   oldest kept trial (deleted on the next rotation)
```

A trial in progress is written to `.trial_in_progress.csv` and is discarded if
the visualizer exits first. Columns:

| column | units | source |
|---|---|---|
| `elapsed_ms` | ms since trial start | Teensy `millis()` |
| `current_A` | A | ESCON AN1 (`measuredMotorCurrentA`) |
| `velocity_deg_s` | deg/s | `measuredVelocityDegPerSec` |
| `acceleration_deg_s2` | deg/s² | `measuredAccelDegPerSec2` |
| `current_position_deg` | deg | encoder (`encDeg`) |
| `target_position_deg` | deg | commanded target (`interpolateEnd`) |
| `setpoint_position_deg` | deg | interpolated setpoint / moving reference the PID tracks |
| `patient_effort_Nm` | N·m | estimated patient torque toward the target (`patientEffortNm`; - = resisting) |

On Windows, close the CSV in Excel before the next trial completes, because
Excel locks open files and the rotation will fail. Copy files out of
`telemetry_data/` to keep them.

## Testing without hardware (Linux/macOS)

```bash
python mock_joint_feed.py --trials 7     # prints e.g. "Mock feed on /dev/pts/3"
python joint_visualizer.py --port /dev/pts/3
```

After about 25 s, `telemetry_data/` holds `trial_run_1..5.csv` (trials 7..3).
