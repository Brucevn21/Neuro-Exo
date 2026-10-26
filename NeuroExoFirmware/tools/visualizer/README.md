# NeuroExo Joint Live Visualizer

Live matplotlib visualization of the exoskeleton joint driven by serial data
streamed from `src/main.cpp`.

## Firmware side

`main.cpp` streams a compact CSV line at 50 Hz:

```
JOINT,<encoderDeg>,<setpointInterpolatedDeg>,<targetDeg>,<motorVoltage>,<velocityDegPerSec>,<motionActive 0|1>,<accelDegPerSec2>,<currentA>,<millis>
```

The Teensy env must be built with `-D USB_MIDI_SERIAL` (see `platformio.ini`)
so it enumerates as a real serial port (`/dev/ttyACM*` / `COMx`). With plain
`USB_MIDI`, `Serial` is HID-emulated and pyserial cannot open it.

Older firmware that sends only the first 7 fields still displays, but trial
logging is disabled.

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
commanded target), plus angle and velocity time-series plots.

The arm and plots only update while a trial is running (`motionActive=1`). Each
new trial clears the plots and starts at t = 0 (device time), and the plots show
the whole trial. When the trial ends the display freezes on it until the next
trial starts; the status line keeps updating so you can see the link is alive.

## 3D visualizer

`joint_visualizer_3d.py` shows the same data as a simple 3D elbow brace: a fixed
upper arm, the motor hub on the elbow axis, and a forearm that rotates about it.
The red dashed line is the trial target and the orange line is the moving
setpoint, with an angle-vs-time plot beside it. Drag to rotate the view and
right-drag or scroll to zoom. It takes the same options, follows the same trial
behavior and writes the same trial CSVs as the 2D visualizer:

```bash
python joint_visualizer_3d.py --port /dev/ttyACM1 --baud 115200
```

Run only one visualizer at a time; both need the Teensy's serial port.

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
| `setpoint_position_deg` | deg | interpolated setpoint the PID tracks |

On Windows, close the CSV in Excel before the next trial completes, because
Excel locks open files and the rotation will fail. Copy files out of
`telemetry_data/` to keep them.

## Testing without hardware (Linux/macOS)

```bash
python mock_joint_feed.py --trials 7     # prints e.g. "Mock feed on /dev/pts/3"
python joint_visualizer.py --port /dev/pts/3
```

After about 25 s, `telemetry_data/` holds `trial_run_1..5.csv` (trials 7..3).
