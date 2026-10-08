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
commanded target), plus live angle and velocity time-series plots.

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
