# NeuroExo Joint Live Visualizer

Live matplotlib visualization of the exoskeleton joint driven by serial data
streamed from `src/main.cpp`.

## Firmware side

`main.cpp` streams a compact CSV line at 50 Hz:

```
JOINT,<encoderDeg>,<setpointInterpolatedDeg>,<targetDeg>,<motorVoltage>,<velocityDegPerSec>,<motionActive 0|1>
```

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

The window shows the rotating joint link (blue = current angle, red dashed =
commanded target), plus live angle and velocity time-series plots.
