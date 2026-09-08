# NeuroExo Rehab Trainer (patient app)

A patient-facing web app for target-reaching trials, driven over Bluetooth Low
Energy. It is a pure **visualizer and session controller** - it does not pick
target angles or generate trial logic itself. It connects to the
**BeagleBone Black bridge** (`tools/beaglebone-bridge`), which owns that
logic and is itself the middleman to the Nano 33 BLE / Teensy hardware:

```
app (this folder) <--BLE--> BeagleBone Black bridge <--BLE--> Nano 33 BLE <--I2C--> Teensy 4.1
```

## Session flow

1. **Setup screen** - patient picks a speed (Slow / Medium / Fast) and
   connects to the BeagleBone, then presses Start.
2. **Trial screen** - the app asks the BeagleBone for a trial (sending only
   mode + speed); the BeagleBone picks the target angle (20&deg;-120&deg;) and the
   app displays whatever target comes back over telemetry, live-visualizing
   the joint (a rotating dial plus an angle-vs-time chart) at 20Hz. A red
   **STOP** button is always visible and immediately halts the motor
   mid-motion.
3. **Success screen** - once the encoder holds within a few degrees of the
   target, the app downloads a CSV of that trial's data and shows
   "Great job!" with a "Next Trial" button (requests a new trial from the
   BeagleBone) and a "Quit" button (returns to Setup).

Pressing STOP mid-trial sends a stop request and returns to Setup without
saving a CSV - only a trial that actually reaches its target gets logged.

## CSV export

Every telemetry sample of the trial in progress (elapsed time, current angle,
target angle, motor current in mA) is buffered in memory. When a trial is
reached, that buffer is written out as `neuroexo_latest_trial.csv` via a
normal browser download - only the most recently *completed* trial is ever
saved; the buffer resets at the start of each new trial. Note that most
browsers will suffix repeat downloads of the same filename (e.g.
`neuroexo_latest_trial (1).csv`) unless "ask where to save each file" is
disabled and the previous copy is deleted/overwritten manually.

## Running it

This is plain HTML/CSS/JS with no build step or external dependencies. Web
Bluetooth requires a "secure context," so either:

- Open `index.html` directly in Chrome or Edge (a `file://` URL counts as
  secure), or
- Serve the folder locally and open it over `http://localhost`, e.g.:

  ```bash
  cd NeuroExoFirmware/tools/patient-app
  python3 -m http.server 8000
  # then open http://localhost:8000 in Chrome
  ```

Web Bluetooth is supported in Chrome/Edge on desktop and Android. It is
**not** supported in Safari (desktop or iOS) or Firefox.

Before connecting, make sure the whole chain is up: the Nano 33 BLE is
powered and running `Nano33BLEFirmware.ino`, and `tools/beaglebone-bridge`'s
`bridge.py` is running on the BeagleBone and has connected to the Nano (see
that folder's README). The browser's device picker will show the bridge's
advertised name, `NeuroExo-BBB-Bridge` - **not** the Nano.

## Known limitations

- **Speed** changes trial duration on the firmware side (slow/medium/fast map
  to `interpCycles` in `src/main.cpp`), but **Mode**
  (Resistive/Assistive/Neutral) is not yet differentiated by
  `ControlAlgorithm.cpp` - the app always requests `Assistive`, and this has
  no functional effect on the device today.
- "Reached" is decided purely from telemetry (within
  `REACHED_TOLERANCE_DEG` of the target for `REACHED_HOLD_SAMPLES` consecutive
  samples, in `app.js`) - tune those constants if trials complete too
  eagerly/slowly for your hardware's settling behavior. A short warm-up
  window (`REACH_DETECTION_WARMUP_SAMPLES`) also ignores the first few
  telemetry samples after requesting a trial, so a stale target value still
  in flight from the previous trial can't trigger a false "reached."
- The BeagleBone bridge's dual BLE role (central to the Nano, peripheral to
  this app) has not been validated against real BeagleBone Black hardware -
  see `tools/beaglebone-bridge/README.md` for details and caveats.
