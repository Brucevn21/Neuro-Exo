# NeuroExo Rehab Trainer (patient app)

A patient-facing web app for target-reaching trials, driven over Bluetooth Low
Energy. It talks to `Nano33BLEFirmware.ino` (service `180C`, command
characteristic `2A56`, telemetry characteristic `2A57`), which bridges to the
Teensy joint controller (`src/main.cpp`) over I2C.

## Session flow

1. **Setup screen** - patient picks a speed (Slow / Medium / Fast) and
   connects to the device, then presses Start.
2. **Trial screen** - the app picks a random target angle between 20&deg; and
   120&deg;, sends it to the device, and live-visualizes the joint (a rotating
   dial plus an angle-vs-time chart) using the 20Hz telemetry notifications.
3. **Success screen** - once the encoder holds within a few degrees of the
   target, the app shows "Great job!" with a "Next Trial" button (picks a new
   random angle) and a "Quit" button (returns to Setup).

## Running it

This is plain HTML/CSS/JS with no build step or external dependencies. Web
Bluetooth requires a "secure context," so either:

- Open `index.html` directly in Chrome or Edge (a `file://` URL counts as
  secure), or
- Serve the folder locally and open it over `http://localhost`, e.g.:

  ```bash
  cd tools/patient-app
  python3 -m http.server 8000
  # then open http://localhost:8000 in Chrome
  ```

Web Bluetooth is supported in Chrome/Edge on desktop and Android. It is
**not** supported in Safari (desktop or iOS) or Firefox.

Before connecting, make sure the Nano 33 BLE is powered, running
`Nano33BLEFirmware.ino`, and advertising (its serial log prints
`System Ready.`). The browser's device picker will show it as
`Nano33BLE_Master`.

## Known limitations

- **Speed** now changes trial duration on the firmware side (slow/medium/fast
  map to `interpCycles` in `src/main.cpp`), but **Mode**
  (Resistive/Assistive/Neutral) is not yet differentiated by
  `ControlAlgorithm.cpp` - the app always sends `Assistive`, and this has no
  functional effect on the device today.
- There's no mid-motion abort in the wire protocol: once the Teensy accepts a
  target and starts moving (`motorMotionActive`), it ignores further command
  packets until that motion finishes or the built-in safety stop trips
  (position error > 10&deg;). The trial screen's "Quit" button stops the app's
  own polling/UI but can't interrupt an in-flight motor move.
- "Reached" is decided purely from telemetry (within
  `REACHED_TOLERANCE_DEG` of the target for `REACHED_HOLD_SAMPLES` consecutive
  samples, in `app.js`) - tune those constants if trials complete too
  eagerly/slowly for your hardware's settling behavior.
