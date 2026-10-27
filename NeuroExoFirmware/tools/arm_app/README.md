# NeuroExo Arm Control

A Windows Python desktop app and a responsive mobile web app for the
`ProtocolDemo` firmware. Both interfaces use the **same controller, Bluetooth
connection, and trial library** on the computer. The phone works in Android or
iPhone browsers; it does not need Web Bluetooth or an app-store installation.
Everything runs locally.

## Windows quick start

1. Install Python 3.10 or newer with the Python launcher and Tcl/Tk support
   (included in the standard python.org Windows installer).
2. Double-click `run_windows.bat` in this folder. On first launch it creates a
   local virtual environment and installs the Bluetooth dependency, `bleak`.
3. The desktop app opens. To try it without hardware, select **simulation**,
   click **Connect**, choose **Neutral**, and click **Start trial**.
4. For phone control, put the phone and computer on the same Wi-Fi network.
   Open the complete **Phone access** link displayed in the desktop app or
   printed in the terminal, including the text after `#`.
   If Windows Firewall prompts, allow Python on your **private network**.

Manual PowerShell setup from this folder:

```powershell
py -3 -m venv .venv
.\.venv\Scripts\python.exe -m pip install -r requirements.txt
.\.venv\Scripts\python.exe arm_app.py --mobile
```

Without phone access, run `arm_app.py` without `--mobile`.
To use just the browser UI on the computer, run `arm_app.py --headless`.
To use only the phone/browser interface over Wi-Fi, run
`arm_app.py --headless --mobile`.
On Linux/macOS, use `python3` and `.venv/bin/python` in the manual commands.
Some Linux distributions require installing `python3-tk` for the desktop UI.
The simulator, 3D views, CSV recording, and web server need only the Python
standard library; `bleak` is needed only for real Bluetooth.

## Connect to hardware

Select **bluetooth** / **Nano 33 BLE**, leave the address blank, and connect.
The app scans for `Nano33BLE_Master` advertising service `180C`. An explicit
BLE address (or macOS device UUID) can be supplied when auto-discovery is not
appropriate. Another central, including the BeagleBone bridge, must release
the Nano before this app connects.

The device chain is:

```text
Windows desktop UI ─┐
                   ├─ Python controller ─ BLE ─ Nano 33 BLE ─ I2C ─ Teensy
Phone browser ──────┘
       local Wi-Fi
```

Uses the current framed/CRC-8 protocol: service `180C`, command `2A56`,
telemetry `2A57`. Control and stop frames use GATT writes **with response**,
then the app checks the telemetry `MotionActive` bit for confirmation.
No firmware changes or old `BrowserApp` packets are used. Bluetooth API usage
follows the [Bleak client documentation](https://bleak.readthedocs.io/en/latest/api/client.html).

The firmware currently contains temporary bench-testing changes, including a
disabled overcurrent trip and unconnected current feedback. Read the
[firmware bench-testing notes](../../README.md#temporary-bench-testing-changes)
before attaching a person. The displayed and exported current is the reported
value; it is not a validated current measurement in that configuration.

## Controls and trial behavior

| Control | Behavior |
| --- | --- |
| Mode | Assistive, Resistive, or Neutral. |
| Target angle | Whole encoder degrees from −150° to 80°, matching current firmware limits. |
| Assistance / resistance | 10–90% in 10% steps, mapped to firmware levels 1–9. Applied to the next trial; settings are locked during motion. |
| Speed | Slow / Medium / High for Neutral, Demo, and Home (3 / 7 / 15° per second). |
| Start trial | Sends one command. Recording begins when active motion is reported, not when the button is clicked. |
| End trial & save | Sends Stop and retains the CSV with result `operator_ended`. |
| STOP | Sends Stop immediately, without waiting for a CSV export or a start request. Also clears the firmware's latched fault. Data is retained with result `operator_stop`. |
| Demo | Commands the actual 0° → 70° → 0° hardware sweep, moving to 0° first if needed. Intended for demonstrating the device with no one attached. |
| Home | Commands a move to 0°. |
| Calibrate to target | Runs the firmware's relaxed-arm calibration from the current position to the selected target and back twice. Range must be at least 10°. |
| Export CSV | Desktop: select a saved trial and choose a destination. Phone: tap the trial's export button to download its CSV. |

Assistive and Resistive require calibration covering the whole trial range.
The app can check the `Calibrated` bit, but **Bluetooth telemetry does not
report calibration bounds**. Firmware remains the authority for range
coverage. If a start is not confirmed within 3 seconds, the app requests Stop
and reports the failure. Recalibrate after a Teensy reset.

Automatic completion, safety faults, timeouts, and patient resistance retain
the data and the reported end reason. An ordinary stop is confirmed only
after an inactive notification following completion of the stop write.
New starts are blocked while a command is awaiting confirmation. Loss of
device telemetry for 1.5 seconds triggers a stop request; if Stop remains
unconfirmed for 3 seconds, the app disconnects. The Nano firmware also sends
Stop on a Bluetooth central disconnect. The app reports unconfirmed stops
explicitly rather than claiming the device stopped.

Closing the desktop app, quitting the headless server with Ctrl+C, or clicking
Disconnect attempts Stop before releasing Bluetooth. Closing a phone tab does
not end the shared computer session. Both interfaces can control the same
session; a second simultaneous start is rejected by the controller.

## 3D view

The 3D arm is a schematic fixed upper arm, elbow motor hub, moving forearm,
brace cuffs, and hand. It follows encoder angles continuously, including idle
telemetry. Zero degrees is drawn as a straight elbow; this is a visual
convention, not an anatomical or CAD calibration.

Drag to orbit; mouse wheel zooms. On a phone, drag with one finger and use
**Reset view** to return to the default camera. The dashed amber line shows
the requested final target, when known. Demo has no target line because its
current waypoint is not transmitted over Bluetooth. The chart retains the
latest 1,500 trial samples; the CSV retains every recorded sample.

## Files and export

By default, CSVs and JSON summary sidecars are saved in `telemetry_data/`
beside this README. Use `--log-dir PATH` to select another directory.
Each trial gets a timestamp and unique filename; files are never rotated or
deleted. Each row is flushed to disk as it arrives. Completed trials reappear
after restarting the app. A CSV left without a summary after a crash is
recovered as `recovered_unfinished`.

| CSV column | Meaning |
| --- | --- |
| `elapsed_ms` | Monotonic computer time since first active notification; not device time. |
| `host_timestamp_utc` | Computer's UTC receipt timestamp. |
| `current_position_deg` | Reported encoder position (BLE has integer-degree resolution). |
| `current_A` | Reported current, converted from mA. See bench-testing note above. |
| `patient_effort_Nm` | Reported estimated patient effort, converted from centi-N·m. |
| `status` | Raw status bit field, including motion, fault, completion and calibration flags. |
| `requested_mode` | Command selected in this app; `unknown` if motion was started externally. |
| `requested_setting` | Requested level or speed, not an echoed device value. |
| `requested_target_deg` | Requested endpoint; ignored by Demo/Home firmware. Not a measured trajectory. |
| `source` | `bluetooth` or `simulation`. |

The first active sample and final inactive sample are included. The JSON
sidecar records the end reason and row count. An unexpected link loss retains
all samples received up to the disconnect. If storage fails during motion,
the app requests Stop and preserves whatever has already been written.

Bluetooth does **not** provide measured velocity, acceleration, device time,
moving setpoint, or the current Demo waypoint. Those fields are deliberately
absent rather than fabricated. Use the existing
[USB visualizer](../visualizer/README.md) when those channels are required.

## Mobile access

`--mobile` explicitly binds the server to all computer interfaces. Otherwise,
`--headless` listens only on `127.0.0.1`. Override with `--host` and `--port`
(default 8765). If automatic IP detection lists multiple links, choose the
computer's Wi-Fi IPv4 address (`ipconfig` on Windows). Guest Wi-Fi and client
isolation may prevent phone-to-computer connections.

The link contains a random per-launch access token. The browser keeps it in
session storage and sends it in an authorization header; API calls and CSV
downloads require it. Requests from unrelated websites are not granted CORS
access. No external scripts, fonts, CDNs, or cloud services are used.

Keep the link private: it grants movement commands and trial download access.
The local server uses HTTP, so use a trusted private network. Do not forward
this port to the Internet. If the phone loses Wi-Fi, its display becomes
offline; use the computer to control the ongoing session.

## Checks

From this folder:

```powershell
py -3 -m unittest discover -s tests -v
```

The checks exercise protocol reference vectors, corrupt telemetry, command
gating, recording, Stop confirmation, interrupted-trial recovery, simulator
behavior, mobile authentication, and the HTTP start/stop/CSV download flow.
They require permission to bind a temporary localhost socket, and do not
connect to hardware. The desktop was smoke-tested on Linux; Windows, actual
Android/iPhone devices, and real Nano/Teensy Bluetooth operation still need
hardware validation.
