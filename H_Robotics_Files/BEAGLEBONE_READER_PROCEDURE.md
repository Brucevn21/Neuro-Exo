> Legacy procedure: this describes the old Classic RFCOMM producer. The current
> Hardware_Interface.cpp uses the BLE trial protocol instead. Follow
> [NEUROEXO_PROTOCOL.md](NEUROEXO_PROTOCOL.md) for the current Nano receiver and
> Python/C++ run commands.

This procedure receives the BeagleBone application's EEG/EOG messages with
[`read_outputs.py`](read_outputs.py) and displays them in a terminal. It assumes
the Classic Bluetooth transport implemented in this folder's C++ code.

**What the reader displays**

The BeagleBone sends one text record per sample:

```text
eeg1;eeg2;eeg3;eeg4;eeg5;eog1;eog2;eog3\n
```

The eight fields are numeric values; `\n` represents the terminating newline.
For example, this synthetic message:

```text
1;2;3;4;5;6;7;8
```

appears as:

```text
2026-09-22T12:00:00.000 [EEG/EOG] eeg1=1  eeg2=2  eeg3=3  eeg4=4  eeg5=5  eog1=6  eog2=7  eog3=8
```

The timestamp comes from the receiving computer. The reader assembles complete
newline-terminated messages even when transport reads split or combine them.
It labels the values rather than preserving the original semicolon layout.
Unknown text remains visible with a `LOG` label.

**Raw binary bits, Bluetooth packet headers, and electrical GPIO/SPI signals
are outside this reader's current capabilities.** It has no hex/binary display
option or serial-port mode. The debug sender transmits filtered EEG/EOG values;
raw ADC bytes, IMU data, and predictions are not included in this stream.

**1. Confirm the equipment and application are ready.**

Use a Linux receiving computer with Python 3.7 or newer, BlueZ, and a working
Classic Bluetooth adapter. The reader requires no third-party Python packages.
Its Bluetooth mode requires Linux socket support; native Windows Python does
not provide the required BlueZ interface, and WSL must actually have access to
a usable Bluetooth controller.

The standard BeagleBone Black needs an added Bluetooth adapter for this route;
the BeagleBone Black Wireless has onboard Bluetooth. Check the actual board
and its Linux adapter support. See the official
[Black hardware overview](https://docs.beagleboard.org/boards/beaglebone/black/ch04.html)
and [Black Wireless specifications](https://www.beagleboard.org/boards/beaglebone-black-wireless).

Live acquisition requires the complete, working C++ application on the board,
including its sensor hardware and required services. **The C++ files in this
folder cannot currently build by themselves:** dependencies are missing and
there are source errors. See the [project explanation](README.md). This guide
cannot supply a build command or executable path for the missing application.
The examples below use `./main` as a placeholder for its actual executable.

Keep two application terminals available:

| Terminal | Runs on | Purpose |
|---|---|---|
| Receiver | Linux receiving computer | Run Python and display incoming messages. |
| BeagleBone | Board, directly or through an interactive SSH session | Run the acquisition application and type its numeric commands. |

An SSH session can carry your keyboard commands to the board while Bluetooth
carries the EEG/EOG data to the receiver.

**2. Test the reader on the receiving computer.**

Open a terminal in the `Neuro-Exo` repository root and run:

```bash
python3 H_Robotics_Files/read_outputs.py demo
```

Expect labeled sample lines. This checks Python and the display using synthetic
data; it does not check the board or Bluetooth connection. The demo also shows
other supported formats which the live EEG/EOG debug stream does not emit.

**3. Identify and pair the Bluetooth adapters.**

On each device, run:

```bash
bluetoothctl show
```

Record each device's `Controller` MAC address. `No default controller available`
means Bluetooth setup must be resolved on that device before continuing.
Use the receiving computer's Bluetooth address in the sender configuration;
an IP address or Wi-Fi MAC is not interchangeable with it.

For a first pairing, open `bluetoothctl` on the receiving computer and enter
these commands at its interactive prompt:

```text
power on
agent on
default-agent
pairable on
discoverable on
show
```

Keep this window open to answer pairing prompts. In a BeagleBone terminal,
open `bluetoothctl` and enter:

```text
power on
agent on
default-agent
scan on
```

Wait for the receiving computer to appear. Replace the example address below
with that computer's Bluetooth MAC, then enter the following commands one at
a time. Complete pairing prompts on both devices before proceeding:

```text
pair AA:BB:CC:DD:EE:FF
trust AA:BB:CC:DD:EE:FF
info AA:BB:CC:DD:EE:FF
scan off
quit
```

Confirm `Paired: yes` and `Trusted: yes`. If already paired, use `info` to check
the existing pairing instead of repeating `pair`. On the receiver's
`bluetoothctl` prompt, trust the BeagleBone using the BeagleBone's MAC:

```text
trust 11:22:33:44:55:66
info 11:22:33:44:55:66
discoverable off
quit
```

Both MAC addresses above are placeholders. These are Bluetooth setup commands
documented by [BlueZ](https://github.com/bluez/bluez/blob/master/doc/bluetoothctl.rst).
The C++ application will make the actual RFCOMM connection directly to channel
1. The reader does not advertise an SDP service, so a generic Bluetooth
profile-connect button is not the test for this data connection.

**4. Set the receiving address in the complete BeagleBone application.**

In the application's `main.cpp`, configure `top_hI` before acquisition. For
example, add the second line immediately after the existing debug-mode setup:

```cpp
top_hI.debugMode();
top_hI.setBluetoothDevice("AA:BB:CC:DD:EE:FF", 1);
```

Use the receiving computer's actual Bluetooth MAC. Rebuild and deploy the
complete application using its own build instructions. This is a required
configuration example, not a change applied to the C++ source by this guide.
The supplied source leaves the address empty and never calls this setter.

**5. Start the receiver before starting acquisition.**

In the receiver terminal, from the repository root:

```bash
python3 H_Robotics_Files/read_outputs.py bluetooth --channel 1
```

Expected startup message:

```text
Waiting for one RFCOMM sender on 00:00:00:00:00:00, channel 1.
Configure the C++ sender with this computer's Bluetooth MAC address.
```

`00:00:00:00:00:00` means listen on any local Bluetooth adapter. Do not put that
address into the BeagleBone's sender configuration. If the receiver has
multiple adapters, select one explicitly:

```bash
python3 H_Robotics_Files/read_outputs.py bluetooth --bind AA:BB:CC:DD:EE:FF --channel 1
```

To display and save labeled output, use this command instead of the first one:

```bash
python3 H_Robotics_Files/read_outputs.py bluetooth --channel 1 | tee -a beaglebone_capture.txt
```

`tee -a` appends displayed records to the file. Connection messages and errors
still appear in the terminal on stderr. Run only one listener on the selected
adapter/channel.

**6. Start acquisition and check for live messages.**

In the BeagleBone terminal, change to the complete application's working
directory, then launch its executable. For an executable named `main`:

```bash
./main
```

After initialization, type the following into the running C++ application's
terminal and press Enter:

```text
9
```

The source maps `9` to EEG/EOG debug acquisition. Expect a Bluetooth connection
message on the board and `Bluetooth sender connected: ...` on the receiver,
followed by repeated `[EEG/EOG]` lines with all eight values. Use exact numeric
commands; the supplied application does not handle arbitrary text input.

**The Python Bluetooth reader receives telemetry only.** Type application
commands into the BeagleBone terminal. Typing `9` into the Python listener does
not send it to the board. Its `tcp` mode is a separate robot-arm status poller
and does not start BeagleBone EEG acquisition.

**7. End the session.**

In the running BeagleBone application, enter `7` and press Enter to request
the end of the current acquisition stage. The supplied acquisition loop checks
that flag and then disconnects Bluetooth. This is an application stage command;
it is not a motor-stop command.

The reader may report `Reader error: peer disconnected` when the sender closes
the connection. If the reader is still running, press Ctrl+C in its terminal.
Ctrl+C in the reader closes the viewing connection; it sends no stop command
to the BeagleBone. For another session, restart the application and listener:
the supplied code does not visibly reset the end-stage flag, and the reader
accepts one connection per run.

**If no data appears**

| What you see | What to check |
|---|---|
| Reader stays at `Waiting for one RFCOMM sender...` | Confirm the C++ app reached acquisition after `9`, the receiver MAC is configured in the deployed build, both channels are 1, and both adapters are powered and paired. |
| Board prints `Bluetooth address not configured...` | Add the setter in step 4 and rebuild the complete application. |
| Board prints `BT connect` or `Failed to connect...` | Start the listener first; check the receiver MAC/channel, pairing, range, and adapter permissions. The acquisition loop does not automatically retry a failed initial connection. |
| Python reports missing Linux BlueZ RFCOMM support | Use Linux with the required Python socket support and a usable Bluetooth adapter. |
| `No default controller available` or an adapter/device error | Check that an adapter is present, enabled, and supported by the OS; a USB cable to the board alone does not create the Bluetooth stream. |
| Permission error | Resolve the Linux user's adapter/socket permissions for the installed distribution. |
| Connected, but no `[EEG/EOG]` lines | Inspect board-side errors and sensor initialization. The sender must write complete records ending in a newline, and the loop must still be running. |
| C++ build fails on a missing header | Restore the complete acquisition project and address its source errors; the Python reader cannot repair the producer. |

The reader's demo and command-line options were checked locally. Live
Bluetooth reception and sensor acquisition have not been verified with a
connected BeagleBone in this workspace.
