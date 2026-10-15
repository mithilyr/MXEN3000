# Rover Control GUI - how to start and use it

This program is a control panel for the line-following rover. You use it on a Windows PC. It talks to the rover through a wireless XBee radio plugged into the PC with a USB cable.

You use it to:
- start and stop the rover,
- choose a driving mode,
- change the rover's speed,
- watch what the sensors see,
- time each run and save the results.

## What you need

- A Windows PC.
- The **PC-side XBee**, plugged in with USB.
- The rover, with its own XBee, switched on. The rover needs the `rover2026` firmware uploaded (see `../README.md`).

You can open the GUI without the rover. It will just show "Disconnected" or "NO LINK".

## How to start it

There are two ways. Use **Way A** if the file `dist\RoverControl.exe` is in this folder.

### Way A: the ready-made program

1. Open the `dist` folder.
2. Double-click `RoverControl.exe`.

If Windows shows a blue "protected your PC" box, click **More info**, then **Run anyway**.

### Way B: run from Python

Do this once on a new PC:

1. Install Python 3 from python.org. Tick **"Add Python to PATH"** during install.
2. Open **PowerShell** in this folder. (In File Explorer, click the address bar, type `powershell`, press Enter.)
3. Type this and press Enter:

```
python -m pip install pyserial
```

Every time you want to start the GUI, type this in PowerShell in this folder:

```
python rover_gui.py
```

### Make your own .exe (optional)

Double-click `build_exe.bat`. It takes a few minutes. The new program appears in `dist\RoverControl.exe`. Do this again after any change to `rover_gui.py`.

## Pair the two XBees (once)

Do this if the GUI never shows the green "Link OK". It sets both XBees to the same channel and PAN ID and points them at each other.

1. Plug the **PC** XBee into the USB adapter. Click **Pair XBees** (top bar). If you are connected, it asks to disconnect first.
2. Pick the adapter's port. Keep the PAN ID and channel, or click **New PAN ID**.
3. Click **Program PC XBee**. Wait for "Done" in the log (up to about 25 seconds if the XBee is at an unusual baud rate).
4. Take the XBee off the rover and put it in the same adapter. Click **Program rover XBee**.
5. When the window shows green **Paired**, put the rover XBee back, close the window and connect (below).

**Read settings** shows what an XBee is set to now, without changing it. Close XCTU first: only one program can use the port.

## Connect to the rover

1. Plug the PC-side XBee into USB.
2. In the GUI, click **Refresh**, then pick the XBee in the "XBee port" box.
3. Click **Connect**.
4. Look at the top-right corner:

| What you see | What it means |
|---|---|
| Red "Disconnected" | Not connected yet. Click Connect. |
| Orange "Port open, no telemetry" | The port is open but the rover is not answering. Check the rover is on and both XBees have the same settings. |
| Green "Link OK ... Hz" | Working. You should see about 20 Hz and 0 checksum errors. |

## Keyboard controls

Click once on the GUI window first, so it gets your key presses.

| Key | What it does |
|---|---|
| `1` | Clockwise mode |
| `2` | Anticlockwise mode |
| `3` | Squiggle mode |
| `4` | Reverse mode |
| `Space` | Start or stop the run |
| `Esc` | Emergency stop. Always stops the rover. |
| `Up` | Base speed +5% (works while the rover is running) |
| `Down` | Base speed -5% (works while the rover is running) |
| `C` | Calibrate the sensors |
| `T` | Next steering controller for this mode: PID, Bang-bang, Adaptive 1/0 |
| `Q` / `A` | Tuning value 1 up / down (PID: Kp. Bang-bang: bang level. Adaptive: turn gain) |
| `W` / `S` | Tuning value 2 up / down (PID: Kd. Bang-bang: max turn. Adaptive: ramp) |
| `E` / `D` | Tuning value 3 up / down (PID: Ki. Adaptive: max turn) |
| `R` / `F` | Tuning value 4 up / down (PID: max turn) |

| `Y` / `N` | Rate the last run for the optimiser: clean lap / failed |
| `G` | Optimiser: send the next settings to try |
| `B` | Optimiser: send the best settings found so far |

Keys `1` to `4`, `C`, `T`, `Y`, `N`, `G` and `B` only work while the rover is **stopped**.

## How to do a run

1. Connect (see above) and wait for the green "Link OK".
2. Press `C` (calibrate). Slide the rover slowly across the white line by hand for 3 seconds. The state shows **CALIBRATING**, then **STOPPED**.
3. Press `1`, `2`, `3` or `4` to choose the mode.
4. Put the rover on the line.
5. Press `Space`. The state turns green **RUNNING** and the timer counts up.
6. Press `Space` again, or `Esc`, to stop. If the rover loses the line, it stops itself.

## What is on the screen

- **Top bar:** port selection and the connection status.
- **RUN panel:** the state (RUNNING / STOPPED / CALIBRATING / NO LINK), the run timer, and the Start/Stop and Emergency Stop buttons.
- **MODE panel:** the four modes, the current base speed, the steering controller for this mode, and a Calibrate button.
- **BENCH TEST panel:** one slider per motor, in volts from -15 V to +15 V (0.1 V steps; drag the slider or use - / +). The line under each slider shows the DAC code it becomes (-15 V = 0, 0 V = 128, +15 V = 255) and the voltage that code really gives. **Send** sends one side, **Send both** sends both. Tick **Same voltage on both** to move both sliders together. Use this only with the rover **lifted off the table** and **stopped**. Press **Both to 0 V** to stop both motors.
- **NEXT STEP bar:** one line under the connection bar that says what to do now (connect, calibrate, pick a mode, press Space). It changes as the rover state changes.
- **SENSORS panel:** two lamps showing what each sensor sees (white lamp = "LINE", dark lamp = "ground"), the live IR numbers, and a graph of the last 10 seconds. Orange is the left sensor, blue is the right sensor, yellow is the line position (0 = centred). The colours are chosen so they can be told apart with red-green colour blindness.
- **TUNING panel:** change how the rover steers while it is driving (see "Live tuning" below).
- **RUN HISTORY:** every finished run in this session.

## Live tuning

The TUNING panel changes the rover's steering settings for the **current mode**, immediately, even while it is running. Press `+` / `-`, or use the keys `Q A W S E D R F` (the panel shows which keys go with which value). The panel only shows the values the current controller uses.

| Value | What it does | Controller |
|---|---|---|
| Kp, Kd | Proportional and damping gains | PID |
| Ki | Removes a steady offset on long curves. Start at 0. **Reset integral** clears it. | PID |
| Bang level | How far off-centre (0-1) the line must be before a full turn | Bang-bang |
| Turn gain | Bigger = sharper turn each time a sensor reaches the line | Adaptive 1/0 |
| Ramp /s | Bigger = the turn keeps getting sharper the longer the correction lasts | Adaptive 1/0 |
| Max turn % | The most the rover is allowed to steer (bang-bang: the turn it always uses) | all |

- The number turns **orange** until the rover confirms it. Then it turns white. The value shown is what the rover is really using.
- The settings live in the rover's memory. **A reset or power-off brings back the values in `config.h`.** The GUI saves each change in `tuning.json` (next to the run log), so press **Apply saved** (while stopped) to send them back after a reset.
- **Copy for config.h** puts the current values on the clipboard. When you have a setting you like, paste it into `MODE_TUNING` in `config.h` and upload again, so it is permanent.
- Each run's tuning is written into the **notes** column of `run-log.csv`, so you can see which settings gave which lap time.
- Tuning is per mode: changing the mode shows that mode's own values.

## Lap optimiser

The LAP OPTIMISER panel finds faster settings for the **current mode and controller** from your own timed runs (Bayesian optimisation, see `optimizer.py`). It tunes speed plus that controller's values (PID: Kp, Kd, Ki, max turn).

1. Hand-tune until the rover does one clean lap. Press `Y` after it.
2. Press `G`. The rover gets the next settings to try. The panel shows them and the predicted lap time.
3. Put the rover on the line, press `Space`, and press `Space` again **as it finishes the lap**. The timer is the lap time.
4. Press `Y` if it was a clean lap, or `N` if it lost the line or left the track. Use **Don't count** if something else went wrong (a bump or a late Space press). The GUI won't suggest anything else until you rate the run.
5. Repeat 2-4. About 15-20 rated runs per mode usually find most of the gain.
6. Press `B` to load the best settings. Do 3 clean laps in a row with them, then press **Copy for config.h** and paste them into `MODE_TUNING`.

- **Failed runs count against nearby settings.** In the competition a failed run scores nothing (best of 2 attempts), so `B` picks settings that are reliably fast, not one lucky lap next to a crash.
- **It never makes a big jump.** Each suggestion stays within a small box around the best settings so far. The box grows after a new best lap and shrinks after a failure.
- The search ranges are in `SEARCH_SPACE` at the top of `optimizer.py`. Narrow them if it keeps trying values you know are useless.
- Ratings are kept separately per mode and controller. Changing the controller (`T`) starts a fresh search for that controller.
- **Undo last rating** removes the most recent rating if you pressed the wrong key.
- The battery affects speed. A run on a flat battery is slower for reasons the optimiser can't see, so charge it, or press **Don't count** for runs on a flat battery.

## Saved files

The GUI saves files in the folder you started it from.

- `run-log.csv` - one row per run: date, mode, attempt number, time, result, and the tuning used.
- `tuning.json` - the last tuning for each mode (used by **Apply saved**).
- `optimizer.json` - every rated run (mode, controller, settings, lap time, clean or failed). The optimiser learns from this file, so keep it between sessions.
- `ir-log_<Mode>_<date_time>.csv` - one file per run with the sensor readings, about 20 rows per second. Columns: `time_s, ir_left, ir_right, position, dac_left, dac_right, speed, line_lost`.

You can open both in Excel. Use **Change log file** in the GUI to save somewhere else.

## Safety

- The GUI sends a "still here" message 10 times a second. If the rover hears nothing for about 0.6 seconds (for example the radio drops), it stops by itself and its LED blinks fast.
- Closing the GUI also sends a stop.
- For the first tests, lift the rover so the wheels are off the table.

## If something goes wrong

| Problem | Try this |
|---|---|
| The port list is empty | Plug in the XBee, then click Refresh. |
| "Could not open port" | Another program (for example XCTU) is using the port. Close it. |
| Orange "no telemetry" | Check the rover is on. Check both XBees use the same channel and PAN ID, and 9600 baud. See `../README.md`. |
| Space or keys do nothing | Click on the GUI window first. Keys 1 to 4 and C do nothing while the rover is running. |
| `No module named serial` | Run `python -m pip install pyserial`. |
| The rover steers away from the line | Change `steerSign` in `firmware/rover2026/config.h` for that mode and upload again. |
| A wheel turns the wrong way | Change `LEFT_MOTOR_DIR` or `RIGHT_MOTOR_DIR` in `config.h` and upload again. |

For wiring, XBee settings and tuning, read `../README.md`.
