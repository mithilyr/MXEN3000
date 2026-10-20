# Track Map - estimated rover position

Two ways to use it:
- `TrackMap.exe` - its own window, live or replay (this folder, below).
- `RoverControlMap.exe` - the full rover GUI with the map as a tile on the right, live only (`../gui/rover_gui_map.py`, built by `../gui/build_exe_map.bat`). `RoverControl.exe` stays exactly as it was.

Both share the saved speed scale in `settings	rack_map.json` next to the .exe.

A separate program from the rover GUI (`../gui/`). It shows a dot moving around the stadium track during a run, and lets you replay old runs. The rover GUI is not changed in any way.

## How it gets the data

The rover GUI writes one `ir-log_<mode>_<controller>_<date>_<time>.csv` per run, in the `logs` folder next to the .exe, with every telemetry sample. Track Map reads those files. It never opens the XBee COM port, so both programs run at the same time.

## How the position is worked out

The rover has no wheel encoders, so the position is an **estimate**, not a measurement:

1. Distance along the line = full-speed scale (cm/s) x the average wheel speed the firmware commanded x time.
2. On the straights the corrections go both ways; in the bends they mostly lean to one side. When the lean starts or stops near a bend entry or exit, the dot jumps there (allowing for the detection delay). This removes most of the drift, four times per lap. Untick "Snap to the bend ends" to see the raw estimate.

Known limits: battery level, wheel slip and pivot turns change the real speed, and a pivot-heavy run (e.g. Reverse with a forward correction wheel) is under-estimated. Squiggle is not mapped (different track).

## Use

1. Build once: double-click `build_track_map.bat`. It puts `TrackMap.exe` in `..\gui\dist\`, next to `RoverControl.exe`.
2. Open `RoverControl.exe` and `TrackMap.exe`.
3. Start a run in the rover GUI. With "Follow live runs" ticked, the map switches to it by itself (LIVE).
4. To replay: click a run in the list, then Play, or drag the slider.
5. If the logs are somewhere else, press "Log folder..." once. It is remembered in `track_map.json`.

## Calibrate (once per division, about 2 minutes)

1. Do a run and note where the rover was when it stopped (or any moment you know).
2. Open that run, drag the slider to that moment.
3. Press "Set true position", then click that spot on the line.

The scale (cm/s at full commanded speed) is saved per division in `track_map.json`.

## Files

- `track_map.py` - the window (Tkinter, no extra packages).
- `map_view.py` - the map canvas and saved settings, shared by both programs.
- `track_model.py` - stadium geometry (140 cm straights, 50 cm outer width, 3 cm line; lap ≈ 428 cm) and the estimator. All track dimensions are at the top.
- `test_track_model.py` - `python test_track_model.py` (13 tests).
- `build_track_map.bat` - builds the single .exe.
