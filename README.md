# rover2026 — wireless line-following rover (MXEN3000, S2 2026)

> **Final demo build (27 Oct 2026):** the code used at the final demonstration is the
> **Front + Reverse, bang-bang only** pair in [`onboard-control/`](onboard-control/README.md):
> firmware `onboard-control/firmware/rover2026_front_reverse/` and GUI
> `onboard-control/gui/rover_gui_front_reverse.py`. The .exe is not stored in git; build it
> with `onboard-control/gui/build_exe_front_reverse.bat`.

## Build the final demo GUI (about 3 minutes)

1. Install 64-bit Python 3.10 or newer from python.org (tick "Add python.exe to PATH").
2. Clone and build:

```
git clone https://github.com/mithilyr/MXEN3000.git
cd MXEN3000\onboard-control\gui
build_exe_front_reverse.bat
```

3. Run `dist\RoverControl_front_reverse.exe`. To upload the matching firmware to the Nano,
   open `onboard-control/firmware/rover2026_front_reverse/rover2026_front_reverse.ino` in the
   Arduino IDE. Full steps: [`onboard-control/README.md`](onboard-control/README.md).

## Versions in this repo

Two complete, separate versions of the rover code. Earlier onboard-control builds were removed on 27 Oct 2026 and are in git history under the tag `pre-cleanup-archive-27oct2026`.
Each version has its own firmware and its own GUI, and neither imports anything from the other.

| Folder | Where the duty cycles are computed | Nano firmware | PC program |
|---|---|---|---|
| [`pc-control/`](pc-control/README.md) | **On the PC** (meets the 20-Oct-2026 instructor rule) | `rover_io.ino` — I/O only | `RoverControlPC.exe` |
| [`onboard-control/`](onboard-control/README.md) | On the Nano (**the final demo build**) | `rover2026_front_reverse.ino` — controller on the Nano | `RoverControl_front_reverse.exe` |

Both use the same hardware, wiring, XBee pairing (38400 baud) and GUI keys.
Flash the firmware that matches the GUI you run: the two radio protocols are
different.

- `rover2026/`
  - `pc-control/` — Version B: `firmware/rover_io` + `gui/` (controller in `gui/pc_brain.py`)
  - **`onboard-control/` — Version A (FINAL, used at the demo)**
    - **`firmware/rover2026_front_reverse/`** — Nano firmware: `rover2026_front_reverse.ino` (main loop, line follower, XBee link), `config.h` (pins, tuning), `protocol.h` (frame format)
    - **`gui/`** — PC control panel: `rover_gui_front_reverse.py`, helpers (`protocol.py`, `optimizer.py`, `telemetry_stats.py`, `log_guide.py`, `glass.py`, `xbee_setup.py`), unit tests, `requirements.txt`
    - **`gui/build_exe_front_reverse.bat`** — builds `dist/RoverControl_front_reverse.exe`
    - **`README.md`** — full build, upload and run guide
  - `tools/`
    - **`xbee_bridge/`** — USB <-> XBee pass-through sketch for the GUI's "Pair XBees" window
    - `track_map/` — `TrackMap.exe`: replays the GUI's ir-log CSVs on the track map
  - `docs/`
    - `gui_preview.png` — screenshot of the GUI (simulated data)

Run the tests for each version from its `gui/` folder:
`python -m unittest discover`.
