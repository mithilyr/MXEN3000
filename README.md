# rover2026 — wireless line-following rover (MXEN3000, S2 2026)

Two complete, separate versions of the rover code. Each one has its own
firmware and its own GUI, and neither imports anything from the other.

| Folder | Where the duty cycles are computed | Nano firmware | PC program |
|---|---|---|---|
| [`pc-control/`](pc-control/README.md) | **On the PC** (meets the 20-Oct-2026 instructor rule) | `rover_io.ino` — I/O only | `RoverControlPC.exe` |
| [`onboard-control/`](onboard-control/README.md) | On the Nano (the original build, used on test days 19–20 Oct) | `rover2026.ino` — full controller | `RoverControlMap.exe` / `RoverControl.exe` |

Both use the same hardware, wiring, XBee pairing (38400 baud) and GUI keys.
Flash the firmware that matches the GUI you run: the two radio protocols are
different.

```
rover2026/
  pc-control/        Version B: firmware/rover_io + gui/ (controller in gui/pc_brain.py)
  onboard-control/   Version A: firmware/rover2026 + gui/ (controller in the .ino)
  tools/
    xbee_bridge/     USB <-> XBee pass-through sketch for the GUI's "Pair XBees" window
    track_map/       TrackMap.exe: replays the GUI's ir-log CSVs on the track map
  docs/
    gui_preview.png  screenshot of the GUI (simulated data)
```

Run the tests for each version from its `gui/` folder:
`python -m unittest discover`.
