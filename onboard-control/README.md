# onboard-control — controller runs on the Nano

## FINAL DEMO BUILD (27 Oct 2026) — build and run it on a new PC

The code used at the final demonstration is the **Front + Reverse, bang-bang only** pair:

| Part | File |
|---|---|
| Firmware (Nano) | `firmware/rover2026_front_reverse/` (`.ino`, `config.h`, `protocol.h`) |
| GUI source | `gui/rover_gui_front_reverse.py` |
| Build script | `gui/build_exe_front_reverse.bat` → `gui/dist/RoverControl_front_reverse.exe` |

Earlier builds (all modes, PID/adaptive, map GUI, 24-Oct fallback) were removed from this folder on 27 Oct 2026. They are kept in git history under the tag `pre-cleanup-archive-27oct2026` (`git checkout pre-cleanup-archive-27oct2026`).

### 1. Get the code

```
git clone https://github.com/mithilyr/MXEN3000.git
cd MXEN3000/onboard-control
```

(If you cloned the outer course repo instead, the same folder is `labs/lab4-final-vehicle/code/rover2026/onboard-control`.)

### 2. Upload the firmware (about 5 minutes)

1. Install the Arduino IDE. No extra libraries are needed.
2. Open `firmware/rover2026_front_reverse/rover2026_front_reverse.ino`. Keep `config.h` and `protocol.h` in the same folder.
3. Board **Arduino Nano**, processor **ATmega328P** (try "Old Bootloader" if the upload fails), then pick the Nano's COM port.
4. Switch both levers of DIP switch **S6** off, press **Upload**, then switch S6 back on.

### 3. Build the GUI .exe (about 3 minutes)

Needs Windows and 64-bit **Python 3.10 or newer from python.org** with "Add python.exe to PATH" ticked. Tkinter comes with that installer; the Microsoft Store Python can fail.

1. Open Command Prompt in `gui/`.
2. Run:

```
build_exe_front_reverse.bat
```

The script installs `pyserial`, `pillow` and `pyinstaller` from `requirements.txt`, then runs PyInstaller. The result is `gui\dist\RoverControl_front_reverse.exe` (about 32 MB, one file, no install needed; copy it to any PC). Antivirus may flag it on first run because it is an unsigned PyInstaller file.

All files the build needs are in `gui/`: `rover_gui_front_reverse.py`, `protocol.py`, `glass.py`, `log_guide.py`, `optimizer.py`, `telemetry_stats.py`, `xbee_setup.py`, `requirements.txt`. The `.exe` creates `logs/` and `settings/` next to itself on first run.

No build, just run from source:

```
pip install -r requirements.txt
python rover_gui_front_reverse.py
```

### 4. Run

1. Plug in the PC XBee, start `RoverControl_front_reverse.exe`, pick its COM port, press **Connect**. The top bar turns green ("Link OK").
2. Keys: `1` Front, `2` Reverse, `Space` start/stop, `Esc` emergency stop.
3. Sensor placement, XBee setup and wiring are below.

The line-following controller runs on the rover every 5 ms (bang-bang), and the PC GUI only starts/stops it and shows the sensor data. The PC and the rover talk over a pair of XBees in transparent mode. The track is a stadium-shaped white line on a black table. This build covers two runs: **Front** (forward) and **Reverse**.

```
firmware/rover2026_front_reverse/
  rover2026_front_reverse.ino   main loop, line follower, XBee link
  config.h                      pins, XBee options, Front/Reverse tuning  <- tune here
  protocol.h                    frame format + command numbers (mirrors gui/protocol.py)
gui/
  rover_gui_front_reverse.py    Tkinter control panel (keyboard driven)
  protocol.py                   frame encode/decode (mirrors protocol.h)
  optimizer.py, telemetry_stats.py, log_guide.py   GUI helpers
  glass.py                      window styling
  xbee_setup.py                 the "Pair XBees" window's XBee programming
  test_*.py                     unit tests (python -m unittest discover)
  build_exe_front_reverse.bat   -> dist/RoverControl_front_reverse.exe
  requirements.txt
```

The version where all duty-cycle processing runs on the PC is in [`../pc-control/`](../pc-control/README.md).

## Sensor placement (2 sensors only)

Recommended:

1. **Both at the front**, on the rover's centreline, about 5–8 cm ahead of the drive-wheel axle. More distance gives earlier warning on curves, but makes the rover cut corners.
2. **Spacing (centre to centre): 3 cm, equal to the 3 cm line width (the plan).** Each sensor then sits over one edge of the line when the rover is centred, so it reads grey, not fully white or black, which gives a smooth proportional error (analog PD, the default). Keep the spacing close to 3 cm and keep both sensors the same height. If you ever switch to the adaptive controller, the sensors must be wider than the line, about 4-5 cm.
3. **Height about 3–8 mm** above the table (check the sensor datasheet; the lecture slide gives "0.12–0.18", most likely inches, which is 3–4.6 mm). Shroud the sensors from room light.

Trade-off: the front position is right for Clockwise (compulsory, 6 marks), Anticlockwise and Squiggle. In Reverse the sensors trail behind the wheels, which is harder to keep stable. With only 2 sensors, **Squiggle or Anticlockwise is the safer choice for the 3-mark division**. Reverse is still implemented, so you can try it in practice.

## XBee setup

Follows the unit's `../../Xbee Pairing Guide.pdf` (XBee S2C, transparent mode, 38400 baud).

**Easiest: the GUI's Pair XBees button.** It sends the same settings as the guide's XCTU steps, no XCTU needed. Hardware as in guide steps 1-2: XBee on the V0 shield, shield switch on **USB**, empty sketch uploaded to that Arduino. On a breadboard instead of the V0 shield, wire it as a pass-through: XBee DIN to Arduino **D0 (RX)**, XBee DOUT to **D1 (TX)** (the opposite of normal use), XBee on 3.3 V, and a 5 V-to-3.3 V divider on DIN unless the XBee sits on a 5 V adapter board.

1. PC XBee on the shield. Press **Pair XBees**, pick the Arduino's port, press **Program PC XBee**.
2. Swap in the rover XBee, press **Program rover XBee**. It gets the PC XBee's serial number as its destination.
3. Swap the PC XBee back in, press **Program PC XBee** again. It gets the rover's serial number.
4. The window says "Paired". Put each XBee on its own board and press **Connect**.

It finds the XBee at 38400 or any other baud rate, writes all settings at once (ATWR, guide step 17), reads them back to check them, remembers both serial numbers in `settings¾e.json`, and refuses to program the wrong XBee in step 3.

**Zigbee firmware:** the guide sets CE = 0 on both. That links on 802.15.4 firmware, but on Zigbee firmware (the guide's screenshot shows "ZIGBEE TH Reg") one XBee must be the coordinator (CE = 1). The GUI checks the firmware (ATZS): on Zigbee it ticks **PC XBee as coordinator** and sets CE = 1 on the PC XBee in the third pass.

**By hand (XCTU):** follow the guide. The settings it ends up with:

| Setting | PC XBee | Rover XBee | Guide step |
|---|---|---|---|
| ID (PAN ID) | random, same on both | same | 11, 19 |
| CE | `0` (`1` on Zigbee firmware) | `0` | 12 |
| NI | `PC` | `ROVER` | 14 |
| BD | `5` (38400) | `5` (38400) | 15 |
| AP | `0` (transparent) | `0` | 16 |
| DH / DL | rover's SH / SL | PC's SH / SL | 20, 22 |

The firmware (`LINK_BAUD` in `config.h`) and the GUI both use 38400 to match.

## Wiring (checked against the PCB schematics in `docs/pcb details/`)

- **XBee:** the PCB's XBee connector J10 reaches Nano D0 (RX) / D1 (TX) through DIP switch S6, which matches the default `config.h` (`LINK_USE_SOFTWARE_SERIAL 0`). J10 supplies +5 V, so the XBee must sit on a 5 V adapter board (regulator + level shifting). Turn S6 off to upload over USB. `LINK_USE_SOFTWARE_SERIAL 1` (D12 RX / D13 TX) is only for a hand-wired XBee that bypasses J10; D12 and D13 are unused on the PCB.
- **DACs (LSB first):** the second board's DAC (called left in `config.h`) = D9, D8, D7, D6, D5, D4, D3, D2 (changed 2026-10-12; the schematic order was D2, D3, D4, D5, D9, D8, D7, D6). The first board's DAC (right) = A2, A3, A4, A5, A1, A0, D11, D10, which is the DAC0800 data inputs B8..B1 order from the schematic. Which DAC drives which physical wheel is not in the schematics: confirm it with the bench test and swap the pin arrays or flip `LEFT_MOTOR_DIR` / `RIGHT_MOTOR_DIR` if needed.
- **IR sensors:** connector J8 = GND, A6, A7, 5 V (LED supply through R2 180 Ω). A6 and A7 have 10 kΩ pull-ups (R3, R4) and no comparator, so the readings are analog: white line = low, black table = high (`WHITE_READS_HIGH false`). The firmware calls A6 left and A7 right; if the GUI shows them swapped, swap `IR_LEFT_PIN` / `IR_RIGHT_PIN`.
- **PWM jumpers (not set by firmware):** each DAC output must reach the LM393 comparator against the ramp. Board 1: J7 on 1-2 (DAC → +RAMP1), J1 on 2-3, J5 on 2-3 (ramp, not +5 V). Board 2: J12 on 1-2 (-RAMP2). With the wrong jumpers a wheel ignores its DAC.
- **Power:** the Nano runs from VIN (L7809 + diode D1); the motor stages from the ±15 V input J14. DAC 128 ≈ 50 % duty ≈ 0 V at the motor.
