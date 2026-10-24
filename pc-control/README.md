# pc-control — all duty-cycle processing on the PC

**Version B** of the rover2026 code. It follows the instructor's requirement
(20-Oct-2026): *"all your duty cycle processing should be done in the pc. The
onboard nano is only there for communication and not for processing."*

This folder is self-contained: its own firmware and its own GUI, with no
imports from `../onboard-control/`. Since 21-Oct-2026 `RoverControlPC.exe` is
built from `rover_gui_min.py`, a black, monospace terminal-style panel with the same keys, logs, tuning, optimiser, run
history, Pair XBees and PWM test windows, but the live display is plain numbers
at 5 Hz (no track map, scorecard, sensor chart, level bars or PC CONTROL LOOP
panel). The full panel is still in `rover_gui.py` / `rover_gui_map.py`.

```
firmware/rover_io/
  rover_io.ino      Nano: read IR -> send; receive DAC codes -> write DACs. No control.
gui/
  pc_brain.py       THE CONTROLLER: calibration, PID / bang-bang / adaptive,
                    lost-line search, MODE_TUNING table  <- tune here
  io_protocol.py    radio frames to/from rover_io.ino
  protocol.py       GUI <-> controller messages (commands, telemetry, tuning)
  rover_gui.py      Tkinter control panel (keyboard driven)
  rover_gui_min.py  terminal-style minimal panel over rover_gui.py, numbers only (entry point)
  rover_gui_map.py  the full panel plus the track map + scorecard (no longer built)
  optimizer.py, track_model.py, map_view.py, glass.py, xbee_setup.py
  test_*.py         unit tests (python -m unittest discover)
  build_exe.bat     -> dist/RoverControlPC.exe
  requirements.txt
```

## Who does what

| | Nano (`rover_io.ino`) | PC (`pc_brain.py`) |
|---|---|---|
| Reads IR sensors | yes, raw `analogRead`, sent every 25 ms | — |
| Calibration, normalising, white/black decision | — | yes |
| Controller (PID / bang-bang / adaptive) | — | yes |
| Lost-line backtrack + spin search | — | yes |
| Duty cycle → DAC code for each wheel | — | yes |
| Writes DAC codes to the R-2R pins | yes, exactly as received | — |
| Safety | motors to neutral if no PC frame for 300 ms | rides through gaps up to 2 s, ends the run after that |

## Radio link (XBee, 38400 baud, same pairing as Version A)

- Rover → PC, 9 bytes: `[FF][30][irLeft][irRight][dacLeft][dacRight][seq][ackSeq][checksum]`
- PC → rover, 6 bytes: `[FF][31][dacLeft][dacRight][seq][checksum]`

Each sensor frame runs one control step on the PC, which answers with one
motor frame. `ackSeq` echoes the last motor frame the rover applied, so the PC
can measure the radio round trip and count lost frames (the Nano only copies
the number). Constants are in `gui/io_protocol.py` and at the top of
`rover_io.ino`; keep them identical.

### Keeping the radio smooth

The XBees (Zigbee, unicast with acknowledgements and retries) handle a few
dozen small packets a second, so every packet saved lowers latency:

- **Motor frames only when needed.** The PC sends the DAC codes only when they
  change, plus a 100 ms keepalive for the rover's 300 ms watchdog. Heartbeats
  and tuning changes cause no radio traffic. Measured in simulation: bang-bang
  drops from 44 to 11 motor frames a second; PID changes every step (35/s).
- **Newest frame wins.** If the radio delivers several sensor frames at once,
  only the newest is acted on (the others are counted as "Bunched"), so the
  rover never gets a queue of out-of-date commands.
- **No polling delay.** The reader blocks until a byte arrives instead of
  sleeping between checks. Simulated round trip max dropped from about 90 ms
  to 28 ms.
- **Dropout ride-through.** A gap of 0.5–2 s (the rover is already neutral)
  clears the controller's memory (integral, derivative, backtrack history) and
  carries on when frames return; longer gaps end the run.
- **Radio check.** "Pair XBees" → Read settings now also shows CH (channel),
  PL (power, 4 = max), PM (boost, 1 = on), SM (sleep, must be 0) and DB
  (signal strength of the last packet, hex -dBm: lower is stronger). These are
  read only; pairing writes the same settings as before.

## Use it

1. Open `firmware/rover_io/rover_io.ino` in the Arduino IDE (board Arduino Nano, ATmega328P). Switch DIP switch S6 off, upload, switch S6 back on.
2. Run `gui/build_exe.bat`, then start `gui/dist/RoverControlPC.exe` (or `python rover_gui_min.py` from `gui/`).
3. Connect, calibrate (C), pick a mode (1–4), Space to start, Esc to stop — as in Version A.
4. Tune live, press **Copy for pc_brain.py**, and paste the values into `MODE_TUNING` in `gui/pc_brain.py`.

Saved tuning, PWM levels and XBee pairing are kept in `settings/` next to the
.exe. Only `xbee.json` is copied over from Version A. Version A's
`tuning.json` and `pwm_levels.json` would override the delay-adjusted defaults
below, so copy them only if you want the onboard test-day values back.

## PC CONTROL LOOP panel

- **Pipeline:** ROVER · IR sensors → XBee → **PC · controller** (controller,
  position, the duty % it computed) → XBee → ROVER · DACs (codes the rover
  reports it applied).
- **Link health** over the last 2 s, green / orange / red: control rate (Hz),
  radio round trip (ms, median and max), lost sensor frames, bunched frames,
  throttled motor frames, dropouts ridden through (since connect), PC step time.
- **Verdict line:** says when the loop is healthy enough to try
  `SENSOR_PERIOD_MS 20` in `rover_io.ino`, or when the radio is dropping frames.

## Overflow protection (keeps the XBees connected)

The XBees have no flow control, so both sides limit their own output:

- **PC:** at most one motor frame per `MIN_MOTOR_GAP_S` (40 ms); while the OS
  transmit buffer holds more than `MAX_TX_BACKLOG_BYTES` the frame is held back
  and only the newest codes are sent once it drains; a stuck write times out
  after `WRITE_TIMEOUT_S` and the buffer is flushed instead of blocking the loop;
  the receive buffer is capped at `MAX_BUFFER_BYTES`. Esc/STOP always goes at once.
  Held-back frames show as **Throttled** in the link health.
- **Rover:** a sensor frame is skipped (and counted as lost) if the serial
  transmit buffer cannot take it whole, and while no motor frame arrives (PC
  closed, out of range) sensor frames drop from 40 Hz to 5 Hz
  (`IDLE_SENSOR_PERIOD_MS`) so a dead link is not flooded. The fast rate
  resumes at the first reply. Re-flash `rover_io.ino` to get this.

## Expect on the track

The control loop runs at about 40 Hz over the radio (Version A: 200 Hz on the
Nano), plus one radio round trip of delay. The starting tuning in
`pc_brain.py` allows for that:

- All four modes start on bang-bang (least affected by delay); PID and
  adaptive stay available with `T`. Clockwise, Anticlockwise and Squiggle use
  duty levels 90 / 90, slow side 55; Reverse uses 30 / 30, slow side 45.
- PID `kd` is halved in every mode; Squiggle's base speed is 30 % (was 35).

The loop times itself with `time.perf_counter()`: on Windows
`time.monotonic()` only ticks every 15.6 ms, which would make the step time
jump between 0 and 16 ms.

## Checks done

- `python -m unittest discover` in `gui/`: 73 tests pass, including the PC controller, radio frames, link health, keepalive, newest-frame and dropout logic.
- GUI run against a simulated rover (15 ms radio delay each way): 38 Hz control rate, 26 ms round trip (max 28), 0 lost frames, under 3 ms PC step time; Clockwise bang-bang sent 70 % / 55 % duty; Esc sent neutral at once.
- `build_exe.bat` builds `RoverControlPC.exe` (about 32 MB). `dist/settings/xbee.json` is copied from Version A so the XBee pairing carries over.
- `rover_io.ino` (simplified 21-Oct-2026: 4-sample IR averaging, DACs written only on change, XBee on D0/D1 only) compiles cleanly with the Arduino AVR core 1.8.8 (Nano, no warnings); not yet confirmed on the rover.
