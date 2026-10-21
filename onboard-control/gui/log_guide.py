"""The guide written as logs/README-logs.md next to the run logs, so anyone
(or Claude) reading the logs later knows what every file and column means and
how to use them to cut the lap time. Rewritten each time the GUI starts, so it
always matches the columns this build writes."""

import os

GUIDE_FILE = "README-logs.md"

GUIDE = """# Rover 2026 run logs (onboard-control, RoverControlMap.exe)

Written by the GUI. Goal of the data: **the fastest clean lap in each division**
(Clockwise, Squiggle, Anticlockwise, Reverse). The controller runs on the Nano
every 5 ms; the PC sees telemetry at 10 Hz, but the debug figures marked
"window" cover every 5 ms step since the previous frame.

## Files

| File | One row per | Use it for |
|---|---|---|
| `run-stats.csv` | run | **Start here.** Lap time, result, the settings the rover was using, and the figures that explain the time. |
| `run-log.csv` | run | Same runs, competition format (`date, division, attempt, time_s, result, notes, track, note`). `note` = what the operator typed after the run; `notes` = tuning summary. |
| `ir-log_<Mode>_<Controller>_<Track>_<YYYYMMDD_HHMMSS>.csv` | telemetry sample (~10 Hz) | What happened when: find where the line was lost, where it weaved, which wheel braked. The file name is in `run-stats.csv` column `ir_log`. |
| `../settings/optimizer.json` | rated run | Operator's rating: `clean` true = valid lap (Y), false = failed (N). Matches runs by `date`/`mode`/`time_s`. |
| `../settings/tuning.json`, `../settings/pwm_levels.json` | mode | Last saved live tuning and bang-bang wheel levels per mode. |

## run-stats.csv columns

Identity: `date` (local time the run ended), `division`, `attempt` (count this session),
`controller` (PID / Bang-bang / Adaptive 1/0), `track` (Test / Competition), `time_s`
(START to STOP or self-stop), `result` (`stopped by operator`, `stopped`, `line lost`).

Lap figures (the ones that explain the time):

| Column | Meaning | Read it as |
|---|---|---|
| `avg_speed` | mean of both wheels' speed, % of full (-100..100) | the real pace; far below `base_speed` = corners are braking it |
| `lost_events`, `lost_s` | times the line was lost, seconds spent searching | the biggest time loss; get to 0 before raising speed |
| `saturated_pct` | % of 5 ms steps with the steering clamped at `max_turn` | > 30 %: raise `max_turn` or lower `kp`; ~0 with lost lines: turns too gently |
| `weave_hz` | side-to-side swings per second (from 10 Hz samples, so up to ~5 Hz) | high with low `rms_pos` = oscillating on straights: lower `kp` / raise `kd` |
| `rms_pos`, `max_pos` | RMS and largest line position error (0 = centred, 1 = fully off one sensor) | high = corrects too late |
| `avg_duty_l`, `avg_duty_r` | mean PWM duty per wheel, % (50 = stopped, 100 = full forward) | unequal on a symmetric track = one motor weaker (drift) |
| `min_duty_l`, `min_duty_r` | lowest duty per wheel | how hard the tightest corner braked that wheel |

Health (rule these out before blaming the tuning):
`step_max_ms`, `period_max_ms` (control loop; target period 5 ms, > 10 ms = the Nano stalled),
`frames_lost` (debug frames lost on the radio), `gap_max_ms` (longest gap between telemetry frames),
`checksum_errors` (damaged frames during the run), `rx_bytes_per_s`, `commands` (commands sent
during the run).

Settings the rover reported using (its tuning echo, not what was requested):
`mode`, `base_speed` (%), `kp`, `ki`, `kd`, `max_turn` (%), `bang_level` (not used by
bang-bang in this firmware), `adapt_gain`, `adapt_ramp`, `pwm_straight_l/r`, `pwm_correct_l/r`
(bang-bang wheel duty %: centred / slowed side).

Sensors and link: `cal_result` (defaults / calibrated / rejected - rejected means the last C
sweep was too small and config.h levels are still in use), `white_l/r`, `black_l/r` (raw ADC
levels the rover normalises with), `port`, `pan_id`, `ir_log` (this run's sample file).

## ir-log columns (one row per telemetry frame)

| Column | Meaning |
|---|---|
| `time_s` | seconds since START |
| `ir_left`, `ir_right` | raw ADC (0-1020; white line reads LOW) |
| `n_left`, `n_right` | normalised level the controller uses: 0 = white, 1 = black (white < 0.35, black > 0.65) |
| `left_white`, `right_white` | the controller's 1/0 decision per sensor (1 = on the line) |
| `position` | -1..1, > 0 = the line is to the LEFT |
| `dac_left`, `dac_right` | DAC code sent to each motor (128 = stopped, 255 = full forward) |
| `duty_left`, `duty_right` | the same as PWM duty % (50 = stopped) |
| `speed_left`, `speed_right` | the same as wheel speed % (-100..100) |
| `speed` | base speed setting, % |
| `line_lost` | 1 while the rover reports the line lost |
| `frame_gap_ms` | time since the previous telemetry frame on the PC |
| `state` | what the controller was doing: FOLLOW, LOST: BACKTRACK / SPIN / CORNER SWEEP, CORNER HOLD, ADAPTIVE TURN, BANG STRAIGHT / TURN LEFT / TURN RIGHT / BOTH BLACK, IDLE |
| `p_term`, `i_term`, `d_term`, `turn` | PID contributions and the actual turn, % (> 0 = clockwise / right) |
| `saturated` | 1 if the steering was clamped at max turn on the last step |
| `adapt_rate` | adaptive controller's correction rate, % |
| `saturated_pct`, `max_error`, `step_max_ms`, `period_max_ms` | window figures over all 5 ms steps since the previous frame |
| `lost_events` | lost-line count so far this run |
| `debug_seq` | debug frame number (gaps = frames lost on the radio) |

Debug columns are blank when the rover runs firmware without the debug frame.

## Competition marks (lap time must be under the limit)

Clockwise < 7 s = 5, < 9 = 4, < 11 = 3, < 16 = 2, else 1. Squiggle < 10 = 5, < 14 = 4,
< 17 = 3, < 25 = 2, else 1. Anticlockwise < 7 = 3, < 9 = 2.5, < 11 = 2, < 16 = 1.5, else 1.
Reverse < 14 = 2, < 20 = 1.5, < 30 = 1, else 0.5. A failed lap scores 0.

## Suggested analysis order

1. Per division and controller, list clean laps (optimizer.json) with `time_s` and settings.
2. Runs with `lost_events` > 0: open their ir-log, find the `time_s` of each loss and what
   `position`, `turn`, `saturated` and the wheel duties did in the 0.5 s before it.
3. Compare `avg_speed`, `saturated_pct`, `weave_hz` across the fastest clean laps to choose
   the next `base_speed`, `kp`, `kd`, `max_turn` (or bang-bang duty levels).
4. Discard runs where the health columns show a stalled loop or radio dropouts.
"""


def write(folder):
    """Writes the guide into the logs folder. Errors are ignored: the guide is a help."""
    try:
        with open(os.path.join(folder, GUIDE_FILE), "w", encoding="utf-8") as guide_file:
            guide_file.write(GUIDE)
    except OSError:
        pass
