#!/usr/bin/env python3
"""MXEN3000 S2 2026 - the rover dashboard (RoverControlMap.exe).

The rover_gui.py control panel (same keys, logs, tuning and optimiser files)
laid out as a telemetry dashboard in a green terminal look (theme.py):

- ROVER (rover_view.py): top-down drawing of our rover. IR sensors, tyres and the
  XBee LED light up when active; each part's readings sit beside it.
- WAVEFORMS (scope_view.py): the last 20 s of IR levels, position, wheel PWM
  and the PID terms.
- DATA: only what decides the lap time - controller state, radio and
  calibration health, and the run's figures (live, then the last run).
Everything else the rover and the link report goes to the logs instead
(logs/run-stats.csv per run, ir-log_*.csv per sample; columns explained in
logs/README-logs.md), for working through later.
- SCORECARD, run controls, tuning and the lap optimiser as before.

The controller insides need the firmware with MSG_DEBUG / MSG_CALIBRATION.
rover_gui.py itself still builds RoverControl.exe.

Build a single .exe with build_exe_map.bat (RoverControlMap.exe).
"""

import math
import time
import tkinter as tk
from tkinter import ttk

import protocol as p
import rover_gui as gui
import rover_view as rv
import scope_view as sv
import theme

theme.apply(gui)                # Before anything below reads the fonts and colours
gui.UI_REFRESH_MS = 100         # Telemetry arrives at 10 Hz; refreshing faster only costs CPU

# Competition time bands (labs/competition/README.md): (under this many s, marks),
# fastest band first. All four divisions are run. A time in a gap the table does not
# cover (e.g. 16-18 s Clockwise) gets the next band down.
MARK_BANDS = {
    p.MODE_CLOCKWISE: ((7, 5), (9, 4), (11, 3), (16, 2), (math.inf, 1)),
    p.MODE_SQUIGGLE: ((10, 5), (14, 4), (17, 3), (25, 2), (math.inf, 1)),
    p.MODE_ANTICLOCKWISE: ((7, 3), (9, 2.5), (11, 2), (16, 1.5), (math.inf, 1)),
    p.MODE_REVERSE: ((14, 2), (20, 1.5), (30, 1), (math.inf, 0.5)),
}
SCORECARD_ORDER = (p.MODE_CLOCKWISE, p.MODE_SQUIGGLE, p.MODE_ANTICLOCKWISE, p.MODE_REVERSE)
TABLE_FONT = (theme.FONT, 10)
TARGET_FONT = (theme.FONT, 14, "bold")
TOTAL_FONT = (theme.FONT, 14, "bold")

# ---- Look ----
HEADER_PIXELS_LEFT = ("##.#..##.#", "#..##.#..#", "##..#.##.#")
HEADER_PIXELS_RIGHT = ("#.##..#.##", "#..#.##..#", "#.##.#..##")
STATUS_SQUARE = 12
ROVER_VIEW_WIDTH = 600
RATING_FONT = (theme.FONT, 10, "bold")
HINT_MARGIN = 40                # px
LOG_LABEL_WRAP = 700            # px
RATING_WRAP = 560               # px, "Last run ... was it a clean lap?" above its buttons
OPTIMISER_WRAP = 560            # px

# ---- Waveforms: 20 s at the 10 Hz telemetry rate ----
SCOPE_WIDTH = 600
SCOPE_STRIP_HEIGHT = 48
SCOPE_SAMPLES = gui.CHART_SAMPLES
WHITE_BELOW = 0.35              # config.h hysteresis
BLACK_ABOVE = 0.65
SCOPE_STRIPS = (
    sv.Strip("IR LEVEL n", 0, 1, (sv.Trace("ir_l", "L", theme.LEFT, ".2f"),
                                  sv.Trace("ir_r", "R", theme.RIGHT, ".2f")),
             refs=(WHITE_BELOW, BLACK_ABOVE)),
    sv.Strip("POSITION", -1, 1, (sv.Trace("pos", "pos", theme.ACCENT),), refs=(0,),
             flag="lost"),
    sv.Strip("PWM DUTY", 0, 100, (sv.Trace("duty_l", "L", theme.LEFT, "5.1f"),
                                  sv.Trace("duty_r", "R", theme.RIGHT, "5.1f")),
             refs=(50,), unit="%"),
    sv.Strip("PID TURN", -100, 100, (sv.Trace("p", "P", theme.ACCENT, "+4.0f"),
                                     sv.Trace("i", "I", theme.WARN, "+4.0f"),
                                     sv.Trace("d", "D", theme.SOFT, "+4.0f"),
                                     sv.Trace("turn", "turn", theme.BRIGHT, "+4.0f")),
             refs=(0,), flag="sat", unit="%"),
)

# ---- DATA box: (column, title, rows of (key, label)). Only what decides the lap
# time; everything else is in the logs (logs/README-logs.md). ----
DATA_SECTIONS = (
    (0, "STATUS", (("state", "state"), ("radio", "radio"), ("cal", "calibration"),
                   ("levels", "white/black"), ("pwm", "bang PWM"))),
    (1, "RUN", (("run_time", "run"), ("avg_speed", "avg speed"), ("avg_duty", "avg duty"),
                ("min_duty", "min duty"), ("rms", "rms/max pos"), ("run_sat", "saturated"),
                ("run_lost", "lost line"), ("weave", "weave"), ("run_loop", "loop max"),
                ("run_radio", "radio lost"))),
)
DATA_FONT = (theme.FONT, 10)
DATA_TITLE_FONT = (theme.FONT, 10, "bold")
DATA_LABEL_WIDTH = 12           # Characters
DATA_VALUE_WIDTH = 17
LINK_FRESH_S = 0.3              # XBee LED on the ROVER drawing stays lit this long per frame
# Warning levels
SATURATED_WARN_PCT = 30         # Steering pinned at max turn this often: raise max turn or lower kp
PERIOD_WARN_MS = 7              # Control loop target 5 ms
GAP_BAD_MS = 300                # The GUI calls the link down at 1.2 s


def top_mark_time(mode):
    """Lap time for the top mark in this division."""
    return MARK_BANDS[mode][0][0]


def marks_for(mode, time_s):
    """Marks a clean lap of time_s earns (0 if there is no clean lap)."""
    if time_s is None:
        return 0
    return next(marks for limit, marks in MARK_BANDS[mode] if time_s < limit)


def max_marks(mode):
    return MARK_BANDS[mode][0][1]


def best_clean_laps(rated_runs, track):
    """Fastest lap rated clean (Y) per division name, on this track."""
    best = {}
    for run in rated_runs:
        if run.get("clean") and run.get("track", gui.TRACKS[0]) == track:
            mode, time_s = run.get("mode"), run.get("time_s")
            if time_s and (mode not in best or time_s < best[mode]):
                best[mode] = time_s
    return best


class RoverGUIWithMap(gui.RoverGUI):
    def __init__(self, root):
        super().__init__(root)
        self._update_scorecard()

    # ================================================================
    # Green terminal look (theme.py)
    # ================================================================
    def _build_styles(self):
        self.root.title("MXEN3000 Rover 2026 - dashboard")
        gui.glass.dark_title_bar(self.root)
        theme.configure_styles(self.root)

    def _card(self, parent, **pack):
        """A box with a 1 px green outline. `.outer` holds the box (and a title row)."""
        outer = tk.Frame(parent, bg=theme.BG)
        outer.pack(**{"padx": 6, "pady": 3, **pack})
        box = tk.Frame(outer, bg=theme.BG, highlightthickness=1,
                       highlightbackground=theme.BORDER, highlightcolor=theme.BORDER)
        box.pack(fill=tk.BOTH, expand=True)
        inner = tk.Frame(box, bg=theme.BG)
        inner.pack(fill=tk.BOTH, expand=True, padx=10, pady=6)
        inner.outer = outer
        return inner

    def _panel(self, parent, title, **pack):
        """Title above the box. Extra widgets for the title row go in `frame.header`."""
        frame = self._card(parent, **pack)
        frame.header = ttk.Frame(frame.outer, style="Plain.TFrame")
        frame.header.pack(fill=tk.X, pady=(0, 2), before=frame.master)
        tk.Frame(frame.header, bg=theme.ACCENT, width=7, height=7).pack(side=tk.LEFT,
                                                                       padx=(0, 6))
        frame.title = ttk.Label(frame.header, text=title, style="Title.TLabel")
        frame.title.pack(side=tk.LEFT)
        return frame

    def _button(self, parent, text, command, kind="normal", min_width=0):
        style = {"primary": "Primary.TButton", "danger": "Danger.TButton"}.get(kind, "TButton")
        return ttk.Button(parent, text=text, command=command, style=style, takefocus=0,
                          width=max(min_width // 11, 0) or None)

    def _build_header(self):
        """Pixel marks, the title strip with the link square, then the port row
        and the next step."""
        top = tk.Frame(self.page, bg=theme.BG)
        top.pack(fill=tk.X, padx=12, pady=(10, 0))
        theme.pixel_blocks(top, HEADER_PIXELS_LEFT).pack(side=tk.LEFT)
        theme.pixel_blocks(top, HEADER_PIXELS_RIGHT).pack(side=tk.RIGHT)
        strip = tk.Frame(top, bg=theme.BG, highlightthickness=1, highlightbackground=theme.TEXT)
        strip.pack(side=tk.LEFT, expand=True)
        self.status_square = tk.Canvas(strip, width=STATUS_SQUARE, height=STATUS_SQUARE,
                                       bg=theme.BG, highlightthickness=0)
        self.status_square.pack(side=tk.LEFT, padx=(12, 0))
        self.status_fill = self.status_square.create_rectangle(
            0, 0, STATUS_SQUARE, STATUS_SQUARE, fill=theme.GRID, outline="")
        for text, colour in (("ROVER 2026 Mk. A", theme.TEXT), ("●", theme.ACCENT),
                             ("MXEN3000 // ONBOARD CONTROL", theme.TEXT)):
            tk.Label(strip, text=text, bg=theme.BG, fg=colour,
                     font=(theme.FONT, 12, "bold")).pack(side=tk.LEFT, padx=8, pady=3)

        row = tk.Frame(self.page, bg=theme.BG)
        row.pack(fill=tk.X, padx=12, pady=(8, 0))
        self.port_box = ttk.Combobox(row, state="readonly", width=34, takefocus=0)
        self.port_box.pack(side=tk.LEFT, padx=(0, 6))
        self.connect_button = self._button(row, "Connect", self.toggle_connection)
        for button in (self._button(row, "Refresh", self.refresh_ports), self.connect_button,
                       self._button(row, "Pair XBees", self.open_pairing),
                       self._button(row, "PWM test", self.open_pwm_window)):
            button.pack(side=tk.LEFT, padx=2)
        self.link_label = ttk.Label(row, text="● Disconnected", style="Panel.TLabel",
                                    foreground=theme.RED)
        self.link_label.pack(side=tk.RIGHT)

        guide = tk.Frame(self.page, bg=theme.BG)
        guide.pack(fill=tk.X, padx=12, pady=(6, 0))
        ttk.Label(guide, text="NEXT STEP >", style="Muted.TLabel",
                  font=(theme.FONT, 10, "bold")).pack(side=tk.LEFT, padx=(0, 8))
        self.guide_label = ttk.Label(guide, text="", style="Panel.TLabel", font=gui.GUIDE_FONT)
        self.guide_label.pack(side=tk.LEFT)
        tk.Frame(self.page, bg=theme.BORDER, height=1).pack(fill=tk.X, padx=12, pady=(6, 2))

    def _update_orb(self):
        """The header square: dark with no link, green when linked, bright while running."""
        if not self.link_alive():
            colour = theme.GRID
        elif self.rover_running():
            colour = theme.BRIGHT
        else:
            colour = theme.ACCENT
        if self.widget_cache.get("status_square") != colour:
            self.widget_cache["status_square"] = colour
            self.status_square.itemconfigure(self.status_fill, fill=colour)

    # ================================================================
    # Layout: controls | rover + tuning | waveforms + data
    # ================================================================
    def _build_layout(self):
        self._build_scroll_area()
        self._build_header()
        self._build_history_panel(self.page)     # Packed to the bottom, before the body
        body = tk.Frame(self.page, bg=theme.BG)
        body.pack(fill=tk.BOTH, expand=True, padx=8, pady=(0, 6))
        left = tk.Frame(body, bg=theme.BG)
        left.pack(side=tk.LEFT, fill=tk.Y)
        tk.Frame(left, width=gui.LEFT_COLUMN_WIDTH, height=1, bg=theme.BG).pack()
        data_column = tk.Frame(body, bg=theme.BG)
        data_column.pack(side=tk.RIGHT, fill=tk.Y)
        middle = tk.Frame(body, bg=theme.BG)
        middle.pack(side=tk.LEFT, fill=tk.BOTH, expand=True)

        self._build_run_panel(left)
        self._build_mode_panel(left)
        self._build_scorecard_panel(left)
        self._build_rover_panel(middle)
        self._build_tuning_panel(middle)
        self._build_optimiser_panel(middle)
        self._build_chart_panel(data_column)     # The waveforms
        self._build_data_panel(data_column)
        self.rating_label.configure(font=RATING_FONT, wraplength=RATING_WRAP)
        self.rating_label.pack_configure(side=tk.TOP, anchor=tk.W, pady=(0, 4))  # Buttons under it
        for label in (self.optimiser_status, self.optimiser_detail):
            label.configure(wraplength=OPTIMISER_WRAP)
        # Monospace text is wider: wrap the long hints instead of widening the window
        self.note_hint.configure(wraplength=gui.LEFT_COLUMN_WIDTH - HINT_MARGIN)
        self.log_label.configure(wraplength=LOG_LABEL_WRAP)
        self._fit_window_to_screen()

    def _build_run_panel(self, parent):
        super()._build_run_panel(parent)
        # Beside the timer: the lap time that earns the top mark in this division.
        self.target_label = ttk.Label(self.timer_label.master, text="", style="Muted.TLabel",
                                      font=TARGET_FONT)
        self.target_label.pack(side=tk.RIGHT, anchor=tk.S, padx=(0, 16), pady=(0, 6))

    def _show_tuning_columns(self, ready):
        """Only the current controller's values, and none before the rover links,
        so the tuning box never needs the width of all seven."""
        shown = tuple(param.key for param in p.TUNING_PARAMS
                      if ready and self._param_active(param))
        if self.widget_cache.get("tuning_columns") == shown:
            return
        self.widget_cache["tuning_columns"] = shown
        for key, widgets in self.tuning_widgets.items():
            if key in shown:
                widgets["cell"].grid(row=0, column=shown.index(key))
            else:
                widgets["cell"].grid_remove()
        for column in range(len(p.TUNING_PARAMS)):
            self.tuning_grid.columnconfigure(column, weight=1 if column < len(shown) else 0)

    def _refresh(self):
        super()._refresh()
        mode = self.run_mode if self.run_start is not None else self.selected_mode.get()
        self._set_widget(self.target_label, text=f"< {top_mark_time(mode)} s")
        self._update_rover()
        self._update_data()

    # ================================================================
    # ROVER drawing (rover_view.py) in place of the lamps, level bars and wheel panel
    # ================================================================
    def _build_rover_panel(self, parent):
        panel = self._panel(parent, "ROVER  (top view, front up; lit = active)", fill=tk.X)
        self.rover_view = rv.RoverView(panel, ROVER_VIEW_WIDTH)
        self.rover_view.canvas.pack()

    def _update_rover(self):
        t = self.telemetry if self.link_alive() else None
        self.rover_view.update(t, self.debug if self.debug_alive() else None, self.calibration,
                               self.tuning.max_turn if self.tuning is not None else None,
                               self.rover_running(),
                               link_fresh=time.monotonic() - self.telemetry_time < LINK_FRESH_S)

    # The ROVER drawing replaces these base-class displays.
    def _update_lamps(self):
        pass

    def _update_levels(self):
        pass

    def _update_wheels(self, t):
        pass

    def _update_readout(self, t):
        pass

    # ================================================================
    # WAVEFORMS (scope_view.py)
    # ================================================================
    def _build_chart_panel(self, parent):
        panel = self._panel(parent, "WAVEFORMS  (last 20 s; red marks = line lost /"
                                    " steering at max)", fill=tk.X)
        self._button(panel.header, "Clear", self._clear_scope).pack(side=tk.RIGHT)
        self.scope = sv.Scope(panel, SCOPE_STRIPS, SCOPE_WIDTH, SCOPE_STRIP_HEIGHT,
                              SCOPE_SAMPLES)
        self.scope.canvas.pack()

    def _clear_scope(self):
        self.scope.clear()

    def _draw_chart(self):
        self.scope.draw()

    def _on_telemetry(self, telemetry):
        super()._on_telemetry(telemetry)
        self.scope.push(self._scope_sample(telemetry))

    def _scope_sample(self, t):
        c = self.calibration
        white = (c.white_left, c.white_right) if c else (rv.DEFAULT_WHITE_LEVEL,) * 2
        black = (c.black_left, c.black_right) if c else (rv.DEFAULT_BLACK_LEVEL,) * 2
        d = self.debug if self.debug_alive() else None
        sample = {
            "ir_l": rv.normalised(t.ir_left, white[0], black[0]),
            "ir_r": rv.normalised(t.ir_right, white[1], black[1]),
            "pos": t.position, "lost": t.line_lost,
            "duty_l": p.dac_to_percent(t.dac_left), "duty_r": p.dac_to_percent(t.dac_right),
            "turn": (p.dac_to_speed(t.dac_left) - p.dac_to_speed(t.dac_right)) / 2,
        }
        if d is not None:
            sample.update(p=d.p_term, i=d.i_term, d=d.d_term, sat=d.saturated,
                          lost=t.line_lost or d.state in p.LOST_STATES)
        return sample

    # ================================================================
    # DATA box: what decides the lap time (the rest is in the logs)
    # ================================================================
    def _build_data_panel(self, parent):
        panel = self._panel(parent, "DATA  (everything else: logs/run-stats.csv,"
                                    " ir-log_*.csv)", fill=tk.BOTH, expand=True)
        columns = [ttk.Frame(panel, style="Plain.TFrame") for _ in range(2)]
        for column in columns:
            column.pack(side=tk.LEFT, anchor=tk.N, padx=(0, 16))
        self.data_values = {}
        for index, title, rows in DATA_SECTIONS:
            frame = ttk.Frame(columns[index], style="Plain.TFrame")
            frame.pack(anchor=tk.W, pady=(0, 6))
            ttk.Label(frame, text=title, style="Panel.TLabel", foreground=theme.ACCENT,
                      font=DATA_TITLE_FONT).grid(row=0, column=0, columnspan=2, sticky=tk.W)
            for row, (key, label) in enumerate(rows, start=1):
                ttk.Label(frame, text=label, style="Muted.TLabel", font=DATA_FONT,
                          width=DATA_LABEL_WIDTH).grid(row=row, column=0, sticky=tk.W)
                value = ttk.Label(frame, text="--", style="Panel.TLabel", font=DATA_FONT,
                                  width=DATA_VALUE_WIDTH)
                value.grid(row=row, column=1, sticky=tk.W)
                self.data_values[key] = value

    def _data(self, key, text, colour=None):
        self._set_widget(self.data_values[key], text=text, foreground=colour or theme.TEXT)

    def _blank(self, keys):
        for key in keys:
            self._data(key, "--", theme.MUTED)

    def _update_data(self):
        self._data_status()
        self._data_run()

    def _data_status(self):
        """Is the rover, the radio and the sensor calibration fit for a timed run?"""
        if not self.link_alive():
            self._blank(("state", "radio", "cal", "levels", "pwm"))
            return
        d = self.debug if self.debug_alive() else None
        if d is None:
            self._data("state", "NO DEBUG (old fw)", theme.WARN)
        else:
            lost = d.state in p.LOST_STATES
            self._data("state", p.STATE_NAMES.get(d.state, str(d.state)),
                       theme.RED if lost else theme.BRIGHT)
        snap = self.link_stats.snapshot(time.monotonic())
        self._data("radio", f"lost {snap.lost} gap {snap.gap_max_ms:.0f} ms",
                   theme.RED if snap.gap_max_ms > GAP_BAD_MS else
                   theme.WARN if snap.lost else None)
        c = self.calibration
        if c is None:
            self._blank(("cal", "levels"))
        else:
            result = {p.CAL_DEFAULTS: "config.h defaults", p.CAL_ACCEPTED: "calibrated",
                      p.CAL_REJECTED: "REJECTED - redo C"}.get(c.result, "?")
            self._data("cal", result, theme.RED if c.result == p.CAL_REJECTED else
                       theme.MUTED if c.result == p.CAL_DEFAULTS else None)
            self._data("levels", f"{c.white_left}/{c.white_right}  {c.black_left}/"
                                 f"{c.black_right}")
        u = self.tuning
        if u is None or u.controller != p.CONTROLLER_BANG_BANG:
            self._data("pwm", "bang-bang only", theme.MUTED)
        else:
            self._data("pwm", f"{u.pwm_straight_left}/{u.pwm_straight_right} > "
                              f"{u.pwm_correct_left}/{u.pwm_correct_right} %")

    def _data_run(self):
        """The run in progress, live; otherwise the last finished run."""
        if self.run_start is not None and self.run_stats is not None:
            name, s = "THIS RUN", self.run_stats.summary(time.monotonic() - self.run_start)
        else:
            name, s = "LAST RUN", self.last_run_summary
        keys = ("run_time", "avg_speed", "avg_duty", "min_duty", "rms", "run_sat",
                "run_lost", "weave", "run_loop", "run_radio")
        if s is None:
            self._blank(keys)
            return
        self._data("run_time", f"{name.split()[0]} {s.duration_s:.2f} s",
                   theme.BRIGHT if name == "THIS RUN" else None)
        self._data("avg_speed", f"{s.avg_speed:+.0f} %")
        self._data("avg_duty", f"{s.avg_duty_left:.0f} / {s.avg_duty_right:.0f} %")
        self._data("min_duty", f"{s.min_duty_left:.0f} / {s.min_duty_right:.0f} %")
        self._data("rms", f"{s.rms_error:.2f} / {s.max_error:.2f}")
        self._data("run_sat", f"{s.saturated_pct:.0f} %",
                   theme.WARN if s.saturated_pct >= SATURATED_WARN_PCT else None)
        self._data("run_lost", f"{s.lost_events}x  {s.lost_s:.1f} s",
                   theme.WARN if s.lost_events else None)
        self._data("weave", f"{s.weave_hz:.1f} Hz")
        self._data("run_loop", f"{s.period_max_ms:.1f} ms",
                   theme.WARN if s.period_max_ms > PERIOD_WARN_MS else None)
        self._data("run_radio", f"{s.frames_lost} frames", theme.WARN if s.frames_lost else None)

    # ================================================================
    # SCORECARD
    # ================================================================
    def _build_scorecard_panel(self, parent):
        panel = self._panel(parent, "SCORECARD  (best lap rated clean with Y)", fill=tk.BOTH,
                            expand=True)
        self.scorecard_track = ttk.Label(panel.header, text="", style="Muted.TLabel")
        self.scorecard_track.pack(side=tk.RIGHT)
        grid = ttk.Frame(panel, style="Plain.TFrame")
        grid.pack(fill=tk.X, pady=(6, 0))
        for column, heading in enumerate(("Division", "Best", "Target", "Marks")):
            ttk.Label(grid, text=heading, style="Muted.TLabel", font=TABLE_FONT).grid(
                row=0, column=column, sticky=tk.W, padx=(0, 22))
        self.score_rows = {}
        for row, mode in enumerate(SCORECARD_ORDER, start=1):
            cells = [ttk.Label(grid, text=text, style="Panel.TLabel", font=TABLE_FONT)
                     for text in (p.MODE_NAMES[mode], "--", f"< {top_mark_time(mode)} s", "--")]
            for column, cell in enumerate(cells):
                cell.grid(row=row, column=column, sticky=tk.W, padx=(0, 22), pady=1)
            self.score_rows[mode] = cells
        self.score_total = ttk.Label(panel, text="", style="Panel.TLabel", font=TOTAL_FONT)
        self.score_total.pack(anchor=tk.W, pady=(8, 0))

    def _update_scorecard(self):
        track = self.track_name.get()
        best = best_clean_laps(self._load_rated_runs(), track)
        total = 0
        for mode, (_name, best_cell, _target, marks_cell) in self.score_rows.items():
            time_s = best.get(p.MODE_NAMES[mode])
            marks = marks_for(mode, time_s)
            total += marks
            best_cell.config(text=f"{time_s:.2f} s" if time_s else "--")
            colour = theme.BRIGHT if marks == max_marks(mode) else \
                theme.TEXT if marks else theme.MUTED
            marks_cell.config(text=f"{marks:g} / {max_marks(mode):g}", foreground=colour)
        top = sum(max_marks(mode) for mode in SCORECARD_ORDER)
        self.score_total.config(text=f"Total  {total:g} / {top:g}")
        self.scorecard_track.config(text=f"{track} track")

    # Recount after anything that changes the ratings or the track.
    def rate_run(self, clean):
        super().rate_run(clean)
        self._update_scorecard()

    def undo_rating(self):
        super().undo_rating()
        self._update_scorecard()

    def select_track(self):
        super().select_track()
        self._update_scorecard()


def main():
    root = tk.Tk()
    RoverGUIWithMap(root)
    root.mainloop()


if __name__ == "__main__":
    main()
