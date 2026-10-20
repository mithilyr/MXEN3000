#!/usr/bin/env python3
"""MXEN3000 S2 2026 - the rover control panel plus a live track-map tile.

Exactly the rover_gui.py control panel (same keys, logs and tuning files),
with one extra column on the right: the stadium track with the rover's ESTIMATED
position during a run (see track_model.py; no wheel encoders).
This is the PC-control build's entry point (RoverControlPC.exe).

Calibrate the map once per division: after a run, press "Set true position"
and click where the rover really stopped. The scale is saved in
track_map.json, shared with TrackMap.exe.

Build a single .exe with build_exe.bat (RoverControlPC.exe).
"""

import math
import time
import tkinter as tk
from tkinter import ttk

import map_view as mv
import protocol as p
import rover_gui as gui
import track_model as tm

MAP_TILE_WIDTH = 400
MAP_TILE_HEIGHT = 170
MAP_READOUT_FONT = (gui.FONT_FAMILY, 10)
MAP_READOUT_KEYS = ("Mode", "Time", "Distance", "Section", "Line lost", "Scale")

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
TARGET_FONT = (gui.FONT_FAMILY, 14, "bold")
TOTAL_FONT = (gui.FONT_FAMILY, 14, "bold")


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
        self.track = tm.Track()
        self.map_settings = mv.MapSettings()
        self.map_mode = ""
        self.map_division = None
        self.map_estimator = None
        self.map_rows = []           # (time_s, dac_left, dac_right, line_lost) of the run
        self.map_states = []
        self.map_picking = False
        super().__init__(root)
        self.root.title(self.root.title() + "  +  track map")
        self._show_map()
        self._update_scorecard()

    # Same as RoverGUI._build_layout, plus the map column on the right.
    def _build_layout(self):
        self._build_scroll_area()
        self._build_header()
        self._build_history_panel(self.page)     # Packed to the bottom, before the body
        body = tk.Frame(self.page, bg=gui.SOLID_BG)
        body.pack(fill=tk.BOTH, expand=True, padx=8, pady=(0, 6))
        left = tk.Frame(body, bg=gui.SOLID_BG)
        left.pack(side=tk.LEFT, fill=tk.Y)
        tk.Frame(left, width=gui.LEFT_COLUMN_WIDTH, height=1, bg=gui.SOLID_BG).pack()
        map_column = tk.Frame(body, bg=gui.SOLID_BG)
        map_column.pack(side=tk.RIGHT, fill=tk.Y)
        right = tk.Frame(body, bg=gui.SOLID_BG)
        right.pack(side=tk.LEFT, fill=tk.BOTH, expand=True)

        self._build_run_panel(left)
        self._build_mode_panel(left)
        self._build_wheels_panel(left)
        self._build_chart_panel(right)
        self._build_pc_loop_panel(right)
        self._build_tuning_panel(right)
        self._build_optimiser_panel(right)
        self._build_map_panel(map_column)
        self._build_scorecard_panel(map_column)
        self._fit_window_to_screen()

    def _build_run_panel(self, parent):
        super()._build_run_panel(parent)
        # Beside the timer: the lap time that earns the top mark in this division.
        self.target_label = ttk.Label(self.timer_label.master, text="", style="Muted.TLabel",
                                      font=TARGET_FONT)
        self.target_label.pack(side=tk.RIGHT, anchor=tk.S, padx=(0, 16), pady=(0, 6))

    def _refresh(self):
        super()._refresh()
        mode = self.run_mode if self.run_start is not None else self.selected_mode.get()
        self._set_widget(self.target_label, text=f"< {top_mark_time(mode)} s")

    def _build_map_panel(self, parent):
        panel = self._panel(parent, "TRACK MAP  (estimated, no encoders)", fill=tk.X)
        self._button(panel.header, "Set true position", self.pick_true_position).pack(
            side=tk.RIGHT)
        self.map = mv.MapView(panel, self.track, MAP_TILE_WIDTH, MAP_TILE_HEIGHT,
                              hint=False, bg=gui.COLOUR_PANEL)
        self.map.canvas.pack(pady=(4, 4))
        self.map.canvas.bind("<Button-1>", self._on_map_click)
        readout = ttk.Frame(panel, style="Plain.TFrame")
        readout.pack(fill=tk.X)
        self.map_values = {}
        for row, key in enumerate(MAP_READOUT_KEYS):
            ttk.Label(readout, text=key, style="Muted.TLabel", font=MAP_READOUT_FONT).grid(
                row=row, column=0, sticky=tk.W, padx=(0, 16))
            value = ttk.Label(readout, text="--", style="Panel.TLabel", font=MAP_READOUT_FONT)
            value.grid(row=row, column=1, sticky=tk.W)
            self.map_values[key] = value
        self.map_note = ttk.Label(panel, text="", style="Muted.TLabel",
                                  wraplength=MAP_TILE_WIDTH)
        self.map_note.pack(anchor=tk.W, pady=(4, 0))

    def _build_scorecard_panel(self, parent):
        panel = self._panel(parent, "SCORECARD  (best lap rated clean with Y)", fill=tk.BOTH,
                            expand=True)
        self.scorecard_track = ttk.Label(panel.header, text="", style="Muted.TLabel")
        self.scorecard_track.pack(side=tk.RIGHT)
        grid = ttk.Frame(panel, style="Plain.TFrame")
        grid.pack(fill=tk.X, pady=(6, 0))
        for column, heading in enumerate(("Division", "Best", "Target", "Marks")):
            ttk.Label(grid, text=heading, style="Muted.TLabel", font=MAP_READOUT_FONT).grid(
                row=0, column=column, sticky=tk.W, padx=(0, 22))
        self.score_rows = {}
        for row, mode in enumerate(SCORECARD_ORDER, start=1):
            cells = [ttk.Label(grid, text=text, style="Panel.TLabel", font=MAP_READOUT_FONT)
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
            colour = gui.COLOUR_OK if marks == max_marks(mode) else \
                gui.COLOUR_TEXT if marks else gui.COLOUR_MUTED
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

    # ================================================================
    # Feeding the map
    # ================================================================
    def start_run(self):
        previous_start = self.run_start
        super().start_run()
        if self.run_start is not None and self.run_start != previous_start:
            self.map_mode = p.MODE_NAMES[self.run_mode]
            self.map_division = tm.DIVISIONS.get(self.map_mode)
            self.map_rows = []
            self.map_states = []
            self.map_estimator = None
            self.map_note.config(text="")
            if self.map_division is not None:
                self.map_estimator = tm.Estimator(
                    self.track, self.map_division, self.map_settings.full_speed(self.map_mode))
            self._show_map()

    def _on_telemetry(self, telemetry):
        running = self.run_start is not None
        elapsed = time.monotonic() - self.run_start if running else 0.0
        super()._on_telemetry(telemetry)
        if running and self.map_estimator is not None:
            row = (elapsed, telemetry.dac_left, telemetry.dac_right, telemetry.line_lost)
            self.map_rows.append(row)
            self.map_states.append(self.map_estimator.step(*row))
            self._show_map()

    def _show_map(self):
        self.map.draw_run(self.map_states, self.map_division)
        rows = {}
        if self.map_mode and self.map_division is None:
            self.map_note.config(text=f"{self.map_mode} is not on the stadium track"
                                      " (only Clockwise, Anticlockwise and Reverse are mapped).")
        elif self.map_states:
            rows = dict(mv.readout_rows(self.map_mode, self.map_states[-1], self.map_states,
                                        self.track, self.map_settings.full_speed(self.map_mode)))
        elif not self.map_mode:
            self.map_note.config(text="Start a run to see the rover on the track.")
        for key, label in self.map_values.items():
            label.config(text=rows.get(key, "--"))

    # ================================================================
    # Calibration
    # ================================================================
    def pick_true_position(self):
        if not self.map_states or self.run_start is not None:
            self.map_note.config(text="Finish a run first, then click where it really stopped.")
            return
        self.map_picking = True
        self.map_note.config(text="Click on the line where the rover really stopped.")

    def _on_map_click(self, event):
        if not self.map_picking:
            return
        self.map_picking = False
        x, y = self.map.to_cm(event.x, event.y)
        scale = tm.calibrated_scale(self.track, self.map_division, self.map_states[-1], x, y)
        if scale is None:
            self.map_note.config(text="That run was too short to calibrate from.")
            return
        error = self.map_settings.set_full_speed(self.map_mode, scale)
        self.map_states = tm.estimate(self.track, self.map_division, self.map_rows, scale)
        self._show_map()
        self.map_note.config(text=error or f"{self.map_mode}: full speed set to {scale:.1f} cm/s.")


def main():
    root = tk.Tk()
    RoverGUIWithMap(root)
    root.mainloop()


if __name__ == "__main__":
    main()
