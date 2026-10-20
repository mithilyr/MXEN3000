#!/usr/bin/env python3
"""MXEN3000 S2 2026 - track map: an ESTIMATED rover position on the stadium track.

A separate program from the rover GUI. It never opens the XBee port: it reads
the ir-log_<mode>_<controller>_<time>.csv file the GUI writes during every run
(one row per telemetry sample), so both can run side by side.

    Live    while the GUI is running a run, the dot follows it.
    Replay  pick any earlier run in the list, then Play or drag the slider.

The position is worked out from the commanded wheel speeds (see
track_model.py). To calibrate it, stop a replay where you know where the rover
really was (for example the moment it stopped), press "Set true position" and
click that spot on the map.

Build a single .exe with build_track_map.bat.
"""

import csv
import os
import time
import tkinter as tk
from tkinter import filedialog

import map_view as mv
import track_model as tm

# ---- Files ----
LOG_PREFIX = "ir-log_"
LOG_SUFFIX = ".csv"
RUN_LIST_SIZE = 40

# ---- Timing ----
POLL_MS = 100            # Live file read and replay step
SCAN_MS = 500            # Folder scan for new runs
LIVE_IDLE_S = 2.0        # A log written to within this long counts as a live run

# ---- Layout ----
MAP_WIDTH = 760
MAP_HEIGHT = 320

FONT = mv.FONT
PANEL = "#171c26"
PANEL_EDGE = "#2b3342"
AMBER = "#ffd60a"


def run_mode(path):
    """Division name from ir-log_<mode>_<controller>_<date>_<time>.csv."""
    parts = os.path.basename(path)[len(LOG_PREFIX):].split("_")
    return parts[0] if parts else ""


def read_rows(lines):
    """(time_s, dac_left, dac_right, line_lost) for each complete CSV line."""
    rows = []
    for record in csv.reader(lines):
        try:
            rows.append((float(record[0]), int(record[4]), int(record[5]), int(record[7])))
        except (ValueError, IndexError):
            continue        # Header or a half-written line
    return rows


class Run:
    """One ir-log file, read in full or followed while it grows."""

    def __init__(self, path):
        self.path = path
        self.mode = run_mode(path)
        self.division = tm.DIVISIONS.get(self.mode)
        self.rows = []
        self.offset = 0
        self.partial = ""

    def read_new(self):
        """Reads lines added since the last call. Returns True if any arrived."""
        try:
            with open(self.path, encoding="utf-8", newline="") as log_file:
                log_file.seek(self.offset)
                text = log_file.read()
                self.offset = log_file.tell()
        except OSError:
            return False
        if not text:
            return False
        lines = (self.partial + text).split("\n")
        self.partial = lines.pop()       # Not ended yet: keep for next time
        new_rows = read_rows(lines)
        self.rows.extend(new_rows)
        return bool(new_rows)


class TrackMapApp:
    def __init__(self, root):
        self.root = root
        self.track = tm.Track()
        self.settings = mv.MapSettings()
        self.folder = (self.settings.values.get("folder")
                       or os.path.join(mv.app_folder(), mv.LOG_FOLDER))
        self.run = None
        self.states = []
        self.live = False
        self.playhead = 0
        self.playing = False
        self.play_started = 0.0     # Wall clock when play began, minus the run time then
        self.picking = False
        self.run_paths = []
        self.follow_live = tk.BooleanVar(value=True)
        self.snap = tk.BooleanVar(value=True)

        root.title("Track Map - estimated rover position")
        root.configure(bg=mv.BG)
        self._build()
        self._update_readout()
        self._scan()
        self._poll()

    # ================================================================
    # Layout
    # ================================================================
    def _build(self):
        self.map = mv.MapView(self.root, self.track, MAP_WIDTH, MAP_HEIGHT)
        self.map.canvas.pack(side=tk.LEFT, padx=12, pady=12)
        self.map.canvas.bind("<Button-1>", self._on_map_click)

        side = tk.Frame(self.root, bg=PANEL, highlightbackground=PANEL_EDGE,
                        highlightthickness=1, padx=14, pady=12)
        side.pack(side=tk.LEFT, fill=tk.Y, padx=(0, 12), pady=12)

        self.status = self._label(side, "", (FONT, 15, "bold"))
        self.run_name = self._label(side, "", (FONT, 9), mv.MUTED)
        self.readout = self._label(side, "", ("Consolas", 12))
        self.readout.config(justify=tk.LEFT)

        controls = tk.Frame(side, bg=PANEL)
        controls.pack(fill=tk.X, pady=(10, 4))
        self.play_button = self._button(controls, "Play", self.toggle_play)
        self.play_button.pack(side=tk.LEFT)
        self._button(controls, "Set true position", self.pick_true_position).pack(
            side=tk.LEFT, padx=6)

        self.slider = tk.Scale(side, from_=0, to=0, orient=tk.HORIZONTAL, showvalue=False,
                               command=self._on_slider, bg=PANEL, fg=mv.TEXT,
                               troughcolor=mv.BG, highlightthickness=0, bd=0, length=300)
        self.slider.pack(fill=tk.X, pady=4)

        for text, variable in (("Follow live runs", self.follow_live),
                               ("Snap to the bend ends", self.snap)):
            tk.Checkbutton(side, text=text, variable=variable, command=self._recompute,
                           bg=PANEL, fg=mv.TEXT, selectcolor=mv.BG, activebackground=PANEL,
                           activeforeground=mv.TEXT, font=(FONT, 10)).pack(anchor=tk.W)

        self._label(side, "Runs (newest first)", (FONT, 10, "bold")).pack_configure(pady=(10, 2))
        self.run_list = tk.Listbox(side, height=14, width=44, bg=mv.BG, fg=mv.TEXT,
                                   selectbackground=mv.BLUE, highlightthickness=0, bd=0,
                                   font=("Consolas", 9), activestyle="none")
        self.run_list.pack(fill=tk.BOTH, expand=True)
        self.run_list.bind("<<ListboxSelect>>", self._on_pick_run)

        folder_row = tk.Frame(side, bg=PANEL)
        folder_row.pack(fill=tk.X, pady=(8, 0))
        self._button(folder_row, "Log folder...", self.choose_folder).pack(side=tk.LEFT)
        self.folder_label = tk.Label(folder_row, text="", bg=PANEL, fg=mv.MUTED,
                                     font=(FONT, 8), wraplength=220, justify=tk.LEFT)
        self.folder_label.pack(side=tk.LEFT, padx=6)

        self.note = self._label(side, "", (FONT, 9), AMBER)
        self.note.config(wraplength=320, justify=tk.LEFT)
        self._label(side, "Estimated from commanded wheel speeds - no encoders.",
                    (FONT, 8), mv.MUTED)

    def _label(self, parent, text, font, colour=mv.TEXT):
        label = tk.Label(parent, text=text, bg=PANEL, fg=colour, font=font, anchor=tk.W)
        label.pack(fill=tk.X)
        return label

    def _button(self, parent, text, command):
        return tk.Button(parent, text=text, command=command, bg=mv.BLUE, fg=mv.TEXT, bd=0,
                         activebackground="#409cff", activeforeground=mv.TEXT,
                         font=(FONT, 10, "bold"), padx=10, pady=4, cursor="hand2")

    # ================================================================
    # Runs
    # ================================================================
    def _log_files(self):
        try:
            entries = [entry for entry in os.scandir(self.folder)
                       if entry.name.startswith(LOG_PREFIX) and entry.name.endswith(LOG_SUFFIX)]
        except OSError:
            return []
        entries.sort(key=lambda entry: entry.stat().st_mtime, reverse=True)
        return entries

    def _scan(self):
        entries = self._log_files()
        paths = [entry.path for entry in entries[:RUN_LIST_SIZE]]
        if paths != self.run_paths:
            self.run_paths = paths
            self.run_list.delete(0, tk.END)
            for path in paths:
                name = os.path.basename(path)[len(LOG_PREFIX):-len(LOG_SUFFIX)]
                self.run_list.insert(tk.END, name.replace("_", "  "))
        self.folder_label.config(text=self.folder)
        if entries and self.follow_live.get():
            newest = entries[0]
            fresh = time.time() - newest.stat().st_mtime < LIVE_IDLE_S
            if fresh and (self.run is None or self.run.path != newest.path):
                self._open_run(newest.path, live=True)
        self.root.after(SCAN_MS, self._scan)

    def _open_run(self, path, live):
        self.run = Run(path)
        self.live = live
        self.playing = False
        self.play_button.config(text="Play")
        self.run.read_new()
        if self.run.division is None:
            self.note.config(text=f"{self.run.mode or 'This run'} is not on the stadium track"
                                  " (only Clockwise, Anticlockwise and Reverse are mapped).")
        else:
            self.note.config(text="")
        self._recompute(to_end=True)

    def _recompute(self, to_end=False):
        if self.run is None or self.run.division is None:
            self.states = []
            self.slider.config(to=0)
            self.map.draw_run([], None)
            self._update_readout()
            return
        at_end = to_end or self.playhead >= len(self.states) - 1
        self.states = tm.estimate(self.track, self.run.division, self.run.rows,
                                  self.settings.full_speed(self.run.mode), self.snap.get())
        last = max(len(self.states) - 1, 0)
        self.slider.config(to=last)
        self._set_playhead(last if at_end else min(self.playhead, last))

    def _set_playhead(self, index):
        self.playhead = index
        self.slider.set(index)
        self._show_playhead()

    def _show_playhead(self):
        self.map.draw_run(self.states[:self.playhead + 1], self.run.division)
        self._update_readout()

    def _update_readout(self):
        if self.run is None:
            self.status.config(text="Waiting for a run", fg=mv.MUTED)
            self.run_name.config(text="Start a run in the rover GUI, or pick one below.")
            self.readout.config(text="")
            return
        if self.live:
            self.status.config(text="LIVE", fg=mv.GREEN)
        else:
            self.status.config(text="REPLAY", fg=mv.BLUE)
        self.run_name.config(text=os.path.basename(self.run.path))
        if not self.states:
            self.readout.config(text="No samples yet")
            return
        self.readout.config(text=mv.readout(
            self.run.mode, self.states[self.playhead], self.states[:self.playhead + 1],
            self.track, self.settings.full_speed(self.run.mode)))

    # ================================================================
    # Events
    # ================================================================
    def _poll(self):
        if self.run is not None and self.live:
            if self.run.read_new():
                self._recompute(to_end=True)
            try:
                idle = time.time() - os.path.getmtime(self.run.path)
            except OSError:
                idle = LIVE_IDLE_S
            if idle >= LIVE_IDLE_S:
                self.live = False
                self._update_readout()
        if self.playing and self.states:
            now_s = time.monotonic() - self.play_started
            index = self.playhead
            while index < len(self.states) - 1 and self.states[index + 1].time_s <= now_s:
                index += 1
            if index != self.playhead:
                self._set_playhead(index)
            if index >= len(self.states) - 1:
                self.playing = False
                self.play_button.config(text="Play")
        self.root.after(POLL_MS, self._poll)

    def _on_pick_run(self, _event):
        selection = self.run_list.curselection()
        if selection:
            self.follow_live.set(False)
            self._open_run(self.run_paths[selection[0]], live=False)

    def _on_slider(self, value):
        index = int(float(value))
        if self.states and index != self.playhead:
            self.playing = False
            self.play_button.config(text="Play")
            self.playhead = index
            self._show_playhead()

    def toggle_play(self):
        if self.live or not self.states:
            return
        if self.playing:
            self.playing = False
            self.play_button.config(text="Play")
            return
        if self.playhead >= len(self.states) - 1:
            self._set_playhead(0)
        self.play_started = time.monotonic() - self.states[self.playhead].time_s
        self.playing = True
        self.play_button.config(text="Pause")

    def pick_true_position(self):
        if not self.states:
            self.note.config(text="Open a run first.")
            return
        self.playing = False
        self.play_button.config(text="Play")
        self.picking = True
        self.note.config(text="Click on the line where the rover really was at this moment.")

    def _on_map_click(self, event):
        if not self.picking:
            return
        self.picking = False
        x, y = self.map.to_cm(event.x, event.y)
        state = self.states[self.playhead]
        scale = tm.calibrated_scale(self.track, self.run.division, state, x, y)
        if scale is None:
            self.note.config(text="Too early in the run to calibrate - pick a later moment.")
            return
        error = self.settings.set_full_speed(self.run.mode, scale)
        self.note.config(text=error or f"{self.run.mode}: full speed set to {scale:.1f} cm/s.")
        self._recompute()

    def choose_folder(self):
        folder = filedialog.askdirectory(initialdir=self.folder,
                                         title="Folder with the GUI's ir-log CSV files")
        if folder:
            self.folder = folder
            self.run_paths = []
            self.settings.values["folder"] = folder
            error = self.settings.save()
            if error:
                self.note.config(text=error)


def main():
    root = tk.Tk()
    TrackMapApp(root)
    root.mainloop()


if __name__ == "__main__":
    main()
