"""The stadium-track map canvas and the saved speed scale, shared by TrackMap.exe
(track_map.py) and the rover GUI with the map tile (gui/rover_gui_map.py)."""

import json
import os
import sys
import tkinter as tk

import track_model as tm

SETTINGS_FILE = "track_map.json"   # In settings/ next to the .exe; both programs share it
SETTINGS_FOLDER = "settings"
LOG_FOLDER = "logs"                # Where the rover GUI writes its run logs

MAP_MARGIN_CM = 8.0
DOT_RADIUS = 8
LOST_RADIUS = 5
TRAIL_WIDTH = 3
START_MARK_HALF_PX = 9   # Tick across the line at the start / finish
ARROW_OFFSET_CM = 7.0    # Direction arrow sits this far outside the line
ARROW_LENGTH_CM = 25.0
TICK_HALF_PX = 8
LINE_PX = 3              # Drawn thinner than the real 3 cm line, to keep the map clean

FONT = "Segoe UI"
BG = "#0b0e14"           # Black table
TEXT = "#f2f4f8"
MUTED = "#8a93a6"
LINE = "#e8ebf0"         # White line
BLUE = "#0a84ff"
GREEN = "#30d158"
RED = "#ff453a"


def app_folder():
    """Folder of the .exe (or of the script that was started)."""
    if getattr(sys, "frozen", False):
        return os.path.dirname(sys.executable)
    return os.path.dirname(os.path.abspath(sys.argv[0]))


class MapSettings:
    """track_map.json: the log folder and the full-speed scale per division."""

    def __init__(self):
        self.path = os.path.join(app_folder(), SETTINGS_FOLDER, SETTINGS_FILE)
        try:
            with open(self.path, encoding="utf-8") as settings_file:
                self.values = json.load(settings_file)
        except (OSError, ValueError):
            self.values = {}

    def save(self):
        """Returns an error message, or "" when saved."""
        try:
            os.makedirs(os.path.dirname(self.path), exist_ok=True)
            with open(self.path, "w", encoding="utf-8") as settings_file:
                json.dump(self.values, settings_file, indent=2)
            return ""
        except OSError as error:
            return f"Could not save {SETTINGS_FILE}: {error}"

    def full_speed(self, mode):
        """cm/s at full commanded wheel speed for this division."""
        return self.values.get("full_speed_cm_s", {}).get(mode, tm.DEFAULT_FULL_SPEED_CM_S)

    def set_full_speed(self, mode, cm_per_s):
        self.values.setdefault("full_speed_cm_s", {})[mode] = round(cm_per_s, 2)
        return self.save()


class MapView:
    """A canvas showing the stadium line and one run's estimated path."""

    def __init__(self, parent, track, width, height, hint=True, bg=BG):
        self.track = track
        self.width, self.height = width, height
        self.canvas = tk.Canvas(parent, width=width, height=height, bg=bg, highlightthickness=0)
        self._fit()
        self._draw_static(hint)

    def _fit(self):
        xs = [x for x, _y in self.track.points]
        ys = [y for _x, y in self.track.points]
        margin = MAP_MARGIN_CM + ARROW_OFFSET_CM
        left, right = min(xs) - margin, max(xs) + margin
        bottom, top = min(ys) - margin, max(ys) + margin
        self.scale = min(self.width / (right - left), self.height / (top - bottom))
        self.left = left - (self.width / self.scale - (right - left)) / 2
        self.top = top + (self.height / self.scale - (top - bottom)) / 2

    def to_px(self, x, y):
        return (x - self.left) * self.scale, (self.top - y) * self.scale

    def to_cm(self, px, py):
        return px / self.scale + self.left, self.top - py / self.scale

    def _draw_static(self, hint):
        canvas = self.canvas
        flat = [value for point in self.track.points for value in self.to_px(*point)]
        canvas.create_line(*flat, fill=LINE, width=LINE_PX, capstyle=tk.ROUND,
                           joinstyle=tk.ROUND)
        if hint:
            for bend in self.track.bends:
                for distance in bend:
                    x, y = self.to_px(*self.track.point(distance))
                    canvas.create_line(x, y - TICK_HALF_PX, x, y + TICK_HALF_PX, fill=MUTED,
                                       dash=(3, 2))
            canvas.create_text(self.width / 2, self.height - 14, fill=MUTED, font=(FONT, 9),
                               text="Dashed ticks: bend entry / exit used for snapping")

    def _draw_start(self, division):
        """A tick across the line at the start / finish, and an arrow outside the
        line showing the direction of the lap."""
        canvas = self.canvas
        reverse = division.reverse_path
        x, y = self.to_px(*self.track.point(0.0, reverse))
        h = START_MARK_HALF_PX
        canvas.create_line(x, y - h, x, y + h, fill=GREEN, width=3, tags="run")
        (ax0, ay0), (ax1, ay1) = (self.track.point(d, reverse) for d in (2.0, ARROW_LENGTH_CM))
        arrow = [self.to_px(ax, ay + ARROW_OFFSET_CM) for ax, ay in ((ax0, ay0), (ax1, ay1))]
        canvas.create_line(*arrow[0], *arrow[1], fill=GREEN, width=2, arrow=tk.LAST,
                           tags="run")
        canvas.create_text(arrow[0][0], arrow[0][1] - 10, text="START / FINISH", fill=GREEN,
                           anchor=tk.E if reverse else tk.W, font=(FONT, 9, "bold"),
                           tags="run")

    def draw_run(self, states, division):
        """Start mark, the trail, line-lost marks and the dot at the last state."""
        canvas = self.canvas
        canvas.delete("run")
        if division is None:
            return
        self._draw_start(division)
        if len(states) > 1:
            trail = [value for state in states for value in self.to_px(state.x, state.y)]
            canvas.create_line(*trail, fill=BLUE, width=TRAIL_WIDTH, tags="run")
        was_lost = False
        for state in states:
            if state.line_lost and not was_lost:
                x, y = self.to_px(state.x, state.y)
                r = LOST_RADIUS
                canvas.create_oval(x - r, y - r, x + r, y + r, fill=RED, outline="", tags="run")
            was_lost = state.line_lost
        if states:
            x, y = self.to_px(states[-1].x, states[-1].y)
            r = DOT_RADIUS
            canvas.create_oval(x - r - 5, y - r - 5, x + r + 5, y + r + 5, outline=BLUE,
                               width=2, tags="run")
            canvas.create_oval(x - r, y - r, x + r, y + r, fill=BLUE, outline=TEXT, width=2,
                               tags="run")


def lost_count(states):
    """How many times the line was lost (rising edges of line_lost)."""
    return sum(1 for a, b in zip(states, states[1:]) if b.line_lost and not a.line_lost)


def readout_rows(mode, state, states, track, full_speed):
    """(label, value) pairs describing where the rover is."""
    return [("Mode", mode),
            ("Time", f"{state.time_s:.2f} s"),
            ("Distance", f"{state.distance:.0f} / {track.length:.0f} cm"
                         f"  ({100 * state.distance / track.length:.0f} % of a lap)"),
            ("Section", track.section(state.distance)),
            ("Line lost", f"{lost_count(states)}"),
            ("Scale", f"{full_speed:.1f} cm/s at full speed")]


def readout(mode, state, states, track, full_speed):
    """The same, as one aligned text block."""
    return "\n".join(f"{label:<10}{value}"
                     for label, value in readout_rows(mode, state, states, track, full_speed))
