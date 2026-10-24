#!/usr/bin/env python3
"""MXEN3000 S2 2026 - wireless rover control panel, PC-control build.

All duty-cycle processing runs on this PC (pc_brain.py). The rover runs
firmware/rover_io, which only streams raw IR readings and applies the DAC
codes sent back, over an XBee pair in transparent mode (the PC XBee shows
up as a COM port). Everything the supervisor needs during a
timed run is on the keyboard:

    1 2 3 4   select Clockwise / Anticlockwise / Squiggle / Reverse
    Space     start / stop
    Esc       emergency stop (always stops)
    Up/Down   change base speed
    C         calibrate sensors (sweep the rover across the line)
    T         next steering controller for this mode (PID / bang-bang / adaptive),
              also while running
    Q/A W/S E/D R/F   tuning values 1-4 up / down
    Y / N     rate the last run: clean lap / failed (for the optimiser)
    G         optimiser: send the next settings to try
    B         optimiser: send the best settings found so far

Build a single .exe with build_exe.bat.
"""

import csv
import json
import math
import os
import queue
import sys
import threading
import time
import tkinter as tk
from collections import deque
from datetime import datetime
from tkinter import filedialog, messagebox, ttk

import serial
from serial.tools import list_ports

import glass
import optimizer as opt
import protocol as p
import xbee_setup as xb
from pc_brain import NOMINAL_PERIOD_S, PcBrainLink

# ---- Link ----
BAUD_RATE = xb.LINK_BAUD       # 38400 (Pairing Guide step 15) = LINK_BAUD in rover_io.ino
HEARTBEAT_MS = 200              # Harmless here: pc_brain answers every sensor frame anyway
TELEMETRY_STALE_S = 1.2         # No telemetry for this long = link down

# ---- UI ----
UI_REFRESH_MS = 50
SPEED_STEP = 5
SPEED_MAX = 100
WHEEL_BAR_WIDTH = 260         # PWM bar: 0 % at the left edge, 100 % at the right
WHEEL_BAR_HEIGHT = 26
CHART_SAMPLES = 200
CHART_WIDTH = 640
CHART_HEIGHT = 140             # Minimum; the chart grows to fill spare height
ADC_FULL_SCALE = 1023
SENSOR_SPACING_CM = 3.0          # IR sensors are 3 cm apart, over a 3 cm white line
LAMP_SIZE = 44
# Sensor levels for calibrating with each sensor's own pot: 0-1 of the ADC
# range, 1 = white line, 0 = black table. The dashed marks are pc_brain.py's
# DEFAULT_WHITE_LEVEL / DEFAULT_BLACK_LEVEL, the levels the controller assumes
# until C is pressed.
DEFAULT_WHITE_LEVEL = 60
DEFAULT_BLACK_LEVEL = 720
LEVEL_WHITE_MARK = 1 - DEFAULT_WHITE_LEVEL / ADC_FULL_SCALE    # 0.94
LEVEL_BLACK_MARK = 1 - DEFAULT_BLACK_LEVEL / ADC_FULL_SCALE    # 0.30
LEVEL_BAR_WIDTH = 260
LEVEL_BAR_HEIGHT = 18
FONT_FAMILY = "Segoe UI"
GUIDE_FONT = (FONT_FAMILY, 12, "bold")
READOUT_FONT = ("Consolas", 12)
# Next to the .exe (or this script): logs/ for run-log.csv and the ir-log CSVs,
# settings/ for the .json files, so the .exe folder stays tidy.
APP_DIR = os.path.dirname(sys.executable if getattr(sys, "frozen", False)
                          else os.path.abspath(__file__))
LOG_DIR = os.path.join(APP_DIR, "logs")
SETTINGS_DIR = os.path.join(APP_DIR, "settings")
DEFAULT_LOG_FILE = "run-log.csv"
# notes = the tuning used (written by the GUI); note = what the operator typed after the run.
RUN_LOG_HEADER = ["date", "division", "attempt", "time_s", "result", "notes", "track", "note"]
TRACKS = ("Test", "Competition")   # Runs and optimiser ratings are kept apart per track
SESSION_FILE = "session.json"   # The last selected track and mode
NOTE_WIDTH = 34                 # Characters
TUNING_FILE = "tuning.json"     # In SETTINGS_DIR, one entry per mode
PWM_FILE = "pwm_levels.json"    # Bang-bang wheel levels from the PWM test window, one entry per mode
PWM_STEP = 5                    # Slider increment, % duty
PWM_USER_HOLD_S = 1.5           # After a slider move, ignore the rover echo this long
OPTIMISER_FILE = "optimizer.json"  # Every rated run
XBEE_FILE = "xbee.json"         # Pair XBees: PAN ID and both serial numbers
PAIR_POLL_MS = 50
TUNING_CONFIRM_S = 0.6          # Wait this long for the rover to echo a change
MODE_RESEND_S = 0.6             # Gap between attempts to put the rover back in the session mode
# Hotkey pairs (increase, decrease) for the tuning slots of the current controller.
TUNING_KEYS = (("q", "a"), ("w", "s"), ("e", "d"), ("r", "f"))
TUNING_SLOTS = {
    p.CONTROLLER_PID: ("kp", "kd", "ki", "max_turn"),
    p.CONTROLLER_BANG_BANG: ("bang_level", "max_turn"),
    p.CONTROLLER_ADAPTIVE: ("adapt_gain", "adapt_ramp", "max_turn"),
}
SAMPLE_LOG_HEADER = ["time_s", "ir_left", "ir_right", "position",
                     "dac_left", "dac_right", "speed", "line_lost"]

# Dark glass look (see glass.py): flat near-black window, rounded dark cards.
# Status text colours are the bright Apple-dark shades; the *_LAMP shades are
# the same hues for filled shapes.
SOLID_BG = glass.BG
COLOUR_PANEL = glass.CARD
COLOUR_PANEL_RAISED = "#0e131b"     # Input fields, table body
COLOUR_BORDER = glass.CARD_BORDER
COLOUR_TEXT = glass.TEXT
COLOUR_MUTED = "#8b95a7"
COLOUR_ACCENT = glass.ACCENT        # Primary action, selection (teal = PC control)
COLOUR_ACCENT_TEXT = glass.ACCENT_TEXT
COLOUR_GREEN = "#30d158"            # Good
COLOUR_GREEN_LAMP = "#30d158"
COLOUR_ORANGE = "#ff9f0a"           # Warning
COLOUR_ORANGE_LAMP = "#ff9f0a"
COLOUR_RED = "#ff453a"              # Bad
COLOUR_OK = COLOUR_GREEN
COLOUR_WARN = COLOUR_ORANGE
COLOUR_BAD = COLOUR_RED
COLOUR_IR_LEFT = COLOUR_ORANGE_LAMP
COLOUR_IR_RIGHT = "#4da3ff"
COLOUR_POSITION = COLOUR_TEXT
COLOUR_LAMP_OFF = "#2b3342"
COLOUR_LAMP_DARK = "#05070a"        # Sensor over the black table
COLOUR_CHART_BG = "#0e131b"
COLOUR_GRID = "#222a37"
COLOUR_CHART_CENTRE = "#3a4455"
COLOUR_ORB_DIM = "#3a4455"
COLOUR_ORB_RUN = "#a6f3b8"
LEFT_COLUMN_WIDTH = 520         # Fixed, so a long readout cannot resize it
SCROLLBAR_ALLOWANCE = 20        # Window width added beside the page for the scrollbar
SCREEN_MARGIN = 40
TASKBAR_ALLOWANCE = 90          # Keeps the window clear of the title bar and taskbar
MIN_WINDOW_WIDTH = 640
MIN_WINDOW_HEIGHT = 300
ORB_SIZE = 44
ORB_PULSE_HZ = 0.7

MODE_KEYS = {"1": p.MODE_CLOCKWISE, "2": p.MODE_ANTICLOCKWISE,
             "3": p.MODE_SQUIGGLE, "4": p.MODE_REVERSE}


def sensor_level(adc):
    """Raw ADC reading -> 0..1, 1 = white line (the sensors read white low)."""
    return max(0.0, min(1.0, 1 - adc / ADC_FULL_SCALE))


class RoverGUI:
    def __init__(self, root):
        self.root = root
        self.link = PcBrainLink()       # The controller itself, see pc_brain.py

        self.telemetry = None
        self.telemetry_time = 0.0
        self.telemetry_count = 0
        self.rate_window_start = time.monotonic()
        self.telemetry_rate = 0

        session = self._load_session()
        mode = session.get("mode")
        self.selected_mode = tk.IntVar(value=mode if mode in p.MODE_NAMES else p.MODE_CLOCKWISE)
        self.mode_sent = 0.0            # Last time the session mode was re-sent to the rover
        self.selected_controller = tk.IntVar(value=p.CONTROLLER_BANG_BANG)
        self.run_start = None
        self.run_mode = None
        self.was_calibrating = False
        self.tuning = None              # Last Tuning frame from the rover
        self.tuning_target = {}         # Requested values the rover has not echoed yet
        self.pwm_window = None          # The PWM test window, while open
        self.pwm_applied = False        # Saved wheel levels sent since the last connect/mode change
        self.pwm_target = {}            # (mode, field): requested wheel level the rover has not echoed
        self.tuning_sent = 0.0
        self.controller_target = None   # Controller asked for with T, until the rover echoes it
        self.controller_sent = 0.0
        self.level_range = {}           # Sensor side -> (lowest, highest) level since reset
        self.widget_cache = {}          # Last options set by _set_widget
        self.calibrated = False
        self.attempts = {mode: 0 for mode in p.MODE_NAMES}
        for folder in (LOG_DIR, SETTINGS_DIR):
            os.makedirs(folder, exist_ok=True)
        self.log_path = os.path.join(LOG_DIR, DEFAULT_LOG_FILE)
        self.sample_file = None
        self.sample_writer = None
        self.sample_path = None
        self.unrated_run = None         # Last finished run, waiting for Y / N
        self.optimiser_note = ""
        self.track_name = tk.StringVar(value=session.get("track", TRACKS[0]))
        self.run_track = None           # Track of the run in progress
        self.last_log_row = None        # run-log.csv row of the last run, for its note
        self.last_history_item = None

        self.chart = {name: deque(maxlen=CHART_SAMPLES)
                      for name in ("left", "right", "position")}

        self._build_styles()
        self._build_layout()
        self._bind_keys()
        self.refresh_ports()

        self.root.protocol("WM_DELETE_WINDOW", self.on_close)
        self.root.after(HEARTBEAT_MS, self._heartbeat)
        self.root.after(UI_REFRESH_MS, self._refresh)

    # ================================================================
    # Layout
    # ================================================================
    def _build_styles(self):
        self.root.title("MXEN3000 Rover 2026 - PC control")
        glass.dark_title_bar(self.root)
        self.root.configure(bg=SOLID_BG)
        style = ttk.Style()
        style.theme_use("clam")
        style.configure(".", background=COLOUR_PANEL, foreground=COLOUR_TEXT,
                        font=(FONT_FAMILY, 10))
        style.configure("Plain.TFrame", background=COLOUR_PANEL)
        style.configure("Panel.TLabel", background=COLOUR_PANEL, foreground=COLOUR_TEXT)
        style.configure("Muted.TLabel", background=COLOUR_PANEL, foreground=COLOUR_MUTED)
        style.configure("Title.TLabel", background=COLOUR_PANEL, foreground=COLOUR_MUTED)
        style.configure("Big.TLabel", background=COLOUR_PANEL, font=(FONT_FAMILY, 26, "bold"))
        for kind, size in (("Radiobutton", 11), ("Checkbutton", 10)):
            name = f"Panel.T{kind}"
            style.configure(name, background=COLOUR_PANEL, foreground=COLOUR_TEXT,
                            font=(FONT_FAMILY, size), focuscolor=COLOUR_PANEL,
                            upperbordercolor=COLOUR_BORDER, lowerbordercolor=COLOUR_BORDER,
                            indicatorforeground="#ffffff")
            style.map(name, background=[("active", COLOUR_PANEL)],
                      indicatorbackground=[("selected", COLOUR_ACCENT),
                                           ("!selected", COLOUR_PANEL_RAISED)],
                      upperbordercolor=[("selected", COLOUR_ACCENT)],
                      lowerbordercolor=[("selected", COLOUR_ACCENT)])
        style.configure("Treeview", background=COLOUR_PANEL_RAISED, fieldbackground=COLOUR_PANEL_RAISED,
                        foreground=COLOUR_TEXT, bordercolor=COLOUR_BORDER, lightcolor=COLOUR_BORDER,
                        darkcolor=COLOUR_BORDER, rowheight=24)
        style.map("Treeview", background=[("selected", COLOUR_ACCENT)],
                  foreground=[("selected", "#ffffff")])
        style.configure("Treeview.Heading", background=COLOUR_PANEL, foreground=COLOUR_MUTED,
                        relief="flat", font=(FONT_FAMILY, 10, "bold"))
        style.map("Treeview.Heading", background=[("active", COLOUR_BORDER)])
        style.configure("TCombobox", fieldbackground=COLOUR_PANEL_RAISED, background=COLOUR_PANEL,
                        foreground=COLOUR_TEXT, arrowcolor=COLOUR_MUTED, bordercolor=COLOUR_BORDER,
                        lightcolor=COLOUR_BORDER, darkcolor=COLOUR_BORDER,
                        selectbackground=COLOUR_PANEL_RAISED, selectforeground=COLOUR_TEXT)
        style.map("TCombobox", fieldbackground=[("readonly", COLOUR_PANEL_RAISED)],
                  foreground=[("readonly", COLOUR_TEXT)])
        style.configure("Vertical.TScrollbar", background=COLOUR_BORDER, troughcolor=COLOUR_PANEL,
                        bordercolor=COLOUR_PANEL, lightcolor=COLOUR_BORDER, darkcolor=COLOUR_BORDER,
                        arrowcolor=COLOUR_MUTED)
        style.map("Vertical.TScrollbar", background=[("active", COLOUR_CHART_CENTRE)])
        self.root.option_add("*TCombobox*Listbox.background", COLOUR_PANEL_RAISED)
        self.root.option_add("*TCombobox*Listbox.foreground", COLOUR_TEXT)
        self.root.option_add("*TCombobox*Listbox.selectBackground", COLOUR_ACCENT)
        self.root.option_add("*TCombobox*Listbox.selectForeground", "#ffffff")

    def _build_scroll_area(self):
        """Put the whole UI in a canvas so a small screen can scroll it. The mouse
        wheel scrolls it too (Up/Down stay reserved for speed). The scrollbar only
        shows when the page does not fit."""
        bar = ttk.Scrollbar(self.root, orient=tk.VERTICAL, takefocus=0)

        def on_scroll(first, last):
            bar.set(first, last)
            if float(first) <= 0 and float(last) >= 1:
                bar.pack_forget()
            else:
                bar.pack(side=tk.RIGHT, fill=tk.Y, before=self.scroll_canvas)

        self.scroll_canvas = tk.Canvas(self.root, bg=SOLID_BG, highlightthickness=0,
                                       yscrollcommand=on_scroll)
        self.scroll_canvas.pack(side=tk.LEFT, fill=tk.BOTH, expand=True)
        bar.config(command=self.scroll_canvas.yview)
        self.page = tk.Frame(self.scroll_canvas, bg=SOLID_BG)
        page_window = self.scroll_canvas.create_window(0, 0, window=self.page, anchor=tk.NW)

        def fit_page(_event=None):
            self.scroll_canvas.itemconfigure(
                page_window, width=self.scroll_canvas.winfo_width(),
                height=max(self.scroll_canvas.winfo_height(), self.page.winfo_reqheight()))
            self.scroll_canvas.configure(scrollregion=self.scroll_canvas.bbox("all"))

        self.page.bind("<Configure>", fit_page)
        self.scroll_canvas.bind("<Configure>", fit_page)
        self.root.bind_all("<MouseWheel>", self._on_mouse_wheel)

    def _on_mouse_wheel(self, event):
        if self.page.winfo_reqheight() > self.scroll_canvas.winfo_height():
            self.scroll_canvas.yview_scroll(-1 if event.delta > 0 else 1, "units")

    def _fit_window_to_screen(self):
        """Open at the content size, but never taller or wider than the desktop."""
        self.root.update_idletasks()
        available_height = self.root.winfo_screenheight() - TASKBAR_ALLOWANCE
        needs_scroll = self.page.winfo_reqheight() > available_height
        width = min(self.page.winfo_reqwidth() + (SCROLLBAR_ALLOWANCE if needs_scroll else 0),
                    self.root.winfo_screenwidth() - SCREEN_MARGIN)
        height = min(self.page.winfo_reqheight(), available_height)
        x = (self.root.winfo_screenwidth() - width) // 2
        y = max((available_height - height) // 2, 0)
        self.root.geometry(f"{width}x{height}+{x}+{y}")
        self.root.minsize(MIN_WINDOW_WIDTH, MIN_WINDOW_HEIGHT)

    def _build_header(self):
        """Title, next-step guidance and the XBee connection controls in one bar."""
        header = self._card(self.page, fill=tk.X, padx=8, pady=(6, 0))
        self.orb = tk.Canvas(header, width=ORB_SIZE, height=ORB_SIZE, bg=COLOUR_PANEL,
                             highlightthickness=0)
        self.orb.pack(side=tk.LEFT)
        centre = ORB_SIZE / 2
        self.orb_ring = self.orb.create_oval(4, 4, ORB_SIZE - 4, ORB_SIZE - 4,
                                             outline=COLOUR_ORB_DIM, width=3)
        self.orb_dot = self.orb.create_oval(centre - 8, centre - 8, centre + 8, centre + 8,
                                            fill=COLOUR_ORB_DIM, outline="")
        titles = tk.Frame(header, bg=COLOUR_PANEL)
        titles.pack(side=tk.LEFT, padx=12)
        title_row = tk.Frame(titles, bg=COLOUR_PANEL)
        title_row.pack(anchor=tk.W)
        tk.Label(title_row, text="Rover 2026 \u2014 PC Control", bg=COLOUR_PANEL,
                 fg=COLOUR_TEXT, font=(FONT_FAMILY, 22, "bold")).pack(side=tk.LEFT)
        tk.Label(title_row, text="PROCESSING: PC", bg=COLOUR_ACCENT, fg=COLOUR_ACCENT_TEXT,
                 font=(FONT_FAMILY, 9, "bold"), padx=8, pady=2).pack(side=tk.LEFT, padx=(12, 0))
        tk.Label(titles, text="MXEN3000  \u00b7  Line-follower control",
                 bg=COLOUR_PANEL, fg=COLOUR_MUTED, font=(FONT_FAMILY, 10)).pack(anchor=tk.W)

        controls = tk.Frame(header, bg=COLOUR_PANEL)
        controls.pack(side=tk.RIGHT)
        row = tk.Frame(controls, bg=COLOUR_PANEL)
        row.pack(anchor=tk.E)
        self.port_box = ttk.Combobox(row, state="readonly", width=34, takefocus=0)
        self.port_box.pack(side=tk.LEFT, padx=(0, 6))
        self._button(row, "Refresh", self.refresh_ports).pack(side=tk.LEFT)
        self.connect_button = self._button(row, "Connect", self.toggle_connection)
        self.connect_button.pack(side=tk.LEFT, padx=(6, 0))
        self._button(row, "Pair XBees", self.open_pairing).pack(side=tk.LEFT, padx=(6, 0))
        self._button(row, "PWM test", self.open_pwm_window).pack(side=tk.LEFT, padx=(6, 0))
        self.link_label = ttk.Label(controls, text="\u25cf Disconnected", style="Panel.TLabel",
                                    foreground=COLOUR_BAD)
        self.link_label.pack(anchor=tk.E, pady=(6, 0))

        guide = tk.Frame(header, bg=COLOUR_PANEL)
        guide.pack(side=tk.LEFT, fill=tk.X, expand=True, padx=24)
        ttk.Label(guide, text="NEXT STEP", style="Title.TLabel",
                  font=(FONT_FAMILY, 9, "bold")).pack(anchor=tk.W)
        self.guide_label = ttk.Label(guide, text="", style="Panel.TLabel", font=GUIDE_FONT,
                                     wraplength=380, width=42, justify=tk.LEFT)
        self.guide_label.pack(anchor=tk.W)

    def _update_orb(self):
        """Grey with no link, teal when linked and idle, pulsing green while running."""
        if not self.link_alive():
            colour = COLOUR_ORB_DIM
        elif self.rover_running():
            level = (math.sin(time.monotonic() * math.tau * ORB_PULSE_HZ) + 1) / 2
            colour = self._blend(COLOUR_GREEN_LAMP, COLOUR_ORB_RUN, level)
        else:
            colour = COLOUR_ACCENT
        self.orb.itemconfig(self.orb_ring, outline=colour)
        self.orb.itemconfig(self.orb_dot, fill=colour)

    @staticmethod
    def _blend(start, end, amount):
        a = [int(start[i:i + 2], 16) for i in (1, 3, 5)]
        b = [int(end[i:i + 2], 16) for i in (1, 3, 5)]
        return "#" + "".join(f"{round(x + (y - x) * amount):02x}" for x, y in zip(a, b))

    def _card(self, parent, **pack):
        card = glass.GlassCard(parent)
        card.pack(**pack)
        return card.inner

    def _panel(self, parent, title, **pack):
        """A card with a title row. Extra widgets for that row go in `frame.header`."""
        frame = self._card(parent, **pack)
        frame.header = ttk.Frame(frame, style="Plain.TFrame")
        frame.header.pack(fill=tk.X)
        frame.title = ttk.Label(frame.header, text=title, style="Title.TLabel",
                                font=(FONT_FAMILY, 9, "bold"))
        frame.title.pack(side=tk.LEFT)
        return frame

    def _button(self, parent, text, command, kind="normal", min_width=0):
        return glass.GlassButton(parent, text, command, kind, min_width)

    def _build_layout(self):
        self._build_scroll_area()
        self._build_header()
        self._build_history_panel(self.page)     # Packed to the bottom, before the body
        body = tk.Frame(self.page, bg=SOLID_BG)
        body.pack(fill=tk.BOTH, expand=True, padx=8, pady=(0, 6))
        left = tk.Frame(body, bg=SOLID_BG)
        left.pack(side=tk.LEFT, fill=tk.Y)
        tk.Frame(left, width=LEFT_COLUMN_WIDTH, height=1, bg=SOLID_BG).pack()
        right = tk.Frame(body, bg=SOLID_BG)
        right.pack(side=tk.LEFT, fill=tk.BOTH, expand=True)

        self._build_run_panel(left)
        self._build_mode_panel(left)
        self._build_wheels_panel(left)
        self._build_chart_panel(right)
        self._build_pc_loop_panel(right)
        self._build_tuning_panel(right)
        self._build_optimiser_panel(right)
        self._fit_window_to_screen()

    def _build_run_panel(self, parent):
        panel = self._panel(parent, "RUN", fill=tk.X)
        ttk.Label(panel.header, text="Up/Down: speed   C: calibrate",
                  style="Muted.TLabel").pack(side=tk.RIGHT)
        readout = ttk.Frame(panel, style="Plain.TFrame")
        readout.pack(fill=tk.X, pady=(2, 0))
        self.state_label = ttk.Label(readout, text="STOPPED", style="Big.TLabel",
                                     foreground=COLOUR_BAD)
        self.state_label.pack(side=tk.LEFT)
        self.timer_label = ttk.Label(readout, text="0.000 s", style="Big.TLabel")
        self.timer_label.pack(side=tk.RIGHT)
        self.detail_label = ttk.Label(panel, text="", style="Panel.TLabel")
        self.detail_label.pack(anchor=tk.W, pady=(0, 4))
        buttons = ttk.Frame(panel, style="Plain.TFrame")
        buttons.pack(fill=tk.X)
        self._button(buttons, "START / STOP  [Space]", self.toggle_run, "primary").pack(fill=tk.X)
        self._button(buttons, "EMERGENCY STOP  [Esc]", self.stop_run,
                     "danger").pack(fill=tk.X, pady=4)

        tracks = ttk.Frame(panel, style="Plain.TFrame")
        tracks.pack(fill=tk.X)
        ttk.Label(tracks, text="Track:", style="Muted.TLabel").pack(side=tk.LEFT, padx=(0, 8))
        for name in TRACKS:
            ttk.Radiobutton(tracks, text=name, value=name, variable=self.track_name,
                            style="Panel.TRadiobutton", command=self.select_track,
                            takefocus=0).pack(side=tk.LEFT, padx=(0, 12))
        notes = ttk.Frame(panel, style="Plain.TFrame")
        notes.pack(fill=tk.X, pady=(4, 0))
        ttk.Label(notes, text="Note on last run:", style="Muted.TLabel").pack(side=tk.LEFT,
                                                                           padx=(0, 8))
        self.note_entry = ttk.Entry(notes, width=NOTE_WIDTH)
        self.note_entry.pack(side=tk.LEFT, fill=tk.X, expand=True)
        self.note_entry.bind("<Return>", self.save_note)
        self.note_hint = ttk.Label(panel, text="Click the box, type, Enter to save."
                                               " Hotkeys pause while typing (Esc still stops).",
                                   style="Muted.TLabel")
        self.note_hint.pack(anchor=tk.W)

    def _build_mode_panel(self, parent):
        panel = self._panel(parent, "MODE  (keys 1-4, only while stopped)", fill=tk.X)
        modes = ttk.Frame(panel, style="Plain.TFrame")
        modes.pack(fill=tk.X)
        for index, (key, mode) in enumerate(MODE_KEYS.items()):
            ttk.Radiobutton(modes, text=f"{key}  {p.MODE_NAMES[mode]}", value=mode,
                            variable=self.selected_mode, style="Panel.TRadiobutton",
                            command=lambda m=mode: self.select_mode(m),
                            takefocus=0).grid(row=index // 2, column=index % 2, sticky=tk.W,
                                              padx=(0, 18))
        self.speed_label = ttk.Label(panel, text="Base speed: --", style="Panel.TLabel",
                                     font=(FONT_FAMILY, 11, "bold"))
        self.speed_label.pack(anchor=tk.W, pady=(4, 0))
        controllers = ttk.Frame(panel, style="Plain.TFrame")
        controllers.pack(fill=tk.X, pady=(4, 0))
        ttk.Label(controllers, text="Controller [T, live]:", style="Muted.TLabel").pack(side=tk.LEFT,
                                                                             padx=(0, 8))
        for controller, name in p.CONTROLLER_NAMES.items():
            ttk.Radiobutton(controllers, text=name, value=controller,
                            variable=self.selected_controller, style="Panel.TRadiobutton",
                            command=lambda c=controller: self.select_controller(c),
                            takefocus=0).pack(side=tk.LEFT, padx=(0, 10))
        self._button(panel, "Calibrate sensors  [C]", self.calibrate).pack(fill=tk.X, pady=(4, 0))

    def _build_wheels_panel(self, parent):
        panel = self._panel(parent, "WHEELS  (PWM %, 50 % = stopped)", fill=tk.BOTH,
                            expand=True)   # Stretches so the left column ends level with the right
        self.wheel_widgets = {}
        for side, colour in (("Left", COLOUR_IR_LEFT), ("Right", COLOUR_IR_RIGHT)):
            ttk.Label(panel, text=side.upper(), style="Muted.TLabel").pack(anchor=tk.W, pady=(6, 0))
            row = ttk.Frame(panel, style="Plain.TFrame")
            row.pack(fill=tk.X)
            value = ttk.Label(row, text="--", style="Big.TLabel", width=7)
            value.pack(side=tk.LEFT)
            bar = tk.Canvas(row, width=WHEEL_BAR_WIDTH, height=WHEEL_BAR_HEIGHT,
                            bg=COLOUR_CHART_BG, highlightthickness=0)
            bar.pack(side=tk.LEFT, padx=8)
            fill = bar.create_rectangle(0, 0, 0, WHEEL_BAR_HEIGHT, fill=colour, outline="")
            bar.create_line(WHEEL_BAR_WIDTH / 2, 0, WHEEL_BAR_WIDTH / 2, WHEEL_BAR_HEIGHT,
                            fill=COLOUR_TEXT, dash=(2, 2))
            detail = ttk.Label(panel, text="", style="Muted.TLabel", font=READOUT_FONT)
            detail.pack(anchor=tk.W)
            self.wheel_widgets[side] = (value, bar, fill, detail)

    def _build_chart_panel(self, parent):
        panel = self._panel(parent, "SENSORS  (level 0-1: 1 = white line, 0 = black table)",
                            fill=tk.BOTH, expand=True)
        self._button(panel.header, "Reset min/max", self.level_range.clear).pack(side=tk.RIGHT)
        lamps = ttk.Frame(panel, style="Plain.TFrame")
        lamps.pack(anchor=tk.W, pady=(4, 0))
        self.lamps = {}
        for side in ("Left", "Right"):
            canvas = tk.Canvas(lamps, width=LAMP_SIZE, height=LAMP_SIZE, bg=COLOUR_PANEL,
                               highlightthickness=0)
            canvas.pack(side=tk.LEFT)
            oval = canvas.create_oval(4, 4, LAMP_SIZE - 4, LAMP_SIZE - 4,
                                      fill=COLOUR_LAMP_OFF, outline=COLOUR_BORDER, width=2)
            text = ttk.Label(lamps, text=f"{side}: --", style="Panel.TLabel",
                             font=(FONT_FAMILY, 12, "bold"), width=14)
            text.pack(side=tk.LEFT, padx=(4, 16))
            self.lamps[side] = (canvas, oval, text)
        self._build_level_bars(panel)
        self.readout_label = ttk.Label(panel, text="", style="Panel.TLabel", font=READOUT_FONT)
        self.readout_label.pack(anchor=tk.W, pady=(4, 4))
        self.chart_canvas = tk.Canvas(panel, width=CHART_WIDTH, height=CHART_HEIGHT,
                                      bg=COLOUR_CHART_BG, highlightthickness=0)
        self.chart_canvas.pack(fill=tk.BOTH, expand=True)
        legend = ttk.Frame(panel, style="Plain.TFrame")
        legend.pack(anchor=tk.W)
        for text, colour in (("IR left", COLOUR_IR_LEFT), ("IR right", COLOUR_IR_RIGHT),
                             ("Position (centre = 0)", COLOUR_POSITION)):
            ttk.Label(legend, text=f"■ {text}", foreground=colour,
                      style="Panel.TLabel").pack(side=tk.LEFT, padx=(0, 12))

    # ================================================================
    # PC control loop panel: where the processing happens, and how well
    # the radio loop is keeping up
    # ================================================================
    def _build_pc_loop_panel(self, parent):
        panel = self._panel(parent, "PC CONTROL LOOP  (all duty-cycle processing on this PC)",
                            fill=tk.X)
        flow = tk.Frame(panel, bg=COLOUR_PANEL)
        flow.pack(fill=tk.X, pady=(6, 6))
        self.pipeline = {}
        stages = (("rover_in", "ROVER  \u00b7  IR sensors", False),
                  ("radio_up", "XBee", None),
                  ("pc", "PC  \u00b7  controller", True),
                  ("radio_down", "XBee", None),
                  ("rover_out", "ROVER  \u00b7  DACs", False))
        for column, (key, title, is_pc) in enumerate(stages):
            if is_pc is None:
                arrow = tk.Label(flow, text=f"{title}\n\u2192", bg=COLOUR_PANEL, fg=COLOUR_MUTED,
                                 font=(FONT_FAMILY, 9), justify=tk.CENTER)
                arrow.grid(row=0, column=column, padx=4)
                continue
            border = COLOUR_ACCENT if is_pc else COLOUR_BORDER
            box = tk.Frame(flow, bg=COLOUR_PANEL_RAISED, highlightbackground=border,
                           highlightcolor=border, highlightthickness=2 if is_pc else 1)
            box.grid(row=0, column=column, sticky="nsew")
            flow.columnconfigure(column, weight=2 if is_pc else 1)
            tk.Label(box, text=title, bg=COLOUR_PANEL_RAISED,
                     fg=COLOUR_ACCENT if is_pc else COLOUR_MUTED,
                     font=(FONT_FAMILY, 9, "bold")).pack(anchor=tk.W, padx=8, pady=(4, 0))
            value = tk.Label(box, text="--", bg=COLOUR_PANEL_RAISED, fg=COLOUR_TEXT,
                             font=READOUT_FONT, justify=tk.LEFT)
            value.pack(anchor=tk.W, padx=8, pady=(0, 4))
            self.pipeline[key] = value

        stats = ttk.Frame(panel, style="Plain.TFrame")
        stats.pack(fill=tk.X)
        self.health_labels = {}
        for column, key in enumerate(("Control rate", "Radio round trip", "Lost frames",
                                      "Bunched", "Throttled", "Dropouts", "PC step time")):
            ttk.Label(stats, text=key, style="Muted.TLabel").grid(
                row=0, column=column, sticky=tk.W, padx=(0, 28))
            value = ttk.Label(stats, text="--", style="Panel.TLabel", font=READOUT_FONT)
            value.grid(row=1, column=column, sticky=tk.W, padx=(0, 28))
            self.health_labels[key] = value
        self.health_verdict = ttk.Label(panel, text="", style="Muted.TLabel")
        self.health_verdict.pack(anchor=tk.W, pady=(4, 0))

    @staticmethod
    def _health_colour(value, good, fair, higher_is_better=False):
        if higher_is_better:
            value, good, fair = -value, -good, -fair
        return COLOUR_OK if value <= good else COLOUR_WARN if value <= fair else COLOUR_BAD

    def _update_pc_loop(self):
        if not self.link_alive():
            for label in list(self.pipeline.values()) + list(self.health_labels.values()):
                self._set_widget(label, text="--", foreground=COLOUR_MUTED)
            self._set_widget(self.health_verdict, text="Connect to the rover to see the loop.")
            return
        t = self.telemetry
        brain = self.link.brain
        self._set_widget(self.pipeline["rover_in"], foreground=COLOUR_TEXT,
                         text=f"L {sensor_level(t.ir_left):.2f}   R {sensor_level(t.ir_right):.2f}")
        self._set_widget(self.pipeline["pc"], foreground=COLOUR_TEXT,
                         text=f"{p.CONTROLLER_NAMES[self._controller()]}   "
                              f"pos {t.position:+.2f}\n"
                              f"duty L {p.dac_to_percent(brain.dac_left):.0f} %   "
                              f"R {p.dac_to_percent(brain.dac_right):.0f} %")
        self._set_widget(self.pipeline["rover_out"], foreground=COLOUR_TEXT,
                         text=f"DAC {t.dac_left} / {t.dac_right}")

        health = self.link.health()
        expected_hz = 1 / NOMINAL_PERIOD_S
        good_hz, bad_hz = 0.875 * expected_hz, 0.625 * expected_hz     # 35 / 25 Hz at 40 Hz
        period_ms = round(NOMINAL_PERIOD_S * 1000)
        rows = {
            "Control rate": (f"{health.rate_hz:.0f} Hz",
                             self._health_colour(health.rate_hz, good_hz, bad_hz, higher_is_better=True)),
            "Radio round trip": (f"{health.round_trip_ms:.0f} ms (max {health.round_trip_max_ms:.0f})",
                                 self._health_colour(health.round_trip_ms, 60, 100)),
            "Lost frames": (f"{health.lost_frames} in 2 s",
                            self._health_colour(health.lost_frames, 0, 2)),
            "Bunched": (f"{health.stale_frames} in 2 s",
                        self._health_colour(health.stale_frames, 1, 4)),
            "Throttled": (f"{health.throttled} in 2 s", self._health_colour(health.throttled, 4, 12)),
            "Dropouts": (f"{health.dropouts}", self._health_colour(health.dropouts, 0, 2)),
            "PC step time": (f"{health.step_ms:.1f} ms", self._health_colour(health.step_ms, 2, 10)),
        }
        for key, (text, colour) in rows.items():
            self._set_widget(self.health_labels[key], text=text, foreground=colour)
        if health.lost_frames == 0 and health.round_trip_ms <= 60 and health.rate_hz >= good_hz:
            verdict = f"Loop healthy at SENSOR_PERIOD_MS {period_ms}."
        elif health.lost_frames > 2 or health.rate_hz < bad_hz:
            verdict = f"Radio is dropping frames: keep {period_ms} ms, check the XBee range and pairing."
        elif health.stale_frames > 4:
            verdict = f"Frames arrive bunched up (radio congested): keep SENSOR_PERIOD_MS at {period_ms}."
        else:
            verdict = f"Loop OK but slow: keep SENSOR_PERIOD_MS at {period_ms}."
        self._set_widget(self.health_verdict, text=verdict)

    def _build_level_bars(self, panel):
        """One bar per sensor for turning its pot: live level, and the lowest and
        highest level seen since Reset min/max."""
        levels = ttk.Frame(panel, style="Plain.TFrame")
        levels.pack(anchor=tk.W, pady=(6, 0))
        self.level_widgets = {}
        for row, (side, colour) in enumerate((("Left", COLOUR_IR_LEFT), ("Right", COLOUR_IR_RIGHT))):
            ttk.Label(levels, text=side, width=6, style="Panel.TLabel").grid(row=row, column=0,
                                                                            sticky=tk.W)
            bar = tk.Canvas(levels, width=LEVEL_BAR_WIDTH, height=LEVEL_BAR_HEIGHT,
                            bg=COLOUR_CHART_BG, highlightthickness=0)
            bar.grid(row=row, column=1, pady=2)
            fill = bar.create_rectangle(0, 0, 0, LEVEL_BAR_HEIGHT, fill=colour, outline="")
            for mark in (LEVEL_BLACK_MARK, LEVEL_WHITE_MARK):
                x = mark * LEVEL_BAR_WIDTH
                bar.create_line(x, 0, x, LEVEL_BAR_HEIGHT, fill=COLOUR_TEXT, dash=(2, 2))
            value = ttk.Label(levels, text="--", style="Panel.TLabel", width=5,
                              font=("Consolas", 14, "bold"))
            value.grid(row=row, column=2, padx=8)
            seen = ttk.Label(levels, text="", style="Muted.TLabel", font=READOUT_FONT)
            seen.grid(row=row, column=3, sticky=tk.W)
            self.level_widgets[side] = (bar, fill, value, seen)
        ttk.Label(panel, style="Muted.TLabel", wraplength=640, justify=tk.LEFT,
                  text=(f"Pot calibration: turn each sensor's pot until it reads above "
                        f"{LEVEL_WHITE_MARK:.2f} over the white line and below "
                        f"{LEVEL_BLACK_MARK:.2f} over the black table (dashed marks), with left "
                        f"and right matching. Green = clear reading, orange = in between.")
                  ).pack(anchor=tk.W, pady=(2, 0))

    def _build_tuning_panel(self, parent):
        panel = self._panel(parent, "TUNING  (live - changes the rover immediately, current mode)",
                            fill=tk.X)
        self.tuning_title = panel.title
        grid = ttk.Frame(panel, style="Plain.TFrame")
        grid.pack(fill=tk.X, pady=(4, 0))
        self.tuning_grid = grid
        self.tuning_widgets = {}
        for column, param in enumerate(p.TUNING_PARAMS):
            grid.columnconfigure(column, weight=1)
            cell = ttk.Frame(grid, style="Plain.TFrame")
            cell.grid(row=0, column=column)
            name = ttk.Label(cell, text=param.label, style="Muted.TLabel")
            name.pack()
            value = ttk.Label(cell, text="--", style="Panel.TLabel", font=("Consolas", 18, "bold"))
            value.pack()
            buttons = ttk.Frame(cell, style="Plain.TFrame")
            buttons.pack()
            minus = self._button(buttons, "-", lambda k=param.key: self.adjust_tuning(k, -1),
                                 min_width=44)
            plus = self._button(buttons, "+", lambda k=param.key: self.adjust_tuning(k, +1),
                                min_width=44)
            minus.pack(side=tk.LEFT, padx=2)
            plus.pack(side=tk.LEFT, padx=2)
            hint = ttk.Label(cell, text="", style="Muted.TLabel")
            hint.pack()
            self.tuning_widgets[param.key] = {"value": value, "minus": minus, "plus": plus,
                                              "hint": hint, "name": name, "cell": cell}
        row = ttk.Frame(panel, style="Plain.TFrame")
        row.pack(fill=tk.X, pady=(6, 0))
        self._button(row, "Apply saved", self.apply_saved_tuning).pack(side=tk.LEFT)
        self.reset_integral_button = self._button(row, "Reset integral", self.reset_integral)
        self.reset_integral_button.pack(side=tk.LEFT, padx=(6, 0))
        self._button(row, "Copy for pc_brain.py", self.copy_tuning).pack(side=tk.LEFT, padx=6)
        self.tuning_status = ttk.Label(row, text="", style="Muted.TLabel")
        self.tuning_status.pack(side=tk.LEFT, padx=6)

    def _build_optimiser_panel(self, parent):
        panel = self._panel(parent, "LAP OPTIMISER  (stopped only)", fill=tk.X)
        self.optimiser_title = panel.title
        self.optimiser_status = ttk.Label(panel, text="", style="Panel.TLabel", wraplength=700,
                                          justify=tk.LEFT)
        self.optimiser_status.pack(anchor=tk.W, pady=(4, 2))
        rating = ttk.Frame(panel, style="Plain.TFrame")
        rating.pack(fill=tk.X)
        self.rating_label = ttk.Label(rating, text="", style="Panel.TLabel",
                                      font=(FONT_FAMILY, 11, "bold"))
        self.rating_label.pack(side=tk.LEFT)
        self.rating_buttons = [
            self._button(rating, "Clean lap  [Y]", lambda: self.rate_run(True), "primary"),
            self._button(rating, "Failed  [N]", lambda: self.rate_run(False), "danger"),
            self._button(rating, "Don't count", lambda: self.rate_run(None)),
        ]
        for button in reversed(self.rating_buttons):
            button.pack(side=tk.RIGHT, padx=(4, 0))
        row = ttk.Frame(panel, style="Plain.TFrame")
        row.pack(fill=tk.X, pady=(6, 0))
        self.suggest_button = self._button(row, "Try next settings  [G]", self.suggest_settings)
        self.suggest_button.pack(side=tk.LEFT)
        self.best_button = self._button(row, "Use best  [B]", self.apply_best_settings)
        self.best_button.pack(side=tk.LEFT, padx=6)
        self._button(row, "Undo last rating", self.undo_rating).pack(side=tk.LEFT)
        self.optimiser_detail = ttk.Label(panel, text="", style="Muted.TLabel", wraplength=700,
                                          justify=tk.LEFT)
        self.optimiser_detail.pack(anchor=tk.W, pady=(4, 0))

    def _build_history_panel(self, parent):
        panel = self._panel(parent, "RUN HISTORY", side=tk.BOTTOM, fill=tk.X, padx=8, pady=(0, 6))
        columns = ("division", "attempt", "time_s", "result", "track", "note")
        self.history = ttk.Treeview(panel, columns=columns, show="headings", height=2,
                                    takefocus=0)
        for column in columns:
            self.history.heading(column, text=column)
            self.history.column(column, width=300 if column == "note" else 100)
        self.history.pack(fill=tk.X, pady=(4, 4))
        self._button(panel.header, "Change log file", self.choose_log_file).pack(side=tk.RIGHT)
        self.log_label = ttk.Label(panel.header, text=f"Logging to {self.log_path}",
                                   style="Muted.TLabel")
        self.log_label.pack(side=tk.RIGHT, padx=10)

    def _bind_keys(self):
        bind = self._bind_hotkey
        bind("<space>", self.toggle_run)
        self.root.bind_all("<Escape>", self._on_escape)    # Works while typing too
        bind("<Up>", lambda: self.change_speed(SPEED_STEP))
        bind("<Down>", lambda: self.change_speed(-SPEED_STEP))
        bind("<KeyPress-c>", self.calibrate)
        bind("<KeyPress-C>", self.calibrate)
        bind("<KeyPress-t>", self.cycle_controller)
        bind("<KeyPress-T>", self.cycle_controller)
        for slot, (up_key, down_key) in enumerate(TUNING_KEYS):
            for key, direction in ((up_key, +1), (down_key, -1)):
                for variant in (key, key.upper()):
                    bind(f"<KeyPress-{variant}>", lambda s=slot, d=direction: self.tune_slot(s, d))
        for key, mode in MODE_KEYS.items():
            bind(key, lambda m=mode: self.select_mode(m))
        for key, action in (("y", lambda: self.rate_run(True)), ("n", lambda: self.rate_run(False)),
                            ("g", self.suggest_settings), ("b", self.apply_best_settings)):
            for variant in (key, key.upper()):
                bind(f"<KeyPress-{variant}>", action)

    def _typing(self):
        """True while a text box (the note, or a pairing field) has the cursor.
        The port list is a Combobox (an Entry too) but is not typed into."""
        try:
            widget = self.root.focus_get()
        except KeyError:               # Focus is in a combobox drop-down
            return False
        return isinstance(widget, (tk.Entry, ttk.Entry)) and not isinstance(widget, ttk.Combobox)

    def _bind_hotkey(self, sequence, action):
        """A window-wide hotkey that does nothing while the note box has the cursor."""
        self.root.bind_all(sequence, lambda _event: None if self._typing() else action())

    def _on_escape(self, _event):
        self.stop_run()
        self.root.focus_set()          # Leave the note box, so the hotkeys work again

    # ================================================================
    # Connection
    # ================================================================
    def refresh_ports(self):
        ports = [f"{port.device} - {port.description}" for port in list_ports.comports()]
        self.port_box["values"] = ports
        if ports and not self.port_box.get():
            self.port_box.current(0)

    def toggle_connection(self):
        if self.link.is_open:
            self.link.send(p.CMD_STOP)
            self.link.close()
            self.pwm_applied = False
            self.connect_button.config(text="Connect")
            return
        selection = self.port_box.get()
        if not selection:
            messagebox.showwarning("No port", "Plug in the PC-side XBee and press Refresh.")
            return
        device = selection.split(" - ")[0]
        try:
            self.link.open(device)
        except serial.SerialException as error:
            messagebox.showerror("Could not open port", str(error))
            return
        self.level_range.clear()
        self.connect_button.config(text="Disconnect")

    def open_pairing(self):
        if self.link.is_open:
            if not messagebox.askyesno("Disconnect first",
                                       "Pairing needs the XBee's COM port. Disconnect now?"):
                return
            self.toggle_connection()
        PairingDialog(self)

    # ================================================================
    # PWM test window: bang-bang wheel levels
    # ================================================================
    def open_pwm_window(self):
        if self.pwm_window is not None:
            self.pwm_window.window.lift()
            return
        self.pwm_window = PwmWindow(self)

    def _pwm_path(self):
        return os.path.join(SETTINGS_DIR, PWM_FILE)

    def _load_saved_pwm(self):
        try:
            with open(self._pwm_path(), encoding="utf-8") as saved_file:
                return json.load(saved_file)
        except (OSError, ValueError):
            return {}

    def _save_pwm(self):
        """Remember this mode's four levels so they come back after a rover reset."""
        if self.tuning is None:
            return
        saved = self._load_saved_pwm()
        levels = self.pwm_levels()
        saved[p.MODE_NAMES[self.tuning.mode]] = {
            field: levels[field] for field, _command, _label in p.PWM_LEVELS}
        try:
            with open(self._pwm_path(), "w", encoding="utf-8") as saved_file:
                json.dump(saved, saved_file, indent=2)
        except OSError:
            pass

    def pwm_levels(self):
        """The levels the window shows: requested values until the rover echoes them."""
        return {field: self.pwm_target.get((self.tuning.mode, field), getattr(self.tuning, field))
                for field, _command, _label in p.PWM_LEVELS}

    def set_pwm_level(self, field, command, value):
        if not self.link_alive() or self.tuning is None:
            return
        value = max(0, min(100, int(value)))
        if self.link.send(command, value):
            self.pwm_target[(self.tuning.mode, field)] = value
            self._save_pwm()

    def _apply_saved_pwm(self):
        """After a connect or mode change, send the saved levels the rover does not have."""
        if self.pwm_applied or self.tuning is None:
            return
        self.pwm_applied = True
        saved = self._load_saved_pwm().get(p.MODE_NAMES[self.tuning.mode])
        if not saved:
            return
        for field, command, _label in p.PWM_LEVELS:
            value = saved.get(field)
            if isinstance(value, int) and value != getattr(self.tuning, field):
                self.link.send(command, max(0, min(100, value)))

    def link_alive(self):
        return self.telemetry is not None and \
            time.monotonic() - self.telemetry_time < TELEMETRY_STALE_S

    def rover_running(self):
        return self.link_alive() and self.telemetry.running

    # ================================================================
    # Commands
    # ================================================================
    def toggle_run(self):
        if self.rover_running() or self.run_start is not None:
            self.stop_run()
        else:
            self.start_run()

    def start_run(self):
        if not self.link_alive():
            # _set_widget keeps the cache in step, so the next refresh can clear it
            self._set_widget(self.detail_label, text="No telemetry from rover - check XBee link")
            return
        mode = self.selected_mode.get()
        if self.telemetry.mode != mode:
            self.link.send(p.CMD_SET_MODE, mode)  # Resets speed to that mode's default
        if self.link.send(p.CMD_START):
            self.run_start = time.monotonic()
            self.run_mode = mode
            self.run_track = self.track_name.get()
            self._open_sample_log(mode)

    def stop_run(self):
        self.link.send(p.CMD_STOP)
        self._finish_run("stopped by operator")

    def select_track(self):
        """Allowed only while stopped: a run keeps the track it started on."""
        if self.run_start is not None:
            self.track_name.set(self.run_track)
            return
        self._save_session()

    def _session_path(self):
        return os.path.join(SETTINGS_DIR, SESSION_FILE)

    def _load_session(self):
        try:
            with open(self._session_path(), encoding="utf-8") as session_file:
                return json.load(session_file)
        except (OSError, ValueError):
            return {}

    def _save_session(self):
        try:
            with open(self._session_path(), "w", encoding="utf-8") as session_file:
                json.dump({"track": self.track_name.get(), "mode": self.selected_mode.get()},
                          session_file, indent=2)
        except OSError as error:
            self.log_label.config(text=f"Could not save {SESSION_FILE}: {error}")

    def select_mode(self, mode):
        if self.rover_running():
            self.selected_mode.set(self.telemetry.mode)
            return
        self.selected_mode.set(mode)
        self.link.send(p.CMD_SET_MODE, mode)
        self._save_session()

    def _keep_session_mode(self, t):
        """The GUI's selected mode is the session's mode. A rover that comes back
        from a link drop may have reset to Clockwise, so while it is stopped it is
        put back in the selected mode instead of the GUI following it."""
        mode = self.selected_mode.get()
        if t.running or t.mode == mode:
            return
        now = time.monotonic()
        if now - self.mode_sent > MODE_RESEND_S and self.link.send(p.CMD_SET_MODE, mode):
            self.mode_sent = now

    def select_controller(self, controller):
        """Steering controller for the current mode, also while running."""
        if not self.link_alive() or self.tuning is None:
            self.selected_controller.set(self._controller())
            self.tuning_status.config(text="T needs the rover link - connect first")
            return
        if self.link.send(p.CMD_SET_CONTROLLER, controller):
            self.controller_target = controller
            self.controller_sent = time.monotonic()
            self.tuning_status.config(text=f"Controller: {p.CONTROLLER_NAMES[controller]}")
        self.selected_controller.set(self._controller())

    def cycle_controller(self):
        self.select_controller((self._controller() + 1) % len(p.CONTROLLER_NAMES))

    def reset_integral(self):
        self.link.send(p.CMD_RESET_INTEGRAL)

    def change_speed(self, step):
        if not self.link_alive():
            return
        speed = max(0, min(SPEED_MAX, self.telemetry.speed + step))
        self.link.send(p.CMD_SET_SPEED, speed)

    def calibrate(self):
        if self.rover_running():
            return
        self.link.send(p.CMD_CALIBRATE)

    def _update_wheels(self, t):
        for side, dac, level, white in (("Left", t.dac_left, t.ir_left, t.left_white),
                                        ("Right", t.dac_right, t.ir_right, t.right_white)):
            value, bar, fill, detail = self.wheel_widgets[side]
            percent = p.dac_to_percent(dac)
            value.config(text=f"{percent:5.1f}%")
            half = WHEEL_BAR_WIDTH / 2
            x = percent / 100 * WHEEL_BAR_WIDTH
            bar.coords(fill, min(x, half), 0, max(x, half), WHEEL_BAR_HEIGHT)
            detail.config(text=f"{p.dac_to_volts(dac):+.1f} V   DAC {dac:3d}   "
                               f"sensor {sensor_level(level):.2f} "
                               f"{'WHITE line' if white else 'black'}")

    def _heartbeat(self):
        self.link.send(p.CMD_PING)
        self.root.after(HEARTBEAT_MS, self._heartbeat)

    # ================================================================
    # Run log
    # ================================================================
    def _finish_run(self, result):
        if self.run_start is None:
            return
        elapsed = time.monotonic() - self.run_start
        mode = self.run_mode
        self.run_start = None
        self._close_sample_log()
        self.attempts[mode] += 1
        row = [datetime.now().strftime("%Y-%m-%d"), p.MODE_NAMES[mode],
               self.attempts[mode], f"{elapsed:.3f}", result, self._tuning_note(),
               self.run_track, ""]
        self.last_history_item = self.history.insert("", 0, values=row[1:5] + row[6:])
        self.timer_label.config(text=f"{elapsed:.3f} s")
        self._hold_for_rating(mode, elapsed, result)
        try:
            self._upgrade_log_header()
            new_file = not os.path.exists(self.log_path)
            with open(self.log_path, "a", newline="") as log_file:
                writer = csv.writer(log_file)
                if new_file:
                    writer.writerow(RUN_LOG_HEADER)
                writer.writerow(row)
            self.last_log_row = [str(value) for value in row]
        except OSError as error:
            self.log_label.config(text=f"Log write failed: {error}")

    def _read_log(self):
        with open(self.log_path, newline="") as log_file:
            return list(csv.reader(log_file))

    def _write_log(self, rows):
        with open(self.log_path, "w", newline="") as log_file:
            csv.writer(log_file).writerows(rows)

    def _upgrade_log_header(self):
        """Older logs lack the track and note columns: add them to the header once."""
        if not os.path.exists(self.log_path):
            return
        rows = self._read_log()
        if rows and rows[0] != RUN_LOG_HEADER and rows[0] == RUN_LOG_HEADER[:len(rows[0])]:
            rows[0] = RUN_LOG_HEADER
            self._write_log(rows)

    def save_note(self, _event=None):
        """Writes the note box into the last run's row of the run log."""
        note = self.note_entry.get().strip()
        self.root.focus_set()          # Hotkeys work again
        if not note:
            return
        if self.last_log_row is None:
            self.note_hint.config(text="No run to attach the note to yet.")
            return
        note_column = RUN_LOG_HEADER.index("note")
        try:
            rows = self._read_log()
            for row in reversed(rows):
                if row[:note_column] == self.last_log_row[:note_column]:
                    row[note_column:] = [note]
                    break
            else:
                self.note_hint.config(text="The last run is not in the log file any more.")
                return
            self._write_log(rows)
        except OSError as error:
            self.note_hint.config(text=f"Could not save the note: {error}")
            return
        self.history.set(self.last_history_item, "note", note)
        self.note_entry.delete(0, tk.END)
        self.note_hint.config(text=f"Saved on the last run: {note}")

    # One CSV per run with every telemetry sample (about 20 per second), so
    # IR readings can be plotted and thresholds tuned after the run.
    def _open_sample_log(self, mode):
        self._close_sample_log()
        stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        controller = p.CONTROLLER_NAMES.get(self._controller(), "")
        controller = controller.replace(" ", "").replace("/", "-")
        name = f"ir-log_{p.MODE_NAMES[mode]}_{controller}_{self.track_name.get()}_{stamp}.csv"
        self.sample_path = os.path.join(os.path.dirname(self.log_path), name)
        try:
            self.sample_file = open(self.sample_path, "w", newline="")
            self.sample_writer = csv.writer(self.sample_file)
            self.sample_writer.writerow(SAMPLE_LOG_HEADER)
        except OSError as error:
            self.sample_file = self.sample_writer = None
            self.log_label.config(text=f"IR log failed: {error}")

    def _log_sample(self, t):
        if self.sample_writer is None or self.run_start is None:
            return
        try:
            self.sample_writer.writerow([
                f"{time.monotonic() - self.run_start:.3f}", t.ir_left, t.ir_right,
                f"{t.position:.3f}", t.dac_left, t.dac_right, t.speed, int(t.line_lost)])
            self.sample_file.flush()
        except OSError as error:
            self._close_sample_log()
            self.log_label.config(text=f"IR log failed: {error}")

    def _close_sample_log(self):
        if self.sample_file:
            self.sample_file.close()
            if self.sample_path:
                self.log_label.config(text=f"IR samples saved to {self.sample_path}")
        self.sample_file = self.sample_writer = None

    # ================================================================
    # Lap optimiser (see optimizer.py)
    # ================================================================
    def _optimiser_path(self):
        return os.path.join(SETTINGS_DIR, OPTIMISER_FILE)

    def _load_rated_runs(self):
        try:
            with open(self._optimiser_path(), encoding="utf-8") as saved_file:
                return json.load(saved_file)
        except (OSError, ValueError):
            return []

    def _save_rated_runs(self, runs):
        try:
            with open(self._optimiser_path(), "w", encoding="utf-8") as saved_file:
                json.dump(runs, saved_file, indent=1)
            return True
        except OSError as error:
            self.optimiser_note = f"Could not save {OPTIMISER_FILE}: {error}"
            return False

    def _current_settings(self, controller):
        """The settings the rover is using now, in optimizer.py's keys."""
        settings = {}
        levels = self.pwm_levels()
        for dimension in opt.SEARCH_SPACE[controller]:
            key = dimension.key
            if key == "speed":
                settings[key] = self.telemetry.speed
            elif key in opt.PWM_PAIRS:
                left, right = opt.PWM_PAIRS[key]
                settings[key] = round((levels[left] + levels[right]) / 2)
            else:
                settings[key] = self._tuning_value(key)
        return settings

    def _hold_for_rating(self, mode, elapsed, result):
        """Keep the run until the operator presses Y (clean lap) or N (failed).
        A radio dropout says nothing about the settings, so it is not offered."""
        self.unrated_run = None
        if self.tuning is None or self.telemetry is None \
                or self.tuning.mode != mode:
            return
        controller = self._controller()
        self.unrated_run = {"date": datetime.now().strftime("%Y-%m-%d %H:%M"),
                            "mode": p.MODE_NAMES[mode],
                            "controller": p.CONTROLLER_NAMES[controller],
                            "settings": self._current_settings(controller),
                            "time_s": round(elapsed, 3), "result": result,
                            "track": self.run_track}

    def rate_run(self, clean):
        """clean: True = valid lap, False = failed, None = do not count it."""
        if self.unrated_run is None or self.rover_running():
            return
        entry, self.unrated_run = self.unrated_run, None
        if clean is None:
            self.optimiser_note = "Last run not counted"
            return
        entry["clean"] = clean
        runs = self._load_rated_runs()
        runs.append(entry)
        if self._save_rated_runs(runs):
            verdict = f"clean lap {entry['time_s']:.3f} s" if clean else "failed"
            self.optimiser_note = f"Saved: {verdict}. Press G for the next settings to try."

    def undo_rating(self):
        runs = self._load_rated_runs()
        if runs and self._save_rated_runs(runs[:-1]):
            last = runs[-1]
            self.optimiser_note = (f"Removed the last rating ({last['mode']} / {last['controller']},"
                                   f" {'clean' if last['clean'] else 'failed'})")

    def _optimiser(self):
        """A LapOptimiser for the rover's current mode and controller (needs self.tuning),
        and the rated runs it was built from."""
        controller = self._controller()
        runs = [opt.Run(run["settings"], run["time_s"], run["clean"])
                for run in self._load_rated_runs()
                if run.get("mode") == p.MODE_NAMES[self.tuning.mode]
                and run.get("controller") == p.CONTROLLER_NAMES[controller]
                and run.get("track", TRACKS[0]) == self.track_name.get()]   # Old = Test
        return opt.LapOptimiser(controller, runs), runs

    def _send_settings(self, settings):
        pwm_commands = {field: command for field, command, _label in p.PWM_LEVELS}
        for key, value in settings.items():
            if key == "speed":
                self.link.send(p.CMD_SET_SPEED, round(value))
            elif key in opt.PWM_PAIRS:
                for field in opt.PWM_PAIRS[key]:
                    self.set_pwm_level(field, pwm_commands[field], value)
            else:
                self._send_tuning(self._param(key), value)

    def _optimiser_blocked(self):
        if not self.link_alive() or self.tuning is None:
            self.optimiser_note = "Connect to the rover first"
        elif self.rover_running():
            self.optimiser_note = "Stop the rover first"
        elif self.unrated_run is not None:
            self.optimiser_note = "Rate the last run first: Y = clean lap, N = failed"
        else:
            return False
        return True

    def suggest_settings(self):
        if self._optimiser_blocked():
            return
        optimiser, _ = self._optimiser()
        if not optimiser.ready:
            self.optimiser_note = ("Do one clean lap with your hand-tuned settings and press Y,"
                                   " so the optimiser has a starting point")
            return
        self.root.config(cursor="watch")
        self.root.update_idletasks()
        try:
            settings = optimiser.suggest()
        finally:
            self.root.config(cursor="")
        if settings is None:
            self.optimiser_note = "No new settings left in the search ranges"
            return
        self._send_settings(settings)
        self.optimiser_note = (f"Trying: {opt.describe(settings, optimiser.dimensions)}"
                               f"  (predicted {optimiser.predicted_time(settings):.2f} s)."
                               f" Put the rover on the line and press Space.")

    def apply_best_settings(self):
        if self._optimiser_blocked():
            return
        optimiser, _ = self._optimiser()
        best = optimiser.best()
        if best is None:
            self.optimiser_note = "No clean laps rated yet for this mode and controller"
            return
        settings, predicted, laps = best
        self._send_settings(settings)
        self.optimiser_note = (f"Using best: {opt.describe(settings, optimiser.dimensions)}"
                               f"  (predicted {predicted:.2f} s, {laps} clean lap(s) with exactly"
                               f" these). Press Copy for pc_brain.py once you trust it.")

    def _update_optimiser_panel(self):
        ready = self.link_alive() and self.tuning is not None
        if ready:
            name = f"{p.MODE_NAMES[self.tuning.mode]} / {p.CONTROLLER_NAMES[self._controller()]}"
        else:
            name = "--"
        self._set_widget(self.optimiser_title,
                         text=f"LAP OPTIMISER - {name}  (G: try next, B: use best, stopped only)")
        if self.unrated_run is not None:
            run = self.unrated_run
            text = f"Last run {run['time_s']:.3f} s ({run['result']}) - was it a clean lap?"
            colour = COLOUR_WARN
        else:
            text, colour = "", COLOUR_TEXT
        self._set_widget(self.rating_label, text=text, foreground=colour)
        for button in self.rating_buttons:
            self._set_enabled(button, self.unrated_run is not None)
        idle = ready and not self.rover_running() and self.unrated_run is None
        self._set_enabled(self.suggest_button, idle)
        self._set_enabled(self.best_button, idle)
        self._set_widget(self.optimiser_status, text=self.optimiser_note)
        if not ready:
            self._set_widget(self.optimiser_detail, text="")
            return
        mode, controller = p.MODE_NAMES[self.tuning.mode], p.CONTROLLER_NAMES[self._controller()]
        key = (mode, controller, os.path.getmtime(self._optimiser_path())
               if os.path.exists(self._optimiser_path()) else 0)
        if self.widget_cache.get("optimiser_summary_key") == key:
            return
        self.widget_cache["optimiser_summary_key"] = key
        self._set_widget(self.optimiser_detail, text=self._optimiser_summary())

    def _optimiser_summary(self):
        optimiser, runs = self._optimiser()
        clean = [run for run in runs if run.clean]
        if not runs:
            return ("No rated runs yet for this mode and controller. After each run press"
                    " Y (clean lap) or N (failed), then G for the next settings to try.")
        text = f"{len(runs)} rated runs, {len(clean)} clean, {len(runs) - len(clean)} failed."
        if clean:
            fastest = min(clean, key=lambda run: run.time_s)
            text += f"  Fastest clean lap {fastest.time_s:.3f} s."
            settings, predicted, laps = optimiser.best()
            text += (f"  Best settings (B): {opt.describe(settings, optimiser.dimensions)},"
                     f" predicted {predicted:.2f} s, {laps} clean lap(s).")
        return text

    def choose_log_file(self):
        path = filedialog.asksaveasfilename(defaultextension=".csv",
                                            initialfile=DEFAULT_LOG_FILE,
                                            filetypes=[("CSV", "*.csv")])
        if path:
            self.log_path = path
            self.log_label.config(text=f"Logging to {path}")

    # ================================================================
    # Periodic UI refresh (runs on the Tk thread)
    # ================================================================
    def _refresh(self):
        while not self.link.messages.empty():
            message = self.link.messages.get_nowait()
            if isinstance(message, p.Tuning):
                self._on_tuning(message)
            else:
                self._on_telemetry(message)
        self._update_link_label()
        self._update_run_labels()
        self._update_lamps()
        self._update_levels()
        self._update_guide()
        self._update_tuning_panel()
        self._update_optimiser_panel()
        self._update_orb()
        self._update_pc_loop()
        self._draw_chart()
        self.root.after(UI_REFRESH_MS, self._refresh)

    def _on_telemetry(self, telemetry):
        previous = self.telemetry
        self.telemetry = telemetry
        self.telemetry_time = time.monotonic()
        self.telemetry_count += 1
        self.chart["left"].append(telemetry.ir_left)
        self.chart["right"].append(telemetry.ir_right)
        self.chart["position"].append(telemetry.position)
        self._log_sample(telemetry)

        rover_stopped_itself = previous is not None and previous.running and not telemetry.running
        if rover_stopped_itself and self.run_start is not None:
            if telemetry.line_lost:
                self._finish_run("line lost")
            else:
                self._finish_run("stopped")

    def _update_link_label(self):
        now = time.monotonic()
        if now - self.rate_window_start >= 1:
            self.telemetry_rate = self.telemetry_count
            self.telemetry_count = 0
            self.rate_window_start = now
        if not self.link.is_open:
            text, colour = "● Disconnected", COLOUR_BAD
        elif not self.link_alive():
            text, colour = "● Port open, no telemetry", COLOUR_WARN
        else:
            text, colour = (f"● Link OK  {self.telemetry_rate} Hz  "
                            f"checksum errors {self.link.parser.checksum_errors}"), COLOUR_OK
        self.link_label.config(text=text, foreground=colour)

    def _update_run_labels(self):
        if self.run_start is not None:
            self.timer_label.config(text=f"{time.monotonic() - self.run_start:.3f} s")
        t = self.telemetry
        if not self.link_alive():
            self.state_label.config(text="NO LINK", foreground=COLOUR_WARN)
            return
        if t.calibrating:
            state, colour = "CALIBRATING", COLOUR_WARN
        elif t.running:
            state, colour = "RUNNING", COLOUR_OK
        else:
            state, colour = "STOPPED", COLOUR_BAD
        self.state_label.config(text=state, foreground=colour)

        flags = [p.MODE_NAMES[t.mode], p.CONTROLLER_NAMES.get(self._controller(), "")]
        if t.line_lost:
            flags.append("LINE LOST")
        self.detail_label.config(text="   ".join(flags))
        self.speed_label.config(text=f"Base speed: {t.speed} %")
        self._update_wheels(t)
        self._keep_session_mode(t)
        if self.tuning is not None and self.selected_controller.get() != self._controller():
            self.selected_controller.set(self._controller())
        self.readout_label.config(
            text=f"IR L {sensor_level(t.ir_left):.2f} ({int(t.left_white)})  "
                 f"IR R {sensor_level(t.ir_right):.2f} ({int(t.right_white)})  "
                 f"pos {t.position:+.2f} (~{t.position * SENSOR_SPACING_CM / 2:+.1f} cm)  "
                 f"DAC {t.dac_left:3d}/{t.dac_right:3d} "
                 f"({p.dac_to_volts(t.dac_left):+.1f}/{p.dac_to_volts(t.dac_right):+.1f} V, "
                 f"PWM {p.dac_to_percent(t.dac_left):.0f}/{p.dac_to_percent(t.dac_right):.0f} %)")

    # ================================================================
    # Live tuning
    # ================================================================
    def _controller(self):
        """The controller in use, or the one just asked for with T until the rover echoes it."""
        if self.tuning is None:
            return p.CONTROLLER_BANG_BANG
        if self.controller_target is not None:
            return self.controller_target
        return self.tuning.controller

    def _active_slots(self):
        return TUNING_SLOTS.get(self._controller(), ())

    def _param(self, key):
        return next(param for param in p.TUNING_PARAMS if param.key == key)

    def _param_active(self, param):
        return self._controller() in param.controllers

    def _tuning_value(self, key):
        """What the panel shows: the requested value until the rover echoes it."""
        return self.tuning_target.get(key, getattr(self.tuning, key))

    def tune_slot(self, slot, direction):
        slots = self._active_slots()
        if slot < len(slots):
            self.adjust_tuning(slots[slot], direction)

    def adjust_tuning(self, key, direction):
        param = self._param(key)
        if not self.link_alive() or self.tuning is None or not self._param_active(param):
            return
        value = round(self._tuning_value(key) + direction * param.step, param.decimals)
        self._send_tuning(param, max(0, min(param.limit, value)))
        self._save_tuning()

    def _send_tuning(self, param, value):
        if self.link.send(param.command, round(value * param.scale)):
            self.tuning_target[param.key] = value
            self.tuning_sent = time.monotonic()

    def _on_tuning(self, tuning):
        mode_changed = self.tuning is None or tuning.mode != self.tuning.mode
        self.tuning = tuning
        if mode_changed:
            self.pwm_applied = False
        self._apply_saved_pwm()
        if mode_changed or time.monotonic() - self.tuning_sent > TUNING_CONFIRM_S:
            self.tuning_target.clear()
        # Echoed, or not taken within the confirm time (the controller refuses some changes while running).
        if mode_changed or tuning.controller == self.controller_target or \
                time.monotonic() - self.controller_sent > TUNING_CONFIRM_S:
            self.controller_target = None

    def _tuning_note(self):
        """The tuning in use, written into the run log's notes column."""
        if self.tuning is None or self.telemetry is None:
            return ""
        parts = [p.CONTROLLER_NAMES.get(self._controller(), "?"), f"speed {self.telemetry.speed}"]
        for key in self._active_slots():
            param = self._param(key)
            parts.append(f"{key} {self._tuning_value(key):.{param.decimals}f}")
        return ", ".join(parts)

    def _tuning_path(self):
        return os.path.join(SETTINGS_DIR, TUNING_FILE)

    def _load_saved_tuning(self):
        try:
            with open(self._tuning_path(), encoding="utf-8") as saved_file:
                return json.load(saved_file)
        except (OSError, ValueError):
            return {}

    def _save_tuning(self):
        """Remember the values for this mode so they survive a rover reset."""
        if self.tuning is None or self.telemetry is None:
            return
        saved = self._load_saved_tuning()
        values = {param.key: self._tuning_value(param.key) for param in p.TUNING_PARAMS}
        values["speed"] = self.telemetry.speed
        values["controller"] = self._controller()
        saved[p.MODE_NAMES[self.tuning.mode]] = values
        try:
            with open(self._tuning_path(), "w", encoding="utf-8") as saved_file:
                json.dump(saved, saved_file, indent=2)
        except OSError as error:
            self.tuning_status.config(text=f"Could not save tuning: {error}")

    def apply_saved_tuning(self):
        """Send the saved values for the current mode (after the rover was reset)."""
        if not self.link_alive() or self.tuning is None or self.rover_running():
            self.tuning_status.config(text="Connect and stop the rover first")
            return
        name = p.MODE_NAMES[self.tuning.mode]
        values = self._load_saved_tuning().get(name)
        if not values:
            self.tuning_status.config(text=f"Nothing saved for {name} yet")
            return
        if values.get("controller") in p.CONTROLLER_NAMES:
            self.link.send(p.CMD_SET_CONTROLLER, values["controller"])
        for param in p.TUNING_PARAMS:
            if param.key in values:
                self._send_tuning(param, values[param.key])
        if "speed" in values:
            self.link.send(p.CMD_SET_SPEED, values["speed"])
        self.tuning_status.config(text=f"Applied saved tuning for {name}")

    def copy_tuning(self):
        """Put the current values on the clipboard, ready to paste into MODE_TUNING."""
        if self.tuning is None or self.telemetry is None:
            return
        v = self._tuning_value
        controller = ("p.CONTROLLER_PID", "p.CONTROLLER_BANG_BANG",
                      "p.CONTROLLER_ADAPTIVE")[self._controller()]
        text = (f"# {p.MODE_NAMES[self.tuning.mode]}: {controller}, base {self.telemetry.speed}, "
                f"kp {v('kp'):.0f}, ki {v('ki'):.0f}, kd {v('kd'):.1f}, "
                f"gain {v('adapt_gain'):.1f}, ramp {v('adapt_ramp'):.0f}, "
                f"maxTurn {v('max_turn'):.0f}, bang {v('bang_level'):.2f}, "
                f"strL strR corL corR {' '.join(str(level) for level in self.pwm_levels().values())}")
        self.root.clipboard_clear()
        self.root.clipboard_append(text)
        self.tuning_status.config(text="Copied - paste into MODE_TUNING in pc_brain.py")

    def _set_widget(self, widget, **options):
        """Configure a widget only when an option changed. Rewriting ttk widgets
        every 50 ms made the whole window repaint slower than the refresh period."""
        cache = self.widget_cache.setdefault(str(widget), {})
        changed = {key: value for key, value in options.items() if cache.get(key) != value}
        if changed:
            cache.update(changed)
            widget.config(**changed)

    def _set_enabled(self, button, enabled):
        if self.widget_cache.setdefault(str(button), {}).get("enabled") != enabled:
            self.widget_cache[str(button)]["enabled"] = enabled
            button.state(["!disabled"] if enabled else ["disabled"])

    def _show_tuning_columns(self, ready):
        """Only the current controller's values, side by side with no gaps."""
        shown = tuple(param.key for param in p.TUNING_PARAMS
                      if not ready or self._param_active(param))
        if self.widget_cache.get("tuning_columns") == shown:
            return
        self.widget_cache["tuning_columns"] = shown
        for column in range(len(p.TUNING_PARAMS)):
            self.tuning_grid.columnconfigure(column, weight=1 if column < len(shown) else 0)
        for key, widgets in self.tuning_widgets.items():
            if key in shown:
                widgets["cell"].grid(row=0, column=shown.index(key))
            else:
                widgets["cell"].grid_remove()

    def _update_tuning_panel(self):
        ready = self.link_alive() and self.tuning is not None
        mode_name = p.MODE_NAMES[self.tuning.mode] if self.tuning else "--"
        self._set_widget(self.tuning_title,
                         text=f"TUNING - {mode_name}  (live, changes the rover immediately)")
        slots = self._active_slots()
        self._set_enabled(self.reset_integral_button,
                          ready and self._controller() == p.CONTROLLER_PID)
        self._show_tuning_columns(ready)
        for param in p.TUNING_PARAMS:
            widgets = self.tuning_widgets[param.key]
            active = ready and self._param_active(param)
            self._set_enabled(widgets["minus"], active)
            self._set_enabled(widgets["plus"], active)
            if not ready:
                self._set_widget(widgets["value"], text="--", foreground=COLOUR_MUTED)
                self._set_widget(widgets["hint"], text="")
                continue
            value = self._tuning_value(param.key)
            pending = param.key in self.tuning_target and \
                abs(value - getattr(self.tuning, param.key)) > param.step / 2
            colour = COLOUR_WARN if pending else (COLOUR_TEXT if active else COLOUR_MUTED)
            self._set_widget(widgets["value"], text=f"{value:.{param.decimals}f}",
                             foreground=colour)
            if param.key in slots:
                up_key, down_key = TUNING_KEYS[slots.index(param.key)]
                self._set_widget(widgets["hint"], text=f"keys {up_key.upper()} / {down_key.upper()}")
            else:
                self._set_widget(widgets["hint"], text="")

    def _update_lamps(self):
        t = self.telemetry if self.link_alive() else None
        for side, white in (("Left", t and t.left_white), ("Right", t and t.right_white)):
            canvas, oval, text = self.lamps[side]
            if t is None:
                colour, label = COLOUR_LAMP_OFF, f"{side}: --"
            elif white:
                colour, label = COLOUR_GREEN_LAMP, f"{side}: 1 LINE"      # Lit: sees the white line
            else:
                colour, label = COLOUR_LAMP_DARK, f"{side}: 0 ground"      # Dark: sees the black table
            canvas.itemconfig(oval, fill=colour)
            text.config(text=label)

    def _update_levels(self):
        t = self.telemetry if self.link_alive() else None
        for side in ("Left", "Right"):
            bar, fill, value, seen = self.level_widgets[side]
            if t is None:
                bar.coords(fill, 0, 0, 0, LEVEL_BAR_HEIGHT)
                self._set_widget(value, text="--", foreground=COLOUR_MUTED)
                continue
            level = sensor_level(t.ir_left if side == "Left" else t.ir_right)
            low, high = self.level_range.get(side, (level, level))
            low, high = min(low, level), max(high, level)
            self.level_range[side] = (low, high)
            bar.coords(fill, 0, 0, level * LEVEL_BAR_WIDTH, LEVEL_BAR_HEIGHT)
            clear = level >= LEVEL_WHITE_MARK or level <= LEVEL_BLACK_MARK
            self._set_widget(value, text=f"{level:.2f}",
                             foreground=COLOUR_OK if clear else COLOUR_WARN)
            self._set_widget(seen, text=f"min {low:.2f}  max {high:.2f}  gap {high - low:.2f}")

    def _guidance(self):
        if not self.link.is_open:
            return "Pick the XBee port, then press Connect"
        if not self.link_alive():
            return "No data from the rover - switch it on and check both XBee settings"
        t = self.telemetry
        if t.calibrating:
            return "Slide the rover slowly across the line"
        if t.running:
            return "Running - Space to stop, Esc for emergency stop, Up/Down for speed"
        if self.unrated_run is not None:
            return ("Rate the run: Y = clean lap, N = failed. Then click the note box,"
                    " type, Enter")
        if not self.calibrated:
            return "Press C and sweep the rover across the line (optional for 1/0 sensors)"
        return f"Mode: {p.MODE_NAMES[t.mode]} - put the rover on the line and press Space"

    def _update_guide(self):
        t = self.telemetry
        calibrating = self.link_alive() and t.calibrating
        if self.was_calibrating and not calibrating:
            self.calibrated = True
        self.was_calibrating = calibrating
        self.guide_label.config(text=self._guidance())

    def _draw_chart(self):
        canvas = self.chart_canvas
        canvas.delete("all")
        width = max(canvas.winfo_width(), 2)
        height = max(canvas.winfo_height(), 2)
        for fraction in (0.25, 0.75):
            canvas.create_line(0, height * fraction, width, height * fraction,
                               fill=COLOUR_GRID, dash=(2, 6))
        canvas.create_line(0, height / 2, width, height / 2, fill=COLOUR_CHART_CENTRE)
        x_step = width / (CHART_SAMPLES - 1)

        def plot(samples, to_y, colour):
            if len(samples) < 2:
                return
            points = []
            for i, value in enumerate(samples):
                points += [i * x_step, to_y(value)]
            canvas.create_line(*points, fill=colour, width=2)

        adc_to_y = lambda value: height - value / ADC_FULL_SCALE * height
        position_to_y = lambda value: height / 2 - value * height / 2
        plot(self.chart["left"], adc_to_y, COLOUR_IR_LEFT)
        plot(self.chart["right"], adc_to_y, COLOUR_IR_RIGHT)
        plot(self.chart["position"], position_to_y, COLOUR_POSITION)

    def on_close(self):
        self.link.send(p.CMD_STOP)
        self.link.close()
        self._close_sample_log()
        self.root.destroy()


class PwmWindow:
    """Separate test window: set the bang-bang wheel levels of the current mode as
    electrical PWM duty (100 % = full forward, 50 % = stopped, 0 % = full reverse).
    Changes reach the rover at once, also while it runs, and are saved per mode."""

    def __init__(self, gui):
        self.gui = gui
        self.user_time = 0.0
        self.syncing = False
        self.window = tk.Toplevel(gui.root)
        self.window.title("PWM test")
        self.window.configure(bg=SOLID_BG)
        glass.dark_title_bar(self.window)
        self.window.protocol("WM_DELETE_WINDOW", self.close)
        panel = gui._panel(self.window, "PWM TEST", fill=tk.BOTH, expand=True, padx=8, pady=8)
        self.note = ttk.Label(panel, text="", style="Panel.TLabel", wraplength=460,
                              justify=tk.LEFT)
        self.note.pack(anchor=tk.W, pady=(0, 6))
        ttk.Label(panel, style="Muted.TLabel", wraplength=460, justify=tk.LEFT,
                  text="100 % = full forward, 50 % = stopped, 0 % = full reverse. "
                       "'Slowed to turn' is the duty a wheel drops to when the other sensor "
                       "reads black.").pack(anchor=tk.W, pady=(0, 6))
        self.vars = {}
        self.value_labels = {}
        for field, command, label in p.PWM_LEVELS:
            ttk.Label(panel, text=label, style="Panel.TLabel").pack(anchor=tk.W, pady=(6, 0))
            row = ttk.Frame(panel, style="Plain.TFrame")
            row.pack(fill=tk.X)
            var = tk.IntVar(value=50)
            self.vars[field] = var
            tk.Scale(row, from_=0, to=100, resolution=PWM_STEP, orient=tk.HORIZONTAL,
                     variable=var, length=300, showvalue=False, bg=COLOUR_PANEL, fg=COLOUR_TEXT,
                     highlightthickness=0, troughcolor=COLOUR_BORDER, activebackground=COLOUR_ACCENT,
                     takefocus=0,
                     command=lambda _v, f=field, c=command: self._moved(f, c)).pack(side=tk.LEFT)
            value = ttk.Label(row, text="--", style="Panel.TLabel", width=31,
                              font=(FONT_FAMILY, 11, "bold"))
            value.pack(side=tk.LEFT, padx=8)
            self.value_labels[field] = value
        self.rover_label = ttk.Label(panel, text="", style="Muted.TLabel", font=READOUT_FONT)
        self.rover_label.pack(anchor=tk.W, pady=(8, 0))
        self.bang_button = gui._button(panel, "Use Bang-bang controller",
                                       lambda: gui.select_controller(p.CONTROLLER_BANG_BANG))
        self.bang_button.pack(fill=tk.X, pady=(6, 0))
        self._poll()

    @staticmethod
    def _describe(duty):
        speed = (duty - 50) * 2
        volts = duty * 30 / 100 - 15
        return f"{duty:3d} %   {volts:+5.1f} V   speed {speed:+4d} %"

    def _moved(self, field, command):
        if self.syncing:
            return
        self.user_time = time.monotonic()
        value = self.vars[field].get()
        self.value_labels[field].config(text=self._describe(value))
        self.gui.set_pwm_level(field, command, value)

    def _poll(self):
        gui = self.gui
        tuning = gui.tuning
        ready = gui.link_alive() and tuning is not None
        if not ready:
            self.note.config(text="Connect to the rover first.", foreground=COLOUR_WARN)
            self.rover_label.config(text="")
        else:
            bang = tuning.controller == p.CONTROLLER_BANG_BANG
            self.note.config(
                text=f"Mode: {p.MODE_NAMES[tuning.mode]}  \u00b7  controller: "
                     f"{p.CONTROLLER_NAMES.get(tuning.controller, '?')}"
                     + ("" if bang else "  -  these levels only apply to the Bang-bang controller"),
                foreground=COLOUR_TEXT if bang else COLOUR_WARN)
            if time.monotonic() - self.user_time > PWM_USER_HOLD_S:
                self.syncing = True
                for field, _command, _label in p.PWM_LEVELS:
                    value = getattr(tuning, field)
                    self.vars[field].set(value)
                    self.value_labels[field].config(text=self._describe(value))
                self.syncing = False
            self.rover_label.config(
                text=f"Rover has: L {tuning.pwm_straight_left}/{tuning.pwm_correct_left}  "
                     f"R {tuning.pwm_straight_right}/{tuning.pwm_correct_right}  (centred/slowed)")
        gui._set_enabled(self.bang_button, ready)
        self.after_id = self.window.after(200, self._poll)

    def close(self):
        self.window.after_cancel(self.after_id)
        self.gui.pwm_window = None
        self.window.destroy()


class PairingDialog:
    """The unit's XBee Pairing Guide (S2C, XCTU) done from the GUI: three passes
    through the V0 shield, PC -> rover -> PC again, so each XBee ends up with
    the other's serial number as its destination (xbee_setup.py)."""

    STEPS = ("Hardware (guide steps 1-2): XBee on the V0 shield, shield switch on USB, "
             "empty sketch uploaded to the Arduino. Close XCTU.\n"
             "1.  PC XBee on the shield: pick the Arduino's port, press Program PC XBee.\n"
             "2.  Swap in the rover XBee: press Program rover XBee.\n"
             "3.  Swap the PC XBee back in: press Program PC XBee again.\n"
             "Wiring for a breadboard pass-through: XBee DIN to Arduino D0 (RX), "
             "DOUT to D1 (TX), 3.3 V supply and a 5 V-to-3.3 V divider on DIN.")

    def __init__(self, gui):
        self.gui = gui
        self.messages = queue.Queue()
        self.busy = False
        self.results = {}               # role: read-back settings from this session
        saved = self._load()
        self.serials = {role: tuple(saved[f"{role}_serial"]) for role in xb.ROLE_NAMES
                        if isinstance(saved.get(f"{role}_serial"), list)}

        self.window = tk.Toplevel(gui.root)
        self.window.title("Pair XBees")
        self.window.configure(bg=SOLID_BG)
        self.window.transient(gui.root)
        glass.dark_title_bar(self.window)
        self.window.protocol("WM_DELETE_WINDOW", self.close)
        panel = gui._panel(self.window, "PAIR XBEES  (S2C, as in the XBee Pairing Guide)",
                           fill=tk.BOTH, expand=True, padx=8, pady=8)
        ttk.Label(panel, text=self.STEPS, style="Panel.TLabel", justify=tk.LEFT,
                  wraplength=640).pack(anchor=tk.W, pady=(6, 8))

        row = ttk.Frame(panel, style="Plain.TFrame")
        row.pack(fill=tk.X)
        ttk.Label(row, text="Port", style="Muted.TLabel").pack(side=tk.LEFT)
        self.port_box = ttk.Combobox(row, state="readonly", width=34, takefocus=0)
        self.port_box.pack(side=tk.LEFT, padx=6)
        gui._button(row, "Refresh", self.refresh_ports).pack(side=tk.LEFT)

        row = ttk.Frame(panel, style="Plain.TFrame")
        row.pack(fill=tk.X, pady=8)
        self.pan = self._entry(row, "PAN ID (hex)", saved.get("pan") or f"{xb.random_pan():X}")
        gui._button(row, "New random PAN ID", self.new_pan).pack(side=tk.LEFT)
        self.coordinator = tk.BooleanVar(value=False)
        ttk.Checkbutton(row, text="PC XBee as coordinator (CE=1)", variable=self.coordinator,
                        style="Panel.TCheckbutton", takefocus=0).pack(side=tk.LEFT, padx=(14, 0))

        row = ttk.Frame(panel, style="Plain.TFrame")
        row.pack(fill=tk.X)
        self.buttons = [
            gui._button(row, "Program PC XBee", lambda: self.program(xb.ROLE_PC), "primary"),
            gui._button(row, "Program rover XBee",
                        lambda: self.program(xb.ROLE_ROVER), "primary"),
            gui._button(row, "Read settings", self.read_settings),
        ]
        for button in self.buttons:
            button.pack(side=tk.LEFT, padx=(0, 6))
        self.status = ttk.Label(panel, text="", style="Panel.TLabel", font=GUIDE_FONT,
                                wraplength=640, justify=tk.LEFT)
        self.status.pack(anchor=tk.W, pady=(8, 4))
        self.log_box = tk.Text(panel, height=14, width=80, bg=COLOUR_PANEL_RAISED,
                               fg=COLOUR_TEXT, font=(READOUT_FONT[0], 10), relief=tk.FLAT,
                               highlightthickness=1, highlightbackground=COLOUR_BORDER,
                               state=tk.DISABLED, takefocus=0)
        self.log_box.pack(fill=tk.BOTH, expand=True)

        self.refresh_ports()
        self._update_status()
        self.window.after(PAIR_POLL_MS, self._poll)

    def _entry(self, parent, label, value):
        ttk.Label(parent, text=label, style="Muted.TLabel").pack(side=tk.LEFT)
        entry = tk.Entry(parent, width=7, bg=COLOUR_PANEL_RAISED, fg=COLOUR_TEXT,
                         insertbackground=COLOUR_TEXT, relief=tk.FLAT, font=READOUT_FONT,
                         highlightthickness=1, highlightbackground=COLOUR_BORDER,
                         highlightcolor=COLOUR_ACCENT)
        entry.insert(0, value)
        entry.pack(side=tk.LEFT, padx=6)
        # Leave out the "all" tag, so typing digits/letters here doesn't fire the
        # main window's run hotkeys (1-4 = mode, Space = start, ...).
        entry.bindtags((str(entry), "Entry", str(self.window)))
        return entry

    # ---- settings file: PAN ID and both serial numbers (guide step 13) ----
    def _path(self):
        return os.path.join(SETTINGS_DIR, XBEE_FILE)

    def _load(self):
        try:
            with open(self._path(), encoding="utf-8") as saved_file:
                saved = json.load(saved_file)
            return saved if isinstance(saved, dict) else {}
        except (OSError, ValueError):
            return {}

    def _save(self):
        saved = {"pan": self.pan.get().strip().upper()}
        saved.update({f"{role}_serial": list(serial_pair)
                      for role, serial_pair in self.serials.items()})
        try:
            with open(self._path(), "w", encoding="utf-8") as saved_file:
                json.dump(saved, saved_file)
        except OSError:
            pass

    # ---- actions ----
    def refresh_ports(self):
        ports = [f"{port.device} - {port.description}" for port in list_ports.comports()]
        self.port_box["values"] = ports
        if ports and self.port_box.get() not in ports:
            main_choice = self.gui.port_box.get()
            self.port_box.set(main_choice if main_choice in ports else ports[0])

    def new_pan(self):
        self.pan.delete(0, tk.END)
        self.pan.insert(0, f"{xb.random_pan():X}")
        self.results.clear()        # Any XBee programmed before now has the old PAN
        self._append("New PAN ID: program both XBees again (all three passes).")
        self._update_status()

    def _device(self):
        selection = self.port_box.get()
        if not selection:
            messagebox.showwarning("No port", "Plug in the Arduino and press Refresh.",
                                   parent=self.window)
            return None
        return selection.split(" - ")[0]

    def program(self, role):
        try:
            pan = xb.parse_pan(self.pan.get())
        except xb.XBeeError as error:
            messagebox.showerror("Check the PAN ID", str(error), parent=self.window)
            return
        device = self._device()
        if device is None:
            return
        other = xb.ROLE_ROVER if role == xb.ROLE_PC else xb.ROLE_PC
        peer = self.serials.get(other)
        coordinator = role == xb.ROLE_PC and self.coordinator.get()
        self._save()
        self._append(f"--- Programming {device} as the {xb.ROLE_NAMES[role]} XBee ---")
        self._start(role, lambda: xb.program(device, role, pan, peer, coordinator,
                                             log=self._log))

    def read_settings(self):
        device = self._device()
        if device is None:
            return
        self._append(f"--- Reading {device} ---")
        self._start("read", lambda: xb.read_info(device, log=self._log))

    def _start(self, job, work):
        def run():
            try:
                self.messages.put(("done", job, work()))
            except xb.XBeeError as error:
                self.messages.put(("error", job, str(error)))
        self.busy = True
        for button in self.buttons:
            button.state(["disabled"])
        threading.Thread(target=run, daemon=True).start()

    # ---- UI updates (main thread) ----
    def _log(self, text):
        """Safe from the worker thread: lines go through the queue."""
        self.messages.put(("log", None, text))

    def _poll(self):
        if not self.window.winfo_exists():
            return
        while True:
            try:
                kind, job, payload = self.messages.get_nowait()
            except queue.Empty:
                break
            if kind == "log":
                self._append(payload)
                continue
            self.busy = False
            for button in self.buttons:
                button.state(["!disabled"])
            if kind == "error":
                self._append("FAILED: " + payload)
            elif job == "read":
                self._append("  " + "   ".join(f"{k}={v}" for k, v in payload.items()))
            else:
                self.results[job] = payload
                self.serials[job] = xb.serial_number(payload)
                self._save()
                if xb.is_zigbee(payload) and not self.coordinator.get():
                    self.coordinator.set(True)  # Two CE = 0 Zigbee routers never link
                    self._append("Zigbee firmware: 'PC XBee as coordinator' is now ticked "
                                 "for the next PC XBee pass.")
            self._update_status()
        self.window.after(PAIR_POLL_MS, self._poll)

    def _append(self, text):
        self.log_box.config(state=tk.NORMAL)
        self.log_box.insert(tk.END, text + "\n")
        self.log_box.see(tk.END)
        self.log_box.config(state=tk.DISABLED)

    def _next_step(self):
        """(text, colour) saying which XBee to put on the shield next."""
        pc, rover = self.results.get(xb.ROLE_PC), self.results.get(xb.ROLE_ROVER)
        if pc is None and rover is None:
            return ("Next: PC XBee on the shield, press Program PC XBee "
                    "(pass 1 of 3)."), COLOUR_WARN
        if rover is None:
            return ("Next: swap in the rover XBee, press Program rover XBee "
                    "(pass 2 of 3)."), COLOUR_WARN
        if pc is None:
            return "Next: swap in the PC XBee, press Program PC XBee.", COLOUR_WARN
        problems = xb.check_pair(pc, rover)
        if not problems:
            return ("Paired. Put each XBee on its own board (rover XBee back on the rover), "
                    "switch the rover on, close this window and Connect."), COLOUR_OK
        for role, prefix in ((xb.ROLE_PC, "PC"), (xb.ROLE_ROVER, "Rover")):
            if all(problem.startswith(prefix) for problem in problems):
                return (f"Next: swap the {xb.ROLE_NAMES[role]} XBee back in, press Program "
                        f"{xb.ROLE_NAMES[role]} XBee again (pass 3 of 3)."), COLOUR_WARN
        return "Not paired: " + " ".join(problems), COLOUR_BAD

    def _update_status(self):
        text, colour = self._next_step()
        self.status.config(text=text, foreground=colour)

    def close(self):
        if self.busy:
            messagebox.showinfo("Still working", "Wait until the XBee finishes saving.",
                                parent=self.window)
            return
        self.gui.refresh_ports()
        self.window.destroy()


def main():
    root = tk.Tk()
    RoverGUI(root)
    root.mainloop()


if __name__ == "__main__":
    main()
