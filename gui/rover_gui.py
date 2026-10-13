#!/usr/bin/env python3
"""MXEN3000 S2 2026 - wireless rover control panel.

Talks to firmware/rover2026 over an XBee pair in transparent mode (the PC
XBee shows up as a COM port). Everything the supervisor needs during a
timed run is on the keyboard:

    1 2 3 4   select Clockwise / Anticlockwise / Squiggle / Reverse
    Space     start / stop
    Esc       emergency stop (always stops)
    Up/Down   change base speed
    C         calibrate sensors (sweep the rover across the line)
    T         next steering controller for this mode (PID / bang-bang / adaptive)
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

# ---- Link ----
BAUD_RATE = 9600                # Must match LINK_BAUD in config.h and the XBees
HEARTBEAT_MS = 100              # Rover failsafe is LINK_TIMEOUT_MS = 600
TELEMETRY_STALE_S = 0.5         # No telemetry for this long = link down
READ_CHUNK_BYTES = 256
READ_IDLE_S = 0.005

# ---- UI ----
UI_REFRESH_MS = 50
SPEED_STEP = 5
SPEED_MAX = 100
BENCH_VOLTS_STEP = 1.0         # One - or + press on the bench test panel
CHART_SAMPLES = 200
CHART_WIDTH = 640
CHART_HEIGHT = 200
ADC_FULL_SCALE = 1023
SENSOR_SPACING_CM = 3.0          # IR sensors are 3 cm apart, over a 3 cm white line
LAMP_SIZE = 44
FONT_FAMILY = "Segoe UI"
GUIDE_FONT = (FONT_FAMILY, 12, "bold")
READOUT_FONT = ("Consolas", 12)
DEFAULT_LOG_FILE = "run-log.csv"
RUN_LOG_HEADER = ["date", "division", "attempt", "time_s", "result", "notes"]
TUNING_FILE = "tuning.json"     # Saved next to the run log, one entry per mode
OPTIMISER_FILE = "optimizer.json"  # Every rated run, saved next to the run log
TUNING_CONFIRM_S = 0.6          # Wait this long for the rover to echo a change
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
COLOUR_BLUE = glass.BLUE            # Primary action, selection
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
LEFT_COLUMN_WIDTH = 520         # Fixed, so a long bench readout cannot resize it
SCROLLBAR_ALLOWANCE = 20        # Window width added beside the page for the scrollbar
SCREEN_MARGIN = 40
TASKBAR_ALLOWANCE = 90          # Keeps the window clear of the title bar and taskbar
MIN_WINDOW_WIDTH = 640
MIN_WINDOW_HEIGHT = 300
ORB_SIZE = 44
ORB_PULSE_HZ = 0.7

MODE_KEYS = {"1": p.MODE_CLOCKWISE, "2": p.MODE_ANTICLOCKWISE,
             "3": p.MODE_SQUIGGLE, "4": p.MODE_REVERSE}


class RoverLink:
    """Owns the serial port and a background reader thread."""

    def __init__(self):
        self.port = None
        self.parser = p.TelemetryParser()
        self.messages = queue.Queue()
        self.reader = None
        self.reading = False

    @property
    def is_open(self):
        return self.port is not None and self.port.is_open

    def open(self, device):
        self.port = serial.Serial(device, BAUD_RATE, timeout=0)
        self.parser = p.TelemetryParser()
        self.reading = True
        self.reader = threading.Thread(target=self._read_loop, daemon=True)
        self.reader.start()

    def close(self):
        self.reading = False
        if self.reader:
            self.reader.join(timeout=1)
        if self.is_open:
            self.port.close()
        self.port = None

    def send(self, command, data=0):
        if not self.is_open:
            return False
        try:
            self.port.write(p.encode_command(command, data))
            return True
        except serial.SerialException:
            return False

    def _read_loop(self):
        while self.reading:
            try:
                data = self.port.read(READ_CHUNK_BYTES)
            except serial.SerialException:
                self.reading = False
                break
            if data:
                for message in self.parser.feed(data):
                    self.messages.put(message)
            else:
                time.sleep(READ_IDLE_S)


class RoverGUI:
    def __init__(self, root):
        self.root = root
        self.link = RoverLink()

        self.telemetry = None
        self.telemetry_time = 0.0
        self.telemetry_count = 0
        self.rate_window_start = time.monotonic()
        self.telemetry_rate = 0

        self.selected_mode = tk.IntVar(value=p.MODE_CLOCKWISE)
        self.selected_controller = tk.IntVar(value=p.CONTROLLER_PID)
        self.run_start = None
        self.run_mode = None
        self.was_calibrating = False
        self.tuning = None              # Last Tuning frame from the rover
        self.tuning_target = {}         # Requested values the rover has not echoed yet
        self.tuning_sent = 0.0
        self.widget_cache = {}          # Last options set by _set_widget
        self.calibrated = False
        self.attempts = {mode: 0 for mode in p.MODE_NAMES}
        self.log_path = os.path.abspath(DEFAULT_LOG_FILE)
        self.sample_file = None
        self.sample_writer = None
        self.sample_path = None
        self.unrated_run = None         # Last finished run, waiting for Y / N
        self.optimiser_note = ""

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
        self.root.title("MXEN3000 Rover 2026")
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
                      indicatorbackground=[("selected", COLOUR_BLUE),
                                           ("!selected", COLOUR_PANEL_RAISED)],
                      upperbordercolor=[("selected", COLOUR_BLUE)],
                      lowerbordercolor=[("selected", COLOUR_BLUE)])
        style.configure("Treeview", background=COLOUR_PANEL_RAISED, fieldbackground=COLOUR_PANEL_RAISED,
                        foreground=COLOUR_TEXT, bordercolor=COLOUR_BORDER, lightcolor=COLOUR_BORDER,
                        darkcolor=COLOUR_BORDER, rowheight=24)
        style.map("Treeview", background=[("selected", COLOUR_BLUE)],
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
        self.root.option_add("*TCombobox*Listbox.selectBackground", COLOUR_BLUE)
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
        tk.Label(titles, text="Rover 2026", bg=COLOUR_PANEL, fg=COLOUR_TEXT,
                 font=(FONT_FAMILY, 22, "bold")).pack(anchor=tk.W)
        tk.Label(titles, text="MXEN3000  \u00b7  Line-follower control", bg=COLOUR_PANEL,
                 fg=COLOUR_MUTED, font=(FONT_FAMILY, 10)).pack(anchor=tk.W)

        controls = tk.Frame(header, bg=COLOUR_PANEL)
        controls.pack(side=tk.RIGHT)
        row = tk.Frame(controls, bg=COLOUR_PANEL)
        row.pack(anchor=tk.E)
        self.port_box = ttk.Combobox(row, state="readonly", width=34, takefocus=0)
        self.port_box.pack(side=tk.LEFT, padx=(0, 6))
        self._button(row, "Refresh", self.refresh_ports).pack(side=tk.LEFT)
        self.connect_button = self._button(row, "Connect", self.toggle_connection)
        self.connect_button.pack(side=tk.LEFT, padx=(6, 0))
        self.link_label = ttk.Label(controls, text="\u25cf Disconnected", style="Panel.TLabel",
                                    foreground=COLOUR_BAD)
        self.link_label.pack(anchor=tk.E, pady=(6, 0))

        guide = tk.Frame(header, bg=COLOUR_PANEL)
        guide.pack(side=tk.LEFT, fill=tk.X, expand=True, padx=24)
        ttk.Label(guide, text="NEXT STEP", style="Title.TLabel",
                  font=(FONT_FAMILY, 9, "bold")).pack(anchor=tk.W)
        self.guide_label = ttk.Label(guide, text="", style="Panel.TLabel", font=GUIDE_FONT,
                                     wraplength=450, width=50, justify=tk.LEFT)
        self.guide_label.pack(anchor=tk.W)

    def _update_orb(self):
        """Grey with no link, blue when linked and idle, pulsing green while running."""
        if not self.link_alive():
            colour = COLOUR_ORB_DIM
        elif self.rover_running():
            level = (math.sin(time.monotonic() * math.tau * ORB_PULSE_HZ) + 1) / 2
            colour = self._blend(COLOUR_GREEN_LAMP, COLOUR_ORB_RUN, level)
        else:
            colour = COLOUR_BLUE
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
        self._build_bench_panel(left)
        self._build_chart_panel(right)
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
        ttk.Label(controllers, text="Controller [T]:", style="Muted.TLabel").pack(side=tk.LEFT,
                                                                             padx=(0, 8))
        for controller, name in p.CONTROLLER_NAMES.items():
            ttk.Radiobutton(controllers, text=name, value=controller,
                            variable=self.selected_controller, style="Panel.TRadiobutton",
                            command=lambda c=controller: self.select_controller(c),
                            takefocus=0).pack(side=tk.LEFT, padx=(0, 10))
        self._button(panel, "Calibrate sensors  [C]", self.calibrate).pack(fill=tk.X, pady=(4, 0))

    def _build_bench_panel(self, parent):
        panel = self._panel(parent, "BENCH TEST  (motor volts, stopped only)", fill=tk.X)
        self.bench_vars = {}
        self.bench_code_labels = {}
        for name, command in (("Left", p.CMD_SET_DAC_LEFT), ("Right", p.CMD_SET_DAC_RIGHT)):
            row = ttk.Frame(panel, style="Plain.TFrame")
            row.pack(fill=tk.X, pady=2)
            var = tk.DoubleVar(value=0.0)
            self.bench_vars[command] = var
            ttk.Label(row, text=name, width=6, style="Panel.TLabel").pack(side=tk.LEFT)
            tk.Scale(row, from_=-p.MOTOR_VOLTS_MAX, to=p.MOTOR_VOLTS_MAX,
                     resolution=BENCH_VOLTS_STEP, orient=tk.HORIZONTAL, variable=var,
                     length=160, bg=COLOUR_PANEL, fg=COLOUR_TEXT, highlightthickness=0,
                     troughcolor=COLOUR_BORDER, activebackground=COLOUR_BLUE,
                     takefocus=0).pack(side=tk.LEFT)
            for text, step in (("-", -BENCH_VOLTS_STEP), ("+", BENCH_VOLTS_STEP)):
                self._button(row, text, lambda c=command, s=step: self.nudge_bench(c, s),
                             min_width=34).pack(side=tk.LEFT, padx=1)
            self._button(row, "Send", lambda c=command: self.send_bench(c)).pack(side=tk.LEFT, padx=4)
            code_label = ttk.Label(panel, text="", style="Muted.TLabel")
            code_label.pack(anchor=tk.W)
            self.bench_code_labels[command] = code_label
            var.trace_add("write", lambda *_, c=command: self._bench_changed(c))
            self._bench_changed(command)
        self.bench_linked = tk.BooleanVar(value=False)
        ttk.Checkbutton(panel.header, text="Same voltage on both", variable=self.bench_linked,
                        style="Panel.TCheckbutton", takefocus=0,
                        command=lambda: self._bench_changed(p.CMD_SET_DAC_LEFT)).pack(side=tk.RIGHT)
        buttons = ttk.Frame(panel, style="Plain.TFrame")
        buttons.pack(fill=tk.X, pady=(4, 0))
        self._button(buttons, "Send both", self.send_bench_both).pack(side=tk.LEFT, expand=True, fill=tk.X)
        self._button(buttons, "Both to 0 V", self.bench_neutral).pack(side=tk.LEFT, expand=True,
                                                                     fill=tk.X, padx=(4, 0))

    def _build_chart_panel(self, parent):
        panel = self._panel(parent, "SENSORS", fill=tk.BOTH, expand=True)
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
        self._button(row, "Copy for config.h", self.copy_tuning).pack(side=tk.LEFT, padx=6)
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
        columns = ("division", "attempt", "time_s", "result")
        self.history = ttk.Treeview(panel, columns=columns, show="headings", height=2,
                                    takefocus=0)
        for column in columns:
            self.history.heading(column, text=column)
            self.history.column(column, width=100)
        self.history.pack(fill=tk.X, pady=(4, 4))
        self._button(panel.header, "Change log file", self.choose_log_file).pack(side=tk.RIGHT)
        self.log_label = ttk.Label(panel.header, text=f"Logging to {self.log_path}",
                                   style="Muted.TLabel")
        self.log_label.pack(side=tk.RIGHT, padx=10)

    def _bind_keys(self):
        self.root.bind_all("<space>", lambda e: self.toggle_run())
        self.root.bind_all("<Escape>", lambda e: self.stop_run())
        self.root.bind_all("<Up>", lambda e: self.change_speed(SPEED_STEP))
        self.root.bind_all("<Down>", lambda e: self.change_speed(-SPEED_STEP))
        self.root.bind_all("<KeyPress-c>", lambda e: self.calibrate())
        self.root.bind_all("<KeyPress-C>", lambda e: self.calibrate())
        self.root.bind_all("<KeyPress-t>", lambda e: self.cycle_controller())
        self.root.bind_all("<KeyPress-T>", lambda e: self.cycle_controller())
        for slot, (up_key, down_key) in enumerate(TUNING_KEYS):
            for key, direction in ((up_key, +1), (down_key, -1)):
                for variant in (key, key.upper()):
                    self.root.bind_all(f"<KeyPress-{variant}>",
                                       lambda e, s=slot, d=direction: self.tune_slot(s, d))
        for key, mode in MODE_KEYS.items():
            self.root.bind_all(key, lambda e, m=mode: self.select_mode(m))
        for key, action in (("y", lambda: self.rate_run(True)), ("n", lambda: self.rate_run(False)),
                            ("g", self.suggest_settings), ("b", self.apply_best_settings)):
            for variant in (key, key.upper()):
                self.root.bind_all(f"<KeyPress-{variant}>", lambda e, a=action: a())

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
        self.connect_button.config(text="Disconnect")

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
            self.detail_label.config(text="No telemetry from rover - check XBee link")
            return
        mode = self.selected_mode.get()
        if self.telemetry.mode != mode:
            self.link.send(p.CMD_SET_MODE, mode)  # Resets speed to that mode's default
        if self.link.send(p.CMD_START):
            self.run_start = time.monotonic()
            self.run_mode = mode
            self._open_sample_log(mode)

    def stop_run(self):
        self.link.send(p.CMD_STOP)
        self._finish_run("stopped by operator")

    def select_mode(self, mode):
        if self.rover_running():
            self.selected_mode.set(self.telemetry.mode)
            return
        self.selected_mode.set(mode)
        self.link.send(p.CMD_SET_MODE, mode)

    def select_controller(self, controller):
        """Steering controller for the current mode (the rover ignores it while running)."""
        if self.rover_running() or self.tuning is None:
            self.selected_controller.set(self._controller())
            return
        self.selected_controller.set(controller)
        self.link.send(p.CMD_SET_CONTROLLER, controller)

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

    def _bench_volts(self, command):
        try:
            return self.bench_vars[command].get()
        except tk.TclError:   # Mid-edit or empty value
            return 0.0

    def _bench_changed(self, command):
        volts = self._bench_volts(command)
        if getattr(self, "bench_linked", None) is not None and self.bench_linked.get():
            for other, var in self.bench_vars.items():
                if other != command and abs(self._bench_volts(other) - volts) > 1e-6:
                    var.set(volts)   # Re-enters here for the other side
        code = p.volts_to_dac(volts)
        self.bench_code_labels[command].config(
            text=f"       {volts:+5.1f} V -> DAC {code:3d} -> PWM {p.dac_to_percent(code):5.1f} %"
                 f" (actual {p.dac_to_volts(code):+.2f} V)")

    def nudge_bench(self, command, step):
        volts = self._bench_volts(command) + step
        volts = max(-p.MOTOR_VOLTS_MAX, min(p.MOTOR_VOLTS_MAX, volts))
        self.bench_vars[command].set(round(volts, 1))

    def send_bench(self, command):
        if not self.rover_running():
            self.link.send(command, p.volts_to_dac(self._bench_volts(command)))

    def send_bench_both(self):
        for command in self.bench_vars:
            self.send_bench(command)

    def bench_neutral(self):
        for var in self.bench_vars.values():
            var.set(0.0)
        self.send_bench_both()

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
               self.attempts[mode], f"{elapsed:.3f}", result, self._tuning_note()]
        self.history.insert("", 0, values=row[1:5])
        self.timer_label.config(text=f"{elapsed:.3f} s")
        self._hold_for_rating(mode, elapsed, result)
        try:
            new_file = not os.path.exists(self.log_path)
            with open(self.log_path, "a", newline="") as log_file:
                writer = csv.writer(log_file)
                if new_file:
                    writer.writerow(RUN_LOG_HEADER)
                writer.writerow(row)
        except OSError as error:
            self.log_label.config(text=f"Log write failed: {error}")

    # One CSV per run with every telemetry sample (about 20 per second), so
    # IR readings can be plotted and thresholds tuned after the run.
    def _open_sample_log(self, mode):
        self._close_sample_log()
        stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        controller = p.CONTROLLER_NAMES.get(self._controller(), "")
        controller = controller.replace(" ", "").replace("/", "-")
        name = f"ir-log_{p.MODE_NAMES[mode]}_{controller}_{stamp}.csv"
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
        return os.path.join(os.path.dirname(self.log_path), OPTIMISER_FILE)

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
        settings = {"speed": self.telemetry.speed}
        for dimension in opt.SEARCH_SPACE[controller]:
            if dimension.key != "speed":
                settings[dimension.key] = self._tuning_value(dimension.key)
        return settings

    def _hold_for_rating(self, mode, elapsed, result):
        """Keep the run until the operator presses Y (clean lap) or N (failed).
        A radio dropout says nothing about the settings, so it is not offered."""
        self.unrated_run = None
        if result == "link failsafe" or self.tuning is None or self.telemetry is None \
                or self.tuning.mode != mode:
            return
        controller = self._controller()
        self.unrated_run = {"date": datetime.now().strftime("%Y-%m-%d %H:%M"),
                            "mode": p.MODE_NAMES[mode],
                            "controller": p.CONTROLLER_NAMES[controller],
                            "settings": self._current_settings(controller),
                            "time_s": round(elapsed, 3), "result": result}

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
                and run.get("controller") == p.CONTROLLER_NAMES[controller]]
        return opt.LapOptimiser(controller, runs), runs

    def _send_settings(self, settings):
        for key, value in settings.items():
            if key == "speed":
                self.link.send(p.CMD_SET_SPEED, round(value))
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
                               f" these). Press Copy for config.h once you trust it.")

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
        self._update_guide()
        self._update_tuning_panel()
        self._update_optimiser_panel()
        self._update_orb()
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
            if telemetry.link_failsafe:
                self._finish_run("link failsafe")
            elif telemetry.line_lost:
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
        if t.link_failsafe:
            flags.append("LINK FAILSAFE TRIPPED")
        self.detail_label.config(text="   ".join(flags))
        self.speed_label.config(text=f"Base speed: {t.speed} %")
        if not t.running and self.selected_mode.get() != t.mode:
            self.selected_mode.set(t.mode)
        if self.tuning is not None and self.selected_controller.get() != self.tuning.controller:
            self.selected_controller.set(self.tuning.controller)
        self.readout_label.config(
            text=f"IR L {t.ir_left:4d} ({int(t.left_white)})  IR R {t.ir_right:4d} ({int(t.right_white)})  "
                 f"pos {t.position:+.2f} (~{t.position * SENSOR_SPACING_CM / 2:+.1f} cm)  "
                 f"DAC {t.dac_left:3d}/{t.dac_right:3d} "
                 f"({p.dac_to_volts(t.dac_left):+.1f}/{p.dac_to_volts(t.dac_right):+.1f} V, "
                 f"PWM {p.dac_to_percent(t.dac_left):.0f}/{p.dac_to_percent(t.dac_right):.0f} %)")

    # ================================================================
    # Live tuning
    # ================================================================
    def _controller(self):
        return p.CONTROLLER_PID if self.tuning is None else self.tuning.controller

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
        if mode_changed or time.monotonic() - self.tuning_sent > TUNING_CONFIRM_S:
            self.tuning_target.clear()

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
        return os.path.join(os.path.dirname(self.log_path), TUNING_FILE)

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
        controller = ("CONTROLLER_PID", "CONTROLLER_BANG_BANG",
                      "CONTROLLER_ADAPTIVE")[self._controller()]
        text = (f"// {p.MODE_NAMES[self.tuning.mode]}: {controller}, base {self.telemetry.speed}, "
                f"kp {v('kp'):.0f}f, ki {v('ki'):.0f}f, kd {v('kd'):.1f}f, "
                f"gain {v('adapt_gain'):.1f}f, ramp {v('adapt_ramp'):.0f}f, "
                f"maxTurn {v('max_turn'):.0f}, bang {v('bang_level'):.2f}f")
        self.root.clipboard_clear()
        self.root.clipboard_append(text)
        self.tuning_status.config(text="Copied - paste into MODE_TUNING in config.h")

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
        if t.link_failsafe:
            return "Link failsafe tripped - check the radio, then press Space to restart"
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


def main():
    root = tk.Tk()
    RoverGUI(root)
    root.mainloop()


if __name__ == "__main__":
    main()
