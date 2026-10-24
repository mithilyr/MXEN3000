#!/usr/bin/env python3
"""MXEN3000 S2 2026 - terminal-style rover control panel, PC-control build.

A minimal look over the same controller, keys, logs and settings files as
rover_gui.py (the run, tuning, optimiser, logging, Pair XBees and PWM test
logic is reused from that class). Black background, monospace text and
[ label: value ] readouts refreshed 5 times a second. Nothing moves except
the numbers: no track map, charts, bars, lamps, animations or link statistics.

Keys: 1-4 mode, Space start/stop, Esc emergency stop, Up/Down speed,
C calibrate, T controller, Q/A W/S E/D R/F tuning, Y/N rate, G/B optimiser.

Build a single .exe with build_exe.bat (RoverControlPC.exe).
"""

import time
import tkinter as tk
from tkinter import ttk

import link_test
import protocol as p
import rover_gui as gui

UI_REFRESH_MS = 200             # 5 Hz; telemetry still arrives (and is logged) at 10 Hz

# ---- Terminal look: black, monospace, warm accents ----
BG = "#000000"
FIELD = "#141414"               # Buttons, inputs
FIELD_ACTIVE = "#262626"
BORDER = "#2b2b2b"
TEXT = "#f2f2f2"
MUTED = "#7c7c7c"
DISABLED = "#3d3d3d"
RED = "#e0442c"
ORANGE = "#f07b3c"
AMBER = "#f2b14c"
YELLOW = "#f6e46b"
GREEN = "#9be37a"
MONO = "Consolas"
FONT = MONO
NUMBER_FONT = (MONO, 12)
LIVE_FONT = (MONO, 15)          # The sensor / wheel readouts, the most-read lines
HISTORY_ROWS = 6
HISTORY_WIDTHS = {"division": 90, "attempt": 65, "time_s": 60, "result": 120, "track": 60,
                  "note": 120}
WRAP_MARGIN = 80                # px
# Fullscreen fit: the window opens maximised and every font is scaled so the
# whole panel fills the screen. BASE_* is the panel's own size at scale 1
# (measured with the rover linked); the scale is capped so text never gets silly.
BASE_WIDTH = 1180
BASE_HEIGHT = 775
SCREEN_RESERVE = (16, 70)       # px kept free for the window frame and title bar
MIN_SCALE = 0.8
MAX_SCALE = 1.6

# The shared windows (PWM test, Pair XBees) read rover_gui's colours and font
# when they open, so pointing them at this palette restyles those windows too.
for name, value in (("SOLID_BG", BG), ("COLOUR_PANEL", BG), ("COLOUR_PANEL_RAISED", FIELD),
                    ("COLOUR_BORDER", BORDER), ("COLOUR_TEXT", TEXT), ("COLOUR_MUTED", MUTED),
                    ("COLOUR_ACCENT", ORANGE), ("COLOUR_OK", GREEN), ("COLOUR_WARN", AMBER),
                    ("COLOUR_BAD", RED), ("FONT_FAMILY", MONO)):
    setattr(gui, name, value)


def bracket(label, value):
    """The readout format: [ label: value ]"""
    return f"[ {label}: {value} ]"


class PlainRoverGUI(gui.RoverGUI):
    # ================================================================
    # Look
    # ================================================================
    def _fit_scale(self):
        """How much bigger than BASE_* the maximised window is (fonts follow it)."""
        width = self.root.winfo_screenwidth() - SCREEN_RESERVE[0]
        height = self.root.winfo_screenheight() - SCREEN_RESERVE[1]
        return max(MIN_SCALE, min(MAX_SCALE, width / BASE_WIDTH, height / BASE_HEIGHT))

    def _build_styles(self):
        self.scale = self._fit_scale()
        # Point-size fonts all grow with Tk's scaling; pixel sizes use self.scale.
        self.root.tk.call("tk", "scaling", float(self.root.tk.call("tk", "scaling")) * self.scale)
        self.root.title("MXEN3000 Rover 2026 - PC control")
        gui.glass.dark_title_bar(self.root)
        self.root.configure(bg=BG)
        style = ttk.Style()
        style.theme_use("clam")
        style.configure(".", background=BG, foreground=TEXT, font=(MONO, 10),
                        bordercolor=BORDER, lightcolor=BG, darkcolor=BG, troughcolor=FIELD,
                        fieldbackground=FIELD, insertcolor=TEXT, selectbackground=FIELD_ACTIVE,
                        selectforeground=TEXT)
        style.configure("Plain.TFrame", background=BG)
        style.configure("Panel.TLabel", background=BG, foreground=TEXT)
        style.configure("Muted.TLabel", background=BG, foreground=MUTED)
        style.configure("Title.TLabel", background=BG, foreground=TEXT)
        style.configure("Big.TLabel", background=BG, font=(MONO, 24, "bold"))
        for kind in ("Radiobutton", "Checkbutton"):
            style.configure(f"Panel.T{kind}", background=BG, foreground=MUTED, font=(MONO, 10),
                            focuscolor=BG, indicatorforeground=BG,
                            upperbordercolor=MUTED, lowerbordercolor=MUTED)
            style.map(f"Panel.T{kind}", background=[("active", BG)],
                      foreground=[("selected", TEXT)],
                      indicatorbackground=[("selected", ORANGE), ("!selected", BG)],
                      upperbordercolor=[("selected", ORANGE)],
                      lowerbordercolor=[("selected", ORANGE)])
        style.configure("TButton", background=FIELD, foreground=TEXT, padding=(8, 2),
                        bordercolor=FIELD, focuscolor=FIELD, relief="flat")
        style.map("TButton", background=[("disabled", BG), ("pressed", FIELD_ACTIVE),
                                         ("active", FIELD_ACTIVE)],
                  foreground=[("disabled", DISABLED)], bordercolor=[("disabled", BORDER)])
        disabled = ("disabled", DISABLED)
        style.configure("Primary.TButton", background=FIELD, foreground=YELLOW,
                        font=(MONO, 11, "bold"), padding=(10, 5), bordercolor=ORANGE,
                        focuscolor=FIELD)
        style.map("Primary.TButton", foreground=[disabled], bordercolor=[("disabled", BORDER)],
                  background=[("disabled", BG), ("pressed", FIELD_ACTIVE),
                              ("active", FIELD_ACTIVE)])
        style.configure("Danger.TButton", background=RED, foreground=BG,
                        font=(MONO, 11, "bold"), padding=(10, 5), bordercolor=RED, focuscolor=RED)
        style.map("Danger.TButton", foreground=[disabled], bordercolor=[("disabled", BORDER)],
                  background=[("disabled", BG), ("pressed", ORANGE), ("active", ORANGE)])
        style.configure("Treeview", background=BG, fieldbackground=BG, foreground=TEXT,
                        rowheight=round(20 * self.scale), font=(MONO, 9))
        style.map("Treeview", background=[("selected", FIELD_ACTIVE)],
                  foreground=[("selected", YELLOW)])
        style.configure("Treeview.Heading", background=BG, foreground=MUTED, relief="flat",
                        font=(MONO, 9))
        style.map("Treeview.Heading", background=[("active", BG)])
        style.configure("TCombobox", fieldbackground=FIELD, background=FIELD, foreground=TEXT,
                        arrowcolor=TEXT, bordercolor=FIELD)
        style.map("TCombobox", fieldbackground=[("readonly", FIELD)],
                  foreground=[("readonly", TEXT)], selectbackground=[("readonly", FIELD)])
        self.root.option_add("*TCombobox*Listbox.background", FIELD)
        self.root.option_add("*TCombobox*Listbox.foreground", TEXT)
        self.root.option_add("*TCombobox*Listbox.selectBackground", FIELD_ACTIVE)
        self.root.option_add("*TCombobox*Listbox.font", (MONO, 10))
        style.configure("TEntry", fieldbackground=FIELD, foreground=TEXT, bordercolor=FIELD)
        style.configure("Vertical.TScrollbar", background=FIELD, troughcolor=BG,
                        arrowcolor=MUTED, bordercolor=BG)

    def _card(self, parent, **pack):
        """A black box with a thin grey border."""
        card = tk.Frame(parent, bg=BG, highlightthickness=1, highlightbackground=BORDER)
        card.pack(**{"padx": 4, "pady": 4, **pack})
        inner = tk.Frame(card, bg=BG)
        inner.pack(fill=tk.BOTH, expand=True, padx=10, pady=6)
        return inner

    def _panel(self, parent, title, **pack):
        frame = self._card(parent, **pack)
        frame.header = ttk.Frame(frame, style="Plain.TFrame")
        frame.header.pack(fill=tk.X)
        frame.title = ttk.Label(frame.header, text=title.upper(), style="Title.TLabel",
                                font=(MONO, 11, "bold"))
        frame.title.pack(side=tk.LEFT)
        return frame

    def _button(self, parent, text, command, kind="normal", min_width=0):
        style = {"primary": "Primary.TButton", "danger": "Danger.TButton"}.get(kind, "TButton")
        return ttk.Button(parent, text=text, command=command, style=style, takefocus=0,
                          width=max(min_width // 12, 0) or None)

    # ================================================================
    # Layout
    # ================================================================
    def _build_layout(self):
        """Two columns that fill a maximised window, bottoms level:
        run controls + live values | tuning + optimiser + run history.
        The last box in each column takes the spare height."""
        self._build_scroll_area()
        self._build_header()
        body = tk.Frame(self.page, bg=BG)
        body.pack(fill=tk.BOTH, expand=True, padx=6, pady=(0, 6))
        left = tk.Frame(body, bg=BG)
        right = tk.Frame(body, bg=BG)
        left.grid(row=0, column=0, sticky="nsew")
        right.grid(row=0, column=1, sticky="nsew")
        body.columnconfigure(0, weight=2)
        body.columnconfigure(1, weight=3)
        body.rowconfigure(0, weight=1)

        self._build_control_panel(left)
        self._build_live_panel(left)
        self._build_tuning_panel(right)
        self._show_tuning_columns(False)
        self._build_optimiser_panel(right)
        # Wrap long text at the right column's width (3/5 of the screen, less margins)
        self.text_wrap = round(self.root.winfo_screenwidth() * 3 / 5) - WRAP_MARGIN
        for label in (self.optimiser_status, self.optimiser_detail, self.optimiser_title,
                      self.tuning_title):
            label.configure(wraplength=self.text_wrap)
        self._build_history(right)
        self.root.minsize(gui.MIN_WINDOW_WIDTH, gui.MIN_WINDOW_HEIGHT)
        self.root.state("zoomed")       # Fullscreen window (title bar kept for closing)

    def _build_history(self, parent):
        panel = self._panel(parent, "RUN HISTORY", fill=tk.BOTH, expand=True)
        self._button(panel.header, "Change log file", self.choose_log_file).pack(side=tk.RIGHT)
        columns = ("division", "attempt", "time_s", "result", "track", "note")
        self.history = ttk.Treeview(panel, columns=columns, show="headings",
                                    height=HISTORY_ROWS, takefocus=0)
        for column in columns:
            self.history.heading(column, text=column)
            self.history.column(column, width=round(HISTORY_WIDTHS.get(column, 60) * self.scale),
                                stretch=True)
        self.history.pack(fill=tk.BOTH, expand=True, pady=(4, 2))
        self.log_label = ttk.Label(panel, text=f"Logging to {self.log_path}", style="Muted.TLabel",
                                   wraplength=self.text_wrap, font=(FONT, 8))
        self.log_label.pack(anchor=tk.W)

    def _build_header(self):
        """Title and link status on the first line; port, buttons and the next
        step on the second, so the header is never the widest thing on screen."""
        top = tk.Frame(self.page, bg=BG)
        top.pack(fill=tk.X, padx=10, pady=(6, 0))
        tk.Label(top, text="ROVER 2026 // PC CONTROL", bg=BG, fg=TEXT,
                 font=(MONO, 18, "bold")).pack(side=tk.LEFT)
        self.link_label = ttk.Label(top, text=bracket("LINK", "OFF"), style="Panel.TLabel",
                                    foreground=RED, font=NUMBER_FONT)
        self.link_label.pack(side=tk.RIGHT)

        row = tk.Frame(self.page, bg=BG)
        row.pack(fill=tk.X, padx=10, pady=(4, 0))
        self.port_box = ttk.Combobox(row, state="readonly", width=28, takefocus=0)
        self.port_box.pack(side=tk.LEFT, padx=(0, 6))
        self.connect_button = self._button(row, "Connect", self.toggle_connection)
        for button in (self._button(row, "Refresh", self.refresh_ports), self.connect_button,
                       self._button(row, "Pair XBees", self.open_pairing),
                       self._button(row, "PWM test", self.open_pwm_window),
                       self._button(row, "Link test", self.open_link_test)):
            button.pack(side=tk.LEFT, padx=2)
        self.guide_label = ttk.Label(row, text="", style="Muted.TLabel", font=(MONO, 10))
        self.guide_label.pack(side=tk.LEFT, padx=(16, 0))
        tk.Frame(self.page, bg=BORDER, height=1).pack(fill=tk.X, padx=10, pady=(6, 0))

    def open_link_test(self):
        window = getattr(self, "link_test_window", None)
        if window is not None:
            window.window.lift()
            return
        self.link_test_window = link_test.LinkTestWindow(self)

    def _build_control_panel(self, parent):
        """Run state and timer, start / stop, mode, controller, calibrate, track, note."""
        panel = self._panel(parent, "RUN", fill=tk.X)
        readout = ttk.Frame(panel, style="Plain.TFrame")
        readout.pack(fill=tk.X, pady=(4, 0))
        self.state_label = ttk.Label(readout, text="STOPPED", style="Big.TLabel", foreground=RED)
        self.state_label.pack(side=tk.LEFT)
        self.timer_label = ttk.Label(readout, text="0.000 s", style="Big.TLabel")
        self.timer_label.pack(side=tk.RIGHT, padx=(24, 0))
        self.detail_label = ttk.Label(panel, text="", style="Muted.TLabel")
        self.detail_label.pack(anchor=tk.W)
        self._button(panel, "START / STOP   [SPACE]", self.toggle_run, "primary").pack(
            fill=tk.X, pady=(6, 3))
        self._button(panel, "EMERGENCY STOP   [ESC]", self.stop_run, "danger").pack(fill=tk.X)

        self._radio_row(panel, "Mode", [(f"{key} {p.MODE_NAMES[mode]}", mode,
                                         lambda m=mode: self.select_mode(m))
                                        for key, mode in gui.MODE_KEYS.items()],
                        self.selected_mode)
        self._radio_row(panel, "Controller  [T]",
                        [(name, controller, lambda c=controller: self.select_controller(c))
                         for controller, name in p.CONTROLLER_NAMES.items()],
                        self.selected_controller)
        self._radio_row(panel, "Track", [(name, name, self.select_track) for name in gui.TRACKS],
                        self.track_name)

        row = ttk.Frame(panel, style="Plain.TFrame")
        row.pack(fill=tk.X, pady=(6, 0))
        self.speed_label = ttk.Label(row, text=bracket("SPEED", "--"), style="Panel.TLabel",
                                     font=NUMBER_FONT)
        self.speed_label.pack(side=tk.LEFT)
        ttk.Label(row, text=" UP/DOWN", style="Muted.TLabel").pack(side=tk.LEFT)
        self._button(row, "CALIBRATE [C]", self.calibrate).pack(side=tk.RIGHT)

        notes = ttk.Frame(panel, style="Plain.TFrame")
        notes.pack(fill=tk.X, pady=(6, 0))
        ttk.Label(notes, text="Note", style="Muted.TLabel").pack(side=tk.LEFT, padx=(0, 6))
        self.note_entry = ttk.Entry(notes, width=gui.NOTE_WIDTH)
        self.note_entry.pack(side=tk.LEFT, fill=tk.X, expand=True)
        self.note_entry.bind("<Return>", self.save_note)
        self.note_hint = ttk.Label(panel, text="Enter saves it on the last run.",
                                   style="Muted.TLabel")
        self.note_hint.pack(anchor=tk.W)

    def _radio_row(self, parent, title, choices, variable):
        row = ttk.Frame(parent, style="Plain.TFrame")
        row.pack(fill=tk.X, pady=(4, 0))
        ttk.Label(row, text=title.upper(), style="Muted.TLabel").pack(anchor=tk.W)
        buttons = ttk.Frame(row, style="Plain.TFrame")
        buttons.pack(anchor=tk.W)
        for text, value, command in choices:
            ttk.Radiobutton(buttons, text=text, value=value, variable=variable, command=command,
                            style="Panel.TRadiobutton", takefocus=0).pack(side=tk.LEFT,
                                                                          padx=(0, 12))

    def _build_live_panel(self, parent):
        """Sensor level (1 = white line, 0 = black table) and wheel output, as text."""
        panel = self._panel(parent, "LIVE", fill=tk.BOTH, expand=True)
        self._button(panel.header, "Reset min/max", self.level_range.clear).pack(side=tk.RIGHT)
        self.sensor_labels = {}
        self.wheel_labels = {}
        for side in ("Left", "Right"):
            self.sensor_labels[side] = ttk.Label(panel, style="Panel.TLabel", font=LIVE_FONT)
            self.sensor_labels[side].pack(anchor=tk.W, pady=(2, 0))
        for side in ("Left", "Right"):
            self.wheel_labels[side] = ttk.Label(panel, style="Panel.TLabel", font=LIVE_FONT)
            self.wheel_labels[side].pack(anchor=tk.W, pady=(2, 0))
        ttk.Label(panel, style="Muted.TLabel",
                  text=(f"pots: line > {gui.LEVEL_WHITE_MARK:.2f}, table < "
                        f"{gui.LEVEL_BLACK_MARK:.2f}  (1 = line, 0 = table)")
                  ).pack(anchor=tk.W, pady=(6, 0))

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

    # ================================================================
    # Refresh (5 Hz, text only)
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
        self._update_live()
        self._update_guide()
        self._update_tuning_panel()
        self._update_optimiser_panel()
        self.root.after(UI_REFRESH_MS, self._refresh)

    def _update_run_labels(self):
        if self.run_start is not None:
            self._set_widget(self.timer_label, text=f"{time.monotonic() - self.run_start:.3f} s")
        if not self.link_alive():
            self._set_widget(self.speed_label, text=bracket("SPEED", "--"))
            self._set_widget(self.state_label, text="NO LINK", foreground=AMBER)
            return
        t = self.telemetry
        if t.calibrating:
            state, colour = "CALIBRATING", AMBER
        elif t.running:
            state, colour = "RUNNING", GREEN
        else:
            state, colour = "STOPPED", RED
        self._set_widget(self.state_label, text=state, foreground=colour)
        self._set_widget(self.detail_label, text="LINE LOST" if t.line_lost else "",
                         foreground=RED)
        self._set_widget(self.speed_label, text=bracket("SPEED", f"{t.speed:3d} %"))
        self._keep_session_mode(t)
        if self.tuning is not None and self.selected_controller.get() != self._controller():
            self.selected_controller.set(self._controller())

    def _update_link_label(self):
        """[ LINK: OK 10 Hz  ERR 3 ]"""
        now = time.monotonic()
        if now - self.rate_window_start >= 1:
            self.telemetry_rate = self.telemetry_count
            self.telemetry_count = 0
            self.rate_window_start = now
        if not self.link.is_open:
            text, colour = bracket("LINK", "OFF"), RED
        elif not self.link_alive():
            text, colour = bracket("LINK", "NO DATA"), AMBER
        else:
            text, colour = bracket("LINK", f"OK {self.telemetry_rate:2d} Hz  "
                                           f"ERR {self.link.parser.checksum_errors}"), GREEN
        self._set_widget(self.link_label, text=text, foreground=colour)

    def _update_live(self):
        t = self.telemetry if self.link_alive() else None
        for side in ("Left", "Right"):
            tag = side[0]
            if t is None:
                self._set_widget(self.sensor_labels[side], text=bracket(f"IR.{tag} ", "--"),
                                 foreground=MUTED)
                self._set_widget(self.wheel_labels[side], text=bracket(f"PWM.{tag}", "--"),
                                 foreground=MUTED)
                continue
            left = side == "Left"
            level = gui.sensor_level(t.ir_left if left else t.ir_right)
            white = t.left_white if left else t.right_white
            low, high = self.level_range.get(side, (level, level))
            low, high = min(low, level), max(high, level)
            self.level_range[side] = (low, high)
            clear = level >= gui.LEVEL_WHITE_MARK or level <= gui.LEVEL_BLACK_MARK
            surface = "LINE " if white else "TABLE"
            self._set_widget(self.sensor_labels[side], foreground=YELLOW if clear else AMBER,
                             text=bracket(f"IR.{tag} ", f"{level:.2f} {surface}"
                                                        f"  lo {low:.2f} hi {high:.2f}"))
            dac = t.dac_left if left else t.dac_right
            self._set_widget(self.wheel_labels[side], foreground=TEXT,
                             text=bracket(f"PWM.{tag}", f"{p.dac_to_percent(dac):5.1f} %"
                                                       f"  DAC {dac:3d}  {p.dac_to_volts(dac):+5.1f} V"))


def main():
    root = tk.Tk()
    PlainRoverGUI(root)
    root.mainloop()


if __name__ == "__main__":
    main()
