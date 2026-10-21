"""Link test window: watches the XBee link while the GUI is connected and says,
in plain English, which part of it is failing.

The figures come from pc_brain.LinkLog (counted since connect or the last
reset): gaps in the rover's sensor frames, rover restarts, downlink stalls,
damaged frames. "Save report" writes them to logs/link-report_<time>.txt.
"""

import os
import tkinter as tk
from datetime import datetime
from tkinter import ttk

import pc_brain as pb
import rover_gui as gui

POLL_MS = 250
EVENT_ROWS = 10
WRAP_PX = 620
STAT_KEYS = ("Connected for", "Sensor frames", "Rate now", "Silent for", "Frames lost",
             "Damaged frames", "Gaps > 0.2 s", "Rover restarts", "Downlink stalls",
             "Motor frames sent", "Round trip")


class LinkTestWindow:
    def __init__(self, app):
        self.app = app
        self.window = tk.Toplevel(app.root)
        self.window.title("Link test")
        self.window.configure(bg=gui.SOLID_BG)
        gui.glass.dark_title_bar(self.window)
        self.window.protocol("WM_DELETE_WINDOW", self.close)
        panel = app._panel(self.window, "LINK TEST  (counts since connect or reset)",
                           fill=tk.BOTH, expand=True, padx=8, pady=8)

        self.state_label = ttk.Label(panel, text="", style="Big.TLabel")
        self.state_label.pack(anchor=tk.W, pady=(4, 4))

        grid = ttk.Frame(panel, style="Plain.TFrame")
        grid.pack(fill=tk.X)
        self.stat_labels = {}
        for row, key in enumerate(STAT_KEYS):
            ttk.Label(grid, text=key, style="Muted.TLabel").grid(row=row, column=0, sticky=tk.W,
                                                                padx=(0, 16))
            value = ttk.Label(grid, text="--", style="Panel.TLabel", font=gui.READOUT_FONT)
            value.grid(row=row, column=1, sticky=tk.W)
            self.stat_labels[key] = value

        ttk.Label(panel, text="DIAGNOSIS", style="Title.TLabel").pack(anchor=tk.W, pady=(10, 0))
        self.findings_frame = ttk.Frame(panel, style="Plain.TFrame")
        self.findings_frame.pack(fill=tk.X)
        self.finding_labels = []

        ttk.Label(panel, text="EVENTS (newest first)", style="Title.TLabel").pack(
            anchor=tk.W, pady=(10, 0))
        self.events = ttk.Treeview(panel, columns=("time", "event"), show="headings",
                                   height=EVENT_ROWS, takefocus=0)
        self.events.heading("time", text="t (s)")
        self.events.heading("event", text="event")
        self.events.column("time", width=70, stretch=False)
        self.events.column("event", width=560, stretch=True)
        self.events.pack(fill=tk.BOTH, expand=True, pady=(4, 6))
        self.shown_events = None

        buttons = ttk.Frame(panel, style="Plain.TFrame")
        buttons.pack(fill=tk.X)
        app._button(buttons, "Reset counters", self.reset).pack(side=tk.LEFT, padx=(0, 6))
        app._button(buttons, "Save report", self.save_report).pack(side=tk.LEFT)
        self.saved_label = ttk.Label(panel, text="", style="Muted.TLabel", wraplength=WRAP_PX)
        self.saved_label.pack(anchor=tk.W, pady=(4, 0))
        self._poll()

    # ---- Refresh ----
    def _poll(self):
        if self.app.link.is_open:
            self._show(*self.app.link.link_report())
        else:
            self.state_label.config(text="NOT CONNECTED", foreground=gui.COLOUR_WARN)
            self._show_findings([("warn", "Pick the PC XBee's COM port and press Connect.")])
        self.after_id = self.window.after(POLL_MS, self._poll)

    def _show(self, log, damaged, findings, now):
        silent = log.silence(now)
        if log.frames and silent <= pb.GAP_EVENT_S:
            self.state_label.config(text="RECEIVING", foreground=gui.COLOUR_OK)
        else:
            self.state_label.config(text=f"NO DATA  {silent:.1f} s", foreground=gui.COLOUR_BAD)
        health = self.app.link.health()
        sent_total = log.frames + log.lost
        expected_hz = 1 / pb.NOMINAL_PERIOD_S
        values = {
            "Connected for": f"{now - log.started:.0f} s",
            "Sensor frames": f"{log.frames}",
            "Rate now": f"{health.rate_hz:.0f} Hz  (expected {expected_hz:.0f})",
            "Silent for": f"{silent:.2f} s",
            "Frames lost": (f"{log.lost}  ({100 * log.lost / sent_total:.1f} %)"
                            if sent_total else "0"),
            "Damaged frames": f"{damaged}",
            "Gaps > 0.2 s": f"{log.counts['gap']}  (longest {log.longest_gap:.1f} s)",
            "Rover restarts": f"{log.counts['restart']}",
            "Downlink stalls": f"{log.counts['downlink']}",
            "Motor frames sent": f"{log.motor_frames}",
            "Round trip": (f"{health.round_trip_ms:.0f} ms  (max {health.round_trip_max_ms:.0f})"
                           if health.round_trip_ms else "--"),
        }
        for key, text in values.items():
            self.stat_labels[key].config(text=text)
        self._show_findings(findings)
        self._show_events(log)

    def _show_findings(self, findings):
        colours = {"bad": gui.COLOUR_BAD, "warn": gui.COLOUR_WARN, "ok": gui.COLOUR_OK}
        while len(self.finding_labels) < len(findings):
            label = ttk.Label(self.findings_frame, style="Panel.TLabel", wraplength=WRAP_PX,
                              justify=tk.LEFT)
            label.pack(anchor=tk.W, pady=(2, 0))
            self.finding_labels.append(label)
        for i, label in enumerate(self.finding_labels):
            if i < len(findings):
                level, text = findings[i]
                label.config(text=f"- {text}", foreground=colours[level])
            else:
                label.config(text="")

    def _show_events(self, log):
        key = (id(log), len(log.events), log.events[-1] if log.events else None)
        if key == self.shown_events:
            return
        self.shown_events = key
        self.events.delete(*self.events.get_children())
        for event in reversed(log.events):
            self.events.insert("", tk.END, values=(f"{event.time_s:.1f}",
                                                   pb.describe_event(event)))

    # ---- Buttons ----
    def reset(self):
        if self.app.link.is_open:
            self.app.link.reset_link_log()
        self.saved_label.config(text="")

    def save_report(self):
        if not self.app.link.is_open:
            self.saved_label.config(text="Connect first.")
            return
        log, damaged, findings, now = self.app.link.link_report()
        lines = [f"Link report {datetime.now():%Y-%m-%d %H:%M:%S}", ""]
        lines += [f"{key}: {label.cget('text')}" for key, label in self.stat_labels.items()]
        lines += ["", "Diagnosis:"] + [f"  [{level}] {text}" for level, text in findings]
        lines += ["", "Events (t = seconds since connect):"]
        lines += [f"  {event.time_s:7.1f}  {pb.describe_event(event)}" for event in log.events]
        path = os.path.join(gui.LOG_DIR, f"link-report_{datetime.now():%Y%m%d_%H%M%S}.txt")
        try:
            with open(path, "w", encoding="utf-8") as file:
                file.write("\n".join(lines) + "\n")
        except OSError as error:
            self.saved_label.config(text=f"Could not save: {error}")
            return
        self.saved_label.config(text=f"Saved to {path}")

    def close(self):
        self.window.after_cancel(self.after_id)
        self.app.link_test_window = None
        self.window.destroy()
