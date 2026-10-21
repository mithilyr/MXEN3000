"""Stacked waveform strips (an oscilloscope view) for the rover dashboard.

Each strip has a fixed range, dashed reference lines and one or more traces;
the newest value of every trace is printed in its colour on the strip's
title row, so the numbers sit next to their waveform. Samples arrive with the
telemetry (10 Hz); the static parts are drawn once and only the traces are
redrawn, and only when a sample was added.
"""

from collections import deque
from dataclasses import dataclass

import tkinter as tk

import theme

TITLE_HEIGHT = 15               # px above each strip for its title and values
STRIP_GAP = 6
VALUE_SPACING = 12              # px between one value and the next on the title row
FONT = (theme.FONT, 9)
FONT_BOLD = (theme.FONT, 9, "bold")
FONT_SMALL = (theme.FONT, 7)
FLAG_HEIGHT = 4                 # px of the red marks under a strip


@dataclass(frozen=True)
class Trace:
    key: str
    label: str
    colour: str
    fmt: str = "+.2f"


@dataclass(frozen=True)
class Strip:
    title: str
    low: float
    high: float
    traces: tuple
    refs: tuple = ()            # Dashed reference lines
    flag: str = ""              # Sample key: a true value draws a red mark under the strip
    unit: str = ""


class Scope:
    def __init__(self, parent, strips, width, strip_height, samples):
        self.strips = strips
        self.width = width
        self.strip_height = strip_height
        self.data = deque(maxlen=samples)
        self.samples = samples
        height = len(strips) * (TITLE_HEIGHT + strip_height + STRIP_GAP)
        self.canvas = tk.Canvas(parent, width=width, height=height, bg=theme.BG,
                                highlightthickness=0)
        self.values = []        # Per strip: {trace key: text item}
        self.tops = []
        self.dirty = True
        self._draw_static()

    def _y(self, strip, top, value):
        value = max(strip.low, min(strip.high, value))
        return top + (strip.high - value) / (strip.high - strip.low) * self.strip_height

    def _draw_static(self):
        c = self.canvas
        y = 0
        for strip in self.strips:
            top = y + TITLE_HEIGHT
            self.tops.append(top)
            c.create_text(0, y + 7, text=strip.title, anchor=tk.W, fill=theme.MUTED,
                          font=FONT_BOLD)
            x = len(strip.title) * 7 + 2 * VALUE_SPACING
            values = {}
            for trace in strip.traces:
                values[trace.key] = c.create_text(x, y + 7, text="", anchor=tk.W,
                                                  fill=trace.colour, font=FONT)
                x += (len(trace.label) + 9) * 7 + VALUE_SPACING
            self.values.append(values)
            c.create_rectangle(0, top, self.width - 1, top + self.strip_height,
                               outline=theme.BORDER)
            for ref in strip.refs:
                ry = self._y(strip, top, ref)
                c.create_line(1, ry, self.width - 2, ry, fill=theme.GRID, dash=(3, 3))
            for value, anchor in ((strip.high, tk.NE), (strip.low, tk.SE)):
                c.create_text(self.width - 3, top + (1 if anchor == tk.NE else self.strip_height),
                              text=f"{value:g}{strip.unit}", anchor=anchor, fill=theme.BORDER,
                              font=FONT_SMALL)
            y = top + self.strip_height + STRIP_GAP

    def push(self, sample):
        """sample: {trace key: value or None}; None leaves a gap in that trace."""
        self.data.append(sample)
        self.dirty = True

    def clear(self):
        self.data.clear()
        self.dirty = True

    def draw(self):
        if not self.dirty:
            return
        self.dirty = False
        c = self.canvas
        c.delete("trace")
        step = (self.width - 2) / (self.samples - 1)
        latest = self.data[-1] if self.data else {}
        for strip, top, values in zip(self.strips, self.tops, self.values):
            for trace in strip.traces:
                self._plot(strip, top, trace, step)
                value = latest.get(trace.key)
                text = f"{trace.label} {value:{trace.fmt}}" if value is not None else \
                    f"{trace.label} --"
                c.itemconfigure(values[trace.key], text=text)
            if strip.flag:
                for i, sample in enumerate(self.data):
                    if sample.get(strip.flag):
                        x = 1 + i * step
                        c.create_rectangle(x, top + self.strip_height - FLAG_HEIGHT, x + step,
                                           top + self.strip_height, fill=theme.RED, outline="",
                                           tags="trace")

    def _plot(self, strip, top, trace, step):
        """One polyline per run of samples that have a value."""
        points = []
        for i, sample in enumerate(self.data):
            value = sample.get(trace.key)
            if value is None:
                self._line(points, trace.colour)
                points = []
                continue
            points += [1 + i * step, self._y(strip, top, value)]
        self._line(points, trace.colour)

    def _line(self, points, colour):
        if len(points) >= 4:
            self.canvas.create_line(*points, fill=colour, width=2, tags="trace")
