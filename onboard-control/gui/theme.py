"""Green-on-black terminal look for the rover dashboard (rover_gui_map.py).

Flat and light: plain Tk frames with 1 px green outlines, a monospace font and
segmented bars that only recolour the segments that change. No Pillow images,
shadows or animations (glass.py is only used for the dark title bar).
Phosphor green for text and a lime accent. Left-side readings are phosphor
white and right-side readings mint green, so the two sides never look alike but
stay in the terminal palette. Yellow is for warnings, light grey for faults.
No orange, red, magenta or cyan.
"""

import tkinter as tk
from tkinter import ttk

FONT = "Consolas"

BG = "#030a04"              # Screen
RAISED = "#0a170c"          # Buttons, inputs
HOVER = "#112614"
BORDER = "#1d5527"          # Box outlines
GRID = "#0d2411"            # Chart grid, unlit segments
MUTED = "#3c8f4c"
TEXT = "#4af078"            # Phosphor green
ACCENT = "#a6ff4d"          # Lime: primary action, selection
BRIGHT = "#e4ffd0"          # Near-white phosphor: the strongest highlight
LEFT = "#f2fff2"            # Phosphor white: everything on the left side
RIGHT = "#33ffaa"           # Mint green: everything on the right side
WARN = "#ffe14d"            # Yellow: warnings (and a wheel running backwards)
SOFT = "#8fcf9f"            # Extra trace colour
RED = "#c8c8c8"             # Faults and emergency stop: light grey
DISABLED = "#1e3a24"
DARK = "#010401"            # Sensor over the black table


def apply(gui):
    """Point rover_gui's colour and font constants at this palette.
    Call before the GUI is built; widgets read the constants when created."""
    for name, value in (("SOLID_BG", BG), ("COLOUR_PANEL", BG), ("COLOUR_PANEL_RAISED", RAISED),
                        ("COLOUR_BORDER", BORDER), ("COLOUR_TEXT", TEXT), ("COLOUR_MUTED", MUTED),
                        ("COLOUR_BLUE", ACCENT), ("COLOUR_GREEN", ACCENT),
                        ("COLOUR_GREEN_LAMP", ACCENT), ("COLOUR_ORANGE", WARN),
                        ("COLOUR_ORANGE_LAMP", WARN), ("COLOUR_RED", RED),
                        ("COLOUR_OK", ACCENT), ("COLOUR_WARN", WARN), ("COLOUR_BAD", RED),
                        ("COLOUR_IR_LEFT", LEFT), ("COLOUR_IR_RIGHT", RIGHT),
                        ("COLOUR_POSITION", BRIGHT), ("COLOUR_LAMP_OFF", GRID),
                        ("COLOUR_LAMP_DARK", DARK), ("COLOUR_CHART_BG", BG),
                        ("COLOUR_GRID", GRID), ("COLOUR_CHART_CENTRE", BORDER),
                        ("FONT_FAMILY", FONT), ("GUIDE_FONT", (FONT, 11, "bold")),
                        ("READOUT_FONT", (FONT, 10))):
        setattr(gui, name, value)


def configure_styles(root):
    root.configure(bg=BG)
    style = ttk.Style()
    style.theme_use("clam")
    style.configure(".", background=BG, foreground=TEXT, font=(FONT, 10), bordercolor=BORDER,
                    lightcolor=BG, darkcolor=BG, troughcolor=RAISED, fieldbackground=RAISED,
                    insertcolor=TEXT, selectbackground=HOVER, selectforeground=BRIGHT)
    style.configure("Plain.TFrame", background=BG)
    style.configure("Panel.TLabel", background=BG, foreground=TEXT)
    style.configure("Muted.TLabel", background=BG, foreground=MUTED)
    style.configure("Title.TLabel", background=BG, foreground=TEXT, font=(FONT, 9, "bold"))
    style.configure("Big.TLabel", background=BG, foreground=TEXT, font=(FONT, 24, "bold"))
    for kind in ("Radiobutton", "Checkbutton"):
        name = f"Panel.T{kind}"
        style.configure(name, background=BG, foreground=MUTED, font=(FONT, 10),
                        focuscolor=BG, indicatorforeground=BG,
                        upperbordercolor=MUTED, lowerbordercolor=MUTED)
        style.map(name, background=[("active", BG)], foreground=[("selected", BRIGHT)],
                  indicatorbackground=[("selected", ACCENT), ("!selected", BG)],
                  upperbordercolor=[("selected", ACCENT)],
                  lowerbordercolor=[("selected", ACCENT)])

    style.configure("TButton", background=RAISED, foreground=TEXT, font=(FONT, 9, "bold"),
                    padding=(8, 3), bordercolor=BORDER, focuscolor=RAISED, relief="flat")
    style.map("TButton", background=[("disabled", BG), ("pressed", BORDER), ("active", HOVER)],
              foreground=[("disabled", DISABLED)], bordercolor=[("disabled", GRID)])
    # Primary / danger: solid blocks, like the lit segments
    for name, colour, pressed in (("Primary.TButton", ACCENT, BRIGHT),
                                  ("Danger.TButton", RED, BRIGHT)):
        style.configure(name, background=colour, foreground=BG, font=(FONT, 11, "bold"),
                        padding=(10, 6), bordercolor=colour, focuscolor=colour)
        style.map(name, background=[("disabled", BG), ("pressed", pressed), ("active", pressed)],
                  foreground=[("disabled", DISABLED)], bordercolor=[("disabled", GRID)])

    style.configure("Treeview", background=BG, fieldbackground=BG, foreground=TEXT,
                    bordercolor=BORDER, rowheight=20, font=(FONT, 9))
    style.map("Treeview", background=[("selected", HOVER)], foreground=[("selected", BRIGHT)])
    style.configure("Treeview.Heading", background=BG, foreground=MUTED, relief="flat",
                    font=(FONT, 9, "bold"))
    style.map("Treeview.Heading", background=[("active", BG)])
    style.configure("TCombobox", fieldbackground=RAISED, background=RAISED, foreground=TEXT,
                    arrowcolor=TEXT, bordercolor=BORDER)
    style.map("TCombobox", fieldbackground=[("readonly", RAISED)],
              foreground=[("readonly", TEXT)], selectbackground=[("readonly", RAISED)],
              selectforeground=[("readonly", TEXT)])
    root.option_add("*TCombobox*Listbox.background", RAISED)
    root.option_add("*TCombobox*Listbox.foreground", TEXT)
    root.option_add("*TCombobox*Listbox.selectBackground", HOVER)
    root.option_add("*TCombobox*Listbox.selectForeground", BRIGHT)
    root.option_add("*TCombobox*Listbox.font", (FONT, 10))
    style.configure("TEntry", fieldbackground=RAISED, foreground=TEXT, bordercolor=BORDER)
    style.configure("Vertical.TScrollbar", background=RAISED, troughcolor=BG,
                    arrowcolor=MUTED, bordercolor=BG)


def pixel_blocks(parent, pattern, cell=5, colour=ACCENT):
    """A static block of 'pixels' (the HUD corner marks). pattern: rows of '#' / '.'."""
    rows = len(pattern)
    columns = max(len(row) for row in pattern)
    canvas = tk.Canvas(parent, width=columns * cell, height=rows * cell, bg=BG,
                       highlightthickness=0)
    for y, row in enumerate(pattern):
        for x, mark in enumerate(row):
            if mark == "#":
                canvas.create_rectangle(x * cell, y * cell, (x + 1) * cell - 1,
                                        (y + 1) * cell - 1, fill=colour, outline="")
    return canvas


class SegmentBar:
    """A row of block segments lit from `centre` towards the value. Only
    segments that change are recoloured."""

    def __init__(self, parent, segments, width, height, low, high, centre=None,
                 colour=ACCENT, gap=2):
        self.low, self.high = low, high
        self.centre = low if centre is None else centre
        self.colour = colour
        self.canvas = tk.Canvas(parent, width=width, height=height, bg=BG, highlightthickness=0)
        step = width / segments
        self.edges = [low + (high - low) * i / segments for i in range(segments + 1)]
        self.items = [self.canvas.create_rectangle(i * step, 0, (i + 1) * step - gap, height - 1,
                                                   fill=BG, outline=BORDER)
                      for i in range(segments)]
        self.lit = [False] * segments
        self.fill = [colour] * segments
        self.marks = []
        self.step = step
        self.height = height

    def set(self, value, colour=None):
        """Lights the segments between the centre and value. None = all dark."""
        colour = colour or self.colour
        for i, item in enumerate(self.items):
            lit = False
            if value is not None:
                middle = (self.edges[i] + self.edges[i + 1]) / 2
                lit = min(self.centre, value) <= middle <= max(self.centre, value)
            if lit != self.lit[i] or (lit and colour != self.fill[i]):
                self.lit[i] = lit
                self.fill[i] = colour
                self.canvas.itemconfigure(item, fill=colour if lit else BG,
                                          outline=colour if lit else BORDER)
