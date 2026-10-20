"""Dark glass-card look for the rover GUI (Tk widgets, Pillow-rendered).

Tk has no rounded corners or soft shadows, so each card is a canvas that draws
an anti-aliased rounded panel (shadow, flat fill, faint light edge) and hosts
the real widgets in `.inner`. The fill is one flat colour so the widgets' own
backgrounds match it exactly.
"""

import ctypes
import os
import tkinter as tk
import tkinter.font as tkfont

from PIL import Image, ImageChops, ImageDraw, ImageFilter, ImageTk

FONT_FAMILY = "Segoe UI"

BG = "#0b0e14"
CARD = "#171c26"
CARD_BORDER = "#2b3342"
TEXT = "#f2f4f8"
# PC-control build: teal accent, so it is never mistaken for the blue
# onboard-control GUI.
ACCENT = "#14c8b4"
ACCENT_HOVER = "#3fdcc9"
ACCENT_PRESSED = "#0ea393"
ACCENT_TEXT = "#04211d"          # Dark text: white on teal is too faint
RED_BUTTON = "#ff453a"
RED_HOVER = "#ff6b61"
RED_PRESSED = "#d73a30"

CARD_MARGIN = 8                 # Room around the card for its shadow
CARD_PADDING = 10               # Card edge to content; keeps square widgets off the corners
CARD_RADIUS = 20
EDGE_ALPHA = 0.16               # Strength of the light edge
SHADOW_ALPHA = 0.6
SHADOW_BLUR = 4
SHADOW_DROP = 3
SUPERSAMPLE = 3                 # For anti-aliased rounded edges

DWMWA_USE_IMMERSIVE_DARK_MODE = 20


def dark_title_bar(root):
    """Ask Windows for a dark title bar (no-op elsewhere)."""
    if os.name != "nt":
        return
    try:
        root.update_idletasks()
        hwnd = ctypes.windll.user32.GetParent(root.winfo_id())
        ctypes.windll.dwmapi.DwmSetWindowAttribute(
            hwnd, DWMWA_USE_IMMERSIVE_DARK_MODE, ctypes.byref(ctypes.c_int(1)), 4)
    except (OSError, AttributeError):
        pass


def _rounded_mask(width, height, box, radius):
    """Anti-aliased rounded-rectangle mask. box is (left, top, right, bottom) in px."""
    scale = SUPERSAMPLE
    big = Image.new("L", (width * scale, height * scale), 0)
    left, top, right, bottom = (v * scale for v in box)
    ImageDraw.Draw(big).rounded_rectangle([left, top, right - 1, bottom - 1],
                                          radius=radius * scale, fill=255)
    return big.resize((width, height), Image.LANCZOS)


class GlassCard(tk.Canvas):
    """Rounded, shadowed panel. Put widgets in `.inner`."""

    def __init__(self, parent):
        super().__init__(parent, bg=BG, width=1, height=1, highlightthickness=0, bd=0)
        self.inner = tk.Frame(self, bg=CARD)
        self.inner.pack(fill=tk.BOTH, expand=True, padx=CARD_MARGIN + CARD_PADDING,
                        pady=CARD_MARGIN + CARD_PADDING)
        self._image_item = self.create_image(0, 0, anchor=tk.NW)
        self._size = None
        self._photo = None
        self.bind("<Configure>", self._paint)

    def _paint(self, event):
        size = (event.width, event.height)
        if size == self._size or min(size) < 2 * CARD_MARGIN + 2:
            return
        self._size = size
        self._photo = ImageTk.PhotoImage(self._render(*size))
        self.itemconfigure(self._image_item, image=self._photo)

    @staticmethod
    def _render(width, height):
        margin = CARD_MARGIN
        box = (margin, margin, width - margin, height - margin)
        shape = _rounded_mask(width, height, box, CARD_RADIUS)
        result = Image.new("RGB", (width, height), BG)

        shadow = ImageChops.offset(shape, 0, SHADOW_DROP).filter(
            ImageFilter.GaussianBlur(SHADOW_BLUR)).point(lambda v: int(v * SHADOW_ALPHA))
        result = Image.composite(Image.new("RGB", result.size, "#000000"), result, shadow)
        result = Image.composite(Image.new("RGB", result.size, CARD), result, shape)

        edge_box = tuple(v + d for v, d in zip(box, (1, 1, -1, -1)))
        edge = ImageChops.subtract(shape, _rounded_mask(width, height, edge_box,
                                                         CARD_RADIUS - 1))
        edge = edge.point(lambda v: int(v * EDGE_ALPHA))
        return Image.composite(Image.new("RGB", result.size, "#ffffff"), result, edge)


class GlassButton(tk.Canvas):
    """Pill button that mimics the bits of ttk.Button the GUI uses:
    state(["disabled"]), config(text=...) and a click command."""

    PALETTE = {     # kind: (fill, hover, pressed, border, text, height)
        "normal": ("#222a38", "#2c3547", "#1a2029", "#343e50", TEXT, 34),
        "primary": (ACCENT, ACCENT_HOVER, ACCENT_PRESSED, ACCENT, ACCENT_TEXT, 42),
        "danger": (RED_BUTTON, RED_HOVER, RED_PRESSED, RED_BUTTON, "#ffffff", 42),
    }
    DISABLED = ("#141922", "#222a37", "#5a6578")        # fill, border, text

    def __init__(self, parent, text, command, kind="normal", min_width=0):
        height = self.PALETTE[kind][5]
        size = 11 if kind != "normal" else 10
        self.kind = kind
        self._font = tkfont.Font(family=FONT_FAMILY, size=size, weight="bold")
        self._command = command
        self._text = text
        self._disabled = False
        self._hover = False
        self._pressed = False
        self._cache = {}
        super().__init__(parent, bg=CARD, highlightthickness=0, bd=0, height=height,
                         width=max(min_width, self._font.measure(text) + 40), takefocus=0)
        self._image_item = self.create_image(0, 0, anchor=tk.NW)
        self._text_item = self.create_text(0, 0, text=text, font=self._font)
        self.bind("<Configure>", lambda e: self._draw())
        self.bind("<Enter>", lambda e: self._set(hover=True))
        self.bind("<Leave>", lambda e: self._set(hover=False, pressed=False))
        self.bind("<ButtonPress-1>", lambda e: self._set(pressed=not self._disabled))
        self.bind("<ButtonRelease-1>", self._release)

    def _set(self, hover=None, pressed=None):
        if hover is not None:
            self._hover = hover
        if pressed is not None:
            self._pressed = pressed
        self._draw()

    def _release(self, _event):
        fire = self._pressed and self._hover and not self._disabled
        self._set(pressed=False)
        if fire and self._command:
            self._command()

    def _draw(self):
        width, height = self.winfo_width(), self.winfo_height()
        if width < 2 or height < 2:
            return
        fill, hover, pressed, border, colour, _height = self.PALETTE[self.kind]
        if self._disabled:
            fill, border, colour = self.DISABLED
        elif self._pressed:
            fill = pressed
        elif self._hover:
            fill = hover
        key = (width, height, fill, border)
        if key not in self._cache:
            scale = SUPERSAMPLE
            big = Image.new("RGB", (width * scale, height * scale), CARD)
            draw = ImageDraw.Draw(big)
            radius = height * scale / 2
            draw.rounded_rectangle([0, 0, width * scale - 1, height * scale - 1],
                                   radius=radius, fill=border)
            draw.rounded_rectangle([scale, scale, width * scale - scale - 1,
                                    height * scale - scale - 1], radius=radius - scale,
                                   fill=fill)
            self._cache[key] = ImageTk.PhotoImage(big.resize((width, height), Image.LANCZOS))
        self.itemconfigure(self._image_item, image=self._cache[key])
        self.coords(self._text_item, width / 2, height / 2)
        self.itemconfigure(self._text_item, text=self._text, fill=colour)

    def state(self, spec):
        self._disabled = "disabled" in spec
        self._draw()

    def configure(self, cnf=None, **options):
        if "text" in options:
            self._text = options.pop("text")
            self._draw()
        if "command" in options:
            self._command = options.pop("command")
        if cnf or options:
            super().configure(cnf, **options)

    config = configure
