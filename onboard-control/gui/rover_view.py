"""Top-down view of our rover for the dashboard, drawn after the photos of the
cart: rounded clear-acrylic chassis, green PCB with its chips, the blue Nano and
the XBee, a battery holder along each side, wires, the two treaded drive wheels
at the sides, a castor at the back and the two IR sensors on a bracket at the
nose.

The live parts light up: each IR sensor in its side's colour while it is over
the white line, each tyre's tread brighter the faster it drives (yellow when it
runs backwards), the XBee LED while frames arrive, and the chassis edge turns
red when the line is lost. Each part's readings sit beside it.

Sizes follow the measured 12.5 x 7 in body and 2.7 cm sensor spacing; the
other positions are drawn by eye from the photos. Everything is created once;
an update only changes the colours, coordinates and text that changed.
"""

import tkinter as tk

import protocol as p
import theme

INCH_CM = 2.54
ROVER_LENGTH_CM = 12.5 * INCH_CM
ROVER_WIDTH_CM = 7.0 * INCH_CM
SENSOR_SPACING_CM = 2.7             # Centre to centre (measured)
SENSOR_FROM_FRONT_CM = 1.8
LINE_WIDTH_CM = 3.0                 # The white line on the track
WHEEL_AXLE_FRACTION = 0.55          # Axle position from the front
TYRE_LENGTH_CM = 7.0
TYRE_WIDTH_CM = 3.0
TYRE_TUCK_CM = 0.4                  # How far the tyre sits under the chassis edge
CORNER_RADIUS_CM = 5.5              # Rounded chassis ends

CM_PX = 9                           # Drawing scale
MARGIN_TOP = 46                     # Room for the line ahead of the rover
MARGIN_BOTTOM = 46                  # Room for the turn gauge
SENSOR_RADIUS = 5
GLOW_RADIUS = 10
TREAD_STEP = 6                      # px between tread lines
ARROW_MAX = 60                      # px of wheel arrow at 100 % speed
ARROW_OFFSET = 10                   # px outside the tyre
LABEL_GAP = 18                      # px from the chassis (sensors) or arrow (wheels) to the label
TURN_GAUGE_HALF = 110               # px either side of centre at 100 % turn
TURN_TICK = 6
SPEED_DEADBAND = 3                  # |speed %| below this counts as stopped
MIN_GLOW = 0.3                      # Tread brightness at the slowest speed, 1 = full colour
LED_RADIUS = 4

# Real-world colours of the parts (the UI around them stays terminal green)
ACRYLIC = "#13282c"
ACRYLIC_EDGE = "#7fb4b9"
HOLE = "#081214"
PCB = "#0d5a2b"
PCB_EDGE = "#3ad070"
CHIP = "#0a0a0a"
TERMINAL = "#2ee86f"
NANO = "#1f55c4"
NANO_EDGE = "#7aa2ff"
XBEE = "#262626"
BATTERY = "#070707"
BATTERY_EDGE = "#3a3a3a"
RUBBER = "#161616"
TREAD_DARK = "#3c3c3c"
METAL = "#9aa4a8"
BRACKET = "#202426"
LENS_OFF = "#050505"
LINE_FILL = "#c9d6cc"               # The white line, ahead of the nose
WIRES = ("#c8c8c8", "#ffe14d", "#3d8bff", "#8fcf9f")

FONT = (theme.FONT, 9)
FONT_BOLD = (theme.FONT, 9, "bold")
FONT_TINY = (theme.FONT, 7, "bold")

DEFAULT_WHITE_LEVEL = 60            # config.h, until the rover reports its calibration
DEFAULT_BLACK_LEVEL = 720


def blend(dark, bright, amount):
    """Colour between dark (0) and bright (1)."""
    a = [int(dark[i:i + 2], 16) for i in (1, 3, 5)]
    b = [int(bright[i:i + 2], 16) for i in (1, 3, 5)]
    return "#" + "".join(f"{round(x + (y - x) * amount):02x}" for x, y in zip(a, b))


def normalised(raw, white, black):
    if black == white:
        return 0.0
    return max(0.0, min(1.0, (raw - white) / (black - white)))


def rounded_rect(x0, y0, x1, y1, r):
    """Polygon points for a rectangle with rounded corners (smooth=True)."""
    return [x0 + r, y0, x1 - r, y0, x1, y0, x1, y0 + r, x1, y1 - r, x1, y1,
            x1 - r, y1, x0 + r, y1, x0, y1, x0, y1 - r, x0, y0 + r, x0, y0]


class RoverView:
    def __init__(self, parent, width):
        self.width = width
        self.height = round(ROVER_LENGTH_CM * CM_PX) + MARGIN_TOP + MARGIN_BOTTOM
        self.canvas = tk.Canvas(parent, width=width, height=self.height, bg=theme.BG,
                                highlightthickness=0)
        self.cache = {}
        self._layout()
        self._draw()

    def cm(self, x_cm, y_cm):
        """Rover coordinates (cm from the centre line, cm back from the nose) -> px."""
        return self.cx + x_cm * CM_PX, self.y0 + y_cm * CM_PX

    # ---- Geometry ----
    def _layout(self):
        self.cx = self.width / 2
        self.half_width = ROVER_WIDTH_CM * CM_PX / 2
        self.x0, self.x1 = self.cx - self.half_width, self.cx + self.half_width
        self.y0 = MARGIN_TOP
        self.y1 = MARGIN_TOP + ROVER_LENGTH_CM * CM_PX
        self.sensor_y = self.y0 + SENSOR_FROM_FRONT_CM * CM_PX
        offset = SENSOR_SPACING_CM / 2 * CM_PX
        self.sensor_x = {"L": self.cx - offset, "R": self.cx + offset}
        self.axle_y = self.y0 + WHEEL_AXLE_FRACTION * ROVER_LENGTH_CM * CM_PX
        tyre_w, tuck = TYRE_WIDTH_CM * CM_PX, TYRE_TUCK_CM * CM_PX
        self.tyre_l = TYRE_LENGTH_CM * CM_PX
        self.tyre_x = {"L": (self.x0 + tuck - tyre_w, self.x0 + tuck),
                       "R": (self.x1 - tuck, self.x1 - tuck + tyre_w)}
        self.arrow_x = {"L": self.tyre_x["L"][0] - ARROW_OFFSET,
                        "R": self.tyre_x["R"][1] + ARROW_OFFSET}
        self.turn_y = self.y1 + MARGIN_BOTTOM / 2 - 4

    # ---- Static parts, and the items updates change ----
    def _draw(self):
        c = self.canvas
        self.line = c.create_rectangle(0, 18, 0, self.sensor_y + 4, fill=LINE_FILL, outline="")
        self.line_text = c.create_text(self.cx, 8, text="", fill=theme.MUTED, font=FONT)
        self._draw_tyres()
        self.body = c.create_polygon(rounded_rect(self.x0, self.y0, self.x1, self.y1,
                                                  CORNER_RADIUS_CM * CM_PX),
                                     smooth=True, fill=ACRYLIC, outline=ACRYLIC_EDGE, width=2)
        for x_cm in (-6.6, 6.6):
            for y_cm in (2.2, ROVER_LENGTH_CM - 2.2, ROVER_LENGTH_CM * 0.32):
                x, y = self.cm(x_cm, y_cm)
                c.create_oval(x - 3, y - 3, x + 3, y + 3, fill=HOLE, outline=ACRYLIC_EDGE)
        self._draw_batteries()
        self._draw_pcb()
        self._draw_wires()
        castor_x, castor_y = self.cm(0, ROVER_LENGTH_CM - 2.6)
        c.create_oval(castor_x - 9, castor_y - 9, castor_x + 9, castor_y + 9, fill=BRACKET,
                      outline=METAL, width=2)
        c.create_oval(castor_x - 4, castor_y - 4, castor_x + 4, castor_y + 4, fill=METAL,
                      outline="")
        self._draw_sensors()
        self._draw_wheel_labels()
        self._draw_turn_gauge()

    def _draw_tyres(self):
        c = self.canvas
        self.tyre_glow, self.tyre, self.treads = {}, {}, {}
        for side in ("L", "R"):
            tx0, tx1 = self.tyre_x[side]
            top, bottom = self.axle_y - self.tyre_l / 2, self.axle_y + self.tyre_l / 2
            self.tyre_glow[side] = c.create_rectangle(tx0 - 3, top - 3, tx1 + 3, bottom + 3,
                                                      outline="", width=2)
            self.tyre[side] = c.create_rectangle(tx0, top, tx1, bottom, fill=RUBBER,
                                                 outline=TREAD_DARK)
            self.treads[side] = [c.create_line(tx0 + 2, y, tx1 - 2, y, fill=TREAD_DARK, width=2)
                                 for y in range(round(top) + 4, round(bottom) - 2, TREAD_STEP)]

    def _draw_batteries(self):
        c = self.canvas
        for x_cm in (-7.0, 4.6):
            x0, y0 = self.cm(x_cm, ROVER_LENGTH_CM * 0.30)
            x1, y1 = self.cm(x_cm + 2.4, ROVER_LENGTH_CM * 0.30 + 12.5)
            c.create_rectangle(x0, y0, x1, y1, fill=BATTERY, outline=BATTERY_EDGE, width=2)
            middle = (y0 + y1) / 2
            c.create_line(x0 + 3, middle, x1 - 3, middle, fill=BATTERY_EDGE)
            c.create_line(x0 + 4, y0 - 1, x0 + 4, y0 - 7, fill="#c8c8c8", width=2)
            c.create_line(x1 - 4, y0 - 1, x1 - 4, y0 - 7, fill="#d8d8d8", width=2)

    def _draw_pcb(self):
        c = self.canvas
        x0, y0 = self.cm(-4.2, 6.0)
        x1, y1 = self.cm(4.2, 25.0)
        c.create_rectangle(x0, y0, x1, y1, fill=PCB, outline=PCB_EDGE, width=2)
        # Chip rows
        for row in range(6):
            for column in range(3):
                cx, cy = self.cm(-2.8 + column * 2.4, 8.2 + row * 2.6)
                c.create_rectangle(cx - 7, cy - 4, cx + 7, cy + 4, fill=CHIP, outline="#2a2a2a")
        # Screw terminals along the edges
        for x_cm, y_cm in ((-3.9, 7.0), (3.3, 7.0), (-3.9, 23.2), (3.3, 23.2), (3.3, 15.0)):
            tx, ty = self.cm(x_cm, y_cm)
            c.create_rectangle(tx, ty, tx + 6, ty + 13, fill=TERMINAL, outline="")
        # Arduino Nano and the XBee with its link LED
        nx0, ny0 = self.cm(-1.0, 19.6)
        nx1, ny1 = self.cm(1.0, 24.2)
        c.create_rectangle(nx0, ny0, nx1, ny1, fill=NANO, outline=NANO_EDGE)
        c.create_text((nx0 + nx1) / 2, (ny0 + ny1) / 2, text="NANO", angle=90, fill="#dfe8ff",
                      font=FONT_TINY)
        xx0, xy0 = self.cm(-3.6, 17.2)
        xx1, xy1 = self.cm(-1.6, 19.8)
        c.create_rectangle(xx0, xy0, xx1, xy1, fill=XBEE, outline="#5a5a5a")
        c.create_text((xx0 + xx1) / 2, (xy0 + xy1) / 2, text="XB", fill="#cfcfcf",
                      font=FONT_TINY)
        self.led = c.create_oval(xx1 + 2, xy0, xx1 + 2 + 2 * LED_RADIUS, xy0 + 2 * LED_RADIUS,
                                 fill=LENS_OFF, outline="")

    def _draw_wires(self):
        c = self.canvas
        routes = (((-4.2, 9), (-8.2, 5), (-6.5, 1.0), (-1.4, 1.2)),       # Sensor L
                  ((4.2, 9), (8.2, 5), (6.5, 1.0), (1.4, 1.2)),           # Sensor R
                  ((-4.2, 13), (-8.4, 16), (-8.2, 21), (-7.4, 17)),
                  ((4.2, 13), (8.4, 16), (8.2, 21), (5.8, 17)))               # Battery leads
        for colour, route in zip(WIRES, routes):
            points = [v for point in route for v in self.cm(*point)]
            c.create_line(*points, smooth=True, fill=colour, width=2)

    def _draw_sensors(self):
        c = self.canvas
        bx0, by0 = self.cm(-2.8, SENSOR_FROM_FRONT_CM - 0.9)
        bx1, by1 = self.cm(2.8, SENSOR_FROM_FRONT_CM + 0.9)
        c.create_rectangle(bx0, by0, bx1, by1, fill=BRACKET, outline=METAL)
        self.glow, self.sensor, self.sensor_text = {}, {}, {}
        for side in ("L", "R"):
            x, y = self.sensor_x[side], self.sensor_y
            self.glow[side] = c.create_oval(x - GLOW_RADIUS, y - GLOW_RADIUS, x + GLOW_RADIUS,
                                            y + GLOW_RADIUS, outline="", width=3)
            self.sensor[side] = c.create_oval(x - SENSOR_RADIUS, y - SENSOR_RADIUS,
                                              x + SENSOR_RADIUS, y + SENSOR_RADIUS,
                                              fill=LENS_OFF, outline=METAL)
            left = side == "L"
            label_x = self.x0 - LABEL_GAP if left else self.x1 + LABEL_GAP
            c.create_line(label_x + (4 if left else -4), y,
                          x + (-GLOW_RADIUS if left else GLOW_RADIUS), y,
                          fill=theme.BORDER, dash=(2, 3))
            self.sensor_text[side] = c.create_text(label_x, y, text="", font=FONT,
                                                   anchor=tk.E if left else tk.W,
                                                   justify=tk.RIGHT if left else tk.LEFT,
                                                   fill=theme.MUTED)

    def _draw_wheel_labels(self):
        c = self.canvas
        self.arrow, self.wheel_text = {}, {}
        for side in ("L", "R"):
            left = side == "L"
            ax = self.arrow_x[side]
            self.arrow[side] = c.create_line(ax, self.axle_y, ax, self.axle_y, width=3,
                                             arrow=tk.LAST, fill=theme.MUTED)
            label_x = ax - LABEL_GAP if left else ax + LABEL_GAP
            self.wheel_text[side] = c.create_text(label_x, self.axle_y, text="", font=FONT,
                                                  anchor=tk.E if left else tk.W,
                                                  justify=tk.RIGHT if left else tk.LEFT,
                                                  fill=theme.MUTED)

    def _draw_turn_gauge(self):
        c = self.canvas
        y = self.turn_y
        c.create_line(self.cx - TURN_GAUGE_HALF, y, self.cx + TURN_GAUGE_HALF, y,
                      fill=theme.BORDER)
        c.create_line(self.cx, y - TURN_TICK, self.cx, y + TURN_TICK, fill=theme.MUTED)
        c.create_text(self.cx - TURN_GAUGE_HALF - 6, y, text="ACW", anchor=tk.E, font=FONT,
                      fill=theme.MUTED)
        c.create_text(self.cx + TURN_GAUGE_HALF + 6, y, text="CW", anchor=tk.W, font=FONT,
                      fill=theme.MUTED)
        self.ticks = [c.create_line(0, 0, 0, 0, fill=theme.BRIGHT, width=2) for _ in range(2)]
        self.turn_arrow = c.create_line(self.cx, y, self.cx, y, width=5, arrow=tk.LAST,
                                        fill=theme.BRIGHT)
        self.turn_text = c.create_text(self.cx, y + 15, text="", font=FONT_BOLD,
                                       fill=theme.MUTED)

    def _set(self, item, **options):
        """itemconfigure only what changed."""
        cache = self.cache.setdefault(item, {})
        changed = {key: value for key, value in options.items() if cache.get(key) != value}
        if changed:
            cache.update(changed)
            self.canvas.itemconfigure(item, **changed)

    def _coords(self, item, *points):
        points = tuple(round(v, 1) for v in points)
        if self.cache.get(("coords", item)) != points:
            self.cache[("coords", item)] = points
            self.canvas.coords(item, *points)

    # ---- Live update ----
    def update(self, t, debug=None, calibration=None, max_turn=None, running=False,
               link_fresh=False):
        """t: Telemetry (None = no link). debug / calibration: the rover's last
        frames, or None with firmware that does not send them. link_fresh: a frame
        arrived within the last few hundred ms (lights the XBee LED)."""
        lost = t is not None and (t.line_lost or
                                  (debug is not None and debug.state in p.LOST_STATES))
        self._set(self.body, outline=theme.RED if lost else ACRYLIC_EDGE,
                  width=3 if lost or running else 2)
        self._set(self.led, fill=theme.ACCENT if link_fresh else LENS_OFF)
        self._update_line(t, lost)
        for side in ("L", "R"):
            self._update_sensor(side, t, calibration)
            self._update_wheel(side, t)
        self._update_turn(t, debug, max_turn)

    def _update_line(self, t, lost):
        if t is None or lost:
            self._set(self.line, state=tk.HIDDEN)
            self._set(self.line_text, text="LINE LOST" if lost else "",
                      fill=theme.RED if lost else theme.MUTED)
            return
        offset_cm = t.position * SENSOR_SPACING_CM / 2     # > 0: line to the left
        x = self.cx - offset_cm * CM_PX
        half = LINE_WIDTH_CM * CM_PX / 2
        self._set(self.line, state=tk.NORMAL)
        self._coords(self.line, x - half, 18, x + half, self.sensor_y + 4)
        self._set(self.line_text, fill=theme.TEXT, text=f"pos {t.position:+.2f}")

    def _update_sensor(self, side, t, calibration):
        if t is None:
            self._set(self.sensor[side], fill=LENS_OFF)
            self._set(self.glow[side], outline="")
            self._set(self.sensor_text[side], text=f"IR {side}  --", fill=theme.MUTED)
            return
        left = side == "L"
        raw = t.ir_left if left else t.ir_right
        white = t.left_white if left else t.right_white
        if calibration is not None:
            levels = ((calibration.white_left, calibration.black_left) if left else
                      (calibration.white_right, calibration.black_right))
        else:
            levels = (DEFAULT_WHITE_LEVEL, DEFAULT_BLACK_LEVEL)
        colour = theme.LEFT if left else theme.RIGHT
        self._set(self.sensor[side], fill=colour if white else LENS_OFF)
        self._set(self.glow[side], outline=colour if white else "")
        surface = "WHITE (line)" if white else "BLACK (table)"
        self._set(self.sensor_text[side], fill=colour if white else blend(theme.BG, colour, 0.6),
                  text=f"IR {side}  {surface}\nlevel {normalised(raw, *levels):.2f}")

    def _update_wheel(self, side, t):
        left = side == "L"
        colour = theme.LEFT if left else theme.RIGHT
        if t is None:
            speed, dac = 0.0, None
        else:
            dac = t.dac_left if left else t.dac_right
            speed = p.dac_to_speed(dac)
        if abs(speed) < SPEED_DEADBAND:
            tread, glow, text_colour = TREAD_DARK, "", blend(theme.BG, colour, 0.6)
        else:
            if speed < 0:
                colour = theme.WARN                     # Backwards (backtrack, spin, Reverse)
            # Brighter the faster it drives, so the slowed wheel in a turn shows dimmer
            amount = MIN_GLOW + (1 - MIN_GLOW) * min(abs(speed), 100) / 100
            tread, glow, text_colour = blend(TREAD_DARK, colour, amount), colour, colour
        if self.cache.get(("tread", side)) != tread:
            self.cache[("tread", side)] = tread
            for line in self.treads[side]:
                self.canvas.itemconfigure(line, fill=tread)
        self._set(self.tyre_glow[side], outline=glow)
        x = self.arrow_x[side]
        tip = self.axle_y - speed / 100 * ARROW_MAX    # Forwards = up the screen
        self._coords(self.arrow[side], x, self.axle_y, x, tip)
        self._set(self.arrow[side], fill=text_colour,
                  state=tk.HIDDEN if abs(speed) < SPEED_DEADBAND else tk.NORMAL)
        if dac is None:
            self._set(self.wheel_text[side], text=f"WHEEL {side}  --", fill=theme.MUTED)
            return
        # Volts and the DAC code are in the ir-log, not on screen
        self._set(self.wheel_text[side], fill=text_colour,
                  text=f"WHEEL {side}\nspeed {speed:+4.0f} %\nduty {p.dac_to_percent(dac):5.1f} %")

    def _update_turn(self, t, debug, max_turn):
        y = self.turn_y
        for tick, sign in zip(self.ticks, (-1, 1)):
            if max_turn is None:
                self._set(tick, state=tk.HIDDEN)
                continue
            x = self.cx + sign * min(max_turn, 100) / 100 * TURN_GAUGE_HALF
            self._set(tick, state=tk.NORMAL)
            self._coords(tick, x, y - TURN_TICK, x, y + TURN_TICK)
        if t is None:
            self._coords(self.turn_arrow, self.cx, y, self.cx, y)
            self._set(self.turn_text, text="")
            return
        # Same as the firmware's debug turn: (left % - right %) / 2, > 0 = clockwise
        turn = (p.dac_to_speed(t.dac_left) - p.dac_to_speed(t.dac_right)) / 2
        saturated = debug is not None and debug.saturated
        colour = theme.RED if saturated else theme.BRIGHT
        tip = self.cx + max(-100, min(100, turn)) / 100 * TURN_GAUGE_HALF
        self._coords(self.turn_arrow, self.cx, y, tip, y)
        self._set(self.turn_arrow, fill=colour, state=tk.HIDDEN if abs(turn) < 1 else tk.NORMAL)
        limit = f" / max {max_turn}" if max_turn is not None else ""
        self._set(self.turn_text, fill=colour if abs(turn) >= 1 else theme.MUTED,
                  text=f"TURN {turn:+.0f}{limit}" + ("   SATURATED" if saturated else ""))
