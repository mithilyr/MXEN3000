"""Track geometry and position estimate for the track map (no GUI code here).

The rover has no wheel encoders, so its position is ESTIMATED from the wheel
speeds the firmware commanded (dac_left / dac_right in the GUI's ir-log CSVs):

1. Distance along the line = full-speed scale (cm/s) x average forward wheel
   command x time. The scale is set by telling the map where the rover really
   was at some moment of a run (see calibrated_scale).
2. Bend snap: on the straights the corrections go both ways, in the bends
   they are mostly to one side. When the steering starts or stops leaning to
   the bend side, and the estimate is close to a bend entry or exit, it jumps
   there. That removes most of the drift four times per lap.

Track: a full stadium (two straights joined by two semicircles), white line
on a black table. Coordinates are in cm from the centre of the stadium, x along
the straights, y across. A lap starts at the start of a straight and ends
back there. Every dimension is in the block below.
"""

import math
from collections import deque
from dataclasses import dataclass

# ---- Track dimensions (cm), measured on the competition track (20 Oct) ----
LINE_WIDTH = 3.0
STRAIGHT_LENGTH = 140.0  # Each straight, between the two bends
OUTER_WIDTH = 50.0       # Outer edge to outer edge of the line, across the straights
ARC_SEGMENTS = 60        # Polyline pieces per bend

# ---- Divisions on this track ----
# reverse_path: the lap goes anticlockwise (seen from above).
# travel: +1 drives forwards, -1 backwards.
# turn_sign: sign of (left - right wheel) in a bend. A clockwise body rotation
# needs the left wheel faster; Anticlockwise and Reverse both rotate the body
# anticlockwise.
@dataclass(frozen=True)
class Division:
    reverse_path: bool
    travel: int
    turn_sign: int


DIVISIONS = {
    "Clockwise": Division(False, 1, 1),
    "Anticlockwise": Division(True, 1, -1),
    "Reverse": Division(True, -1, -1),
}

# ---- Estimator ----
DAC_NEUTRAL = 128        # config.h DAC_NEUTRAL / DAC_SPAN
DAC_SPAN = 127
DEFAULT_FULL_SPEED_CM_S = 50.0   # Guess until calibrated
MIN_CALIBRATION_FRACTION = 0.05  # Ignore a calibration this early in a run (of full speed x 1 s)
MAX_STEP_S = 0.5         # Longer gaps between samples (radio dropout) count as this
TURN_WINDOW_S = 1.0      # Steering lean is averaged over this long
TURN_MIN_SAMPLES = 5
BEND_ON = 0.12           # (left - right) / forward above this = in a bend
BEND_OFF = 0.05          # ... and below this = out of it again
SNAP_WINDOW_CM = 30.0    # Only snap when the estimate is this close to a bend end
# The averaged lean crosses BEND_ON / BEND_OFF some time after the rover really
# entered or left the bend. That delay is worked out from how strongly the rover
# leans in a bend (the last bend's, or this guess for the first one).
DEFAULT_BEND_LEAN = 0.5


class Track:
    """The line's centre as a closed polyline, clockwise seen from above,
    starting at the start of the top straight."""

    def __init__(self):
        radius = (OUTER_WIDTH - LINE_WIDTH) / 2
        half = STRAIGHT_LENGTH / 2
        points = [(-half, radius)]
        for centre_x, start in ((half, math.pi / 2), (-half, -math.pi / 2)):
            # Each bend turns clockwise through 180 degrees; the straight before
            # it is the line from the previous point to its first point.
            points += [(centre_x + radius * math.cos(start - math.pi * i / ARC_SEGMENTS),
                        radius * math.sin(start - math.pi * i / ARC_SEGMENTS))
                       for i in range(ARC_SEGMENTS + 1)]
        points.append(points[0])
        self.points = points
        self.radius = radius
        self.lengths = [0.0]
        for (x0, y0), (x1, y1) in zip(points, points[1:]):
            self.lengths.append(self.lengths[-1] + math.hypot(x1 - x0, y1 - y0))
        self.length = self.lengths[-1]
        bend = self.length / 2 - STRAIGHT_LENGTH
        # (start, end) of each bend in lap distance. The same in either
        # direction, because each lap starts at the start of a straight.
        self.bends = ((STRAIGHT_LENGTH, STRAIGHT_LENGTH + bend),
                      (2 * STRAIGHT_LENGTH + bend, self.length))

    def _clockwise_distance(self, distance, reverse_path):
        """Lap distance of a run -> distance along the clockwise polyline."""
        if reverse_path:
            # Anticlockwise laps start at the other end of the top straight.
            distance = STRAIGHT_LENGTH - distance
        return distance % self.length

    def point(self, distance, reverse_path=False):
        """(x, y) of the point this far along the run (any number of laps)."""
        distance = self._clockwise_distance(distance, reverse_path)
        for i in range(1, len(self.lengths)):
            if self.lengths[i] >= distance:
                piece = self.lengths[i] - self.lengths[i - 1]
                share = (distance - self.lengths[i - 1]) / piece if piece else 0.0
                (x0, y0), (x1, y1) = self.points[i - 1], self.points[i]
                return x0 + (x1 - x0) * share, y0 + (y1 - y0) * share
        return self.points[-1]

    def nearest(self, x, y, reverse_path=False):
        """Lap distance (0 .. length) of the line point closest to (x, y)."""
        best, best_gap = 0.0, math.inf
        for i in range(1, len(self.points)):
            (x0, y0), (x1, y1) = self.points[i - 1], self.points[i]
            dx, dy = x1 - x0, y1 - y0
            piece_sq = dx * dx + dy * dy
            share = 0.0 if piece_sq == 0 else \
                min(max(((x - x0) * dx + (y - y0) * dy) / piece_sq, 0.0), 1.0)
            gap = math.hypot(x0 + dx * share - x, y0 + dy * share - y)
            if gap < best_gap:
                best_gap = gap
                best = self.lengths[i - 1] + share * (self.lengths[i] - self.lengths[i - 1])
        if reverse_path:
            best = STRAIGHT_LENGTH - best
        return best % self.length

    def section(self, distance):
        """"Straight 1", "Bend 1", "Straight 2" or "Bend 2" for a run distance."""
        lap = distance % self.length
        (bend1_start, bend1_end), (bend2_start, _bend2_end) = self.bends
        if lap < bend1_start:
            return "Straight 1"
        if lap < bend1_end:
            return "Bend 1"
        if lap < bend2_start:
            return "Straight 2"
        return "Bend 2"

    def nearest_bend_end(self, distance, which):
        """The bend start (which=0) or end (which=1) closest to this run
        distance, over this lap and the ones either side."""
        lap = math.floor(distance / self.length)
        candidates = [(lap + k) * self.length + bend[which]
                      for k in (-1, 0, 1) for bend in self.bends]
        return min(candidates, key=lambda target: abs(target - distance))


def wheel_share(dac):
    """DAC code -> commanded wheel speed, -1 (full reverse) .. 1 (full forward)."""
    return min(max((dac - DAC_NEUTRAL) / DAC_SPAN, -1.0), 1.0)


@dataclass
class State:
    time_s: float
    distance: float      # cm along the run, after snaps (can pass one lap)
    raw: float           # Full-speed seconds travelled, no snaps (for calibration)
    x: float
    y: float
    line_lost: bool
    in_bend: bool
    snapped: str         # "", "bend entry" or "bend exit"


class Estimator:
    def __init__(self, track, division, full_speed_cm_s, snap=True):
        self.track = track
        self.division = division
        self.full_speed = full_speed_cm_s
        self.snap = snap
        self.distance = 0.0
        self.raw = 0.0
        self.last_time = None
        self.lean = deque()          # (time, left - right, forward) in the window
        self.in_bend = False
        self.bend_leans = []         # Averaged lean during the current bend
        self.last_bend_lean = DEFAULT_BEND_LEAN

    def step(self, time_s, dac_left, dac_right, line_lost):
        left, right = wheel_share(dac_left), wheel_share(dac_right)
        forward = (left + right) / 2 * self.division.travel
        dt = 0.0 if self.last_time is None else min(max(time_s - self.last_time, 0.0), MAX_STEP_S)
        self.last_time = time_s
        self.raw += forward * dt
        self.distance = max(self.distance + self.full_speed * forward * dt, 0.0)
        snapped = self._update_bend(time_s, left - right, abs(forward))
        x, y = self.track.point(self.distance, self.division.reverse_path)
        return State(time_s, self.distance, self.raw, x, y, bool(line_lost), self.in_bend, snapped)

    def _update_bend(self, time_s, difference, forward):
        self.lean.append((time_s, difference, forward))
        while self.lean and self.lean[0][0] < time_s - TURN_WINDOW_S:
            self.lean.popleft()
        if len(self.lean) < TURN_MIN_SAMPLES:
            return ""
        mean_forward = sum(item[2] for item in self.lean) / len(self.lean)
        if mean_forward < 1e-3:
            return ""
        lean = self.division.turn_sign * sum(item[1] for item in self.lean) / len(self.lean)
        lean /= mean_forward
        speed = self.full_speed * mean_forward          # cm/s, recent average
        if self.in_bend:
            self.bend_leans.append(lean)
        if not self.in_bend and lean > BEND_ON:
            self.in_bend = True
            self.bend_leans = []
            delay = TURN_WINDOW_S * min(BEND_ON / self.last_bend_lean, 1.0)
            return self._snap(0, "bend entry", speed * delay)
        if self.in_bend and lean < BEND_OFF:
            self.in_bend = False
            if self.bend_leans:
                self.last_bend_lean = max(max(self.bend_leans), BEND_ON)
            delay = TURN_WINDOW_S * (1 - min(BEND_OFF / self.last_bend_lean, 1.0))
            return self._snap(1, "bend exit", speed * delay)
        return ""

    def _snap(self, which, label, travelled_since):
        """Jump to the bend entry / exit plus the distance covered since the
        rover really passed it, when the estimate is close enough."""
        target = self.track.nearest_bend_end(self.distance - travelled_since, which)
        target += travelled_since
        if self.snap and abs(self.distance - target) < SNAP_WINDOW_CM:
            self.distance = target
            return label
        return ""


def estimate(track, division, rows, full_speed_cm_s, snap=True):
    """States for rows of (time_s, dac_left, dac_right, line_lost)."""
    estimator = Estimator(track, division, full_speed_cm_s, snap)
    return [estimator.step(*row) for row in rows]


def calibrated_scale(track, division, state, true_x, true_y):
    """Full-speed cm/s that puts the rover at (true_x, true_y) at this state,
    from straight dead reckoning (snaps ignored). The lap count is taken from
    the estimate. None if too early in the run."""
    if state.raw < MIN_CALIBRATION_FRACTION:
        return None
    lap_distance = track.nearest(true_x, true_y, division.reverse_path)
    laps = round((state.distance - lap_distance) / track.length)
    return (max(laps, 0) * track.length + lap_distance) / state.raw
