"""Lap-time optimiser: suggests the next tuning to try from past timed runs.

Bayesian optimisation in plain Python (no numpy, so the .exe stays small):

1. Every rated run is one observation: the settings used and the lap time,
   or "failed" (line lost, left the track). A failed run is scored as a lap
   FAIL_PENALTY times slower than the slowest clean lap, so settings near a
   failure look bad. In the competition a failed run scores nothing, so
   reliability matters more than a few tenths of a second.
2. A Gaussian process models lap time over the settings, with an estimate of
   how unsure it is at each point.
3. The next suggestion is the candidate with the highest expected
   improvement on the best lap so far, picked from a box around the current
   best settings (a "trust region"), so a real rover is never sent a wild
   jump. The box starts small, grows after each new best lap and shrinks
   after each failed run.

Ranges are in SEARCH_SPACE. Narrow them if the optimiser keeps suggesting
values you know are useless (for example a speed the motors cannot reach).
"""

import math
import random
from dataclasses import dataclass

import protocol as p

FAIL_PENALTY = 1.25         # Failed run = slowest clean lap x this
NOISE = 0.1                 # Run-to-run lap time scatter, as a share of the spread
LENGTH_SCALES = (0.15, 0.3, 0.6)  # Tried when fitting, in normalised (0-1) units
FIXED_LENGTH_SCALE = 0.3    # Used until there are enough runs to fit it
MIN_RUNS_TO_FIT = 6
CANDIDATES = 1500
# Trust region half-width, as a share of each range (0.1 of speed 25-100 = 7.5)
START_STEP = 0.1
MAX_STEP = 0.25
MIN_STEP = 0.03
GROW_ON_BEST = 1.5
SHRINK_ON_FAIL = 0.5
EXPLORATION = 0.01          # Expected-improvement margin, in standardised units


@dataclass(frozen=True)
class Dimension:
    key: str                # "speed", a protocol.TUNING_PARAMS key or a PWM_PAIRS key
    low: float
    high: float
    step: float             # The rover's own resolution for this value
    decimals: int

    def clip(self, value):
        value = round(round(value / self.step) * self.step, self.decimals)
        return min(max(value, self.low), self.high)

    def to_unit(self, value):
        return (value - self.low) / (self.high - self.low)

    def from_unit(self, unit):
        return self.clip(self.low + unit * (self.high - self.low))


SPEED = Dimension("speed", 25, 100, 1, 0)
MAX_TURN = Dimension("max_turn", 20, 100, 1, 0)
# Bang-bang ignores speed, bang_level and max_turn: it only uses its four wheel
# duties. The optimiser tunes them as two values, each sent to both wheels.
PWM_PAIRS = {
    "pwm_straight": ("pwm_straight_left", "pwm_straight_right"),
    "pwm_correct": ("pwm_correct_left", "pwm_correct_right"),
}
SEARCH_SPACE = {
    p.CONTROLLER_PID: (SPEED, Dimension("kp", 10, 200, 1, 0), Dimension("kd", 0, 6, 0.1, 1),
                       Dimension("ki", 0, 60, 1, 0), MAX_TURN),
    p.CONTROLLER_BANG_BANG: (Dimension("pwm_straight", 55, 100, 1, 0),
                             Dimension("pwm_correct", 40, 80, 1, 0)),
    p.CONTROLLER_ADAPTIVE: (SPEED, Dimension("adapt_gain", 1, 25, 0.1, 1),
                            Dimension("adapt_ramp", 0, 150, 1, 0), MAX_TURN),
}


@dataclass
class Run:
    settings: dict          # Dimension key -> value used for the run
    time_s: float
    clean: bool             # False = failed (no valid lap)


def allowed(settings):
    """Bang-bang: the slow wheel must stay slower than the straight one, or the
    rover steers away from the line."""
    if "pwm_correct" in settings:
        return settings["pwm_correct"] < settings["pwm_straight"]
    return True


def describe(settings, dimensions):
    return ", ".join(f"{d.key} {settings[d.key]:.{d.decimals}f}" for d in dimensions)


def _normal_cdf(z):
    return 0.5 * (1 + math.erf(z / math.sqrt(2)))


def _normal_pdf(z):
    return math.exp(-0.5 * z * z) / math.sqrt(2 * math.pi)


def _cholesky(matrix):
    n = len(matrix)
    lower = [[0.0] * n for _ in range(n)]
    for i in range(n):
        for j in range(i + 1):
            total = matrix[i][j] - sum(lower[i][k] * lower[j][k] for k in range(j))
            if i == j:
                lower[i][j] = math.sqrt(max(total, 1e-12))
            else:
                lower[i][j] = total / lower[j][j]
    return lower


def _solve_lower(lower, vector):
    result = []
    for i, row in enumerate(lower):
        result.append((vector[i] - sum(row[k] * result[k] for k in range(i))) / row[i])
    return result


def _solve_upper_from_lower(lower, vector):
    """Solves L^T x = vector."""
    n = len(lower)
    result = [0.0] * n
    for i in reversed(range(n)):
        total = sum(lower[k][i] * result[k] for k in range(i + 1, n))
        result[i] = (vector[i] - total) / lower[i][i]
    return result


class GaussianProcess:
    """Squared-exponential kernel, standardised targets, fixed noise."""

    def __init__(self, points, targets, length_scale):
        self.points = points
        self.length_scale = length_scale
        n = len(points)
        matrix = [[self.kernel(points[i], points[j]) + (NOISE if i == j else 0.0)
                   for j in range(n)] for i in range(n)]
        self.lower = _cholesky(matrix)
        self.alpha = _solve_upper_from_lower(self.lower, _solve_lower(self.lower, targets))
        fit = sum(t * a for t, a in zip(targets, self.alpha))
        log_det = sum(math.log(self.lower[i][i]) for i in range(n))
        self.log_likelihood = -0.5 * fit - log_det

    def kernel(self, a, b):
        distance = sum((x - y) ** 2 for x, y in zip(a, b))
        return math.exp(-0.5 * distance / self.length_scale ** 2)

    def predict(self, point):
        k = [self.kernel(point, other) for other in self.points]
        mean = sum(x * y for x, y in zip(k, self.alpha))
        v = _solve_lower(self.lower, k)
        variance = max(1.0 - sum(x * x for x in v), 1e-9)
        return mean, math.sqrt(variance)


class LapOptimiser:
    def __init__(self, controller, runs, rng=None):
        self.dimensions = SEARCH_SPACE[controller]
        self.runs = [run for run in runs if self._complete(run.settings)]
        self.rng = rng or random.Random()
        self.model = None
        clean_times = [run.time_s for run in self.runs if run.clean]
        if not clean_times:
            return
        penalty = max(clean_times) * FAIL_PENALTY
        laps = [run.time_s if run.clean else penalty for run in self.runs]
        self.offset = sum(laps) / len(laps)
        spread = math.sqrt(sum((t - self.offset) ** 2 for t in laps) / len(laps))
        self.scale = spread if spread > 1e-6 else 1.0
        targets = [(t - self.offset) / self.scale for t in laps]
        points = [self._to_unit(run.settings) for run in self.runs]
        scales = LENGTH_SCALES if len(points) >= MIN_RUNS_TO_FIT else (FIXED_LENGTH_SCALE,)
        self.model = max((GaussianProcess(points, targets, s) for s in scales),
                         key=lambda gp: gp.log_likelihood)
        self.best_target = min(targets)

    @property
    def ready(self):
        """Needs at least one clean lap to know what "good" looks like."""
        return self.model is not None

    def _complete(self, settings):
        return all(d.key in settings for d in self.dimensions)

    def _to_unit(self, settings):
        return [d.to_unit(settings[d.key]) for d in self.dimensions]

    def _from_unit(self, unit):
        return {d.key: d.from_unit(u) for d, u in zip(self.dimensions, unit)}

    def predicted_time(self, settings):
        mean, _ = self.model.predict(self._to_unit(settings))
        return self.offset + mean * self.scale

    def best(self):
        """The clean-lap settings with the lowest predicted time, so one lucky
        lap next to a failure does not win. Returns (settings, predicted s, laps)."""
        if not self.ready:
            return None
        clean = [run for run in self.runs if run.clean]
        best = min(clean, key=lambda run: self.predicted_time(run.settings))
        laps = sum(1 for run in clean if run.settings == best.settings)
        return dict(best.settings), self.predicted_time(best.settings), laps

    def _expected_improvement(self, unit):
        mean, sigma = self.model.predict(unit)
        gain = self.best_target - mean - EXPLORATION
        z = gain / sigma
        return gain * _normal_cdf(z) + sigma * _normal_pdf(z)

    def step_size(self):
        """Trust region half-width, replayed from the runs in the order they happened."""
        step, fastest = START_STEP, None
        for run in self.runs:
            if run.clean and (fastest is None or run.time_s < fastest):
                if fastest is not None:
                    step = min(step * GROW_ON_BEST, MAX_STEP)
                fastest = run.time_s
            elif not run.clean:
                step = max(step * SHRINK_ON_FAIL, MIN_STEP)
        return step

    def suggest(self):
        """Next settings to try, rounded to what the rover can be sent."""
        if not self.ready:
            return None
        centre = self._to_unit(self.best()[0])
        step = self.step_size()
        tried = {tuple(sorted(run.settings.items())) for run in self.runs}
        best_score, best_settings = -1.0, None
        # At least two of the rover's own steps either side, so a coarse value
        # (a whole duty %) still has untried neighbours after the box shrinks.
        widths = [max(step, 2 * d.step / (d.high - d.low)) for d in self.dimensions]
        for _ in range(CANDIDATES):
            unit = [min(max(c + self.rng.uniform(-w, w), 0.0), 1.0)
                    for c, w in zip(centre, widths)]
            settings = self._from_unit(unit)
            if tuple(sorted(settings.items())) in tried or not allowed(settings):
                continue
            score = self._expected_improvement(self._to_unit(settings))
            if score > best_score:
                best_score, best_settings = score, settings
        return best_settings
