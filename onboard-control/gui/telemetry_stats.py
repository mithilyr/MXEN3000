"""Link health and per-run tuning figures, worked out from the rover's frames.

Pure logic: time comes in as an argument, so it is tested without Tk or a rover.
"""

import math
from collections import deque
from dataclasses import dataclass

import protocol as p

LINK_WINDOW_S = 2.0             # Link figures cover the last 2 s
WEAVE_HYSTERESIS = 0.15         # |position| must pass this to count as a side change
MAX_FRAME_GAP_S = 0.3           # Longer gaps are a dropout, not time spent in a state
DEBUG_STALE_S = 2.0             # No debug frame for this long while telemetry arrives = old firmware


@dataclass
class LinkSnapshot:
    rate_hz: float          # Telemetry frames per second
    gap_max_ms: float       # Longest silence between telemetry frames
    lost: int               # Debug frames that never arrived (seq gaps)
    lost_total: int         # Since connect
    debug_frames: int       # Received since connect


class LinkStats:
    """Telemetry rate, gaps and lost debug frames over the last LINK_WINDOW_S."""

    def __init__(self):
        self.arrivals = deque()     # Telemetry arrival times
        self.gaps = deque()         # (time, gap s)
        self.lost = deque()         # (time, frames lost)
        self.lost_total = 0
        self.debug_frames = 0
        self.last_seq = None
        self.last_gap = None        # s between the last two telemetry frames

    def telemetry(self, now):
        if self.arrivals:
            self.last_gap = now - self.arrivals[-1]
            self.gaps.append((now, self.last_gap))
        self.arrivals.append(now)
        self._trim(now)

    def debug(self, seq, now):
        if self.last_seq is not None:
            missed = (seq - self.last_seq - 1) % p.SEQ_COUNT
            if missed:
                self.lost.append((now, missed))
                self.lost_total += missed
        self.last_seq = seq
        self.debug_frames += 1

    def _trim(self, now):
        cutoff = now - LINK_WINDOW_S
        while self.arrivals and self.arrivals[0] < cutoff:
            self.arrivals.popleft()
        for samples in (self.gaps, self.lost):
            while samples and samples[0][0] < cutoff:
                samples.popleft()

    def snapshot(self, now):
        self._trim(now)
        return LinkSnapshot(
            rate_hz=len(self.arrivals) / LINK_WINDOW_S,
            gap_max_ms=max((gap * 1000 for _, gap in self.gaps), default=0.0),
            lost=sum(count for _, count in self.lost),
            lost_total=self.lost_total,
            debug_frames=self.debug_frames)


@dataclass
class RunSummary:
    duration_s: float
    rms_error: float        # RMS of position over the telemetry samples
    max_error: float        # Largest |position| over every control step
    saturated_pct: float    # % of control steps with the steering clamped
    lost_events: int
    lost_s: float           # Time spent searching for the line
    weave_hz: float         # Side-to-side swings per second (10 Hz samples: up to ~5 Hz)
    step_max_ms: float
    period_max_ms: float    # Longest gap between control steps (5 ms target)
    frames_lost: int        # Debug frames lost on the radio during the run
    avg_speed: float = 0.0      # Mean of both wheels' speed, % (how hard the run pushed)
    avg_duty_left: float = 0.0  # Mean PWM duty per wheel, %
    avg_duty_right: float = 0.0
    min_duty_left: float = 0.0  # Lowest duty per wheel: how hard it braked a wheel to turn
    min_duty_right: float = 0.0
    gap_max_ms: float = 0.0     # Longest wait between telemetry frames during the run

    HEADER = ("rms_pos", "max_pos", "saturated_pct", "lost_events", "lost_s", "weave_hz",
              "step_max_ms", "period_max_ms", "frames_lost", "avg_speed", "avg_duty_l",
              "avg_duty_r", "min_duty_l", "min_duty_r", "gap_max_ms")

    def row(self):
        return [f"{self.rms_error:.3f}", f"{self.max_error:.2f}", f"{self.saturated_pct:.1f}",
                self.lost_events, f"{self.lost_s:.2f}", f"{self.weave_hz:.2f}",
                f"{self.step_max_ms:.1f}", f"{self.period_max_ms:.1f}", self.frames_lost,
                f"{self.avg_speed:.1f}", f"{self.avg_duty_left:.1f}",
                f"{self.avg_duty_right:.1f}", f"{self.min_duty_left:.1f}",
                f"{self.min_duty_right:.1f}", f"{self.gap_max_ms:.0f}"]


class RunStats:
    """Collects one run's telemetry and debug frames."""

    def __init__(self):
        self.square_sum = 0.0
        self.samples = 0
        self.side = 0               # -1 / +1: which side of the line, past the hysteresis
        self.side_changes = 0
        self.steps = 0
        self.saturated_steps = 0.0
        self.max_error = 0.0
        self.lost_events = 0
        self.lost_s = 0.0
        self.step_max_ms = 0.0
        self.period_max_ms = 0.0
        self.last_debug = None      # (time, Debug)
        self.first_seq = None
        self.debug_frames = 0
        self.last_seq = None
        self.duty_sum = [0.0, 0.0]
        self.duty_min = [100.0, 100.0]
        self.speed_sum = 0.0
        self.gap_max = 0.0

    def telemetry(self, t, gap_s=None):
        """gap_s: time since the previous telemetry frame, if known."""
        if gap_s is not None and self.samples:     # The first gap is from before the run
            self.gap_max = max(self.gap_max, gap_s)
        self.square_sum += t.position ** 2
        self.samples += 1
        for side, dac in enumerate((t.dac_left, t.dac_right)):
            duty = p.dac_to_percent(dac)
            self.duty_sum[side] += duty
            self.duty_min[side] = min(self.duty_min[side], duty)
        self.speed_sum += (p.dac_to_speed(t.dac_left) + p.dac_to_speed(t.dac_right)) / 2
        if abs(t.position) > WEAVE_HYSTERESIS:
            side = 1 if t.position > 0 else -1
            if self.side and side != self.side:
                self.side_changes += 1
            self.side = side

    def debug(self, d, now):
        if self.last_debug is not None:
            then, previous = self.last_debug
            gap = now - then
            if previous.state in p.LOST_STATES and gap < MAX_FRAME_GAP_S:
                self.lost_s += gap
        self.last_debug = (now, d)
        if self.first_seq is None:
            self.first_seq = d.seq
        self.last_seq = d.seq
        self.debug_frames += 1
        self.steps += d.steps
        self.saturated_steps += d.steps * d.saturated_pct / 100
        self.max_error = max(self.max_error, d.max_error)
        self.lost_events = d.lost_events
        self.step_max_ms = max(self.step_max_ms, d.step_max_ms)
        self.period_max_ms = max(self.period_max_ms, d.period_max_ms)

    def summary(self, duration_s):
        frames_lost = 0
        if self.debug_frames:
            expected = (self.last_seq - self.first_seq) % p.SEQ_COUNT + 1
            frames_lost = max(expected - self.debug_frames, 0)
        return RunSummary(
            duration_s=duration_s,
            rms_error=math.sqrt(self.square_sum / self.samples) if self.samples else 0.0,
            max_error=self.max_error,
            saturated_pct=100 * self.saturated_steps / self.steps if self.steps else 0.0,
            lost_events=self.lost_events,
            lost_s=self.lost_s,
            weave_hz=self.side_changes / 2 / duration_s if duration_s > 0 else 0.0,
            step_max_ms=self.step_max_ms,
            period_max_ms=self.period_max_ms,
            frames_lost=frames_lost,
            avg_speed=self.speed_sum / self.samples if self.samples else 0.0,
            avg_duty_left=self.duty_sum[0] / self.samples if self.samples else 0.0,
            avg_duty_right=self.duty_sum[1] / self.samples if self.samples else 0.0,
            min_duty_left=self.duty_min[0] if self.samples else 0.0,
            min_duty_right=self.duty_min[1] if self.samples else 0.0,
            gap_max_ms=self.gap_max * 1000)
