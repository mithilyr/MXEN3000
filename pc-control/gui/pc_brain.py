"""The rover's controller, running on the PC.

The Nano (../firmware/rover_io/rover_io.ino) only streams raw IR readings and
writes the DAC codes it is sent. Everything that decides a duty cycle lives
here: sensor normalisation and calibration, the three steering controllers
(PID, bang-bang, adaptive), the lost-line backtrack and spin search, and the
per-mode tuning table. It is a port of the onboard-control firmware, so
the tuning numbers and behaviour carry over.

PcBrainLink is the GUI's link (open / close / send / messages / parser).
The GUI's commands (protocol.py CMD_*) go into Brain.handle_command, and the
rover's sensor frames come back to the GUI as Telemetry and Tuning messages.
Only io_protocol.py frames go over the radio.
"""

import math
import queue
import threading
import time                     # perf_counter: monotonic() ticks only every ~16 ms on Windows
import statistics
from collections import deque
from dataclasses import dataclass, replace

import serial

import io_protocol as io
import protocol as p
import xbee_setup as xb

# ---- Link ----
BAUD_RATE = xb.LINK_BAUD
READ_WAKE_TIMEOUT_S = 0.05      # Reader blocks until a byte arrives (no polling delay)
KEEPALIVE_S = 0.1               # Resend unchanged DAC codes this often (rover watchdog 0.3 s)
TELEMETRY_PERIOD_S = 0.1        # GUI telemetry rate, 10 Hz
TUNING_PERIOD_S = 1.0           # Slow tuning refresh; changes are echoed at once
# Radio dropouts while running. The rover's own watchdog has gone neutral after
# 0.3 s. A gap longer than LINK_LOST_S restarts the controller's memory but
# keeps the run going when frames return; longer than LINK_GIVE_UP_S ends it.
LINK_LOST_S = 0.5
LINK_GIVE_UP_S = 2.0
HEALTH_WINDOW_S = 2.0           # Link health figures cover the last 2 s

# ---- Motor output (DAC0800: 128 = 0 V, 255 = +15 V) ----
DAC_NEUTRAL = 128
DAC_SPAN = 127
LEFT_MOTOR_DIR = 1              # -1 if that wheel is wired the other way round
RIGHT_MOTOR_DIR = 1
SPEED_MAX = 100                 # Wheel speeds are % of DAC_SPAN

# ---- Sensors ----
ADC_MAX = 1023
WHITE_READS_HIGH = False        # The controller wants white = low
SENSORS_STRADDLE_LINE = True    # Adaptive only: True = sensors sit either side of the line
DEFAULT_WHITE_LEVEL = 60
DEFAULT_BLACK_LEVEL = 720
CALIBRATION_S = 3.0
MIN_CALIBRATION_SPAN = 100

# ---- Controller ----
NOMINAL_PERIOD_S = 0.025        # rover_io.ino SENSOR_PERIOD_MS
MIN_PERIOD_S = 0.005            # Clamp on the measured step time
MAX_PERIOD_S = 0.1
LOST_LEVEL = 0.75
SIDE_MEMORY_LEVEL = 0.2
WHITE_BELOW = 0.35
BLACK_ABOVE = 0.65
D_FILTER_TAU_S = 0.022          # Derivative low-pass time constant (about a 25 ms average)
MIN_CORRECTION_GAP_S = 0.02
CORRECTION_MEMORY_S = 1.0
MIN_CORRECTION_TURN = 15
BACKTRACK_SAMPLE_S = 0.02
BACKTRACK_SAMPLES = 30

TURN_CLOCKWISE = 1
TURN_ANTICLOCKWISE = -1
LEFT, RIGHT = 0, 1


@dataclass
class ModeTuning:
    travel_dir: int         # +1 forwards, -1 backwards
    default_turn: int       # Search direction before the line has been seen off-centre
    controller: int
    base_speed: int         # % of DAC_SPAN
    search_speed: int
    kp: float
    ki: float
    kd: float
    adapt_gain: float
    adapt_ramp: float
    max_turn: int
    bang_level: float
    steer_sign: int         # -1 if this mode steers the wrong way on test
    pwm_straight_left: int  # Bang-bang wheel levels, PWM duty %: 100 fwd, 50 stop, 0 rev
    pwm_straight_right: int
    pwm_correct_left: int
    pwm_correct_right: int


# Starting values, adapted from the onboard-control config.h for a loop that
# is about 8x slower and 35-55 ms late (one sensor period + the radio):
#   - Clockwise and Anticlockwise use bang-bang: it reacts to black/white
#     crossings, so delay hurts it least. Anticlockwise mirrors Clockwise.
#   - kd is halved everywhere: the derivative term suffers most from delay.
#   - Squiggle's PID base speed drops from 35 to 30 %.
# The GUI tunes them live; press "Copy for pc_brain.py" and paste the best ones here.
MODE_TUNING = (
    #          travel defaultTurn          controller             base search kp    ki   kd   gain  ramp  maxT bang steer strL strR corL corR
    ModeTuning(1,  TURN_CLOCKWISE,     p.CONTROLLER_BANG_BANG, 85, 30, 60.0, 0.0, 0.5, 8.0,  40.0, 20, 0.3, 1, 70, 70, 55, 55),  # Clockwise
    ModeTuning(1,  TURN_ANTICLOCKWISE, p.CONTROLLER_BANG_BANG, 85, 30, 60.0, 0.0, 0.5, 8.0,  40.0, 60, 0.3, 1, 70, 70, 55, 55),  # Anticlockwise
    ModeTuning(1,  TURN_CLOCKWISE,     p.CONTROLLER_PID,       30, 30, 80.0, 0.0, 0.8, 10.0, 60.0, 70, 0.3, 1, 65, 65, 50, 50),  # Squiggle
    ModeTuning(-1, TURN_ANTICLOCKWISE, p.CONTROLLER_PID,       25, 25, 50.0, 0.0, 0.8, 6.0,  30.0, 50, 0.3, 1, 35, 35, 50, 50),  # Reverse
)

TUNING_COMMANDS = {p.CMD_SET_MODE, p.CMD_SET_SPEED, p.CMD_SET_ADAPT_GAIN, p.CMD_SET_ADAPT_RAMP,
                   p.CMD_SET_MAX_TURN, p.CMD_SET_KP, p.CMD_SET_KD, p.CMD_SET_KI,
                   p.CMD_SET_BANG_LEVEL, p.CMD_SET_CONTROLLER, p.CMD_SET_PWM_STRAIGHT_LEFT,
                   p.CMD_SET_PWM_STRAIGHT_RIGHT, p.CMD_SET_PWM_CORRECT_LEFT,
                   p.CMD_SET_PWM_CORRECT_RIGHT}


def clamp(value, low, high):
    return max(low, min(high, value))


def speed_to_dac(speed_percent, motor_dir):
    speed_percent = clamp(speed_percent, -SPEED_MAX, SPEED_MAX)
    return int(DAC_NEUTRAL + motor_dir * speed_percent * DAC_SPAN / SPEED_MAX)


def duty_to_speed(duty):
    """PWM duty (100 = full forward, 50 = stopped, 0 = full reverse) -> wheel speed %."""
    return (clamp(duty, 0, 100) - 50) * 2


def normalise(raw, white, black):
    return clamp((raw - white) / (black - white), 0.0, 1.0)


class Brain:
    """All control decisions. Pure logic: time comes in as an argument."""

    def __init__(self):
        self.tuning = [replace(t) for t in MODE_TUNING]
        self.mode = p.MODE_CLOCKWISE
        self.base_speed = self.tuning[self.mode].base_speed
        self.running = False
        self.line_lost = False
        self.calibrating = False

        self.raw = [0, 0]
        self.white_level = [DEFAULT_WHITE_LEVEL] * 2
        self.black_level = [DEFAULT_BLACK_LEVEL] * 2
        self.calibration_min = [0, 0]
        self.calibration_max = [0, 0]
        self.calibration_start = 0.0
        self.norm = [0.0, 0.0]          # 0 = white, 1 = black
        self.white = [False, False]     # Steady 1/0: True = on the white line

        self.position = 0.0             # > 0: the line has moved to the left
        self.last_position = 0.0
        self.last_turn = TURN_CLOCKWISE

        self.integral = 0.0
        self.filtered_derivative = 0.0

        self.correcting = False
        self.last_correction_end = 0.0
        self.last_correction_dir = TURN_CLOCKWISE
        self.correction_rate = 0.0

        self.left_percent = 0
        self.right_percent = 0
        self.dac_left = DAC_NEUTRAL
        self.dac_right = DAC_NEUTRAL

        self.dropouts = 0               # Radio gaps ridden through
        self.history = []               # Recent (left %, right %) while following
        self.last_history = 0.0
        self.next_backtrack = 0.0
        self.last_step = None

    # ---- Motor output ----
    def set_wheel_speeds(self, left_percent, right_percent):
        self.left_percent = clamp(round(left_percent), -SPEED_MAX, SPEED_MAX)
        self.right_percent = clamp(round(right_percent), -SPEED_MAX, SPEED_MAX)
        self.dac_left = speed_to_dac(left_percent, LEFT_MOTOR_DIR)
        self.dac_right = speed_to_dac(right_percent, RIGHT_MOTOR_DIR)

    def hold_neutral(self):
        self.set_wheel_speeds(0, 0)

    # ---- Sensors ----
    def update_sensors(self, ir_left, ir_right):
        for side, reading in ((LEFT, ir_left), (RIGHT, ir_right)):
            self.raw[side] = ADC_MAX - reading if WHITE_READS_HIGH else reading
            self.norm[side] = normalise(self.raw[side], self.white_level[side],
                                        self.black_level[side])
            if self.norm[side] < WHITE_BELOW:
                self.white[side] = True
            elif self.norm[side] > BLACK_ABOVE:
                self.white[side] = False
        self.last_position = self.position
        self.position = self.norm[RIGHT] - self.norm[LEFT]

    # ---- Calibration: sweep the rover across the line by hand ----
    def start_calibration(self, now):
        if self.running:
            return
        self.hold_neutral()
        self.calibrating = True
        self.calibration_start = now
        self.calibration_min = list(self.raw)
        self.calibration_max = list(self.raw)

    def update_calibration(self, now):
        for side in (LEFT, RIGHT):
            self.calibration_min[side] = min(self.calibration_min[side], self.raw[side])
            self.calibration_max[side] = max(self.calibration_max[side], self.raw[side])
        if now - self.calibration_start < CALIBRATION_S:
            return
        self.calibrating = False
        if all(self.calibration_max[s] - self.calibration_min[s] >= MIN_CALIBRATION_SPAN
               for s in (LEFT, RIGHT)):
            self.white_level = list(self.calibration_min)
            self.black_level = list(self.calibration_max)

    # ---- Run control ----
    def start_run(self, now):
        if self.calibrating:
            return
        self.running = True
        self.history.clear()
        self.line_lost = False
        self.last_turn = self.tuning[self.mode].default_turn
        self.last_position = self.position
        self.integral = 0.0
        self.filtered_derivative = 0.0
        self.correcting = False
        self.last_correction_end = now
        self.last_correction_dir = self.last_turn
        self.correction_rate = 0.0

    def forget_after_dropout(self, now):
        """The rover sat at neutral during a radio gap, so anything the controller
        remembers from before it (integral, rate, backtrack history) is stale."""
        self.dropouts += 1
        self.integral = 0.0
        self.filtered_derivative = 0.0
        self.correcting = False
        self.last_correction_end = now
        self.correction_rate = 0.0
        self.history.clear()
        self.left_percent = self.right_percent = 0     # Bang-bang: no stale command to keep

    def stop_run(self):
        self.running = False
        self.hold_neutral()

    def set_mode(self, mode):
        if self.running or mode not in p.MODE_NAMES:
            return
        self.mode = mode
        self.base_speed = self.tuning[mode].base_speed

    def set_controller(self, controller, now):
        if controller >= len(p.CONTROLLER_NAMES):
            return
        self.tuning[self.mode].controller = controller
        self.integral = 0.0
        self.filtered_derivative = 0.0
        self.correcting = False
        self.last_correction_end = now
        self.correction_rate = 0.0

    def handle_command(self, command, data, now):
        """Applies one GUI command. Returns True if the tuning echo should go out now."""
        tuning = self.tuning[self.mode]
        if command == p.CMD_START:
            self.start_run(now)
        elif command == p.CMD_STOP:
            self.stop_run()
        elif command == p.CMD_SET_MODE:
            self.set_mode(data)
        elif command == p.CMD_SET_SPEED:
            self.base_speed = min(data, SPEED_MAX)
            tuning.base_speed = self.base_speed
        elif command == p.CMD_SET_ADAPT_GAIN:
            tuning.adapt_gain = data / p.GAIN_SCALE
        elif command == p.CMD_SET_ADAPT_RAMP:
            tuning.adapt_ramp = data
        elif command == p.CMD_SET_MAX_TURN:
            tuning.max_turn = min(data, SPEED_MAX)
        elif command == p.CMD_SET_KP:
            tuning.kp = data
        elif command == p.CMD_SET_KD:
            tuning.kd = data / p.KD_SCALE
        elif command == p.CMD_SET_KI:
            tuning.ki = data
        elif command == p.CMD_SET_BANG_LEVEL:
            tuning.bang_level = min(data / p.BANG_LEVEL_SCALE, 1.0)
        elif command == p.CMD_SET_CONTROLLER:
            self.set_controller(data, now)
        elif command == p.CMD_RESET_INTEGRAL:
            self.integral = 0.0
        elif command == p.CMD_CALIBRATE:
            self.start_calibration(now)
        elif command == p.CMD_SET_PWM_STRAIGHT_LEFT:
            tuning.pwm_straight_left = min(data, 100)
        elif command == p.CMD_SET_PWM_STRAIGHT_RIGHT:
            tuning.pwm_straight_right = min(data, 100)
        elif command == p.CMD_SET_PWM_CORRECT_LEFT:
            tuning.pwm_correct_left = min(data, 100)
        elif command == p.CMD_SET_PWM_CORRECT_RIGHT:
            tuning.pwm_correct_right = min(data, 100)
        return command in TUNING_COMMANDS

    # ---- Line follower ----
    def search_if_line_lost(self, line_missing, now, tuning):
        """Backtrack, then spin towards the side the line was last seen on.
        Returns True while it is handling a lost line."""
        if not line_missing:
            self.line_lost = False
            return False
        if not self.line_lost:
            self.line_lost = True
            self.next_backtrack = now
        if self.history:
            if now >= self.next_backtrack:
                self.next_backtrack = now + BACKTRACK_SAMPLE_S
                left, right = self.history.pop()
                self.set_wheel_speeds(-left, -right)
            return True
        spin = self.last_turn * tuning.search_speed
        self.set_wheel_speeds(spin, -spin)
        return True

    def start_correction(self, direction, now, tuning):
        gap = max(now - self.last_correction_end, MIN_CORRECTION_GAP_S)
        rate = tuning.adapt_gain / gap
        if direction == self.last_correction_dir and gap < CORRECTION_MEMORY_S:
            rate += self.correction_rate
        self.correction_rate = clamp(rate, MIN_CORRECTION_TURN, tuning.max_turn)
        self.last_correction_dir = direction
        self.correcting = True

    def update_adaptive(self, now, dt):
        tuning = self.tuning[self.mode]
        left_on, right_on = self.white
        both_off = not left_on and not right_on
        if self.search_if_line_lost(both_off and not SENSORS_STRADDLE_LINE, now, tuning):
            return

        forward = tuning.travel_dir * self.base_speed
        if left_on == right_on:
            if self.correcting:
                self.correcting = False
                self.last_correction_end = now
            self.set_wheel_speeds(forward, forward)
            return

        direction = TURN_ANTICLOCKWISE if left_on else TURN_CLOCKWISE
        self.last_turn = direction
        if not self.correcting or direction != self.last_correction_dir:
            self.start_correction(direction, now, tuning)
        else:
            self.correction_rate = min(self.correction_rate + tuning.adapt_ramp * dt,
                                       tuning.max_turn)
        turn = tuning.steer_sign * direction * self.correction_rate   # > 0 = clockwise
        self.set_wheel_speeds(forward + turn, forward - turn)

    def analog_line_lost(self, now, tuning):
        both_black = self.norm[LEFT] > LOST_LEVEL and self.norm[RIGHT] > LOST_LEVEL
        if self.search_if_line_lost(both_black, now, tuning):
            return True
        if self.position > SIDE_MEMORY_LEVEL:
            self.last_turn = TURN_ANTICLOCKWISE
        elif self.position < -SIDE_MEMORY_LEVEL:
            self.last_turn = TURN_CLOCKWISE
        return False

    def drive_with_turn(self, tuning, turn):
        turn = clamp(turn, -tuning.max_turn, tuning.max_turn)   # > 0 = clockwise
        forward = tuning.travel_dir * self.base_speed
        self.set_wheel_speeds(forward + turn, forward - turn)

    def update_pid(self, now, dt):
        tuning = self.tuning[self.mode]
        if self.analog_line_lost(now, tuning):
            self.integral = 0.0
            self.filtered_derivative = 0.0
            return

        derivative = (self.position - self.last_position) / dt
        alpha = 1 - math.exp(-dt / D_FILTER_TAU_S)
        self.filtered_derivative += alpha * (derivative - self.filtered_derivative)

        # Anti-windup: the integral term alone can never ask for more than max_turn.
        if tuning.ki > 0:
            limit = tuning.max_turn / tuning.ki
            self.integral = clamp(self.integral + self.position * dt, -limit, limit)
        else:
            self.integral = 0.0

        # A positive position (line to the left) needs an anticlockwise turn.
        turn = -tuning.steer_sign * (tuning.kp * self.position + tuning.ki * self.integral +
                                     tuning.kd * self.filtered_derivative)
        self.drive_with_turn(tuning, turn)

    def update_bang_bang(self):
        """Each wheel runs at its own duty; the wheel opposite a sensor that
        reads black drops to its correction duty. Both black: keep the last command."""
        tuning = self.tuning[self.mode]
        self.line_lost = False
        left_black = self.norm[LEFT] > BLACK_ABOVE
        right_black = self.norm[RIGHT] > BLACK_ABOVE
        if left_black and right_black and (self.left_percent or self.right_percent):
            return
        if tuning.steer_sign < 0:
            left_black, right_black = right_black, left_black
        self.set_wheel_speeds(
            duty_to_speed(tuning.pwm_correct_left if right_black else tuning.pwm_straight_left),
            duty_to_speed(tuning.pwm_correct_right if left_black else tuning.pwm_straight_right))

    def record_history(self, now):
        if now - self.last_history < BACKTRACK_SAMPLE_S:
            return
        self.last_history = now
        self.history.append((self.left_percent, self.right_percent))
        if len(self.history) > BACKTRACK_SAMPLES:
            del self.history[0]

    def update_line_follower(self, now, dt):
        controller = self.tuning[self.mode].controller
        if controller == p.CONTROLLER_BANG_BANG:
            self.update_bang_bang()
        elif controller == p.CONTROLLER_ADAPTIVE:
            self.update_adaptive(now, dt)
        else:
            self.update_pid(now, dt)
        if self.running and not self.line_lost:
            self.record_history(now)

    def step(self, ir_left, ir_right, now):
        """One control period, run for every sensor frame. Returns the DAC codes."""
        resumed = False
        if self.last_step is None:
            dt = NOMINAL_PERIOD_S
        else:
            gap = now - self.last_step
            if self.running and gap > LINK_GIVE_UP_S:
                self.stop_run()         # Too long: end the run
            elif self.running and gap > LINK_LOST_S:
                self.forget_after_dropout(now)
                resumed = True
            dt = clamp(gap, MIN_PERIOD_S, MAX_PERIOD_S)
        self.last_step = now

        self.update_sensors(ir_left, ir_right)
        if resumed:
            self.last_position = self.position  # No derivative kick across the gap
        if self.calibrating:
            self.update_calibration(now)
        elif self.running:
            self.update_line_follower(now, dt)
        return self.dac_left, self.dac_right

    # ---- Messages for the GUI (protocol.py Telemetry / Tuning) ----
    def telemetry(self, applied_left, applied_right):
        return p.Telemetry(
            ir_left=self.raw[LEFT], ir_right=self.raw[RIGHT],
            dac_left=applied_left, dac_right=applied_right,
            position=self.position, speed=self.base_speed,
            running=self.running, line_lost=self.line_lost,
            calibrating=self.calibrating, mode=self.mode,
            left_white=self.white[LEFT], right_white=self.white[RIGHT])

    def tuning_message(self):
        t = self.tuning[self.mode]
        return p.Tuning(
            mode=self.mode, controller=t.controller,
            adapt_gain=t.adapt_gain, adapt_ramp=t.adapt_ramp, max_turn=t.max_turn,
            kp=t.kp, kd=t.kd, ki=t.ki, bang_level=t.bang_level,
            pwm_straight_left=t.pwm_straight_left, pwm_straight_right=t.pwm_straight_right,
            pwm_correct_left=t.pwm_correct_left, pwm_correct_right=t.pwm_correct_right)


@dataclass
class Health:
    """How well the control loop is keeping up, over the last HEALTH_WINDOW_S."""
    rate_hz: float          # Control steps per second (= sensor frames received)
    round_trip_ms: float    # Median: motor frame sent -> rover reports it applied
    round_trip_max_ms: float
    step_ms: float          # Worst PC processing time per step
    lost_frames: int        # Sensor frames that never arrived
    stale_frames: int = 0   # Frames that arrived bunched up; only the newest was used
    dropouts: int = 0       # Radio gaps ridden through since connect


class LinkHealth:
    """Counts control steps, lost sensor frames and the radio round trip.

    The round trip includes waiting for the rover's next sensor frame, so it is
    the real age of a command by the time the PC sees it confirmed."""

    def __init__(self):
        self.steps = deque()        # Time of each control step
        self.trips = deque()        # (time, round trip s)
        self.step_times = deque()   # (time, PC processing s)
        self.lost = deque()         # (time, frames lost)
        self.stale = deque()        # (time, frames skipped as stale)
        self.sent_at = {}           # Motor seq -> time sent
        self.last_seq = None
        self.last_ack = io.NO_SEQ

    def motor_sent(self, seq, now):
        self.sent_at[seq] = now

    def sensors_received(self, frame, now):
        self.steps.append(now)
        if self.last_seq is not None:
            missed = (frame.seq - self.last_seq - 1) % io.SEQ_COUNT
            if missed:
                self.lost.append((now, missed))
        self.last_seq = frame.seq
        if frame.ack != self.last_ack and frame.ack in self.sent_at:
            self.trips.append((now, now - self.sent_at.pop(frame.ack)))
        self.last_ack = frame.ack
        self._trim(now)

    def step_done(self, seconds, now):
        self.step_times.append((now, seconds))

    def stale_skipped(self, count, now):
        if count:
            self.stale.append((now, count))

    def _trim(self, now):
        cutoff = now - HEALTH_WINDOW_S
        while self.steps and self.steps[0] < cutoff:
            self.steps.popleft()
        for samples in (self.trips, self.step_times, self.lost, self.stale):
            while samples and samples[0][0] < cutoff:
                samples.popleft()
        if len(self.sent_at) > io.SEQ_COUNT:   # Old frames that were never confirmed
            self.sent_at.clear()

    def snapshot(self, now):
        self._trim(now)
        trips = [trip * 1000 for _, trip in self.trips]
        return Health(
            rate_hz=len(self.steps) / HEALTH_WINDOW_S,
            round_trip_ms=statistics.median(trips) if trips else 0.0,
            round_trip_max_ms=max(trips, default=0.0),
            step_ms=max((s * 1000 for _, s in self.step_times), default=0.0),
            lost_frames=sum(count for _, count in self.lost),
            stale_frames=sum(count for _, count in self.stale))


class PcBrainLink:
    """The GUI's link: the controller runs here, on the PC."""

    def __init__(self):
        self.port = None
        self.parser = io.SensorParser()
        self.messages = queue.Queue()
        self.brain = Brain()
        self.lock = threading.Lock()
        self.reader = None
        self.reading = False
        self.last_telemetry = 0.0
        self.last_tuning = 0.0
        self.tuning_dirty = True
        self.motor_seq = 0
        self.link_health = LinkHealth()
        self.sent_dacs = None           # Last DAC codes written to the radio
        self.sent_time = 0.0

    @property
    def is_open(self):
        return self.port is not None and self.port.is_open

    def open(self, device):
        self.port = xb.open_port(device, BAUD_RATE)     # No Arduino reset on open
        self.port.timeout = READ_WAKE_TIMEOUT_S
        self.parser = io.SensorParser()
        self.link_health = LinkHealth()
        self.brain.dropouts = 0
        self.sent_dacs = None
        self.tuning_dirty = True
        self.reading = True
        self.reader = threading.Thread(target=self._read_loop, daemon=True)
        self.reader.start()

    def close(self):
        if self.is_open:
            with self.lock:
                self.brain.stop_run()
                self._send_motors(time.perf_counter(), force=True)
        self.reading = False
        if self.reader:
            self.reader.join(timeout=1)
        if self.is_open:
            self.port.close()
        self.port = None

    def send(self, command, data=0):
        """A GUI command: applied to the PC controller, not sent to the rover."""
        if not self.is_open:
            return False
        with self.lock:
            now = time.perf_counter()
            if self.brain.handle_command(command, int(data), now):
                self.tuning_dirty = True
            # Only a command that changed the motors (STOP, calibrate) goes on air
            # at once; heartbeats and tuning cost no radio traffic.
            return self._send_motors(now) is not False

    def _send_motors(self, now, force=False):
        """Writes the DAC codes if they changed or the keepalive is due.
        Returns None if nothing needed sending. Caller holds self.lock."""
        dacs = (self.brain.dac_left, self.brain.dac_right)
        if not force and dacs == self.sent_dacs and now - self.sent_time < KEEPALIVE_S:
            return None
        self.motor_seq = (self.motor_seq + 1) % io.SEQ_COUNT
        try:
            self.port.write(io.encode_motors(*dacs, self.motor_seq))
        except (serial.SerialException, AttributeError):
            return False
        self.sent_dacs = dacs
        self.sent_time = now
        self.link_health.motor_sent(self.motor_seq, now)
        return True

    def _read_loop(self):
        while self.reading:
            try:
                data = self.port.read(1)            # Wakes as soon as a byte arrives
                if data and self.port.in_waiting:
                    data += self.port.read(self.port.in_waiting)
            except serial.SerialException:
                self.reading = False
                break
            frames = self.parser.feed(data) if data else []
            if frames:
                self._on_sensors(frames)

    def _on_sensors(self, frames):
        """frames arrived together (the radio can bunch them up). Only the newest
        is acted on: the older ones describe where the rover was, not where it is."""
        now = time.perf_counter()
        frame = frames[-1]
        with self.lock:
            for each in frames:
                self.link_health.sensors_received(each, now)
            self.link_health.stale_skipped(len(frames) - 1, now)
            self.brain.step(frame.ir_left, frame.ir_right, now)
            self._send_motors(now)
            self.link_health.step_done(time.perf_counter() - now, now)
            if now - self.last_telemetry >= TELEMETRY_PERIOD_S:
                self.last_telemetry = now
                self.messages.put(self.brain.telemetry(frame.dac_left, frame.dac_right))
            if self.tuning_dirty or now - self.last_tuning >= TUNING_PERIOD_S:
                self.tuning_dirty = False
                self.last_tuning = now
                self.messages.put(self.brain.tuning_message())

    def health(self):
        """Link health for the GUI (safe from the GUI thread)."""
        with self.lock:
            return replace(self.link_health.snapshot(time.perf_counter()),
                           dropouts=self.brain.dropouts)
