"""Messages between the GUI and the PC-side controller (pc_brain.py).

In this build these frames never go over the radio: pc_brain.Brain takes the
CMD_* commands and produces the Telemetry / Tuning messages. The radio
frames are in io_protocol.py. The byte encoding is kept so the tests and the
log formats match the onboard-control build.

PC -> rover, 4 bytes:  [START][command][data][checksum]
Rover -> PC, 11 bytes: [START][TELEMETRY][irLeft][irRight][dacLeft]
                       [dacRight][position][speed][status][sensors][checksum]
Rover -> PC, 12 bytes: [START][TUNING][mode][controller][gain x10][ramp]
                       [maxTurn][kp][kd x10][ki][bangLevel x100][checksum]

checksum = sum of all previous bytes in the frame, & 0xFF. Data bytes are
never FRAME_START, so a receiver can re-synchronise on the next FRAME_START.
"""

from dataclasses import dataclass

FRAME_START = 0xFF
DATA_MAX = 0xFE
BYTE_MASK = 0xFF

# Commands (PC -> rover)
CMD_PING = 0x01
CMD_START = 0x02
CMD_STOP = 0x03
CMD_SET_MODE = 0x04
CMD_SET_SPEED = 0x05
CMD_CALIBRATE = 0x06
CMD_SET_ADAPT_GAIN = 0x09
CMD_SET_ADAPT_RAMP = 0x0A
CMD_SET_MAX_TURN = 0x0B
CMD_SET_KP = 0x0C
CMD_SET_KD = 0x0D
CMD_SET_KI = 0x0E
CMD_SET_BANG_LEVEL = 0x0F
CMD_SET_CONTROLLER = 0x10   # Also while running
CMD_RESET_INTEGRAL = 0x11
# Bang-bang wheel levels (current mode), as electrical PWM duty %: 100 = full
# forward, 50 = stopped, 0 = full reverse. data = duty %.
CMD_SET_PWM_STRAIGHT_LEFT = 0x12
CMD_SET_PWM_STRAIGHT_RIGHT = 0x13
CMD_SET_PWM_CORRECT_LEFT = 0x14
CMD_SET_PWM_CORRECT_RIGHT = 0x15
GAIN_SCALE = 10.0
KD_SCALE = 10.0
BANG_LEVEL_SCALE = 100.0

# Telemetry (rover -> PC)
MSG_TELEMETRY = 0x20
TELEMETRY_FRAME_LENGTH = 11
MSG_TUNING = 0x21
TUNING_FRAME_LENGTH = 16
FRAME_LENGTHS = {MSG_TELEMETRY: TELEMETRY_FRAME_LENGTH, MSG_TUNING: TUNING_FRAME_LENGTH}

STATUS_RUNNING = 0x01
STATUS_LINE_LOST = 0x02
STATUS_CALIBRATING = 0x04
STATUS_MODE_SHIFT = 4
STATUS_MODE_MASK = 0x07

# Sensors byte: 1 = sensor is over the white line, 0 = over black
SENSOR_LEFT_WHITE = 0x01
SENSOR_RIGHT_WHITE = 0x02

# Driving modes, in firmware order
MODE_CLOCKWISE = 0
MODE_ANTICLOCKWISE = 1
MODE_SQUIGGLE = 2
MODE_REVERSE = 3
MODE_NAMES = {
    MODE_CLOCKWISE: "Clockwise",
    MODE_ANTICLOCKWISE: "Anticlockwise",
    MODE_SQUIGGLE: "Squiggle",
    MODE_REVERSE: "Reverse",
}

# Steering controllers, in firmware order (one per mode)
CONTROLLER_PID = 0
CONTROLLER_BANG_BANG = 1
CONTROLLER_ADAPTIVE = 2
CONTROLLER_NAMES = {
    CONTROLLER_PID: "PID",
    CONTROLLER_BANG_BANG: "Bang-bang",
    CONTROLLER_ADAPTIVE: "Adaptive 1/0",
}

ADC_TO_BYTE_SCALE = 4  # Firmware sends the 10-bit ADC reading >> 2

# DAC0800 -> PWM -> push-pull stage: code 0 = -15 V, 128 ~ 0 V, 255 = +15 V at
# the motor (Lab 3 sweep). A data byte stops at DATA_MAX, so DATA_MAX on a telemetry
# DAC field stands for full scale (255). Code 254 is never used.
DAC_FULL_SCALE = 255
MOTOR_VOLTS_MAX = 15.0


def dac_to_volts(code):
    return code * 2 * MOTOR_VOLTS_MAX / DAC_FULL_SCALE - MOTOR_VOLTS_MAX


def dac_to_percent(code):
    """DAC code -> expected PWM duty cycle, % (0 = -15 V, 255 = +15 V)."""
    return code * 100.0 / DAC_FULL_SCALE


def dac_from_data(data):
    """Undo the DATA_MAX clamp on a DAC value received from the rover."""
    return DAC_FULL_SCALE if data == DATA_MAX else data


def checksum(frame_bytes):
    return sum(frame_bytes) & BYTE_MASK


def encode_command(command, data=0):
    """Build a 4-byte command frame. data is clamped to 0..DATA_MAX."""
    data = max(0, min(int(data), DATA_MAX))
    body = [FRAME_START, command, data]
    return bytes(body + [checksum(body)])


@dataclass
class Telemetry:
    ir_left: int        # Raw ADC, 0-1020
    ir_right: int
    dac_left: int       # 0-255, 128 = neutral
    dac_right: int
    position: float     # -1..1, > 0 means the line is to the left
    speed: int          # Base speed, % of full
    running: bool
    line_lost: bool
    calibrating: bool
    mode: int
    left_white: bool = False    # Steady 1/0 from the rover: True = on the white line
    right_white: bool = False


def decode_telemetry(frame):
    status = frame[8]
    return Telemetry(
        ir_left=frame[2] * ADC_TO_BYTE_SCALE,
        ir_right=frame[3] * ADC_TO_BYTE_SCALE,
        dac_left=dac_from_data(frame[4]),
        dac_right=dac_from_data(frame[5]),
        position=frame[6] * 2 / DATA_MAX - 1,
        speed=frame[7],
        running=bool(status & STATUS_RUNNING),
        line_lost=bool(status & STATUS_LINE_LOST),
        calibrating=bool(status & STATUS_CALIBRATING),
        mode=(status >> STATUS_MODE_SHIFT) & STATUS_MODE_MASK,
        left_white=bool(frame[9] & SENSOR_LEFT_WHITE),
        right_white=bool(frame[9] & SENSOR_RIGHT_WHITE),
    )


@dataclass
class Tuning:
    """The tuning values the rover is using for its current mode."""
    mode: int
    controller: int
    adapt_gain: float   # Adaptive controller: turn % x seconds
    adapt_ramp: float   # Adaptive controller: % turn per second
    max_turn: int       # % of full speed (bang-bang: the turn it uses)
    kp: float           # PID
    kd: float
    ki: float
    bang_level: float   # Bang-bang: |position| that triggers a full turn, 0-1
    pwm_straight_left: int = 50    # Bang-bang wheel levels, PWM duty %
    pwm_straight_right: int = 50
    pwm_correct_left: int = 50
    pwm_correct_right: int = 50


def decode_tuning(frame):
    return Tuning(
        mode=frame[2],
        controller=frame[3],
        adapt_gain=frame[4] / GAIN_SCALE,
        adapt_ramp=frame[5],
        max_turn=frame[6],
        kp=frame[7],
        kd=frame[8] / KD_SCALE,
        ki=frame[9],
        bang_level=frame[10] / BANG_LEVEL_SCALE,
        pwm_straight_left=frame[11],
        pwm_straight_right=frame[12],
        pwm_correct_left=frame[13],
        pwm_correct_right=frame[14],
    )


# The four bang-bang wheel levels: (Tuning field, command, label)
PWM_LEVELS = (
    ("pwm_straight_left", CMD_SET_PWM_STRAIGHT_LEFT, "Left wheel, line centred"),
    ("pwm_straight_right", CMD_SET_PWM_STRAIGHT_RIGHT, "Right wheel, line centred"),
    ("pwm_correct_left", CMD_SET_PWM_CORRECT_LEFT, "Left wheel, slowed to turn left"),
    ("pwm_correct_right", CMD_SET_PWM_CORRECT_RIGHT, "Right wheel, slowed to turn right"),
)


DECODERS = {MSG_TELEMETRY: decode_telemetry, MSG_TUNING: decode_tuning}


@dataclass(frozen=True)
class TuningParam:
    """One live-tunable value. "scale" converts the display value to the data byte."""
    key: str
    label: str
    command: int
    scale: float
    step: float         # Size of one + or - press, in display units
    controllers: tuple  # The controllers that use this value
    decimals: int
    maximum: float      # Largest display value

    @property
    def limit(self):
        return min(self.maximum, DATA_MAX / self.scale)


ALL_CONTROLLERS = tuple(CONTROLLER_NAMES)
TUNING_PARAMS = (
    TuningParam("kp", "Kp", CMD_SET_KP, 1, 5, (CONTROLLER_PID,), 0, DATA_MAX),
    TuningParam("kd", "Kd", CMD_SET_KD, KD_SCALE, 0.1, (CONTROLLER_PID,), 1, DATA_MAX),
    TuningParam("ki", "Ki", CMD_SET_KI, 1, 5, (CONTROLLER_PID,), 0, DATA_MAX),
    TuningParam("bang_level", "Bang level", CMD_SET_BANG_LEVEL, BANG_LEVEL_SCALE, 0.05,
                (CONTROLLER_BANG_BANG,), 2, 1.0),
    TuningParam("adapt_gain", "Turn gain", CMD_SET_ADAPT_GAIN, GAIN_SCALE, 0.5,
                (CONTROLLER_ADAPTIVE,), 1, DATA_MAX),
    TuningParam("adapt_ramp", "Ramp /s", CMD_SET_ADAPT_RAMP, 1, 5, (CONTROLLER_ADAPTIVE,), 0, DATA_MAX),
    TuningParam("max_turn", "Max turn %", CMD_SET_MAX_TURN, 1, 5, ALL_CONTROLLERS, 0, 100),
)


class TelemetryParser:
    """Turns a raw byte stream into Telemetry and Tuning objects, resyncing on errors."""

    def __init__(self):
        self.buffer = bytearray()
        self.checksum_errors = 0

    def feed(self, data):
        self.buffer.extend(data)
        messages = []
        while True:
            start = self.buffer.find(FRAME_START)
            if start < 0:
                self.buffer.clear()
                break
            del self.buffer[:start]
            if len(self.buffer) < 2:
                break
            length = FRAME_LENGTHS.get(self.buffer[1])
            if length is None:
                self.checksum_errors += 1
                del self.buffer[:1]
                continue
            if len(self.buffer) < length:
                break
            frame = self.buffer[:length]
            if checksum(frame[:-1]) == frame[-1]:
                messages.append(DECODERS[frame[1]](frame))
                del self.buffer[:length]
            else:
                self.checksum_errors += 1
                del self.buffer[:1]  # Drop this START and look for the next
        return messages
