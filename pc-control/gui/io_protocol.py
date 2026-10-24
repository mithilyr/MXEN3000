"""Radio frames between the PC and the rover I/O firmware.

Keep every constant here identical to ../firmware/rover_io/rover_io.ino.

Rover -> PC, 9 bytes: [START][MSG_SENSORS][irLeft][irRight][dacLeft][dacRight]
                      [seq][ackSeq][checksum]
PC -> rover, 6 bytes: [START][CMD_MOTORS][dacLeft][dacRight][seq][checksum]

checksum = sum of all previous bytes in the frame, & 0xFF. Data bytes are
never FRAME_START (clamped to DATA_MAX), so a receiver re-synchronises on the
next FRAME_START. On a DAC field DATA_MAX stands for full scale (255).
seq counts 0..SEQ_COUNT-1 on each side; ackSeq is the seq of the last motor
frame the rover applied (NO_SEQ before the first one).
"""

from dataclasses import dataclass

FRAME_START = 0xFF
DATA_MAX = 0xFE
BYTE_MASK = 0xFF

MSG_SENSORS = 0x30
CMD_MOTORS = 0x31
SENSOR_FRAME_LENGTH = 9
MOTOR_FRAME_LENGTH = 6
SEQ_COUNT = 254             # seq runs 0..253
NO_SEQ = DATA_MAX

MAX_BUFFER_BYTES = SENSOR_FRAME_LENGTH * 20   # Cap on unparsed bytes (about half a second of frames)
ADC_TO_BYTE_SCALE = 4       # The rover sends the 10-bit ADC reading >> 2
DAC_FULL_SCALE = 255


def checksum(frame_bytes):
    return sum(frame_bytes) & BYTE_MASK


def dac_to_data(code):
    return max(0, min(int(code), DATA_MAX))


def dac_from_data(data):
    return DAC_FULL_SCALE if data == DATA_MAX else data


def encode_motors(dac_left, dac_right, seq=0):
    """Motor frame carrying the two DAC codes (0-255, 128 = neutral)."""
    body = [FRAME_START, CMD_MOTORS, dac_to_data(dac_left), dac_to_data(dac_right),
            seq % SEQ_COUNT]
    return bytes(body + [checksum(body)])


@dataclass
class SensorFrame:
    ir_left: int        # Raw ADC, 0-1016
    ir_right: int
    dac_left: int       # DAC code the rover has applied, 0-255
    dac_right: int
    seq: int = 0        # This frame's number
    ack: int = NO_SEQ   # seq of the last motor frame the rover applied


def decode_sensors(frame):
    return SensorFrame(
        ir_left=frame[2] * ADC_TO_BYTE_SCALE,
        ir_right=frame[3] * ADC_TO_BYTE_SCALE,
        dac_left=dac_from_data(frame[4]),
        dac_right=dac_from_data(frame[5]),
        seq=frame[6],
        ack=frame[7],
    )


class SensorParser:
    """Turns the raw byte stream into SensorFrames, resyncing on errors."""

    def __init__(self):
        self.buffer = bytearray()
        self.checksum_errors = 0
        self.overflows = 0          # Times the buffer hit MAX_BUFFER_BYTES

    def feed(self, data):
        self.buffer.extend(data)
        if len(self.buffer) > MAX_BUFFER_BYTES:
            # Far behind the radio: the oldest bytes describe the past, so keep
            # only the newest and resync on the next FRAME_START.
            del self.buffer[:-MAX_BUFFER_BYTES]
            self.overflows += 1
        frames = []
        while True:
            start = self.buffer.find(FRAME_START)
            if start < 0:
                self.buffer.clear()
                break
            del self.buffer[:start]
            if len(self.buffer) < SENSOR_FRAME_LENGTH:
                break
            frame = self.buffer[:SENSOR_FRAME_LENGTH]
            if frame[1] == MSG_SENSORS and checksum(frame[:-1]) == frame[-1]:
                frames.append(decode_sensors(frame))
                del self.buffer[:SENSOR_FRAME_LENGTH]
            else:
                self.checksum_errors += 1
                del self.buffer[:1]     # Drop this START and look for the next
        return frames
