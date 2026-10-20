"""Tests for the PC-side controller and the rover I/O protocol.
Run from this folder: python -m unittest test_pc_brain
"""

import unittest

import io_protocol as io
import pc_brain as pb
import protocol as p

WHITE = pb.DEFAULT_WHITE_LEVEL
BLACK = pb.DEFAULT_BLACK_LEVEL
GREY = (WHITE + BLACK) // 2


def sensor_frame(ir_left, ir_right, dac_left=128, dac_right=128, seq=0, ack=io.NO_SEQ):
    body = [io.FRAME_START, io.MSG_SENSORS, ir_left // 4, ir_right // 4, dac_left, dac_right,
            seq, ack]
    return bytes(body + [io.checksum(body)])


class ProtocolTests(unittest.TestCase):
    def test_motor_frame(self):
        frame = io.encode_motors(128, 255, seq=7)
        self.assertEqual(frame[:5], bytes([0xFF, io.CMD_MOTORS, 128, io.DATA_MAX, 7]))
        self.assertEqual(frame[5], io.checksum(frame[:5]))

    def test_parser_resyncs(self):
        parser = io.SensorParser()
        good = sensor_frame(400, 800)
        frames = parser.feed(b"\x01\xff\x30" + good + good[:3])
        self.assertEqual(len(frames), 1)
        self.assertEqual((frames[0].ir_left, frames[0].ir_right), (400, 800))
        self.assertEqual(len(parser.feed(good[3:])), 1)


class HealthTests(unittest.TestCase):
    def test_round_trip_and_lost_frames(self):
        health = pb.LinkHealth()
        health.motor_sent(5, 1.000)
        health.sensors_received(io.SensorFrame(0, 0, 128, 128, seq=10), 1.010)
        health.sensors_received(io.SensorFrame(0, 0, 128, 128, seq=13, ack=5), 1.040)
        snap = health.snapshot(1.040)
        self.assertAlmostEqual(snap.round_trip_ms, 40.0)
        self.assertEqual(snap.lost_frames, 2)      # seq 11 and 12 never came
        self.assertEqual(snap.rate_hz, 2 / pb.HEALTH_WINDOW_S)

    def test_seq_wraps(self):
        health = pb.LinkHealth()
        health.sensors_received(io.SensorFrame(0, 0, 128, 128, seq=io.SEQ_COUNT - 1), 0.0)
        health.sensors_received(io.SensorFrame(0, 0, 128, 128, seq=0), 0.025)
        self.assertEqual(health.snapshot(0.025).lost_frames, 0)


class BrainTests(unittest.TestCase):
    def setUp(self):
        self.brain = pb.Brain()
        self.t = 0.0

    def step(self, left, right):
        self.t += pb.NOMINAL_PERIOD_S
        return self.brain.step(left, right, self.t)

    def start(self, mode, controller):
        self.brain.handle_command(p.CMD_SET_MODE, mode, self.t)
        self.brain.handle_command(p.CMD_SET_CONTROLLER, controller, self.t)
        self.step(GREY, GREY)
        self.brain.handle_command(p.CMD_START, 0, self.t)

    def test_neutral_until_started(self):
        self.assertEqual(self.step(WHITE, BLACK), (128, 128))

    def test_bang_bang_uses_duty_levels(self):
        self.start(p.MODE_CLOCKWISE, p.CONTROLLER_BANG_BANG)
        tuning = self.brain.tuning[p.MODE_CLOCKWISE]
        straight = pb.speed_to_dac(pb.duty_to_speed(tuning.pwm_straight_left), 1)
        self.assertEqual(self.step(GREY, GREY), (straight, straight))
        # Right sensor on black: the left wheel slows, so the rover turns left.
        left, right = self.step(GREY, BLACK)
        self.assertLess(left, right)

    def test_pid_steers_towards_line(self):
        self.start(p.MODE_ANTICLOCKWISE, p.CONTROLLER_PID)
        left, right = self.step(WHITE + 100, BLACK - 100)   # Line to the left
        self.assertLess(left, right)
        left, right = self.step(BLACK - 100, WHITE + 100)   # Line to the right
        self.assertGreater(left, right)

    def test_lost_line_backtracks_then_spins(self):
        self.start(p.MODE_ANTICLOCKWISE, p.CONTROLLER_PID)
        for _ in range(5):
            self.step(GREY, GREY)
        forward = self.brain.left_percent
        self.step(BLACK, BLACK)
        self.assertTrue(self.brain.line_lost)
        self.assertEqual(self.brain.left_percent, -forward)    # Retracing
        for _ in range(10):
            self.step(BLACK, BLACK)
        self.assertEqual(self.brain.left_percent, -self.brain.right_percent)  # Spinning

    def test_stop_is_neutral(self):
        self.start(p.MODE_CLOCKWISE, p.CONTROLLER_BANG_BANG)
        self.step(GREY, GREY)
        self.brain.handle_command(p.CMD_STOP, 0, self.t)
        self.assertEqual((self.brain.dac_left, self.brain.dac_right), (128, 128))

    def test_short_dropout_is_ridden_through(self):
        self.start(p.MODE_ANTICLOCKWISE, p.CONTROLLER_PID)
        self.brain.handle_command(p.CMD_SET_KI, 50, self.t)
        for _ in range(5):
            self.step(WHITE + 100, BLACK - 100)
        self.assertNotEqual(self.brain.integral, 0.0)
        self.t += pb.LINK_LOST_S + 0.1
        self.brain.step(GREY, GREY, self.t)
        self.assertTrue(self.brain.running)
        self.assertEqual(self.brain.dropouts, 1)
        self.assertEqual(self.brain.filtered_derivative, 0.0)   # No kick across the gap
        self.assertEqual(self.brain.history, [(self.brain.left_percent,
                                               self.brain.right_percent)])

    def test_long_dropout_stops_run(self):
        self.start(p.MODE_CLOCKWISE, p.CONTROLLER_BANG_BANG)
        self.t += pb.LINK_GIVE_UP_S + 0.1
        self.assertEqual(self.brain.step(GREY, GREY, self.t), (128, 128))
        self.assertFalse(self.brain.running)


    def test_calibration_sets_levels(self):
        self.step(400, 400)
        self.brain.handle_command(p.CMD_CALIBRATE, 0, self.t)
        self.step(100, 100)
        self.step(800, 800)
        self.t += pb.CALIBRATION_S
        self.step(400, 400)
        self.assertFalse(self.brain.calibrating)
        self.assertEqual(self.brain.white_level, [100, 100])
        self.assertEqual(self.brain.black_level, [800, 800])

    def test_tuning_commands_echo(self):
        self.assertTrue(self.brain.handle_command(p.CMD_SET_KP, 90, self.t))
        self.assertEqual(self.brain.tuning_message().kp, 90)
        self.assertFalse(self.brain.handle_command(p.CMD_PING, 0, self.t))


class FakePort:
    is_open = True

    def __init__(self):
        self.written = []

    def write(self, data):
        self.written.append(bytes(data))


class LinkTests(unittest.TestCase):
    def setUp(self):
        self.link = pb.PcBrainLink()
        self.link.port = FakePort()

    def frame(self, seq, ir_left=GREY, ir_right=GREY):
        return io.SensorFrame(ir_left, ir_right, 128, 128, seq=seq)

    def test_heartbeat_and_tuning_stay_off_air(self):
        self.link._on_sensors([self.frame(0)])
        sent = len(self.link.port.written)
        self.link.send(p.CMD_PING)
        self.link.send(p.CMD_SET_KP, 70)
        self.assertEqual(len(self.link.port.written), sent)

    def test_stop_goes_on_air_at_once(self):
        self.link.send(p.CMD_START)
        self.link._on_sensors([self.frame(0)])
        self.link.send(p.CMD_STOP)
        self.assertEqual(self.link.port.written[-1][2:4], bytes([128, 128]))

    def test_bunched_frames_act_on_newest_only(self):
        self.link.send(p.CMD_START)
        self.link._on_sensors([self.frame(0, BLACK, BLACK), self.frame(1, WHITE, WHITE)])
        self.assertEqual(self.link.brain.raw, [WHITE, WHITE])
        self.assertEqual(self.link.health().stale_frames, 1)

    def test_unchanged_motors_only_resent_as_keepalive(self):
        self.link._on_sensors([self.frame(0)])
        sent = len(self.link.port.written)
        self.link._on_sensors([self.frame(1)])          # Same DACs, keepalive not due
        self.assertEqual(len(self.link.port.written), sent)
        self.link.sent_time -= pb.KEEPALIVE_S
        self.link._on_sensors([self.frame(2)])
        self.assertEqual(len(self.link.port.written), sent + 1)


if __name__ == "__main__":
    unittest.main()
