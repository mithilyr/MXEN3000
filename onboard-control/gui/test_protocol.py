"""Checks for protocol.py. Run: python test_protocol.py"""

import unittest

import protocol as p


def telemetry_frame(ir_left=10, ir_right=200, dac_left=128, dac_right=140,
                    position=127, speed=45, status=p.STATUS_RUNNING, sensors=0):
    body = [p.FRAME_START, p.MSG_TELEMETRY, ir_left, ir_right, dac_left,
            dac_right, position, speed, status, sensors]
    return bytes(body + [p.checksum(body)])


def tuning_frame(mode=p.MODE_SQUIGGLE, controller=p.CONTROLLER_BANG_BANG, gain=105, ramp=60,
                 max_turn=70, kp=80, kd=15, ki=20, bang_level=35, pwm=(85, 80, 65, 60)):
    body = [p.FRAME_START, p.MSG_TUNING, mode, controller, gain, ramp, max_turn, kp, kd,
            ki, bang_level, *pwm]
    return bytes(body + [p.checksum(body)])


def debug_frame(seq=7, state=p.STATE_FOLLOW, terms=(-20, 0, 15, -5), saturated_pct=25,
                max_error=42, steps=20, step_max=13, period_max=56, lost=2):
    signed = [term + p.SIGNED_OFFSET for term in terms]
    body = [p.FRAME_START, p.MSG_DEBUG, seq, state, *signed, 0, saturated_pct, max_error,
            steps, step_max, period_max, lost, p.DEBUG_SATURATED]
    return bytes(body + [p.checksum(body)])


class ProtocolTests(unittest.TestCase):
    def test_command_frame(self):
        frame = p.encode_command(p.CMD_SET_SPEED, 50)
        self.assertEqual(frame, bytes([0xFF, 0x05, 50, (0xFF + 0x05 + 50) & 0xFF]))

    def test_full_scale_dac_survives_data_clamp(self):
        [t] = p.TelemetryParser().feed(telemetry_frame(dac_left=p.DATA_MAX))
        self.assertEqual(t.dac_left, p.DAC_FULL_SCALE)
        self.assertAlmostEqual(p.dac_to_percent(t.dac_left), 100.0)

    def test_decode_fields(self):
        status = p.STATUS_RUNNING | p.STATUS_LINE_LOST | (p.MODE_REVERSE << p.STATUS_MODE_SHIFT)
        [t] = p.TelemetryParser().feed(telemetry_frame(position=p.DATA_MAX, status=status))
        self.assertEqual(t.ir_left, 40)
        self.assertAlmostEqual(t.position, 1.0)
        self.assertTrue(t.running and t.line_lost)
        self.assertFalse(t.calibrating)
        self.assertEqual(t.mode, p.MODE_REVERSE)

    def test_resync_after_garbage_and_split_frames(self):
        parser = p.TelemetryParser()
        stream = b"\x01\x02" + telemetry_frame()[:6] + telemetry_frame() + telemetry_frame()
        messages = []
        for i in range(0, len(stream), 3):
            messages += parser.feed(stream[i:i + 3])
        self.assertEqual(len(messages), 2)
        self.assertEqual(parser.checksum_errors, 1)

    def test_tuning_frame_decodes_with_scaling(self):
        [t] = p.TelemetryParser().feed(tuning_frame())
        self.assertEqual(t.mode, p.MODE_SQUIGGLE)
        self.assertEqual(t.controller, p.CONTROLLER_BANG_BANG)
        self.assertAlmostEqual(t.adapt_gain, 10.5)
        self.assertEqual((t.adapt_ramp, t.max_turn, t.kp, t.ki), (60, 70, 80, 20))
        self.assertAlmostEqual(t.kd, 1.5)
        self.assertAlmostEqual(t.bang_level, 0.35)
        self.assertEqual((t.pwm_straight_left, t.pwm_straight_right,
                          t.pwm_correct_left, t.pwm_correct_right), (85, 80, 65, 60))

    def test_mixed_frame_types_in_one_stream(self):
        stream = telemetry_frame() + tuning_frame() + telemetry_frame()
        kinds = [type(m).__name__ for m in p.TelemetryParser().feed(stream)]
        self.assertEqual(kinds, ["Telemetry", "Tuning", "Telemetry"])

    def test_every_controller_has_tuning_values(self):
        for controller in p.CONTROLLER_NAMES:
            self.assertTrue(any(controller in param.controllers for param in p.TUNING_PARAMS))

    def test_tuning_limits_fit_in_a_data_byte(self):
        for param in p.TUNING_PARAMS:
            self.assertLessEqual(round(param.limit * param.scale), p.DATA_MAX)
        self.assertEqual(next(q for q in p.TUNING_PARAMS if q.key == "bang_level").limit, 1.0)

    def test_white_black_sensor_bits(self):
        [t] = p.TelemetryParser().feed(telemetry_frame(sensors=p.SENSOR_RIGHT_WHITE))
        self.assertFalse(t.left_white)
        self.assertTrue(t.right_white)

    def test_tuning_steps_fit_in_a_data_byte(self):
        for param in p.TUNING_PARAMS:
            data = p.encode_command(param.command, round(2 * param.step * param.scale))[2]
            self.assertLessEqual(data, p.DATA_MAX)

    def test_debug_frame_decodes_signed_terms_and_timing(self):
        [d] = p.TelemetryParser().feed(debug_frame())
        self.assertEqual((d.seq, d.state), (7, p.STATE_FOLLOW))
        self.assertEqual((d.p_term, d.i_term, d.d_term, d.turn), (-20, 0, 15, -5))
        self.assertEqual((d.adapt_rate, d.saturated_pct, d.steps, d.lost_events), (0, 25, 20, 2))
        self.assertAlmostEqual(d.max_error, 0.42)
        self.assertAlmostEqual(d.step_max_ms, 1.3)
        self.assertAlmostEqual(d.period_max_ms, 5.6)
        self.assertTrue(d.saturated)

    def test_calibration_frame(self):
        body = [p.FRAME_START, p.MSG_CALIBRATION, 15, 16, 180, 175, p.CAL_REJECTED]
        [c] = p.TelemetryParser().feed(bytes(body + [p.checksum(body)]))
        self.assertEqual((c.white_left, c.white_right, c.black_left, c.black_right),
                         (60, 64, 720, 700))
        self.assertEqual(c.result, p.CAL_REJECTED)
        self.assertAlmostEqual(p.normalised(390, 60, 720), 0.5)

    def test_all_four_frame_types_in_one_stream(self):
        stream = debug_frame() + telemetry_frame() + tuning_frame() + debug_frame(seq=8)
        kinds = [type(m).__name__ for m in p.TelemetryParser().feed(stream)]
        self.assertEqual(kinds, ["Debug", "Telemetry", "Tuning", "Debug"])

    def test_corrupt_checksum_rejected(self):
        bad = bytearray(telemetry_frame())
        bad[-1] ^= 0x01
        parser = p.TelemetryParser()
        self.assertEqual(parser.feed(bytes(bad)), [])
        self.assertEqual(parser.checksum_errors, 1)


if __name__ == "__main__":
    unittest.main()
