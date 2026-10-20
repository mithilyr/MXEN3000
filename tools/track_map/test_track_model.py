"""Checks for track_model.py. Run: python test_track_model.py"""

import math
import unittest

import track_model as tm

TRACK = tm.Track()
CW = tm.DIVISIONS["Clockwise"]
ACW = tm.DIVISIONS["Anticlockwise"]
REV = tm.DIVISIONS["Reverse"]
SAMPLE_S = 0.1
STRAIGHT = 191           # 75 % duty -> half speed forwards
SLOW = 140               # 55 % duty
RADIUS = (tm.OUTER_WIDTH - tm.LINE_WIDTH) / 2


def drive(seconds, left, right, start=0.0):
    count = round(seconds / SAMPLE_S)
    return [(start + i * SAMPLE_S, left, right, 0) for i in range(count)]


class TrackTests(unittest.TestCase):
    def test_lap_length(self):
        expected = 2 * tm.STRAIGHT_LENGTH + 2 * math.pi * RADIUS
        self.assertAlmostEqual(TRACK.length, expected, delta=0.1)

    def test_clockwise_starts_on_the_top_straight_going_right(self):
        x0, y0 = TRACK.point(0)
        x1, y1 = TRACK.point(10)
        self.assertAlmostEqual(y0, RADIUS)
        self.assertAlmostEqual(y1, RADIUS)
        self.assertGreater(x1, x0)

    def test_anticlockwise_starts_on_the_top_straight_going_left(self):
        x0, y0 = TRACK.point(0, reverse_path=True)
        x1, y1 = TRACK.point(10, reverse_path=True)
        self.assertAlmostEqual(y0, RADIUS)
        self.assertLess(x1, x0)

    def test_a_lap_ends_where_it_started(self):
        for reverse in (False, True):
            start = TRACK.point(0, reverse)
            end = TRACK.point(TRACK.length, reverse)
            self.assertAlmostEqual(start[0], end[0], places=3)
            self.assertAlmostEqual(start[1], end[1], places=3)

    def test_nearest_inverts_point(self):
        for distance in (10, 150, 200, 300, 400):
            for reverse in (False, True):
                x, y = TRACK.point(distance, reverse)
                self.assertAlmostEqual(TRACK.nearest(x, y, reverse), distance, places=3)

    def test_sections(self):
        bend = math.pi * RADIUS
        self.assertEqual(TRACK.section(10), "Straight 1")
        self.assertEqual(TRACK.section(tm.STRAIGHT_LENGTH + bend / 2), "Bend 1")
        self.assertEqual(TRACK.section(tm.STRAIGHT_LENGTH + bend + 10), "Straight 2")
        self.assertEqual(TRACK.section(TRACK.length - 5), "Bend 2")
        self.assertEqual(TRACK.section(TRACK.length + 10), "Straight 1")   # Second lap


class EstimatorTests(unittest.TestCase):
    def test_straight_drive_distance(self):
        states = tm.estimate(TRACK, CW, drive(2.0, STRAIGHT, STRAIGHT), 40.0)
        expected = 40.0 * tm.wheel_share(STRAIGHT) * (2.0 - SAMPLE_S)
        self.assertAlmostEqual(states[-1].distance, expected, places=3)

    def test_reverse_drives_backwards_but_progresses(self):
        back = 2 * tm.DAC_NEUTRAL - STRAIGHT
        states = tm.estimate(TRACK, REV, drive(2.0, back, back), 40.0)
        self.assertGreater(states[-1].distance, 30)

    def test_keeps_going_past_one_lap(self):
        states = tm.estimate(TRACK, CW, drive(60.0, 255, 255), 50.0)
        self.assertGreater(states[-1].distance, TRACK.length)

    def test_snaps_to_both_bends(self):
        # Speed scale 15 % too low: straights at half speed, bends with the right wheel slowed.
        true_speed = 40.0
        straight_s = tm.STRAIGHT_LENGTH / (true_speed * tm.wheel_share(STRAIGHT))
        bend_cm = TRACK.bends[0][1] - TRACK.bends[0][0]
        bend_s = bend_cm / (true_speed * (tm.wheel_share(STRAIGHT) + tm.wheel_share(SLOW)) / 2)
        rows = []
        for left, right, seconds in ((STRAIGHT, STRAIGHT, straight_s), (STRAIGHT, SLOW, bend_s),
                                     (STRAIGHT, STRAIGHT, straight_s), (STRAIGHT, SLOW, bend_s),
                                     (STRAIGHT, STRAIGHT, 2.0)):
            rows += drive(seconds, left, right, rows[-1][0] + SAMPLE_S if rows else 0.0)
        states = tm.estimate(TRACK, CW, rows, true_speed * 0.85)
        snaps = [s.snapped for s in states if s.snapped]
        self.assertEqual(snaps, ["bend entry", "bend exit", "bend entry", "bend exit"])

    def test_wrong_lean_does_not_count_as_a_bend(self):
        rows = drive(4.0, STRAIGHT, STRAIGHT)
        rows += drive(4.0, SLOW, STRAIGHT, rows[-1][0] + SAMPLE_S)   # Turning anticlockwise
        states = tm.estimate(TRACK, CW, rows, 40.0)
        self.assertFalse(any(s.in_bend for s in states))

    def test_anticlockwise_lean(self):
        rows = drive(3.0, STRAIGHT, STRAIGHT)
        rows += drive(4.0, SLOW, STRAIGHT, rows[-1][0] + SAMPLE_S)
        states = tm.estimate(TRACK, ACW, rows, 40.0)
        self.assertTrue(any(s.in_bend for s in states))

    def test_calibration_puts_the_rover_where_clicked(self):
        states = tm.estimate(TRACK, CW, drive(3.0, STRAIGHT, STRAIGHT), 10.0, snap=False)
        true_x, true_y = TRACK.point(50.0)
        scale = tm.calibrated_scale(TRACK, CW, states[-1], true_x, true_y)
        again = tm.estimate(TRACK, CW, drive(3.0, STRAIGHT, STRAIGHT), scale, snap=False)
        self.assertAlmostEqual(again[-1].distance, 50.0, places=3)

    def test_calibration_after_a_full_lap(self):
        rows = drive(40.0, STRAIGHT, STRAIGHT)
        states = tm.estimate(TRACK, CW, rows, 23.0, snap=False)   # Estimate ~ 1.04 laps
        true_x, true_y = TRACK.point(0.0)                         # Really back at the start
        scale = tm.calibrated_scale(TRACK, CW, states[-1], true_x, true_y)
        again = tm.estimate(TRACK, CW, rows, scale, snap=False)
        self.assertAlmostEqual(again[-1].distance, TRACK.length, places=1)

    def test_calibration_refused_at_the_start(self):
        states = tm.estimate(TRACK, CW, drive(0.1, STRAIGHT, STRAIGHT), 40.0)
        self.assertIsNone(tm.calibrated_scale(TRACK, CW, states[-1], 0.0, RADIUS))


if __name__ == "__main__":
    unittest.main()
