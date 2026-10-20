"""Checks for the scorecard in rover_gui_map.py. Run: python test_scorecard.py"""

import unittest

import protocol as p
import rover_gui_map as rgm

CW, SQ, ACW, REV = (p.MODE_CLOCKWISE, p.MODE_SQUIGGLE, p.MODE_ANTICLOCKWISE,
                    p.MODE_REVERSE)


class ScorecardTests(unittest.TestCase):
    def test_band_edges(self):
        self.assertEqual(rgm.marks_for(CW, 6.99), 5)
        self.assertEqual(rgm.marks_for(CW, 7.0), 4)      # "< 7 s" is strict
        self.assertEqual(rgm.marks_for(SQ, 16.9), 3)
        self.assertEqual(rgm.marks_for(ACW, 8.0), 2.5)
        self.assertEqual(rgm.marks_for(REV, 45), 0.5)

    def test_gap_gets_the_next_band_down(self):
        self.assertEqual(rgm.marks_for(CW, 17), 1)        # 16-18 s is not in the table
        self.assertEqual(rgm.marks_for(SQ, 26), 1)

    def test_no_clean_lap_scores_zero(self):
        self.assertEqual(rgm.marks_for(CW, None), 0)

    def test_maximum_is_15(self):
        self.assertEqual(sum(rgm.max_marks(m) for m in rgm.SCORECARD_ORDER), 15)

    def test_targets(self):
        self.assertEqual([rgm.top_mark_time(m) for m in (CW, SQ, ACW, REV)], [7, 10, 7, 14])

    def test_best_clean_lap_per_track(self):
        runs = [{"mode": "Clockwise", "time_s": 9.0, "clean": True},
                {"mode": "Clockwise", "time_s": 8.0, "clean": False},
                {"mode": "Clockwise", "time_s": 8.5, "clean": True, "track": "Competition"},
                {"mode": "Reverse", "time_s": 18.0, "clean": True, "track": "Test"}]
        self.assertEqual(rgm.best_clean_laps(runs, "Test"), {"Clockwise": 9.0, "Reverse": 18.0})
        self.assertEqual(rgm.best_clean_laps(runs, "Competition"), {"Clockwise": 8.5})


if __name__ == "__main__":
    unittest.main()
