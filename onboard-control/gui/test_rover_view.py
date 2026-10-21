"""Checks for rover_view.py's helpers. Run: python test_rover_view.py"""

import unittest

import protocol as p
import rover_view as rv


class RoverViewHelperTests(unittest.TestCase):
    def test_dac_to_speed_matches_firmware(self):
        self.assertEqual(p.dac_to_speed(128), 0)
        self.assertAlmostEqual(p.dac_to_speed(255), 100)
        self.assertAlmostEqual(p.dac_to_speed(1), -100)

    def test_blend_ends(self):
        self.assertEqual(rv.blend("#000000", "#a0ff40", 0), "#000000")
        self.assertEqual(rv.blend("#000000", "#a0ff40", 1), "#a0ff40")
        self.assertEqual(rv.blend("#000000", "#a0ff40", 0.5), "#508020")

    def test_normalised_clamps(self):
        self.assertEqual(rv.normalised(0, 60, 720), 0.0)
        self.assertEqual(rv.normalised(1000, 60, 720), 1.0)
        self.assertEqual(rv.normalised(100, 50, 50), 0.0)


if __name__ == "__main__":
    unittest.main()
