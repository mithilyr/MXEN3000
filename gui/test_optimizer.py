"""Checks for optimizer.py on a simulated track. Run: python test_optimizer.py"""

import random
import time
import unittest

import optimizer as opt
import protocol as p

PID = p.CONTROLLER_PID


def simulated_lap(settings):
    """Faster with speed, but too fast or too little steering leaves the line.
    The best clean lap is near speed 75, kp 90, kd 2.0."""
    speed, kp, kd = settings["speed"], settings["kp"], settings["kd"]
    grip = kp / 90 + kd / 2 - abs(kp - 90) / 60 - abs(kd - 2) / 2
    if speed > 50 + 25 * grip:
        return None                                     # Lost the line
    return 600 / speed + abs(kp - 90) / 40 + abs(kd - 2) * 0.8


def run(settings):
    lap = simulated_lap(settings)
    return opt.Run(dict(settings), lap or 0.0, lap is not None)


BASELINE = {"speed": 40, "kp": 60, "kd": 1.0, "ki": 0, "max_turn": 60}


class OptimiserTests(unittest.TestCase):
    def test_needs_a_clean_lap(self):
        failed = opt.Run(dict(BASELINE), 0.0, False)
        self.assertIsNone(opt.LapOptimiser(PID, [failed]).suggest())

    def test_suggestion_is_in_range_and_rounded(self):
        optimiser = opt.LapOptimiser(PID, [run(BASELINE)], random.Random(1))
        suggestion = optimiser.suggest()
        for dimension in opt.SEARCH_SPACE[PID]:
            value = suggestion[dimension.key]
            self.assertTrue(dimension.low <= value <= dimension.high)
            self.assertEqual(value, dimension.clip(value))

    def test_first_suggestion_is_a_small_step(self):
        optimiser = opt.LapOptimiser(PID, [run(BASELINE)], random.Random(4))
        suggestion = optimiser.suggest()
        for dimension in opt.SEARCH_SPACE[PID]:
            change = abs(dimension.to_unit(suggestion[dimension.key]) -
                         dimension.to_unit(BASELINE[dimension.key]))
            self.assertLessEqual(change, opt.START_STEP + dimension.step / (dimension.high - dimension.low))

    def test_step_shrinks_after_a_failure_and_grows_after_a_new_best(self):
        failed = opt.Run(dict(BASELINE, speed=50), 0.0, False)
        faster = opt.Run(dict(BASELINE, speed=45), 14.0, True)
        self.assertLess(opt.LapOptimiser(PID, [run(BASELINE), failed]).step_size(), opt.START_STEP)
        self.assertGreater(opt.LapOptimiser(PID, [run(BASELINE), faster]).step_size(), opt.START_STEP)

    def test_never_repeats_tried_settings(self):
        runs = [run(BASELINE)]
        optimiser = opt.LapOptimiser(PID, runs, random.Random(2))
        self.assertNotEqual(optimiser.suggest(), BASELINE)

    def test_runs_from_another_controller_are_ignored(self):
        bang = opt.Run({"speed": 40, "bang_level": 0.3, "max_turn": 60}, 12.0, True)
        self.assertFalse(opt.LapOptimiser(PID, [bang]).ready)

    def test_best_prefers_settings_away_from_failures(self):
        lucky = dict(BASELINE, speed=60)
        runs = [run(BASELINE), opt.Run(lucky, 9.0, True),
                opt.Run(dict(lucky, speed=61), 0.0, False),
                opt.Run(dict(lucky, speed=59), 0.0, False),
                opt.Run(dict(BASELINE, speed=45), 13.5, True)]
        settings, _, laps = opt.LapOptimiser(PID, runs).best()
        self.assertNotEqual(settings, lucky)
        self.assertEqual(laps, 1)

    def test_finds_a_faster_lap_in_20_runs(self):
        start = simulated_lap(BASELINE)
        rng = random.Random(3)
        runs = [run(BASELINE)]
        slowest_step = 0.0
        for _ in range(20):
            began = time.perf_counter()
            suggestion = opt.LapOptimiser(PID, runs, rng).suggest()
            slowest_step = max(slowest_step, time.perf_counter() - began)
            runs.append(run(suggestion))
        best_lap = min(r.time_s for r in runs if r.clean)
        self.assertLess(best_lap, start * 0.85)
        self.assertLess(slowest_step, 3.0)   # Fast enough to press G between runs


if __name__ == "__main__":
    unittest.main()
