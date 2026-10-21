"""Checks for telemetry_stats.py. Run: python test_telemetry_stats.py"""

import unittest

import protocol as p
import telemetry_stats as ts
from test_protocol import debug_frame


def telemetry(position):
    return p.Telemetry(ir_left=0, ir_right=0, dac_left=128, dac_right=128, position=position,
                       speed=50, running=True, line_lost=False, calibrating=False, mode=0)


def debug(**fields):
    [d] = p.TelemetryParser().feed(debug_frame(**fields))
    return d


class LinkStatsTests(unittest.TestCase):
    def test_rate_gap_and_lost_debug_frames(self):
        link = ts.LinkStats()
        for i in range(10):
            link.telemetry(i * 0.1)
        link.telemetry(1.4)                      # 0.5 s silence
        for seq in (250, 251, 253, 1):           # 252, 0 lost across the wrap
            link.debug(seq, 1.4)
        snap = link.snapshot(1.4)
        self.assertAlmostEqual(snap.rate_hz, 11 / ts.LINK_WINDOW_S)
        self.assertAlmostEqual(snap.gap_max_ms, 500, places=3)
        self.assertEqual((snap.lost, snap.lost_total, snap.debug_frames), (2, 2, 4))
        self.assertAlmostEqual(link.last_gap, 0.5)

    def test_old_figures_leave_the_window(self):
        link = ts.LinkStats()
        link.debug(0, 0.0)
        link.debug(5, 0.1)
        self.assertEqual(link.snapshot(5.0).lost, 0)
        self.assertEqual(link.snapshot(5.0).lost_total, 4)


class RunStatsTests(unittest.TestCase):
    def test_summary(self):
        stats = ts.RunStats()
        for position in (0.3, -0.3, 0.3, -0.3, 0.0):     # 3 side changes
            stats.telemetry(telemetry(position))
        stats.debug(debug(seq=0, saturated_pct=50, steps=20, max_error=60), 0.0)
        stats.debug(debug(seq=1, state=p.STATE_SPIN, saturated_pct=0, steps=20, lost=1), 0.1)
        stats.debug(debug(seq=3, state=p.STATE_FOLLOW, steps=20, lost=1,
                          period_max=70), 0.2)            # seq 2 lost; 0.1 s in SPIN
        summary = stats.summary(1.5)
        self.assertAlmostEqual(summary.rms_error, (4 * 0.09 / 5) ** 0.5)
        self.assertAlmostEqual(summary.max_error, 0.6)
        self.assertAlmostEqual(summary.saturated_pct, (10 + 0 + 5) / 60 * 100)
        self.assertEqual((summary.lost_events, summary.frames_lost), (1, 1))
        self.assertAlmostEqual(summary.lost_s, 0.1)
        self.assertAlmostEqual(summary.weave_hz, 1.5 / 1.5)
        self.assertAlmostEqual(summary.period_max_ms, 7.0)
        self.assertEqual(len(summary.row()), len(ts.RunSummary.HEADER))
        self.assertAlmostEqual(summary.avg_speed, 0.0)       # All samples at DAC 128
        self.assertAlmostEqual(summary.avg_duty_left, 128 * 100 / 255)

    def test_empty_run(self):
        summary = ts.RunStats().summary(0.0)
        self.assertEqual((summary.rms_error, summary.weave_hz, summary.frames_lost), (0, 0, 0))


if __name__ == "__main__":
    unittest.main()
