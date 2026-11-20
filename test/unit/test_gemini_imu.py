#!/usr/bin/env python3
"""Check gravity geometry and reject unusable calibration samples without hardware."""

import importlib.util
import math
from pathlib import Path
import unittest


SPEC = importlib.util.spec_from_file_location(
    "gemini_imu", Path(__file__).resolve().parents[2] / "tools/gemini_imu.py")
imu = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(imu)
IDENTITY = [[1., 0., 0.], [0., 1., 0.], [0., 0., 1.]]


class ImuTest(unittest.TestCase):
    def test_inclination_is_unsigned_and_scale_independent(self):
        self.assertEqual(imu.inclination([0., 9.8, 0.]), 0.)
        self.assertEqual(imu.inclination([0., 0., -9.8]), 90.)
        for sign in (-1, 1):
            self.assertAlmostEqual(imu.inclination([0., sign * math.sqrt(3), sign]), 30.)
        for vector in ([0., 0., 0.], [0., math.nan, 1.], [math.inf, 0., 1.]):
            with self.assertRaises(ValueError):
                imu.inclination(vector)

    def test_source_to_color_rotation_uses_rows(self):
        rows = [[i / 20., i * 50000, 1., 2., 3., 25.] for i in range(41)]
        result = imu.summarize(rows, [[0., 0., 1.], [0., 1., 0.], [-1., 0., 0.]])
        self.assertEqual(result["mean_color_xyz"], [3., 2., -1.])
        self.assertEqual(result["stddev_color_xyz"], [0., 0., 0.])
        self.assertEqual(result["sample_rate_hz"], 20.)
        rows[-1][2] = 5.
        self.assertGreater(imu.summarize(rows, IDENTITY)["stddev_color_xyz"][0], 0.)

    def test_rejects_missing_invalid_or_unordered_samples(self):
        rows = [[i / 20., i * 50000, 0., 9.8, 0., 25.] for i in range(41)]
        for invalid in ([], rows[:19], rows[:20], rows[::-1], rows + [rows[-1]],
                        rows[:-1] + [[2., 2000000, math.nan, 9.8, 0., 25.]]):
            with self.assertRaises(ValueError):
                imu.summarize(invalid, IDENTITY)

    def test_rejects_low_rate_short_coverage_and_large_gaps(self):
        good = [[i / 200., i * 5000, 0., 9.8, 0., 25.] for i in range(1601)]
        self.assertEqual(imu.summarize(good, IDENTITY, 200, 8)["sample_rate_hz"], 200.)
        gap = [row[:] for row in good]
        for row in gap[800:]:
            row[1] += 200000
        for invalid in (good[::80], good[:401], gap):
            with self.assertRaises(ValueError):
                imu.summarize(invalid, IDENTITY, 200, 8)


if __name__ == "__main__":
    unittest.main()
