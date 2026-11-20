"""Run a complete stationary-board stroke through the Python approach state machine."""

import math
from pathlib import Path
import sys
import unittest

import numpy as np

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / "src/thirdparty"))
from vision.arm_approach import ArmApproach
from vision.arm_tracking import CAMERA_FRAME, load_config


class VerifiedApproachTests(unittest.TestCase):
    def setUp(self):
        cfg = load_config(ROOT / "src/config/arm_tracking.json")
        cfg["approach_standoff_m"] = .065
        self.board_optical_x = cfg["approach_lateral_offset_m"]
        self.tracker = ArmApproach(cfg, verify_travel=True)
        self.start = [.14, .15, math.pi / 2]
        self.pose = self.start[:]
        self.time = 1.
        self.state = None
        self.image = np.zeros((480, 640, 3), dtype=np.uint8)
        self.k = np.array([[420., 0., 320.], [0., 420., 240.], [0., 0., 1.]])

    def tick(self, dx=0., dz=0., *, tags=True, imu=True, depth=True, old_imu=False):
        self.time += .035
        stamp = 1e3 * self.time
        z = .26 - dx
        frame = dict(color_bgr=self.image, timestamp_ms=stamp,
                     camera_matrix=self.k, distortion=np.zeros(8),
                     depth_m=np.full((480, 640), z) if depth else None,
                     depth_status="available" if depth else "missing_depth", base_tilt_rad=0.,
                     wrist_imu=dict(valid=imu, reason="stationary" if imu else "imu_missing",
                                    gravity_color=[0., 1., 0.], gyro_norm_rad_s=0.,
                                    start_ms=stamp - (350 if old_imu else 125), end_ms=stamp))
        codes = []
        if tags:
            for identity, center in enumerate((-.022, .022)):
                xy = np.array([[-.010, -.010], [.010, -.010], [.010, .010], [-.010, .010]])
                corners = (xy + [center + self.board_optical_x, dz]) * 420 / z + [320, 240]
                codes.append(dict(type="aruco", id=identity, dictionary="DICT_4X4_50",
                                  corners_px=corners.tolist()))
        detection = dict(valid=tags, codes=codes, device_timestamp_ms=stamp,
                         pair_indices=[0, 1] if tags else [],
                         position_3d=dict(valid=tags and depth, frame_id=CAMERA_FRAME,
                                          midpoint_m=[self.board_optical_x, dz, z]
                                          if tags and depth else None))
        return self.tracker.evaluate(detection, self.time, self.time, self.pose, self.time,
                                     frame=frame, capture_ns=round(self.time * 1e9),
                                     stroke_state=self.state)

    def acquire(self, imu=True):
        for _ in range(40):
            r = self.tick(imu=imu)
            if r["command_kind"] == "goal":
                return r
        self.fail("fixed goal was not acquired")

    def ack(self, stage):
        ns = round(self.time * 1e9)
        self.state = dict(stage=stage, settled_ns=self.state["settled_ns"] if self.state else ns,
                          observed_ns=ns)

    def test_visual_difference_despite_encoder_at_goal_then_independent_second_measurement(self):
        first = self.acquire()
        np.testing.assert_allclose(first["stroke"]["requested_m"], [.195, .022], atol=1e-9)
        self.pose = [first["goal_m"][0], first["goal_m"][2], self.start[2]]
        self.assertEqual(self.tick(tags=False, depth=False)["phase"], "approaching")
        self.ack("settled")
        for _ in range(12):
            r = self.tick(.195, .003, depth=False, old_imu=True)
            self.assertNotIn("correction_m", r)
        sent = [self.tick(.195, .003, depth=False) for _ in range(8)]
        commands = [r["correction_m"] for r in sent if "correction_m" in r]
        self.assertEqual(len(commands), 1, sent[-1])
        np.testing.assert_allclose(commands[0], [0., .019], atol=.0004)
        self.assertTrue(all(r["target_m"] == first["goal_m"] for r in sent))
        self.ack("correcting")
        self.assertEqual(self.tick(.195, .013)["phase"], "correcting")
        self.ack("complete")
        self.pose[1] += .019
        final = [self.tick(.195, .022, depth=False) for _ in range(9)]
        self.assertTrue(all("correction_m" not in r for r in final))
        self.assertEqual(final[-1]["reason"], "corrected_travel_within_tolerance", final[-1])
        measured = final[-1]["stroke"]["after_correction"]
        np.testing.assert_allclose(measured["travel_m"], [.195, .022], atol=.0004)

    def test_within_tolerance_closes_without_movement(self):
        self.acquire()
        self.ack("settled")
        records = [self.tick(.195, .022, depth=False) for _ in range(8)]
        self.assertEqual([r["correction_m"] for r in records if "correction_m" in r], [[0., 0.]])
        self.ack("complete")
        self.assertEqual(self.tick()["reason"], "travel_within_measurement_tolerance")

    def test_occlusion_missing_baseline_and_large_error_hold(self):
        for branch in ("occluded", "baseline_missing", "large_error"):
            with self.subTest(branch=branch):
                self.setUp()
                self.acquire(imu=branch != "baseline_missing")
                self.ack("settled")
                records = [self.tick(.155, .02, depth=False, tags=branch != "occluded")
                           for _ in range(66)]
                self.assertFalse(any("correction_m" in r for r in records))
                self.assertEqual(records[-1]["phase"], "complete", records[-1])
                self.assertEqual(records[-1]["reason"], {
                    "occluded": "travel_unverified", "baseline_missing": "travel_unverified_no_start_reference",
                    "large_error": "travel_error_exceeds_correction_limit",
                }[branch])

    def test_cancelled_stroke_cannot_reactivate(self):
        self.acquire()
        self.ack("cancelled")
        self.assertEqual(self.tick()["phase"], "cancelled")
        self.ack("settled")
        self.assertIsNone(self.tick()["target_m"])


if __name__ == "__main__":
    unittest.main()
