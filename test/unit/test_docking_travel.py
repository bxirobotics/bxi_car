"""Rigid-board RGB-D acquisition and RGB/wrist-IMU endpoint metrology, offline."""

import math
from pathlib import Path
import sys
import unittest

import cv2
import numpy as np

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / "src/thirdparty"))
from vision.arm_tracking import load_config
from vision.docking_travel import DockingTravel, END_FRAMES, START_FRAMES


class TravelTests(unittest.TestCase):
    def setUp(self):
        self.travel = DockingTravel(load_config(ROOT / "src/config/arm_tracking.json"))
        self.k = np.array([[460., 0., 320.], [0., 462., 240.], [0., 0., 1.]])
        self.d = np.array([.015, -.02, .0001, -.0003, 0.])
        self.color = np.zeros((480, 640, 3), np.uint8)
        y, x = np.indices(self.color.shape[:2])
        pixels = np.stack((x, y), axis=-1).astype(float).reshape(-1, 1, 2)
        xy = cv2.undistortPoints(pixels, self.k, self.d).reshape(480, 640, 2)
        self.rays = np.dstack((xy, np.ones(x.shape)))
        self.normal = cv2.Rodrigues(np.deg2rad([4., 7., -3.]))[0]
        self.center = np.array([0., 0., .245])
        square = np.array([[-.012, -.012, 0.], [.012, -.012, 0.],
                           [.012, .012, 0.], [-.012, .012, 0.]])
        self.points = [self.center + (square + [side, 0., 0.]) @ self.normal.T
                       for side in (-.032, .032)]
        self.stamp = 0.

    def frame(self, dx=0., dz=0., ids=(0, 1), rotation=(0., 0., 0.), depth=True, dy=0.):
        self.stamp += 33.
        rot = cv2.Rodrigues(np.deg2rad(np.array(rotation, dtype=float)))[0]
        # Express the specified TCP translation directly in the original color frame.
        lever = np.array([0., .0625, 0.])
        camera = lever + np.array([-dy, -dz, dx]) - rot.T @ lever
        normal = rot @ self.normal[:, 2]
        plane_offset = np.dot(normal, rot @ (self.center - camera))
        frame = {"timestamp_ms": self.stamp, "color_bgr": self.color,
                 "camera_matrix": self.k.copy(), "distortion": self.d.copy(),
                 "depth_m": plane_offset / (self.rays @ normal) if depth else None, "base_tilt_rad": 0.,
                 "depth_status": "available" if depth else "missing_depth",
                 "wrist_imu": {"valid": True, "gravity_color": rot[:, 1].copy(),
                               "gyro_norm_rad_s": .002}}
        codes = []
        for tag in ids:
            xyz = (self.points[tag] - camera) @ rot.T
            corners = cv2.projectPoints(xyz, np.zeros(3), np.zeros(3), self.k, self.d)[0]
            codes.append({"type": "aruco", "dictionary": "DICT_4X4_50", "id": tag,
                          "corners_px": corners.reshape(4, 2).tolist()})
        return frame, {"valid": set(ids) == {0, 1}, "codes": codes,
                       "pair_indices": [0, 1] if set(ids) == {0, 1} else [],
                       "device_timestamp_ms": self.stamp}

    def lock(self):
        for _ in range(START_FRAMES):
            result = self.travel.observe_start(*self.frame(), math.pi / 2)
        self.assertTrue(result["valid"], result)

    def measure(self, **kwargs):
        for _ in range(END_FRAMES):
            result = self.travel.measure(*self.frame(**kwargs))
        return result

    def assert_travel(self, result, expected):
        self.assertTrue(result["valid"], result)
        np.testing.assert_allclose(result["travel_m"], expected, atol=.00015)
        self.assertGreater(result["uncertainty_m"], 0.)

    def test_fresh_stationary_pair_measures_both_axes_with_both_signs(self):
        self.lock()
        for dx, dz in ((.085, .019), (-.020, -.007), (0., 0.)):
            with self.subTest(dx=dx, dz=dz):
                self.travel.reset_endpoint()
                result = self.measure(dx=dx, dz=dz)
                self.assert_travel(result, [dx, dz])

    def test_one_complete_large_registered_tag_measures_without_endpoint_depth(self):
        self.lock()
        for tag in (0, 1):
            self.travel.reset_endpoint()
            result = self.measure(dx=.16, dz=.012, ids=(tag,), depth=False)
            self.assert_travel(result, [.16, .012])
            self.assertEqual(result["visible_ids"], [tag])

    def test_tilt_rotation_and_lever_do_not_masquerade_as_tcp_travel(self):
        self.lock()
        for angle, dx, dz in ((14., 0., 0.), (-8., .055, -.01)):
            self.travel.reset_endpoint()
            result = self.measure(dx=dx, dz=dz, rotation=(angle, 3., 1.))
            self.assert_travel(result, [dx, dz])

    def test_inverted_wrist_and_unstable_gravity_fail_closed(self):
        frame, det = self.frame()
        frame["wrist_imu"]["gravity_color"] *= -1
        result = self.travel.observe_start(frame, det, math.pi / 2)
        self.assertEqual(result["reason"], "wrist_mount_orientation_mismatch")
        self.lock()
        for field, bad in (("gravity_color", [0., -1., 0.]), ("gravity_color", [0., 0., 0.]),
                           ("gyro_norm_rad_s", .5), ("valid", False)):
            frame, det = self.frame(depth=False)
            frame["wrist_imu"][field] = bad
            with self.subTest(field=field, bad=bad):
                self.assertFalse(self.travel.measure(frame, det)["valid"])

    def test_missing_duplicated_small_and_inconsistent_tags_cannot_supply_a_measurement(self):
        self.lock()
        for tags, reason in (((), "no_docking_tag"), ((0, 0), "duplicate_docking_tag"),
                             ((1,), "single_tag_too_small")):
            frame, det = self.frame(dx=-.12, ids=tags)
            self.assertEqual(self.travel.measure(frame, det)["reason"], reason)
        frame, det = self.frame(dx=.12, depth=False)
        det["codes"][0]["corners_px"][0][1] += 7
        self.assertFalse(self.travel.measure(frame, det)["valid"])
        frame, det = self.frame(dx=.12)
        frame["depth_m"] += .025
        self.assertFalse(self.travel.measure(frame, det)["valid"])

    def test_start_needs_contiguous_coplanar_metric_support(self):
        for _ in range(START_FRAMES - 1):
            self.assertFalse(self.travel.observe_start(*self.frame(), math.pi / 2)["valid"])
        self.assertFalse(self.travel.observe_start(*self.frame(depth=False), math.pi / 2)["valid"])
        self.assertFalse(self.travel.observe_start(*self.frame(), math.pi / 2)["valid"])
        self.assertEqual(self.travel.measure(*self.frame())["reason"], "start_model_missing")
        frame, det = self.frame()
        frame["depth_m"][:, 320:] += .05
        self.assertFalse(self.travel.observe_start(frame, det, math.pi / 2)["valid"])
        frame, det = self.frame()
        det["pair_indices"] = [1]
        self.assertEqual(self.travel.observe_start(frame, det, math.pi / 2)["reason"], "start_pair_missing")
        self.lock()
        self.stamp += 500.
        self.assertFalse(self.travel.observe_start(*self.frame(), math.pi / 2)["valid"])
        self.assertEqual(self.travel.measure(*self.frame())["reason"], "start_model_missing")

    def test_close_single_tag_with_noisy_corners_rejects_loss_then_reacquires(self):
        self.lock()
        rng = np.random.default_rng(9)
        for i in range(END_FRAMES + 1):
            frame, det = self.frame(dx=.16, dz=.012, ids=(0,), depth=False)
            corners = np.array(det["codes"][0]["corners_px"])
            det["codes"][0]["corners_px"] = (corners + rng.normal(0, .20, (4, 2))).tolist()
            result = self.travel.measure(frame, det)
            if i == END_FRAMES - 1:
                self.assertTrue(result["valid"], result)
                error = np.linalg.norm(np.array(result["travel_m"]) - [.16, .012])
                self.assertLess(error, result["uncertainty_m"])
                self.assertFalse(self.travel.measure(*self.frame(ids=(), depth=False))["valid"])
        self.assertEqual(result["reason"], "acquiring_endpoint")

    def test_reference_distance_and_near_dead_zone_use_measurement_uncertainty(self):
        # Representative dimensions are supplied only by this scene generator.
        # The estimator learns all sizes from its initial depth frames.
        self.center = np.array([.005, 0., .290])
        square = np.array([[-.01,-.01,0.], [.01,-.01,0.], [.01,.01,0.], [-.01,.01,0.]])
        self.points = [self.center + (square + [x,0,0]) @ self.normal.T for x in (-.022,.022)]
        for _ in range(START_FRAMES):
            frame, det = self.frame()
            frame["base_tilt_rad"] = math.radians(.26)
            result = self.travel.observe_start(frame, det, math.pi / 2)
        self.assertTrue(result["valid"], result)
        result = self.measure(dx=.250, ids=(0,), depth=False)
        self.assert_travel(result, [.250, 0.])
        self.assertLess(result["uncertainty_m"], .010)
        self.travel.reset_endpoint()
        result = self.measure(dx=.170, ids=(1,), depth=False)
        self.assertFalse(result["valid"])
        self.assertEqual(result["reason"], "travel_uncertainty_excessive")

    def test_signed_wrist_levels_biased_encoder_pitch_and_rejects_chassis_tilt(self):
        for tilt in (None, math.radians(4.), float("nan")):
            frame, det = self.frame()
            frame["base_tilt_rad"] = tilt
            self.assertFalse(self.travel.observe_start(frame, det, math.radians(96.))["valid"])
        for _ in range(START_FRAMES):
            frame, det = self.frame()
            frame["base_tilt_rad"] = math.radians(.26)
            start = self.travel.observe_start(frame, det, math.radians(96.))
        self.assertTrue(start["valid"], start)
        self.assert_travel(self.measure(dx=.12), [.12, 0.])
        self.assertEqual(self.measure(dx=.12, dy=.024)["reason"], "unexpected_lateral_travel")

    def test_stale_frames_changes_of_profile_and_movement_break_endpoint_window(self):
        self.lock()
        frame, det = self.frame(dx=.08)
        self.assertEqual(self.travel.measure(frame, det)["reason"], "acquiring_endpoint")
        self.assertEqual(self.travel.measure(frame, det)["reason"], "invalid_frame_timestamp")
        frame, det = self.frame(dx=.08)
        frame["camera_matrix"][0, 0] += 5
        self.assertEqual(self.travel.measure(frame, det)["reason"], "camera_calibration_changed")
        for dx in (.02, .03, .04, .05):
            self.assertEqual(self.travel.measure(*self.frame(dx=dx))["reason"], "acquiring_endpoint")
        self.stamp += 500.
        self.assertFalse(self.travel.measure(*self.frame(dx=.05))["valid"])
        self.assert_travel(self.measure(dx=.05), [.05, 0.])


if __name__ == "__main__":
    unittest.main()
