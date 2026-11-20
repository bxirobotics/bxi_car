"""Measure TCP travel relative to a stationary, rigid ArUco board after settling.

The baseline is metric RGB-D in the first camera frame. At the endpoint, gravity
fixes two rotational degrees of freedom. A reprojection fit estimates heading
and translation from the visible labelled corners. No motor translation enters
the fit. Distances and reported uncertainty are in metres in shoulder X/Z.
"""

from collections import deque
import math

import cv2
import numpy as np

from .arm_tracking import _mount_to_shoulder_rotation, validate_config


START_FRAMES = 5
END_FRAMES = 3
MAX_FRAME_GAP_MS = 150.
MAX_UNCERTAINTY_M = .012
PIXEL_NOISE = .5  # Corner localization floor, even for perfect reprojection.
TAG_IDS = (0, 1)


def _require(condition, reason):
    if not condition:
        raise ValueError(reason)


def _unit(v):
    return v / np.linalg.norm(v)


def _calibration(frame):
    shape = np.shape(frame.get("color_bgr"))
    k = np.asarray(frame.get("camera_matrix"), dtype=float)
    d = np.asarray(frame.get("distortion"), dtype=float)
    _require(len(shape) == 3 and shape[2] == 3 and k.shape == (3, 3)
             and d.shape in ((4,), (5,), (8,)) and np.isfinite(k).all()
             and np.isfinite(d).all() and min(k[0, 0], k[1, 1]) > 0
             and np.allclose(k[2], [0, 0, 1]) and np.allclose([k[0, 1], k[1, 0]], 0),
             "invalid_camera_calibration")
    return k, d, shape[:2]


def _rays(pixels, k, d):
    pixels = np.ascontiguousarray(pixels, dtype=float).reshape(-1, 1, 2)
    xy = cv2.undistortPointsIter(pixels, k, d, None, None,
                                (cv2.TERM_CRITERIA_COUNT | cv2.TERM_CRITERIA_EPS, 20, 1e-10))
    rays = np.column_stack((xy.reshape(-1, 2), np.ones(len(xy))))
    check = cv2.projectPoints(rays, np.zeros(3), np.zeros(3), k, d)[0]
    _require(np.max(np.linalg.norm(check - pixels, axis=2)) < .2, "invalid_distortion_inverse")
    return rays


def _gravity(frame):
    imu = frame.get("wrist_imu")
    if not isinstance(imu, dict) or imu.get("valid") is not True:
        raise ValueError(imu.get("reason", "wrist_imu_unavailable")
                         if isinstance(imu, dict) else "wrist_imu_unavailable")
    g = np.asarray(imu.get("gravity_color"), dtype=float)
    gyro = imu.get("gyro_norm_rad_s")
    _require(g.shape == (3,) and np.isfinite(g).all() and .98 <= np.linalg.norm(g) <= 1.02
             and isinstance(gyro, (int, float, np.number)) and 0 <= gyro <= .06,
             "wrist_imu_not_stationary")
    return _unit(g)


def _corners(detection, shape):
    result = {}
    height, width = shape
    codes = detection.get("codes")
    _require(isinstance(codes, (list, tuple)), "invalid_tag_detections")
    for code in codes:
        _require(isinstance(code, dict), "invalid_tag_detections")
        tag = code.get("id")
        if code.get("type") != "aruco" or type(tag) is not int or tag not in TAG_IDS:
            continue
        _require(tag not in result, "duplicate_docking_tag")
        _require(code.get("dictionary") == "DICT_4X4_50", "wrong_tag_dictionary")
        p = np.asarray(code.get("corners_px"), dtype=float)
        _require(p.shape == (4, 2) and np.isfinite(p).all() and np.min(p) >= 2
                 and np.max(p[:, 0]) <= width - 3 and np.max(p[:, 1]) <= height - 3
                 and cv2.isContourConvex(p.astype(np.float32)), "incomplete_tag_corners")
        lengths = np.linalg.norm(p - np.roll(p, 1, axis=0), axis=1)
        _require(min(lengths) >= 12 and min(lengths) >= .35 * max(lengths),
                 "insufficient_corner_support")
        result[tag] = p
    _require(result, "no_docking_tag")
    return result


def _depth_samples(corners, depth):
    # Sample inside the ink and paper; depth at the outer edge may hit background.
    u, v = np.meshgrid(np.linspace(.16, .84, 7), np.linspace(.16, .84, 7))
    u, v = u.reshape(-1, 1), v.reshape(-1, 1)
    p = ((1 - v) * ((1 - u) * corners[0] + u * corners[1])
         + v * ((1 - u) * corners[3] + u * corners[2]))
    pixels = np.rint(p).astype(int)
    z = depth[pixels[:, 1], pixels[:, 0]]
    good = np.isfinite(z) & (z > 0)
    return pixels[good], z[good]


def _baseline_model(markers, frame, k, d, shape):
    depth = frame.get("depth_m")
    _require(frame.get("depth_status") == "available" and np.shape(depth) == shape,
             "start_aligned_depth_missing")
    rays, values, groups = [], [], []
    for tag in TAG_IDS:
        pixels, z = _depth_samples(markers[tag], depth)
        _require(len(z) >= 25, "start_depth_support")
        median = np.median(z)
        good = np.abs(z - median) <= max(.008, .10 * median)
        _require(good.sum() >= 25, "start_depth_support")
        rays.extend(_rays(pixels[good], k, d))
        values.extend(z[good])
        groups.extend([tag] * good.sum())
    rays, z, groups = np.array(rays), np.array(values), np.array(groups)
    keep = np.ones(len(z), dtype=bool)
    # A plane's inverse optical depth is affine in undistorted normalized x,y.
    for _ in range(3):
        plane, _, rank, _ = np.linalg.lstsq(rays[keep], 1. / z[keep], rcond=None)
        _require(rank == 3 and plane[2] > .35 * np.linalg.norm(plane), "start_plane_grazing")
        denominators = rays @ plane
        _require(min(denominators) > 0, "start_plane_invalid")
        residual = np.abs(1. / denominators - z)
        keep = residual <= np.clip(3 * np.median(residual), .0015, .0035)
        for tag in TAG_IDS:
            _require(np.count_nonzero(keep & (groups == tag)) >= max(24, .70 * sum(groups == tag)),
                     "start_markers_not_coplanar")
    noise = np.quantile(residual[keep], .95)
    _require(noise <= .0025, "start_depth_noisy")
    pixels = np.vstack([markers[tag] for tag in TAG_IDS])
    corner_rays = _rays(pixels, k, d)
    scale = corner_rays @ plane
    _require(min(scale) > 0, "start_plane_invalid")
    points = corner_rays / scale[:, None]
    for square in points.reshape(2, 4, 3):
        edges = np.roll(square, -1, axis=0) - square
        lengths = np.linalg.norm(edges, axis=1)
        directions = edges / lengths[:, None]
        _require(min(lengths) > .82 * max(lengths)
                 and max(np.abs(np.sum(directions * np.roll(directions, 1, axis=0), axis=1))) < .22,
                 "start_metric_tag_not_square")
    return points, pixels, float(noise), _unit(plane)


def _rotation_between(first, last):
    cross = np.cross(first, last)
    angle = math.atan2(np.linalg.norm(cross), np.dot(first, last))
    _require(angle <= math.radians(35.), "wrist_orientation_changed_excessively")
    return np.eye(3) if angle < 1e-9 else cv2.Rodrigues(_unit(cross) * angle)[0]


def _leveled_mount(nominal, gravity):
    """Retain the calibrated optical heading and set vertical from the signed wrist gravity."""
    _require((nominal @ gravity)[2] < -math.cos(math.radians(20.)),
             "wrist_mount_orientation_mismatch")
    heading = nominal[:, 2].copy()
    heading[2] = 0.
    up_color = -gravity
    forward_color = np.array([0., 0., 1.]) - up_color[2] * up_color
    _require(min(np.linalg.norm(heading), np.linalg.norm(forward_color)) > .70,
             "wrist_heading_unobservable")
    heading = _unit(heading)
    forward_color = _unit(forward_color)
    up = np.array([0., 0., 1.])
    camera_axes = np.column_stack((forward_color, np.cross(up_color, forward_color), up_color))
    shoulder_axes = np.column_stack((heading, np.cross(up, heading), up))
    return shoulder_axes @ camera_axes.T


def _live_ranges(frame, markers, ids, k, d, shape):
    """One central range per supported tag; nearby pixels share systematic errors."""
    result = []
    if frame.get("depth_status") == "available" and np.shape(frame.get("depth_m")) == shape:
        for tag in ids:
            pixels, z = _depth_samples(markers[tag], frame["depth_m"])
            if len(z) >= 25:
                result.append((_rays(np.median(pixels, axis=0), k, d)[0], float(np.median(z))))
    return result


def _fit_pose(points, pixels, k, d, gravity, start, lever, live_ranges):
    # For each heading about gravity, solve a linear bearing-translation system.
    # The single remaining angle can have two minima with a near-frontal tag.
    # Search both sides of the board's normal; reject a competing solution.
    center = points.mean(axis=0)
    local = points - center
    observed = _rays(pixels, k, d)[:, :2]
    n = len(points)
    design = np.zeros((n, 2, 3))
    design[:, 0, 0] = design[:, 1, 1] = 1.
    design[:, :, 2] = -observed
    design = design.reshape(-1, 3)
    pinv = np.linalg.pinv(design)
    tilt = _rotation_between(start["gravity"], gravity)
    focal = np.array([k[0, 0], k[1, 1]])
    mount = start["shoulder_rotation"]

    def fit(angle):
        rot = cv2.Rodrigues(gravity * angle)[0] @ tilt
        rotated = local @ rot.T
        shift = pinv @ (observed * rotated[:, 2:3] - rotated[:, :2]).ravel()
        projected = rotated + shift
        normal = rot @ start["normal"]
        plane_offset = np.dot(normal, shift)
        if min(projected[:, 2]) < .015 or plane_offset <= 0:
            return math.inf, None
        errors = (projected[:, :2] / projected[:, 2:3] - observed) * focal
        travel = mount @ (center + rot.T @ (lever - shift) - lever)
        range_errors = [plane_offset / np.dot(normal, ray) - z for ray, z in live_ranges]
        # Convert range residuals to the same noise units as corner residuals.
        cost = np.sum(errors * errors)
        for error, (_, z) in zip(range_errors, live_ranges):
            cost += (PIXEL_NOISE * error / (.0015 + .005*z))**2
        return float(cost), (travel, rot, projected, errors, range_errors)

    step = math.radians(4.)
    angles = np.linspace(-math.pi, math.pi, 91)
    costs = [fit(a)[0] for a in angles]
    minima = []
    for i in range(1, len(costs) - 1):
        if not (math.isfinite(costs[i]) and costs[i] <= costs[i - 1] and costs[i] <= costs[i + 1]):
            continue
        left, right = angles[i] - step, angles[i] + step
        for _ in range(14):
            a, b = (2 * left + right) / 3., (left + 2 * right) / 3.
            if fit(a)[0] < fit(b)[0]:
                right = b
            else:
                left = a
        angle = (left + right) / 2.
        cost, solution = fit(angle)
        if solution is not None:
            minima.append((cost, angle, solution))
    _require(minima, "visual_pose_unsolved")
    minima.sort(key=lambda entry: entry[0])
    cost, heading, (travel, rotation, projected, residual, range_errors) = minima[0]
    _require(abs(heading) <= math.radians(15), "unexpected_board_heading")
    _require(max(np.linalg.norm(residual, axis=1)) <= 1.7, "visual_reprojection_outlier")
    _require(all(abs(error) <= max(.006, .03*z) for error, (_, z) in zip(range_errors, live_ranges)),
             "live_depth_disagrees")
    _require(abs(travel[1]) <= .015, "unexpected_lateral_travel")
    rms = float(np.sqrt(np.mean(residual**2)))
    sigma = max(PIXEL_NOISE, math.sqrt(cost / (2 * n + len(live_ranges) - 4)))
    confidence_cost = cost + (3 * sigma)**2
    axes = [0, 2]
    for other_cost, other_angle, (other, *_) in minima[1:]:
        if other_cost <= confidence_cost:
            _require(abs(other_angle - heading) < math.radians(18)
                     and np.linalg.norm((other - travel)[axes]) < .002, "ambiguous_planar_pose")
    # Propagate poorly observed heading along the axes that would be corrected.
    # Interpolate both boundaries of the connected 3-sigma cost interval.
    heading_bound = 0.
    for sign in (-1, 1):
        inside = heading
        outside = inside + sign * step
        while abs(outside - heading) < math.pi and fit(outside)[0] <= confidence_cost:
            intermediate = fit(outside)[1]
            heading_bound = max(heading_bound, float(np.linalg.norm((intermediate[0] - travel)[axes])))
            inside, outside = outside, outside + sign * step
        for _ in range(7):
            middle = (inside + outside) / 2.
            if fit(middle)[0] <= confidence_cost:
                inside = middle
            else:
                outside = middle
        alternative = fit(inside)[1]
        heading_bound = max(heading_bound, float(np.linalg.norm((alternative[0] - travel)[axes])))

    jac = np.zeros((n, 2, 3))
    jac[:, 0, 0] = focal[0] / projected[:, 2]
    jac[:, 1, 1] = focal[1] / projected[:, 2]
    jac[:, :, 2] = -projected[:, :2] * focal / projected[:, 2:3]**2
    axes_rotation = (mount @ rotation.T)[axes, :]
    jac = jac.reshape(-1, 3)
    normal = rotation @ start["normal"]
    for ray, z in live_ranges:
        derivative = PIXEL_NOISE * normal / (np.dot(normal, ray) * (.0015 + .005*z))
        jac = np.vstack((jac, derivative))
    sensitivity = axes_rotation @ np.linalg.pinv(jac)
    translation_bound = 3 * sigma * np.linalg.norm(sensitivity, ord=2)
    return travel, heading_bound + translation_bound, rms, float(np.mean(projected[:, 2]))


class DockingTravel:
    def __init__(self, config):
        self.config = validate_config(config)
        self.mount = np.asarray(self.config["rotation"])
        self.lever = -self.mount.T @ np.asarray(self.config["translation_m"])
        self._last_stamp = None
        self.reset_start()

    def reset_endpoint(self):
        """Require fresh stationary endpoint samples while retaining the same board."""
        self._ends = deque(maxlen=END_FRAMES)

    def reset_start(self):
        """Discard the board and all samples when settling is interrupted."""
        self._starts = deque(maxlen=START_FRAMES)
        self._base = None
        self.reset_endpoint()

    def _frame_stamp(self, frame, detection):
        _require(isinstance(frame, dict) and isinstance(detection, dict), "missing_camera_frame")
        stamp = float(frame["timestamp_ms"])
        _require(math.isfinite(stamp) and stamp >= 0 and stamp == detection.get("device_timestamp_ms")
                 and (self._last_stamp is None or stamp > self._last_stamp), "invalid_frame_timestamp")
        if self._last_stamp is not None and stamp - self._last_stamp > MAX_FRAME_GAP_MS:
            self._starts.clear()
            self.reset_endpoint()
        self._last_stamp = stamp
        return stamp

    def observe_start(self, frame, detection, pitch_rad):
        self.reset_endpoint()
        try:
            self._frame_stamp(frame, detection)
            k, d, shape = _calibration(frame)
            g = _gravity(frame)
            _require(self.config["camera_mount"] == "tool" and math.isfinite(pitch_rad),
                     "missing_wrist_mount")
            base_tilt = frame.get("base_tilt_rad")
            _require(not isinstance(base_tilt, bool) and isinstance(base_tilt, (int, float, np.number)) and
                     0 <= base_tilt <= math.radians(3.), "base_not_level_or_tilt_unavailable")
            # The current upright mount maps physical down (optical +Y) to shoulder -Z.
            nominal = _mount_to_shoulder_rotation(self.config, pitch_rad) @ self.mount
            shoulder = _leveled_mount(nominal, g)
            markers = _corners(detection, shape)
            pair = detection.get("pair_indices", [])
            codes = detection.get("codes", [])
            _require(len(markers) == 2 and detection.get("valid") is True and len(pair) == 2
                     and all(type(i) is int and 0 <= i < len(codes) for i in pair)
                     and {codes[i]["id"] for i in pair} == set(TAG_IDS), "start_pair_missing")
            points, pixels, noise, normal = _baseline_model(markers, frame, k, d, shape)
            sample = dict(points=points, pixels=pixels, gravity=g, noise=noise, normal=normal,
                          shoulder_rotation=shoulder, base_tilt=float(base_tilt), k=k.copy(),
                          d=d.copy(), shape=shape)
            if self._starts:
                first = self._starts[0]
                _require(self._same_camera(first, k, d, shape), "camera_calibration_changed")
                if (np.max(np.linalg.norm(points - first["points"], axis=1)) > .003
                        or np.dot(g, first["gravity"]) < math.cos(math.radians(1.))
                        or np.linalg.norm(shoulder - first["shoulder_rotation"]) > .025):
                    self.reset_start()
            self._starts.append(sample)
            _require(len(self._starts) == START_FRAMES, "acquiring_start")
            def median(key):
                return np.median([s[key] for s in self._starts], axis=0)
            points = median("points")
            g = _unit(median("gravity"))
            self._base = dict(sample, points=points, gravity=g, shoulder_rotation=_leveled_mount(nominal, g),
                              pixels=median("pixels"), normal=_unit(median("normal")),
                              base_tilt=max(s["base_tilt"] for s in self._starts),
                              spread=max(np.max(np.linalg.norm(s["points"] - points, axis=1))
                                         for s in self._starts), noise=max(s["noise"] for s in self._starts))
            return {"valid": True, "reason": "start_locked", "samples": START_FRAMES}
        except (ValueError, KeyError, TypeError, OverflowError, cv2.error, np.linalg.LinAlgError) as error:
            reason = str(error)
            self._base = None
            if reason != "acquiring_start":
                self.reset_start()
            return {"valid": False, "reason": reason}

    @staticmethod
    def _same_camera(base, k, d, shape):
        return (shape == base["shape"] and d.shape == base["d"].shape
                and np.allclose(base["k"], k, atol=1e-6, rtol=1e-6)
                and np.allclose(base["d"], d, atol=1e-6, rtol=1e-6))

    def measure(self, frame, detection):
        details = {}
        try:
            self._frame_stamp(frame, detection)
            base = self._base
            _require(base is not None, "start_model_missing")
            k, d, shape = _calibration(frame)
            _require(self._same_camera(base, k, d, shape), "camera_calibration_changed")
            gravity = _gravity(frame)
            markers = _corners(detection, shape)
            ids = tuple(sorted(markers))
            indices = [4 * tag + corner for tag in ids for corner in range(4)]
            pixels = np.vstack([markers[tag] for tag in ids])
            if len(ids) == 1:
                _require(min(np.linalg.norm(pixels - np.roll(pixels, 1, axis=0), axis=1)) >= 36,
                         "single_tag_too_small")
            ranges = _live_ranges(frame, markers, ids, k, d, shape)
            full_travel, fit_bound, rms, mean_range = _fit_pose(
                base["points"][indices], pixels, k, d, gravity, base, self.lever, ranges)
            stroke = np.linalg.norm(full_travel)
            # Include systematic floors for baseline range/scale, finite corner
            # sampling, and wrist gravity direction. More frames cannot remove them.
            span = np.linalg.norm(np.ptp(base["pixels"][indices], axis=0))
            bound = (fit_bound + base["spread"] + base["noise"] + .001
                     + .005 * stroke + 2 * PIXEL_NOISE * mean_range / span
                     + math.radians(.4) * (mean_range + np.linalg.norm(self.lever) + stroke)
                     + 2 * math.sin(base["base_tilt"] / 2) * stroke)
            _require(math.isfinite(bound), "travel_uncertainty_excessive")
            details = {"visible_ids": list(ids), "uncertainty_m": float(bound), "reprojection_rms_px": rms}
            _require(bound <= MAX_UNCERTAINTY_M, "travel_uncertainty_excessive")
            if self._ends and (ids != self._ends[0][0]
                               or np.linalg.norm(full_travel - self._ends[0][1]) > .0025
                               or np.dot(gravity, self._ends[0][3]) < math.cos(math.radians(.8))):
                self.reset_endpoint()
            self._ends.append((ids, full_travel, bound, gravity))
            _require(len(self._ends) == END_FRAMES, "acquiring_endpoint")
            estimate = np.median([x[1][[0, 2]] for x in self._ends], axis=0)
            uncertainty = max(u + np.linalg.norm(t[[0, 2]] - estimate) for _, t, u, _ in self._ends)
            details["uncertainty_m"] = float(uncertainty)
            _require(uncertainty <= MAX_UNCERTAINTY_M, "travel_uncertainty_excessive")
            return dict(details, valid=True, reason="measured", travel_m=estimate.tolist(), samples=END_FRAMES)
        except (ValueError, KeyError, TypeError, OverflowError, cv2.error, np.linalg.LinAlgError) as error:
            reason = str(error)
            if reason != "acquiring_endpoint":
                self.reset_endpoint()
            return dict(details, valid=False, reason=reason)
