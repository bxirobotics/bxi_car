"""Gemini 335 acquisition and camera-frame geometry; no robot control."""

import importlib
import math
import threading
import time
from collections import deque

import cv2
import numpy as np


def _imu_unavailable(reason):
    return {"valid": False, "reason": reason, "gravity_color": None,
            "gyro_norm_rad_s": None, "start_ms": None, "end_ms": None}


class _WristImu:
    """Keep bounded, device-timed accelerometer and gyro history on the RGB-D device."""

    def __init__(self):
        self.pipeline = None
        self.rotations = {}
        self.samples = {kind: deque(maxlen=96) for kind in ("accel", "gyro")}
        self.high_water = {kind: 0. for kind in self.samples}
        self.lock = threading.Lock()
        self.last_color_ms = -math.inf
        self.invalid_reason = "imu_warming_up"

    def start(self, ob, device, color_profile):
        # The SDK's multi-stream example runs video and IMU in separate pipelines.
        # Bind both to the selected device, including when more than one is attached.
        config = ob.Config()
        config.disable_all_stream()
        for kind, sensor in (("accel", ob.OBSensorType.ACCEL_SENSOR),
                             ("gyro", ob.OBSensorType.GYRO_SENSOR)):
            profiles = device.get_sensor(sensor).get_stream_profile_list()
            for index in range(profiles.get_count()):
                profile = profiles.get_stream_profile_by_index(index)
                typed = (profile.as_accel_stream_profile() if kind == "accel"
                         else profile.as_gyro_stream_profile())
                if typed.get_sample_rate() == ob.OBGyroSampleRate.SAMPLE_RATE_200_HZ:
                    break
            else:
                raise ValueError(f"no 200 Hz {kind} stream")
            rotation = np.asarray(profile.get_extrinsic_to(color_profile).rot, dtype=float)
            if (rotation.shape != (3, 3) or not np.isfinite(rotation).all()
                    or not np.allclose(rotation.T @ rotation, np.eye(3), atol=1e-4, rtol=0.)
                    or abs(np.linalg.det(rotation) - 1.) > 1e-4):
                raise ValueError(f"invalid {kind}-to-color rotation")
            self.rotations[kind] = rotation
            config.enable_stream(profile)
        config.set_frame_aggregate_output_mode(ob.OBFrameAggregateOutputMode.ANY_SITUATION)
        self.pipeline = ob.Pipeline(device)
        self.pipeline.start(config, self._receive)

    def _receive(self, frames):
        # The SDK invokes this on a native thread. Copy numbers, never retain frames.
        with self.lock:
            try:
                for kind, sample in (("accel", frames.get_accel_frame()),
                                     ("gyro", frames.get_gyro_frame())):
                    if sample is None:
                        continue
                    stamp = float(sample.get_timestamp_us()) / 1000.
                    if not math.isfinite(stamp) or stamp <= 0 or stamp < self.high_water[kind]:
                        raise ValueError("invalid or regressed device time")
                    if stamp == self.high_water[kind]:
                        continue  # The SDK can repeat one stream when the other updates.
                    vector = sample.get_value()
                    xyz = (float(vector.x), float(vector.y), float(vector.z))
                    if not all(math.isfinite(v) for v in xyz):
                        raise ValueError("nonfinite IMU reading")
                    self.samples[kind].append((stamp, time.monotonic(), xyz))
                    self.high_water[kind] = stamp
            except Exception:
                for rows in self.samples.values():
                    rows.clear()
                self.invalid_reason = "invalid_imu_sample"

    def for_image(self, color_ms):
        # Use the device clock for alignment. Receipt time can veto a stale stream
        # but must never make an unrelated device timestamp match the exposure.
        if color_ms <= self.last_color_ms:
            return _imu_unavailable("out_of_order_color")
        self.last_color_ms = color_ms
        with self.lock:
            windows = {kind: [row for row in rows if color_ms - 130. <= row[0] <= color_ms]
                       for kind, rows in self.samples.items()}
            empty_reasons = {kind: "imu_time_mismatch" if rows else self.invalid_reason
                             for kind, rows in self.samples.items()}
        starts, ends, vectors = [], [], {}
        now = time.monotonic()
        for kind, rows in windows.items():
            if not rows:
                return _imu_unavailable(empty_reasons[kind])
            stamps = np.asarray([row[0] for row in rows])
            span = stamps[-1] - stamps[0]
            if (color_ms - stamps[-1] > 15. or now - rows[-1][1] > .12):
                return _imu_unavailable("stale_imu")
            if (len(rows) < 18 or span < 100.
                    or not 160. <= (len(rows) - 1) * 1000. / span <= 240.
                    or np.max(np.diff(stamps)) > 12.5):
                return _imu_unavailable("insufficient_imu_coverage")
            # Each extrinsic maps a source column vector into optical axes.
            vectors[kind] = np.asarray([row[2] for row in rows]) @ self.rotations[kind].T
            starts.append(float(stamps[0]))
            ends.append(float(stamps[-1]))
        acceleration = vectors["accel"]
        mean = acceleration.mean(axis=0)
        norm = float(np.linalg.norm(mean))
        gyro_max = float(np.max(np.linalg.norm(vectors["gyro"], axis=1)))
        if (not math.isfinite(norm) or not math.isfinite(gyro_max)
                or abs(norm - 9.80665) > .5 or np.max(acceleration.std(axis=0)) > .05
                or np.max(np.linalg.norm(acceleration - mean, axis=1)) > .20
                or gyro_max > .03):
            return _imu_unavailable("wrist_not_stationary")
        # The stationary accelerometer measures specific force towards physical up.
        # On the upright wrist this is optical -Y; gravity is in the opposite direction.
        return {"valid": True, "reason": "stationary", "gravity_color": (-mean / norm).tolist(),
                "specific_force_color_m_s2": mean.tolist(),
                "gyro_norm_rad_s": gyro_max, "start_ms": min(starts), "end_ms": max(ends)}

    def stop(self):
        pipeline, self.pipeline = self.pipeline, None
        if pipeline is not None:
            pipeline.stop()


class GeminiCamera:
    """Read the official Orbbec SDK v2. Importing this module opens no device."""

    def __init__(self, serial=None, color_only=False, width=640, height=480,
                 fps=30, timeout_ms=1000, sdk=None, depth_width=0, depth_height=0,
                 wrist_imu=False):
        for name, value in (("width", width), ("height", height), ("fps", fps),
                            ("timeout_ms", timeout_ms)):
            if isinstance(value, bool) or not isinstance(value, int) or value <= 0:
                raise ValueError(f"{name} must be a positive integer")
        for name, value in (("depth_width", depth_width), ("depth_height", depth_height)):
            if isinstance(value, bool) or not isinstance(value, int) or value < 0:
                raise ValueError(f"{name} must be a non-negative integer")
        if (depth_width == 0) != (depth_height == 0):
            raise ValueError("depth_width and depth_height must both be zero or both positive")
        self.serial = serial
        self.color_only = color_only
        self.width, self.height, self.fps = width, height, fps
        self.depth_width, self.depth_height = depth_width, depth_height
        self.timeout_ms = timeout_ms
        self.wrist_imu = wrist_imu
        self._imu = None
        self._imu_status = None
        self._sdk = sdk
        self._context = self._device = self._pipeline = self._align = None
        self._started = False
        self._start_attempted = False
        self._metadata = None
        self._metric_depth_status = "disabled"

    def start(self):
        if self._started:
            return dict(self._metadata)
        if self._sdk is None:
            try:
                self._sdk = importlib.import_module("pyorbbecsdk")
            except ImportError as exc:
                raise RuntimeError(
                    "Gemini 335 requires the official pyorbbecsdk2 package "
                    "(import name: pyorbbecsdk); install pyorbbecsdk2."
                ) from exc
        ob = self._sdk
        try:
            self._context = ob.Context()
            devices = self._context.query_devices()
            candidates = []
            for index in range(devices.get_count()):
                device = devices.get_device_by_index(index)
                info = device.get_device_info()
                name = str(info.get_name()).strip()
                normalized = "".join(ch.lower() for ch in name if ch.isalnum())
                # Gemini 335L/336 and other cameras have different calibration
                # contracts; do not silently reuse the 335 Brown K6 model.
                if normalized not in ("gemini335", "orbbecgemini335"):
                    continue
                serial = str(info.get_serial_number()).strip()
                if self.serial is None or serial == self.serial:
                    candidates.append((device, name, serial))
            if not candidates:
                suffix = f" with serial {self.serial}" if self.serial else ""
                raise RuntimeError(f"No Gemini 335 camera found{suffix}")
            if len(candidates) != 1:
                raise RuntimeError("Multiple Gemini 335 cameras found; specify --serial")
            self._device, name, serial = candidates[0]
            self._pipeline = ob.Pipeline(self._device)
            config = ob.Config()
            profiles = self._pipeline.get_stream_profile_list(ob.OBSensorType.COLOR_SENSOR)
            color_profile = None
            for frame_format in (ob.OBFormat.BGR, ob.OBFormat.RGB, ob.OBFormat.MJPG):
                try:
                    color_profile = profiles.get_video_stream_profile(
                        self.width, self.height, frame_format, self.fps)
                    break
                except Exception:
                    continue
            if color_profile is None:
                raise RuntimeError(
                    f"Gemini 335 has no BGR/RGB/MJPG color profile "
                    f"{self.width}x{self.height}@{self.fps}"
                )
            config.enable_stream(color_profile)
            depth_profile = None
            if not self.color_only:
                profiles = self._pipeline.get_stream_profile_list(ob.OBSensorType.DEPTH_SENSOR)
                # Color and depth need equal frame rates, not equal dimensions.
                try:
                    depth_profile = profiles.get_video_stream_profile(
                        self.depth_width, self.depth_height, ob.OBFormat.Y16, self.fps)
                except Exception as exc:
                    raise RuntimeError(
                        f"Gemini 335 has no Y16 depth profile "
                        f"{self.depth_width}x{self.depth_height}@{self.fps}"
                    ) from exc
                config.enable_stream(depth_profile)
                config.set_frame_aggregate_output_mode(
                    ob.OBFrameAggregateOutputMode.COLOR_FRAME_REQUIRE)
                self._pipeline.enable_frame_sync()
                self._align = ob.AlignFilter(ob.OBStreamType.COLOR_STREAM)
                # SDK defaults to rectified target pixels, whereas QR detection
                # uses raw RGB. Keep the aligned depth in raw RGB pixel space.
                self._align.set_config_value("TargetDistortion", 1.0)
                if self._align.get_config_value("TargetDistortion") != 1.0:
                    raise RuntimeError("SDK cannot align depth to distorted color pixels")
                try:
                    enabled = self._device.get_bool_property(
                        ob.OBPropertyID.OB_PROP_DISPARITY_TO_DEPTH_BOOL)
                    self._metric_depth_status = (
                        "available" if enabled else "disparity_conversion_disabled")
                except Exception as exc:
                    self._metric_depth_status = f"disparity_conversion_unverified: {exc}"
            self._start_attempted = True
            self._pipeline.start(config)
            self._started = True
            if self.wrist_imu:
                self._imu = _WristImu()
                try:
                    self._imu.start(ob, self._device, color_profile)
                    self._imu_status = "streaming"
                except Exception as exc:
                    # Video continues to serve the nominal approach if only the
                    # optional wrist measurement is unavailable.
                    self._imu_status = f"imu_unavailable: {exc}"
                    try:
                        self._imu.stop()
                    except Exception as stop_error:
                        self._imu_status += f" (IMU cleanup: {stop_error})"
                    self._imu = None
            self._metadata = {
                "name": name, "serial": serial,
                "color_profile": self._profile_metadata(color_profile),
                "depth_profile": self._profile_metadata(depth_profile),
                "depth_aligned_to": None if self.color_only else "color",
                "depth_status": self._metric_depth_status,
                "frame_id": "gemini_color_optical_frame",
            }
            if self.wrist_imu:
                self._metadata["wrist_imu_status"] = self._imu_status
            return dict(self._metadata)
        except BaseException:
            try:
                self.stop()
            except Exception:
                pass  # Preserve the actionable startup failure.
            raise

    @staticmethod
    def _profile_metadata(profile):
        if profile is None:
            return None
        return {"width": int(profile.get_width()), "height": int(profile.get_height()),
                "fps": int(profile.get_fps()), "format": str(profile.get_format())}

    def _color_array(self, frame):
        width, height = int(frame.get_width()), int(frame.get_height())
        if width <= 0 or height <= 0:
            raise ValueError("invalid color dimensions")
        data = np.frombuffer(frame.get_data(), dtype=np.uint8)
        frame_format, formats = frame.get_format(), self._sdk.OBFormat
        if frame_format in (formats.BGR, formats.RGB):
            if data.size != width * height * 3:
                raise ValueError("color buffer length does not match dimensions")
            result = data.reshape(height, width, 3)
            if frame_format == formats.RGB:
                result = cv2.cvtColor(result, cv2.COLOR_RGB2BGR)
        elif frame_format == formats.MJPG:
            if not data.size:
                raise ValueError("empty MJPG color buffer")
            result = cv2.imdecode(data, cv2.IMREAD_COLOR)
            if result is None or result.shape != (height, width, 3):
                raise ValueError("MJPG color data or dimensions are invalid")
        else:
            raise ValueError(f"unsupported color format: {frame_format}")
        # Keep the returned image alive independently of SDK frame ownership.
        return np.array(result, dtype=np.uint8, copy=True, order="C")

    @staticmethod
    def _calibration(frame, image_shape):
        profile = frame.get_stream_profile().as_video_stream_profile()
        intrinsic, distortion = profile.get_intrinsic(), profile.get_distortion()
        height, width = image_shape[:2]
        if intrinsic.width != width or intrinsic.height != height:
            raise ValueError("color intrinsic dimensions do not match image")
        values = np.asarray([intrinsic.fx, intrinsic.fy, intrinsic.cx, intrinsic.cy],
                            dtype=np.float64)
        if not np.isfinite(values).all() or values[0] <= 0 or values[1] <= 0:
            raise ValueError("invalid color intrinsics")
        fx, fy, cx, cy = values
        matrix = np.array([[fx, 0., cx], [0., fy, cy], [0., 0., 1.]])
        # Gemini 335's SDK G330AlgParamManager uses Brown-Conrady K6 for RGB.
        coeffs = np.asarray([distortion.k1, distortion.k2, distortion.p1,
                             distortion.p2, distortion.k3, distortion.k4,
                             distortion.k5, distortion.k6], dtype=np.float64)
        if not np.isfinite(coeffs).all():
            raise ValueError("invalid color distortion coefficients")
        return matrix, coeffs

    def read(self):
        if not self._started:
            raise RuntimeError("Gemini camera has not been started")
        frames = self._pipeline.wait_for_frames(self.timeout_ms)
        if frames is None:
            raise TimeoutError(f"No Gemini frame received within {self.timeout_ms} ms")
        color = frames.get_color_frame()
        if color is None:
            raise TimeoutError("Gemini frameset contains no color frame")
        image = self._color_array(color)
        timestamp = float(color.get_timestamp())
        if not math.isfinite(timestamp):
            raise ValueError("invalid color timestamp")
        result = {"color_bgr": image, "depth_m": None, "camera_matrix": None,
                  "distortion": None, "timestamp_ms": timestamp,
                  "depth_status": ("missing_depth" if self._metric_depth_status == "available"
                                   else self._metric_depth_status)}
        if self.wrist_imu:
            result["wrist_imu"] = (self._imu.for_image(timestamp) if self._imu is not None
                                   else _imu_unavailable(self._imu_status))
        try:
            result["camera_matrix"], result["distortion"] = self._calibration(
                color, image.shape)
        except Exception as exc:
            if not self.color_only:
                result["depth_status"] = f"invalid_calibration: {exc}"
        if (self.color_only or result["camera_matrix"] is None
                or self._metric_depth_status != "available"):
            return result
        try:
            original_depth = frames.get_depth_frame()
            if original_depth is None:
                return result
            depth_timestamp = float(original_depth.get_timestamp())
            if (not math.isfinite(depth_timestamp)
                    or abs(depth_timestamp - timestamp) > max(2., 500. / self.fps)):
                result["depth_status"] = "depth_timestamp_mismatch"
                return result
            aligned = self._align.process(frames)
            if aligned is None:
                result["depth_status"] = "alignment_failed"
                return result
            # The Python SDK exposes Filter.process as a generic Frame.
            aligned = aligned.as_frame_set()
            if aligned is None:
                result["depth_status"] = "alignment_failed"
                return result
            depth = aligned.get_depth_frame()
            if depth is None:
                return result
            height, width = image.shape[:2]
            if int(depth.get_width()) != width or int(depth.get_height()) != height:
                result["depth_status"] = "alignment_size_mismatch"
                return result
            depth_matrix, depth_distortion = self._calibration(depth, image.shape)
            if (not np.allclose(depth_matrix, result["camera_matrix"], rtol=1e-4, atol=1e-6)
                    or not np.allclose(depth_distortion, result["distortion"],
                                       rtol=1e-4, atol=1e-6)):
                result["depth_status"] = "alignment_calibration_mismatch"
                return result
            if depth.get_format() != self._sdk.OBFormat.Y16:
                raise ValueError("aligned depth is not Y16")
            data = memoryview(depth.get_data())
            if data.nbytes != width * height * 2:
                raise ValueError("depth buffer length does not match dimensions")
            scale = float(depth.get_depth_scale())
            if not math.isfinite(scale) or scale <= 0:
                raise ValueError("invalid depth scale")
            raw = np.frombuffer(data, dtype=np.uint16).reshape(height, width)
            depth_m = raw.astype(np.float64) * (scale / 1000.0)
            if not np.isfinite(depth_m).all():
                raise ValueError("depth scale overflows meters")
            result["depth_m"] = depth_m
            result["depth_status"] = "available" if np.any(raw > 0) else "no_valid_depth"
        except Exception as exc:
            result["depth_status"] = f"invalid_depth: {exc}"
        return result

    def stop(self):
        pipeline, attempted = self._pipeline, self._start_attempted
        imu, self._imu = self._imu, None
        self._imu_status = None
        self._pipeline = self._align = self._device = self._context = None
        self._started = self._start_attempted = False
        self._metadata = None
        self._metric_depth_status = "disabled"
        try:
            if imu is not None:
                imu.stop()
        finally:
            if pipeline is not None and attempted:
                pipeline.stop()


def locate_midpoint(detection, frame, sample_radius=2):
    """Back-project the selected center, or back-project both centers then average."""
    result = {"valid": False, "status": "invalid_detection",
              "frame_id": "gemini_color_optical_frame",
              "centers_m": None, "midpoint_m": None}
    if isinstance(sample_radius, bool) or not isinstance(sample_radius, int) or sample_radius < 0:
        raise ValueError("sample_radius must be a non-negative integer")
    if not detection.get("valid", False):
        return result
    try:
        indices = detection["pair_indices"]
        count = len(indices)
        if (count not in (1, 2) or (count == 2 and indices[0] == indices[1])
                or any(isinstance(i, bool) or not isinstance(i, int) or i < 0 for i in indices)):
            return result
        selected = [detection["codes"][index] for index in indices]
        centers = np.asarray([code["center_px"] for code in selected], dtype=np.float64)
        corners = np.asarray([code["corners_px"] for code in selected], dtype=np.float64)
        if centers.shape != (count, 2) or corners.shape != (count, 4, 2):
            return result
        if not np.isfinite(centers).all() or not np.isfinite(corners).all():
            return result
    except (KeyError, IndexError, TypeError, ValueError):
        return result
    depth = frame.get("depth_m")
    if depth is None:
        result["status"] = frame.get("depth_status", "missing_depth")
        return result
    depth = np.asarray(depth)
    if depth.ndim != 2:
        result["status"] = "invalid_depth"
        return result
    height, width = depth.shape
    if frame.get("color_bgr") is not None and frame["color_bgr"].shape[:2] != depth.shape:
        result["status"] = "alignment_size_mismatch"
        return result
    try:
        matrix = np.asarray(frame["camera_matrix"], dtype=np.float64)
        distortion = np.asarray(frame["distortion"], dtype=np.float64)
        if (matrix.shape != (3, 3) or distortion.shape != (8,)
                or not np.isfinite(matrix).all() or not np.isfinite(distortion).all()
                or matrix[0, 0] <= 0 or matrix[1, 1] <= 0
                or not np.allclose(matrix[2], [0., 0., 1.])
                or not np.allclose([matrix[0, 1], matrix[1, 0]], 0.)):
            raise ValueError("invalid calibration")
    except (KeyError, TypeError, ValueError):
        result["status"] = "invalid_calibration"
        return result
    depths = []
    for center, quad in zip(centers, corners):
        contour = quad.astype(np.float32)
        if (not cv2.isContourConvex(contour) or abs(cv2.contourArea(contour)) < 4
                or cv2.pointPolygonTest(contour, tuple(center), False) < 0
                or not (0 <= center[0] < width and 0 <= center[1] < height)):
            return result
        u, v = (int(round(float(value))) for value in center)
        left, right = max(0, u - sample_radius), min(width, u + sample_radius + 1)
        top, bottom = max(0, v - sample_radius), min(height, v + sample_radius + 1)
        samples = [depth[y, x] for y in range(top, bottom) for x in range(left, right)
                   if cv2.pointPolygonTest(contour, (float(x), float(y)), False) >= 0]
        values = np.asarray(samples, dtype=np.float64)
        values = values[np.isfinite(values) & (values > 0)]
        if values.size < 3:
            result["status"] = "insufficient_depth_samples"
            return result
        median = float(np.median(values))
        # A small center patch should be one surface. Reject edge/background
        # contamination rather than quietly mixing separate depth layers.
        if float(np.ptp(values)) > max(0.02, median * 0.03):
            result["status"] = "depth_discontinuity"
            return result
        depths.append(median)
    try:
        rays = cv2.undistortPointsIter(
            centers.reshape(count, 1, 2), matrix, distortion, None, None,
            (cv2.TERM_CRITERIA_COUNT | cv2.TERM_CRITERIA_EPS, 20, 1e-9)).reshape(count, 2)
        points = np.column_stack((rays, np.ones(count))) * np.asarray(depths)[:, None]
        if not np.isfinite(points).all():
            raise ValueError("invalid back projection")
        projected, _ = cv2.projectPoints(
            points, np.zeros(3), np.zeros(3), matrix, distortion)
        error = np.linalg.norm(projected.reshape(count, 2) - centers, axis=1)
        if not np.isfinite(error).all() or np.any(error > .5):
            raise ValueError("distortion inversion did not converge")
    except (cv2.error, ValueError):
        result["status"] = "invalid_calibration"
        return result
    result.update(valid=True, status="available", centers_m=points.tolist(),
                  midpoint_m=points.mean(axis=0).tolist())
    return result
