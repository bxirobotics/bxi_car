#!/usr/bin/env python3
"""Read Gemini 335 IMU data without opening a robot control interface."""

import argparse
from datetime import datetime, timezone
import json
import math
from pathlib import Path
import statistics
import time


def summarize(samples, rotation, expected_hz=None, expected_seconds=None):
    """Rows: elapsed seconds, device microseconds, x, y, z, temperature C."""
    if len(samples) < 20:
        raise ValueError("Too few IMU samples")
    if any(len(row) != 6 or not all(math.isfinite(x) for x in row) for row in samples):
        raise ValueError("Invalid IMU sample")
    if any(b[1] <= a[1] for a, b in zip(samples, samples[1:])):
        raise ValueError("IMU device timestamps must increase")
    span = (samples[-1][1] - samples[0][1]) / 1e6
    if span < 1:
        raise ValueError("At least one second of IMU data is required")
    observed_hz = (len(samples) - 1) / span
    max_gap = max((b[1] - a[1]) / 1e6 for a, b in zip(samples, samples[1:]))
    if expected_hz is not None and (
            not 0.8 * expected_hz <= observed_hz <= 1.2 * expected_hz
            or max_gap > max(0.05, 10 / expected_hz)):
        raise ValueError("IMU data has an unexpected sample rate or a large gap")
    if expected_seconds is not None and span < 0.85 * expected_seconds:
        raise ValueError("IMU data does not cover the requested duration")
    if (len(rotation) != 3 or any(len(row) != 3 for row in rotation)
            or not all(math.isfinite(x) for row in rotation for x in row)):
        raise ValueError("Invalid IMU-to-color rotation")
    # Official SDK CoordinateUtil uses row-major rotations: destination = R * source.
    vectors = [[sum(rotation[i][j] * row[j + 2] for j in range(3))
                for i in range(3)] for row in samples]
    axes = list(zip(*vectors))
    mean = [statistics.fmean(axis) for axis in axes]
    return {
        "count": len(samples), "device_span_s": span,
        "sample_rate_hz": observed_hz, "max_sample_gap_s": max_gap,
        "mean_color_xyz": mean,
        "stddev_color_xyz": [statistics.pstdev(axis) for axis in axes],
        "mean_vector_norm": math.hypot(*mean),
        "temperature_mean_c": statistics.fmean(row[5] for row in samples),
    }


def inclination(accel_color):
    """Unsigned optical +Z elevation; independent of the gravity-vector sign."""
    if len(accel_color) != 3 or not all(math.isfinite(x) for x in accel_color):
        raise ValueError("Invalid acceleration vector")
    norm = math.hypot(*accel_color)
    if norm < 1e-6:
        raise ValueError("Gravity direction is unavailable")
    return math.degrees(math.asin(min(1.0, abs(accel_color[2]) / norm)))


def read_imu(seconds, serial=None):
    # Keep --help and offline math tests independent of the installed camera SDK.
    import pyorbbecsdk as ob

    ob.Context.set_logger_to_console(ob.OBLogLevel.ERROR)
    context = ob.Context()
    devices = context.query_devices()
    matches = []
    for index in range(devices.get_count()):
        device = devices.get_device_by_index(index)
        info = device.get_device_info()
        name = "".join(c.lower() for c in info.get_name() if c.isalnum())
        if name in ("gemini335", "orbbecgemini335"):
            if serial is None or info.get_serial_number() == serial:
                matches.append(device)
    if len(matches) != 1:
        raise RuntimeError("Select exactly one Gemini 335; use --serial if needed")
    device = matches[0]
    info = device.get_device_info()
    pipeline = ob.Pipeline(device)
    config = ob.Config()
    config.disable_all_stream()
    color = device.get_sensor(ob.OBSensorType.COLOR_SENSOR)
    color_profile = color.get_stream_profile_list().get_stream_profile_by_index(0)
    metadata = {
        "device": info.get_name(), "serial": info.get_serial_number(),
        "firmware": info.get_firmware_version(), "sdk_version": ob.__version__,
        "captured_at": datetime.now(timezone.utc).isoformat(),
        "duration_requested_s": seconds,
        "frame": "color_optical: X right, Y down, Z forward",
        "accel_units": "m/s^2", "gyro_units": "rad/s",
        "sample_columns": ["elapsed_s", "device_timestamp_us", "x", "y", "z", "temperature_c"],
    }
    rotations = {}
    for label, kind in (("accel", ob.OBSensorType.ACCEL_SENSOR),
                        ("gyro", ob.OBSensorType.GYRO_SENSOR)):
        profiles = device.get_sensor(kind).get_stream_profile_list()
        candidates = [profiles.get_stream_profile_by_index(i) for i in range(profiles.get_count())]
        if not candidates:
            raise RuntimeError(f"No {label} stream profiles")
        convert = (lambda p: p.as_accel_stream_profile()) if label == "accel" else (
            lambda p: p.as_gyro_stream_profile())
        profile = next((p for p in candidates if convert(p).get_sample_rate()
                        == ob.OBGyroSampleRate.SAMPLE_RATE_200_HZ), None)
        if profile is None:
            raise RuntimeError(f"No 200 Hz {label} stream profile")
        config.enable_stream(profile)
        rotations[label] = profile.get_extrinsic_to(color_profile).rot.tolist()
        metadata[label + "_profile"] = {
            "sample_rate": str(convert(profile).get_sample_rate()),
            "full_scale_range": str(convert(profile).get_full_scale_range()),
            "rotation_to_color": rotations[label],
        }
    config.set_frame_aggregate_output_mode(ob.OBFrameAggregateOutputMode.ANY_SITUATION)
    samples = {"accel": [], "gyro": []}
    try:
        pipeline.start(config)
        start = time.monotonic()
        while time.monotonic() - start < seconds + 1:
            frames = pipeline.wait_for_frames(200)
            elapsed = time.monotonic() - start
            if frames is None or elapsed < 1:
                continue
            for label, frame in (("accel", frames.get_accel_frame()),
                                 ("gyro", frames.get_gyro_frame())):
                if frame is None:
                    continue
                timestamp = frame.get_timestamp_us()
                if samples[label] and timestamp == samples[label][-1][1]:
                    continue
                value = frame.get_value()
                samples[label].append([elapsed, timestamp, value.x, value.y, value.z,
                                       frame.get_temperature()])
    finally:
        pipeline.stop()
    summary = {label: summarize(rows, rotations[label], 200, seconds)
               for label, rows in samples.items()}
    accel, gyro = summary["accel"], summary["gyro"]
    summary["optical_axis_inclination_abs_deg"] = inclination(accel["mean_color_xyz"])
    # A quiet sample is a candidate for static calibration, not proof of accuracy.
    summary["stationary_candidate"] = (
        abs(accel["mean_vector_norm"] - 9.80665) < 0.5
        and max(accel["stddev_color_xyz"]) < 0.05
        and gyro["mean_vector_norm"] < 0.03
        and max(gyro["stddev_color_xyz"]) < 0.01)
    summary["tilt_sign_verified"] = False
    return {"metadata": metadata, "summary": summary, "samples": samples}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--seconds", type=float, default=8, help="Sample duration, 2..60 seconds")
    parser.add_argument("--serial")
    parser.add_argument("--output", type=Path, help="Save metadata, summary and raw samples as JSON")
    args = parser.parse_args()
    if not math.isfinite(args.seconds) or not 2 <= args.seconds <= 60:
        parser.error("--seconds must be finite and between 2 and 60")
    try:
        result = read_imu(args.seconds, args.serial)
        if args.output:
            args.output.write_text(json.dumps(result, indent=2, allow_nan=False) + "\n")
        print(json.dumps({key: result[key] for key in ("metadata", "summary")}, indent=2,
                         allow_nan=False))
    except (Exception, KeyboardInterrupt) as exc:
        parser.exit(1, f"IMU read failed: {exc}\n")


if __name__ == "__main__":
    main()
