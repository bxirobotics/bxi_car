"""Check a single fixed-board docking stroke against its independently measured TCP travel."""

import math

from .docking_travel import DockingTravel


# These are decision bounds, not a claim about the camera's physical accuracy.
MAX_UNCERTAINTY_M = .010
DEADBAND_M = .003
MAX_CORRECTION_M = .025
OBSERVATION_TIMEOUT_S = 2.1


class StrokeVerification:
    def __init__(self, config):
        self.meter = DockingTravel(config)
        self.reset_start()
        self.start_pose = None
        self.report = {}
        self.stage = None
        self.near_since = None
        self.stage_since = None
        self.request = None
        self.request_at = None

    def reset_start(self):
        self.meter.reset_start()
        self.baseline = dict(valid=False, reason="start_not_observed")

    def observe_start(self, frame, detection, pitch):
        self.baseline = self.meter.observe_start(frame, detection, pitch)
        return self.baseline["valid"]

    def capture(self, goal, pose):
        self.start_pose = (float(pose[0]), float(pose[1]))
        self.report = {
            "requested_m": [goal[0] - self.start_pose[0], goal[2] - self.start_pose[1]],
            "baseline": self.baseline["reason"],
        }

    def _measurement(self, frame, detection, capture_ns, barrier_ns, fresh):
        imu = (frame or {}).get("wrist_imu") or {}
        if not fresh or capture_ns is None or capture_ns <= barrier_ns:
            reason = "waiting_for_post_settle_image"
        elif not imu.get("valid"):
            reason = imu.get("reason", "wrist_imu_unavailable")
        else:
            # The entire stationary IMU window, as well as the image, must
            # postdate the receipt of the corresponding hardware standstill.
            first = imu.get("start_ms")
            timestamp = frame["timestamp_ms"]
            if (isinstance(first, (int, float)) and math.isfinite(first)
                    and 0 <= timestamp - first <= 250
                    and capture_ns - round((timestamp - first) * 1e6) > barrier_ns):
                return self.meter.measure(frame, detection)
            reason = "waiting_for_post_settle_imu"
        self.meter.reset_endpoint()
        return dict(valid=False, reason=reason)

    def update(self, frame, detection, pose, now, capture_ns, state, fresh, near):
        """Return phase, reason, and an optional one-use correction in shoulder X/Z."""
        self.report["encoder_travel_m"] = [float(pose[i]) - self.start_pose[i] for i in (0, 1)]
        if state is None:
            if near and self.near_since is None:
                self.near_since = now
            if self.near_since is not None and now - self.near_since > 3.:
                return "complete", "travel_unverified_no_settle_ack", None
            return "approaching", "approaching", None
        stage = state["stage"]
        if stage != self.stage:
            self.stage, self.stage_since = stage, now
            self.meter.reset_endpoint()
        if stage == "cancelled":
            return "cancelled", "stroke_cancelled_by_controller", None
        if stage == "correcting":
            if now - self.stage_since > 8.:
                return "cancelled", "correction_settle_timeout", None
            return "correcting", "correcting", None
        if stage == "complete" and self.request is None:
            return "complete", "travel_unverified_no_correction_request", None
        if stage == "complete" and self.request == [0., 0.]:
            return "complete", "travel_within_measurement_tolerance", None
        if stage == "settled" and self.request is not None:
            if now - self.request_at > 1.:
                return "complete", "correction_ack_missing", None
            return "verifying", "waiting_for_correction_ack", None

        final = stage == "complete"
        if not self.baseline["valid"]:
            return "complete", "travel_unverified_no_start_reference", None
        measurement = self._measurement(frame, detection, capture_ns, state["observed_ns"], fresh)
        key = "after_correction" if final else "at_nominal_goal"
        self.report[key] = measurement
        if measurement["valid"]:
            travel = measurement["travel_m"]
            uncertainty = measurement["uncertainty_m"]
            error = [cmd - actual for cmd, actual in zip(self.report["requested_m"], travel)]
            self.report[key] = dict(measurement, residual_m=error)
            if (len(error) == 2 and math.isfinite(math.hypot(*error))
                    and 0 <= uncertainty <= MAX_UNCERTAINTY_M):
                threshold = uncertainty + DEADBAND_M
                if final:
                    reason = ("corrected_travel_within_tolerance"
                              if all(abs(e) <= threshold for e in error)
                              else "correction_finished_residual_remains")
                    return "complete", reason, None
                if math.hypot(*error) > MAX_CORRECTION_M:
                    return "complete", "travel_error_exceeds_correction_limit", None
                self.request = [e if abs(e) > threshold else 0. for e in error]
                self.request_at = now
                self.report["correction_requested_m"] = list(self.request)
                return "verifying", "correction_requested", list(self.request)
            self.report[key]["reason"] = "travel_uncertainty_too_large"
        if now - self.stage_since > OBSERVATION_TIMEOUT_S:
            return "complete", ("corrected_travel_unverified" if final else "travel_unverified"), None
        return "verifying", ("measuring_after_correction" if final else "measuring_travel"), None
