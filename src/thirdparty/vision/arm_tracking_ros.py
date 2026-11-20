"""ROS transport for the optional arm tracker; preview mode only subscribes."""

import json
import math
import secrets
import time


_OWNERSHIP_CHANGED_REASON = (
    "shared control ownership changed; use the active source command interface")


class TrackingRosLink:
    def __init__(self, execute=False, joystick=False, track_x=False, goal_mode=False):
        if joystick and not execute:
            raise ValueError("joystick mode requires control transport")
        if goal_mode and not track_x:
            raise ValueError("recorded-goal control requires X tracking")
        import rclpy
        from geometry_msgs.msg import PoseStamped
        from rclpy.context import Context
        from rclpy.duration import Duration
        from rclpy.executors import SingleThreadedExecutor
        from rclpy.qos import QoSProfile, ReliabilityPolicy

        self.rclpy = rclpy
        self.context = Context()
        rclpy.init(args=[], context=self.context)
        self.node = rclpy.create_node("arm_visual_tracker", context=self.context)
        self.executor = SingleThreadedExecutor(context=self.context)
        self.executor.add_node(self.node)
        self.execute = execute
        self.track_x = track_x
        self.goal_mode = goal_mode
        self.enabled = False
        self.enable_requested = False
        self.shared_control_generation = None
        self.stroke_id = None
        self.stroke_state = None
        self._base_tilt = None
        self._base_stamp_ns = 0
        self.joystick_received_at = None
        self.joystick_released = False
        self.joystick_request = False
        self.joystick_cancel_pending = False
        self.pose = self.pose_received_at = None
        self.pose_stamp_ns = None
        self.sending = False
        self.qos = QoSProfile(depth=1, lifespan=Duration(seconds=0.25))
        pose_qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT)
        self.subscription = self.node.create_subscription(
            PoseStamped, "/arm_pose", self._observe_pose, pose_qos)
        if execute:
            from geometry_msgs.msg import PointStamped
            from std_msgs.msg import Empty, String
            from rcl_interfaces.srv import GetParameters, SetParametersAtomically

            self.target_publisher = self.node.create_publisher(
                PointStamped, "/arm_tracking_target", self.qos)
            self.shared_target_publisher = self.node.create_publisher(
                String, "/car2/arm_tracking_target", self.qos)
            if goal_mode:
                from sensor_msgs.msg import Imu
                self.base_imu_subscription = self.node.create_subscription(
                    Imu, "/hardware/imu_data", self._observe_base_tilt, pose_qos)
                self.goal_publisher = self.node.create_publisher(
                    PointStamped, "/arm_tracking_goal_target", self.qos)
                self.shared_goal_publisher = self.node.create_publisher(
                    String, "/car2/arm_tracking_goal_target", self.qos)
                self.correction_publisher = self.node.create_publisher(
                    String, "/car2/arm_tracking_correction", self.qos)
                self.stroke_subscription = self.node.create_subscription(
                    String, "/car2/arm_tracking_stroke_state", self._observe_stroke, self.qos)
            self.stop_publisher = self.node.create_publisher(
                Empty, "/arm_tracking_stop", self.qos)
            self.get_client = self.node.create_client(GetParameters, "/chassis/get_parameters")
            self.set_client = self.node.create_client(
                SetParametersAtomically, "/chassis/set_parameters_atomically")
        if joystick:
            from std_msgs.msg import Bool
            self.joystick_subscription = self.node.create_subscription(
                Bool, "/arm_tracking_requested", self._observe_joystick, self.qos)

    def _observe_joystick(self, message):
        now = time.monotonic()
        if (self.joystick_received_at is None
                or now - self.joystick_received_at > 0.3):
            self.require_joystick_release()
        self.joystick_received_at = now
        if not message.data:
            self.joystick_cancel_pending |= self.joystick_request
            self.joystick_released = True
            self.joystick_request = False
        elif self.joystick_released:
            self.joystick_request = True

    def _observe_base_tilt(self, message):
        # The external IMU is mounted Z up. Its heading/XY mounting angle is
        # immaterial for bounding the angle between true vertical and shoulder Z.
        stamp = message.header.stamp.sec * 1_000_000_000 + message.header.stamp.nanosec
        age_ns = self.node.get_clock().now().nanoseconds - stamp
        self._base_tilt = None
        if (message.header.frame_id != "imu_link" or message.orientation_covariance[0] == -1.
                or stamp <= self._base_stamp_ns or not 0 <= age_ns <= 200_000_000):
            return
        self._base_stamp_ns = stamp
        q = message.orientation
        norm = math.hypot(q.x, q.y, q.z, q.w)
        if .9 <= norm <= 1.1:
            self._base_tilt = 2 * math.asin(min(1., math.hypot(q.x, q.y) / norm))

    def base_tilt_at(self, capture_ns):
        if capture_ns is not None and abs(capture_ns - self._base_stamp_ns) <= 150_000_000:
            return self._base_tilt
        return None

    def begin_verified_stroke(self):
        """Opt in only with a shared controller advertising the verification protocol."""
        if (not self.enabled or not self.goal_mode or self.shared_control_generation is None
                or self.correction_publisher.get_subscription_count() == 0
                or self.node.count_publishers("/car2/arm_tracking_stroke_state") == 0):
            return None
        self.stroke_id = secrets.randbits(62) + 1
        self.stroke_state = None
        return self.stroke_id

    def _observe_stroke(self, message):
        if self.stroke_id is None or not self.enabled or len(message.data) > 512:
            return
        try:
            data = json.loads(message.data)
            if not isinstance(data, dict) or any(type(data.get(k)) is not int for k in
                                                 ("generation", "stroke_id", "settled_ns")):
                return
            stage = data["stage"]
            ranks = {"settled": 1, "correcting": 2, "complete": 3, "cancelled": 3}
            now_ns = self.node.get_clock().now().nanoseconds
            if (data["generation"] != self.shared_control_generation
                    or data["stroke_id"] != self.stroke_id or stage not in ranks
                    or not 0 <= data["settled_ns"] <= now_ns
                    or (stage != "cancelled" and data["settled_ns"] == 0)):
                return
            old = self.stroke_state
            if old and (old["settled_ns"] != data["settled_ns"]
                        or ranks[stage] <= ranks[old["stage"]]):
                return
            self.stroke_state = dict(stage=stage, settled_ns=data["settled_ns"], observed_ns=now_ns)
        except (ValueError, KeyError, TypeError):
            return

    def require_joystick_release(self):
        # A restarted vision process or recovered stream cannot replay a held request.
        self.joystick_cancel_pending |= self.joystick_request
        self.joystick_received_at = None
        self.joystick_released = self.joystick_request = False

    def take_joystick_cancel(self):
        cancelled = self.joystick_cancel_pending
        self.joystick_cancel_pending = False
        return cancelled

    def joystick_requested(self, now):
        if (self.joystick_received_at is None
                or not 0 <= now - self.joystick_received_at <= 0.3):
            self.require_joystick_release()
            return False
        return self.joystick_request

    def _observe_pose(self, message):
        p, q = message.pose.position, message.pose.orientation
        values = (p.x, p.y, p.z, q.x, q.y, q.z, q.w)
        stamp = message.header.stamp.sec * 1000000000 + message.header.stamp.nanosec
        age = (self.node.get_clock().now().nanoseconds - stamp) / 1e9
        if (message.header.frame_id != "arm_shoulder"
                or not all(math.isfinite(v) for v in values)
                or abs(p.y) > 1e-6 or abs(q.x) > 1e-6 or abs(q.z) > 1e-6
                or abs(q.y * q.y + q.w * q.w - 1.0) > 1e-6
                or stamp <= 0 or not 0 <= age <= 0.25
                or (self.pose_stamp_ns is not None and stamp <= self.pose_stamp_ns)):
            self.pose = self.pose_received_at = None
            return
        pitch = -2.0 * math.atan2(q.y, q.w)
        self.pose = [float(p.x), float(p.z), math.atan2(math.sin(pitch), math.cos(pitch))]
        self.pose_received_at = time.monotonic() - age
        self.pose_stamp_ns = stamp

    def spin(self):
        self.executor.spin_once(timeout_sec=0)

    def fresh_pose(self, now, max_age=0.25):
        if (self.pose is None or self.pose_received_at is None
                or not 0 <= now - self.pose_received_at <= max_age):
            return None, None
        return list(self.pose), self.pose_received_at

    def capture_stamp(self, age_s=0.0):
        now = self.node.get_clock().now()
        ns = now.nanoseconds - int(max(0.0, age_s) * 1e9)
        stamp = now.to_msg()
        stamp.sec, stamp.nanosec = divmod(ns, 1000000000)
        return stamp

    def _wait(self, future, timeout=2.0):
        deadline = time.monotonic() + timeout
        while not future.done() and time.monotonic() < deadline:
            self.executor.spin_once(timeout_sec=0.02)
        if not future.done():
            raise RuntimeError("arm tracking parameter service timed out")
        result = future.result()
        if result is None:
            raise RuntimeError("arm tracking parameter service returned no result")
        return result

    def _set_enabled(self, value):
        from rcl_interfaces.srv import SetParametersAtomically
        from rclpy.parameter import Parameter

        request = SetParametersAtomically.Request()
        request.parameters = [
            Parameter("arm_tracking_enabled", value=value).to_parameter_msg(),
            Parameter("arm_tracking_x_enabled", value=bool(value and self.track_x)).to_parameter_msg(),
            Parameter("arm_tracking_goal_enabled", value=bool(value and self.goal_mode)).to_parameter_msg(),
        ]
        if self.shared_control_generation is not None:
            request.parameters.append(Parameter(
                "shared_control_request_generation",
                value=self.shared_control_generation).to_parameter_msg())
        response = self._wait(self.set_client.call_async(request))
        if not response.result.successful:
            # Handover already stops tracking. Never retry cleanup using a newer
            # owner's generation, and do not hide unrelated controller failures.
            if (not value and self.shared_control_generation is not None
                    and response.result.reason == _OWNERSHIP_CHANGED_REASON):
                return
            raise RuntimeError("arm tracking mode rejected: " + response.result.reason)

    def enable(self):
        if not self.execute:
            raise RuntimeError("preview cannot enable motion")
        from rcl_interfaces.msg import ParameterType
        from rcl_interfaces.srv import GetParameters

        if not self.get_client.wait_for_service(timeout_sec=2.0) or not self.set_client.wait_for_service(timeout_sec=2.0):
            raise RuntimeError("/chassis tracking services unavailable; controller must be running")
        request = GetParameters.Request()
        request.names = ["arm_tracking_enabled", "arm_tracking_x_enabled", "arm_tracking_goal_enabled",
                         "shared_control_generation"]
        response = self._wait(self.get_client.call_async(request))
        if not response.values:
            # Humble rejects the whole read when any name is undeclared.
            # A legacy node can still provide the original three parameters.
            request.names = request.names[:3]
            response = self._wait(self.get_client.call_async(request))
        if (len(response.values) not in (3, 4)
                or any(value.type != ParameterType.PARAMETER_BOOL for value in response.values[:3])):
            raise RuntimeError("running controller does not provide recorded-goal control; load the new build first")
        generation = response.values[3] if len(response.values) == 4 else None
        if generation is None or generation.type == ParameterType.PARAMETER_NOT_SET:
            request_generation = None  # Legacy controllers do not declare this parameter.
        elif generation.type == ParameterType.PARAMETER_INTEGER and generation.integer_value >= 0:
            request_generation = generation.integer_value
        else:
            raise RuntimeError("running controller returned an invalid shared_control_generation")
        if response.values[0].bool_value:
            raise RuntimeError("arm tracking is already enabled; refusing to take another session")
        # Keep this snapshot through cleanup, including an accepted request whose
        # response is lost. Reading a new generation would authorize stale work.
        self.shared_control_generation = request_generation
        # A response may be lost after acceptance. Cleanup must still send stop/disable.
        self.enable_requested = True
        self._set_enabled(True)
        self.enabled = True

    def publish(self, point, stamp):
        if not self.execute or not self.enabled:
            raise RuntimeError("arm tracking was not enabled")
        if self.shared_control_generation is not None:
            self._publish_shared(self.shared_target_publisher, point, stamp)
        else:
            from geometry_msgs.msg import PointStamped

            message = PointStamped()
            message.header.frame_id = "arm_shoulder"
            message.header.stamp = stamp
            message.point.x, message.point.y, message.point.z = map(float, point)
            self.target_publisher.publish(message)
        self.sending = True

    def _publish_shared(self, publisher, point, stamp, stroke_id=None):
        from std_msgs.msg import String

        x, y, z = map(float, point)
        # Fresh timestamps never grant a stale vision session a newer owner.
        # Only an explicit enable() can capture another generation.
        payload = {
            "generation": self.shared_control_generation,
            "stamp_ns": stamp.sec * 1000000000 + stamp.nanosec,
            "frame_id": "arm_shoulder", "x": x, "y": y, "z": z,
        }
        if stroke_id is not None:
            payload["stroke_id"] = stroke_id
        publisher.publish(String(data=json.dumps(payload, allow_nan=False)))

    def stop(self, force=False):
        if self.execute and self.enable_requested and (force or self.sending):
            if self.shared_control_generation is not None:
                # Empty cannot identify its owner. Use the same guarded cleanup
                # as disable so a delayed stop cannot affect a newer session.
                self._disable_parameters()
            else:
                from std_msgs.msg import Empty
                self.stop_publisher.publish(Empty())
        self.sending = False

    def publish_goal(self, point):
        if not self.execute or not self.enabled or not self.goal_mode:
            raise RuntimeError("recorded-goal control was not enabled")
        # This topic timestamps a current execution command, not a new camera observation.
        stamp = self.node.get_clock().now().to_msg()
        if self.shared_control_generation is not None:
            self._publish_shared(self.shared_goal_publisher, point, stamp, self.stroke_id)
        else:
            from geometry_msgs.msg import PointStamped

            message = PointStamped()
            message.header.frame_id = "arm_shoulder"
            message.header.stamp = stamp
            message.point.x, message.point.y, message.point.z = map(float, point)
            self.goal_publisher.publish(message)
        self.sending = True

    def publish_correction(self, correction, capture_ns):
        from std_msgs.msg import String

        state = self.stroke_state
        now_ns = self.node.get_clock().now().nanoseconds
        if (not self.enabled or self.stroke_id is None or state is None
                or state["stage"] != "settled" or type(capture_ns) is not int
                or not state["settled_ns"] < capture_ns <= now_ns
                or now_ns - capture_ns > 250_000_000):
            return False
        dx, dz = correction
        if not math.isfinite(math.hypot(dx, dz)) or math.hypot(dx, dz) > .025:
            return False
        payload = dict(generation=self.shared_control_generation, stroke_id=self.stroke_id,
                       settled_ns=state["settled_ns"], stamp_ns=capture_ns, dx=dx, dz=dz)
        self.correction_publisher.publish(String(data=json.dumps(payload, allow_nan=False)))
        return True

    def _disable_parameters(self):
        self._set_enabled(False)
        self.enabled = self.enable_requested = self.sending = False
        self.shared_control_generation = None
        self.stroke_id = self.stroke_state = None

    def disable(self):
        if self.execute and self.enable_requested:
            if self.shared_control_generation is None:
                self.stop(force=True)
            self._disable_parameters()

    def close(self):
        try:
            self.disable()
        finally:
            self.executor.shutdown(timeout_sec=1.0)
            self.node.destroy_node()
            if self.context.ok():
                self.rclpy.shutdown(context=self.context)
