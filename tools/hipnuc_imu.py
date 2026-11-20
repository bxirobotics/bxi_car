#!/usr/bin/env python3
"""Publish the external HiPNUC HI91 IMU by passively reading its serial stream."""

import argparse
import binascii
from contextlib import contextmanager, suppress
import errno
import fcntl
import math
import os
from pathlib import Path
import select
import signal
import stat
import struct
import termios
import time
from typing import NamedTuple


USB_SERIAL = os.environ.get("CAR2_IMU_USB_SERIAL", "").strip()
DEFAULT_PORT = (
    "/dev/serial/by-id/usb-Silicon_Labs_CP2102N_USB_to_UART_Bridge_Controller_"
    f"{USB_SERIAL}-if00-port0"
)
TOPIC = "/hardware/imu_data"
FRAME_ID = "imu_link"

SYNC = b"\x5a\xa5"
HEADER = struct.Struct("<2sHH")
PAYLOAD = struct.Struct("<BHbfI3f3f3f3f4f")
FRAME_SIZE = HEADER.size + PAYLOAD.size
MS_NS = 1_000_000
MAX_AGE_NS = 50 * MS_NS
MAX_LEAD_NS = 25 * MS_NS
TIMEOUT_NS = 200 * MS_NS
MAX_BURST_BYTES = (MAX_AGE_NS // (2 * MS_NS) + 1) * FRAME_SIZE
MAX_READ_BYTES = 4096
# Allow small oscillator drift without moving the timestamp forward to hide lag.
DRIFT_PPM = 1000


class ImuError(RuntimeError):
    pass


class Sample(NamedTuple):
    tick_ms: int
    acceleration: tuple[float, float, float]
    gyro: tuple[float, float, float]
    quaternion: tuple[float, float, float, float]  # w, x, y, z


class Decoder:
    def __init__(self):
        self._pending = bytearray()

    def feed(self, data: bytes) -> list[Sample]:
        pending = self._pending
        pending.extend(data)
        samples = []
        offset = 0
        while True:
            offset = pending.find(SYNC, offset)
            if offset < 0:
                pending[:] = SYNC[:1] if pending.endswith(SYNC[:1]) else b""
                return samples
            available = len(pending) - offset
            if available < HEADER.size:
                break
            _, length, crc = HEADER.unpack_from(pending, offset)
            if length != PAYLOAD.size:
                offset += 1
                continue
            if available < FRAME_SIZE:
                break
            payload = pending[offset + HEADER.size:offset + FRAME_SIZE]
            if binascii.crc_hqx(pending[offset:offset + 4] + payload, 0) != crc:
                offset += 1
                continue
            offset += FRAME_SIZE
            fields = PAYLOAD.unpack(payload)
            if fields[0] != 0x91:
                continue
            accel, gyro, quat = fields[5:8], fields[8:11], fields[17:21]
            norm = math.hypot(*quat)
            if not all(map(math.isfinite, accel + gyro)) or not 0.9 <= norm <= 1.1:
                raise ImuError("Invalid HI91 acceleration, angular rate or quaternion")
            samples.append(Sample(fields[4], tuple(x * 9.8 for x in accel),
                                  tuple(map(math.radians, gyro)), tuple(x / norm for x in quat)))
        del pending[:offset]
        return samples


class SampleClock:
    def __init__(self):
        self._tick = None
        self._sample_ns = 0
        self._first_receive_ns = 0
        self._previous_receive_ns = 0
        self._previous_wall_ns = 0
        self._previous_stamp_ns = 0

    def stamp(self, samples: list[Sample], received_ns: int, wall_ns: int) -> int | None:
        """Validate every sample in a batch and stamp only its newest sample."""
        if not samples:
            return None
        tick = self._tick
        elapsed_ns = 0
        between_reads_ns = 0
        for index, sample in enumerate(samples):
            if tick is not None:
                delta_ns = ((sample.tick_ms - tick) & 0xffffffff) * MS_NS
                if not 0 < delta_ns < TIMEOUT_NS:
                    raise ImuError("HI91 tick duplicated, reordered, or reset")
                if index == 0:
                    between_reads_ns = delta_ns
                elapsed_ns += delta_ns
            tick = sample.tick_ms

        burst_ns = elapsed_ns - between_reads_ns
        if self._tick is None:
            self._first_receive_ns = received_ns
            candidate_ns = received_ns
        else:
            host_elapsed_ns = received_ns - self._previous_receive_ns
            if not 0 < host_elapsed_ns < TIMEOUT_NS:
                raise ImuError("No timely HI91 samples")
            if abs(wall_ns - self._previous_wall_ns - host_elapsed_ns) > MAX_AGE_NS:
                raise ImuError("Host wall clock stepped during IMU sampling")
            candidate_ns = self._sample_ns + elapsed_ns

        lag_ns = received_ns - candidate_ns
        if lag_ns < -MAX_LEAD_NS or max(lag_ns, 0) + burst_ns > MAX_AGE_NS:
            raise ImuError("HI91 data is buffered or advancing at an unexpected rate")

        correction_ns = elapsed_ns * DRIFT_PPM // 1_000_000
        self._sample_ns = candidate_ns + max(-correction_ns, min(correction_ns, lag_ns))
        self._tick = tick
        self._previous_receive_ns = received_ns
        self._previous_wall_ns = wall_ns
        # Keep the unclamped estimate above so repeated fast reads cannot hide a backlog.
        stamp_ns = wall_ns - max(0, received_ns - self._sample_ns)
        if stamp_ns <= self._previous_stamp_ns:
            raise ImuError("IMU timestamps stopped increasing")
        self._previous_stamp_ns = stamp_ns
        # Establish progress over a full lag window before publishing.
        return stamp_ns if received_ns - self._first_receive_ns >= MAX_AGE_NS else None


def verify_usb_serial(fd: int):
    status = os.fstat(fd)
    if stat.S_ISCHR(status.st_mode):
        device = Path("/sys/dev/char", f"{os.major(status.st_rdev)}:{os.minor(status.st_rdev)}")
        for parent in device.resolve(strict=True).parents:
            if (parent / "idVendor").is_file():
                serial = parent / "serial"
                if serial.is_file() and serial.read_text(encoding="ascii").strip() == USB_SERIAL:
                    return
                break
    raise ImuError(f"Expected IMU USB adapter with serial {USB_SERIAL}")


@contextmanager
def open_serial(port: str, verify_identity: bool = True):
    if verify_identity and not USB_SERIAL:
        raise ImuError("Set CAR2_IMU_USB_SERIAL to the USB serial number of the external IMU")
    fd = os.open(port, os.O_RDONLY | os.O_NONBLOCK | os.O_NOCTTY)
    original = None
    exclusive = False
    try:
        if verify_identity:
            verify_usb_serial(fd)
        fcntl.flock(fd, fcntl.LOCK_EX | fcntl.LOCK_NB)
        fcntl.ioctl(fd, termios.TIOCEXCL)
        exclusive = True
        original = termios.tcgetattr(fd)
        raw = original[:]
        raw[0] = raw[1] = raw[3] = 0
        raw[2] &= ~(termios.CSIZE | termios.PARENB | termios.CSTOPB | termios.CRTSCTS)
        raw[2] |= termios.CS8 | termios.CLOCAL | termios.CREAD
        raw[4] = raw[5] = termios.B921600
        raw[6] = original[6][:]
        raw[6][termios.VMIN] = 1
        raw[6][termios.VTIME] = 0
        termios.tcsetattr(fd, termios.TCSANOW, raw)
        termios.tcflush(fd, termios.TCIFLUSH)
        yield fd
    finally:
        try:
            if original is not None:
                try:
                    termios.tcsetattr(fd, termios.TCSANOW, original)
                except termios.error as exc:
                    if exc.args[0] not in (errno.EIO, errno.ENXIO, errno.ENODEV):
                        raise
        finally:
            if exclusive:
                with suppress(OSError):
                    fcntl.ioctl(fd, termios.TIOCNXCL)
            os.close(fd)


def read_samples(fd: int):
    decoder = Decoder()
    clock = SampleClock()
    deadline_ns = time.monotonic_ns() + TIMEOUT_NS
    while True:
        timeout = (deadline_ns - time.monotonic_ns()) / 1e9
        if timeout <= 0 or not select.select([fd], [], [], timeout)[0]:
            raise ImuError("Timed out waiting for valid HI91 data")
        try:
            data = os.read(fd, MAX_READ_BYTES)
        except BlockingIOError:
            continue
        received_ns = time.monotonic_ns()
        wall_ns = time.time_ns()
        if not data:
            raise ImuError("IMU serial adapter disconnected")
        if received_ns >= deadline_ns:
            raise ImuError("Timed out waiting for valid HI91 data")
        if len(data) > MAX_BURST_BYTES:
            raise ImuError("IMU serial input backlog is too large")
        samples = decoder.feed(data)
        if samples:
            stamp_ns = clock.stamp(samples, received_ns, wall_ns)
            deadline_ns = received_ns + TIMEOUT_NS
            if stamp_ns is not None:
                yield stamp_ns, samples[-1]


def publish_imu(port: str):
    # Keep diagnostics and parser tests independent of the ROS installation.
    import rclpy
    from rclpy.duration import Duration
    from rclpy.qos import QoSProfile, ReliabilityPolicy
    from rclpy.signals import SignalHandlerOptions
    from sensor_msgs.msg import Imu

    rclpy.init(args=[], signal_handler_options=SignalHandlerOptions.NO)
    try:
        node = rclpy.create_node("car2_imu", start_parameter_services=False)
        try:
            qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT,
                             lifespan=Duration(nanoseconds=MAX_AGE_NS))
            publisher = node.create_publisher(Imu, TOPIC, qos)
            # Only explicit pseudo terminals bypass USB identity verification for testing.
            verify_identity = port == DEFAULT_PORT or Path(port).resolve().parent != Path("/dev/pts")
            with open_serial(port, verify_identity) as fd:
                node.get_logger().info(f"Reading HI91 IMU on {port} at 921600 baud")
                for stamp_ns, sample in read_samples(fd):
                    message = Imu()
                    message.header.frame_id = FRAME_ID
                    message.header.stamp.sec, message.header.stamp.nanosec = divmod(stamp_ns, 10**9)
                    q = message.orientation
                    q.w, q.x, q.y, q.z = sample.quaternion
                    a, g = message.linear_acceleration, message.angular_velocity
                    a.x, a.y, a.z = sample.acceleration
                    g.x, g.y, g.z = sample.gyro
                    if not 0 <= time.time_ns() - stamp_ns <= MAX_AGE_NS:
                        raise ImuError("HI91 sample expired before publishing")
                    publisher.publish(message)
        finally:
            node.destroy_node()
    finally:
        rclpy.shutdown()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--port", default=DEFAULT_PORT,
                        help="Serial path (default: the USB adapter selected by CAR2_IMU_USB_SERIAL)")
    args = parser.parse_args()
    signal.signal(signal.SIGTERM, signal.default_int_handler)
    try:
        publish_imu(args.port)
    except KeyboardInterrupt:
        return
    except (ImuError, ImportError, OSError, termios.error) as exc:
        parser.exit(1, f"IMU publisher stopped: {exc}\n")


if __name__ == "__main__":
    main()
