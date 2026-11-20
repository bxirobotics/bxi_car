#!/usr/bin/env python3
"""Test the real main.cpp lifecycle only with the dedicated fake-vendor binary.

CMake must build main.cpp + shared_daemon_vendor_stub.cpp as a non-installed
test target with CHASSIS_CONTROL_MODE="chassis_only_remote" and the normal
chassis_controller/chassis_arm_control/chassis_remote_control static libraries.
Never point this test at a production controller. The embedded fake identity
is checked before execution and its startup token is checked before any start.
No ROS services, physical joystick, CAN device or motor hardware are accessed.
"""

import argparse
import importlib.util
import json
import mmap
import os
from pathlib import Path
import signal
import socket
import struct
import subprocess
import tempfile
import time
import unittest


ROOT = Path(__file__).resolve().parents[2]
FAKE_IDENTITY = b"CAR2_SHARED_DAEMON_FAKE_VENDOR_V1"
CLIENT_SPEC = importlib.util.spec_from_file_location(
    "shared_control", ROOT / "scripts/shared_control.py")
client = importlib.util.module_from_spec(CLIENT_SPEC)
CLIENT_SPEC.loader.exec_module(client)


def verified_fake_binary(path):
    path = path.resolve(strict=True)
    if not path.is_file() or not os.access(path, os.X_OK):
        raise ValueError("--binary must be an executable fake-vendor test target")
    with path.open("rb") as executable:
        if executable.read(4) != b"\x7fELF":
            raise ValueError("--binary must be the compiled fake-vendor ELF target")
        with mmap.mmap(executable.fileno(), 0, access=mmap.ACCESS_READ) as contents:
            if contents.find(FAKE_IDENTITY) < 0:
                raise ValueError("refusing to execute a binary without the fake-vendor identity")
    return path


def exchange(socket_path, action, pid, *, expired=False, timeout=2.0):
    with socket.socket(socket.AF_UNIX, socket.SOCK_STREAM) as connection:
        connection.settimeout(timeout)
        connection.connect(str(socket_path))
        peer_pid, peer_uid, _ = struct.unpack(
            "3i", connection.getsockopt(socket.SOL_SOCKET, socket.SO_PEERCRED, 12))
        if peer_pid != pid or peer_uid != os.geteuid():
            raise AssertionError("test socket belongs to an unexpected process")
        deadline = int(time.monotonic() * 1000) + (-1000 if expired else 5000)
        connection.sendall(f"v1 {action} {deadline}\n".encode("ascii"))
        data = bytearray()
        while b"\n" not in data:
            part = connection.recv(4096 - len(data))
            if not part or len(data) + len(part) >= 4096:
                raise AssertionError("incomplete or oversized lifecycle response")
            data.extend(part)
        response = json.loads(data)
        if response.get("pid") != pid or response.get("protocol") != 1:
            raise AssertionError("lifecycle response identity mismatch")
        return response


class SharedDaemonRecycle(unittest.TestCase):
    binary = None

    def setUp(self):
        self.temporary = tempfile.TemporaryDirectory(prefix="car2-recycle-")
        self.directory = Path(self.temporary.name)
        self.socket_path = self.directory / "control.sock"
        self.children = []
        self.logs = []

    def tearDown(self):
        try:
            for process, output in reversed(self.children):
                if process.poll() is None:
                    process.send_signal(signal.SIGINT)
                    try:
                        process.wait(timeout=6)
                    except subprocess.TimeoutExpired:
                        process.kill()
                        process.wait(timeout=3)
                output.close()
        finally:
            self.temporary.cleanup()

    def log_text(self, log):
        return log.read_text(errors="replace")

    def wait_until(self, predicate, process, log, timeout=10):
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            try:
                if predicate():
                    return
            except socket.timeout:
                # Hardware cleanup temporarily blocks the executor. Retry only
                # read-only predicates, within the overall cleanup deadline.
                pass
            if process.poll() is not None:
                self.fail(f"daemon exited {process.returncode}:\n{self.log_text(log)[-20000:]}")
            time.sleep(0.02)
        self.fail(f"daemon condition timed out:\n{self.log_text(log)[-20000:]}")

    def launch(self, *, fail_power_off=False):
        log = self.directory / f"daemon-{len(self.children)}.log"
        output = log.open("w")
        environment = dict(
            os.environ, ROS_LOCALHOST_ONLY="1", ROS_DOMAIN_ID="198",
            ROS_LOG_DIR=str(self.directory), CHASSIS_CAN_LOG_DIR=str(self.directory / "can"),
            CHASSIS_ARM_TORQUE_LOG_DIR=str(self.directory / "arm"),
            CHASSIS_CONFIG_DIR=str(ROOT / "src/config"), CAR2_FAKE_VENDOR_TEST="1",
            FAKE_BXI_FAIL_POWER_OFF="1" if fail_power_off else "0")
        command = [str(self.binary), "--ros-args", "-p", "shared_control:=true",
                   "-p", f"shared_control_socket:={self.socket_path}",
                   "-p", f"device_path:={self.directory}/missing-joystick"]
        try:
            process = subprocess.Popen(command, cwd=self.directory, env=environment,
                                       stdout=output, stderr=subprocess.STDOUT)
        except BaseException:
            output.close()
            raise
        self.children.append((process, output))
        self.logs.append(log)
        self.wait_until(
            lambda: self.socket_path.exists() and "SHARED_CONTROL waiting" in self.log_text(log),
            process, log)
        self.assertIn("FAKE_BXI_STUB_READY " + FAKE_IDENTITY.decode(), self.log_text(log))
        status = exchange(self.socket_path, "status", process.pid)
        self.assertTrue(status["ok"])
        self.assertEqual(status["stage"], "waiting")
        self.assertEqual(status["owner"], "none")
        self.assertEqual(status["generation"], 0)
        self.assertFalse(status["hardware_active"])
        self.assertFalse(status["restart_required"])
        self.assertFalse(status["arm_enabled"], "test target must compile chassis_only_remote")
        self.assert_no_hardware(log)
        return process, log

    def assert_no_hardware(self, log):
        text = self.log_text(log)
        self.assertNotIn("FAKE_BXI_INIT", text)
        self.assertNotIn("FAKE_BXI_POWER_", text)
        self.assertNotIn("FAKE_BXI_EXIT", text)

    def begin_then_cancel(self, process, log):
        accepted = exchange(self.socket_path, "start", process.pid)
        self.assertTrue(accepted["ok"])
        self.assertIn(accepted["stage"], ("requested", "starting"))
        self.wait_until(lambda: "FAKE_BXI_POWER_ON" in self.log_text(log), process, log)
        # The real controller has a four-second cancellable power-settle wait;
        # stop immediately on the flushed token, before any calibration starts.
        stopped = exchange(self.socket_path, "stop", process.pid)
        self.assertTrue(stopped["ok"])
        self.assertIn(stopped["stage"], ("stopping", "waiting", "fault"))

    def test_clean_stop_exits_and_new_process_waits_without_reusing_hardware(self):
        process, log = self.launch()
        self.begin_then_cancel(process, log)
        observed = {}

        def stopped():
            observed.update(exchange(self.socket_path, "status", process.pid))
            return not observed["hardware_active"] and observed["stage"] == "waiting"

        self.wait_until(stopped, process, log, timeout=20)
        self.assertTrue(observed["restart_required"])
        self.assertEqual(observed["owner"], "none")
        self.assertGreater(observed["generation"], 0)
        rejected = exchange(self.socket_path, "start", process.pid)
        self.assertFalse(rejected["ok"])
        self.assertTrue(rejected["restart_required"])
        self.assertIn("restarting", rejected["error"])
        process.wait(timeout=5)
        self.assertEqual(process.returncode, 0, self.log_text(log))
        self.assertFalse(self.socket_path.exists(), "old endpoint must be removed before restart")
        text = self.log_text(log)
        self.assertEqual(text.count("FAKE_BXI_INIT"), 1)
        self.assertIn("FAKE_BXI_POWER_OFF", text)
        self.assertIn("FAKE_BXI_POWER_GET 0", text)
        self.assertIn("FAKE_BXI_EXIT", text)
        self.assertIn("SHARED_CONTROL restarting after clean stop", text)
        self.assertNotIn("SHARED_CONTROL fault:", text)

        fresh, fresh_log = self.launch()
        self.assertNotEqual(fresh.pid, process.pid)
        old_request = exchange(self.socket_path, "start", fresh.pid, expired=True)
        self.assertFalse(old_request["ok"])
        self.assertIn("expired", old_request["error"])
        # Observe beyond the old process's restart delay. No new start request
        # was sent, and no old source/owner/generation may resume the hardware.
        time.sleep(1.3)
        status = exchange(self.socket_path, "status", fresh.pid)
        self.assertEqual(status["stage"], "waiting")
        self.assertEqual(status["generation"], 0)
        self.assertEqual(status["owner"], "none")
        self.assertFalse(status["hardware_active"])
        self.assertFalse(status["restart_required"])
        self.assertIsNone(fresh.poll())
        self.assert_no_hardware(fresh_log)

    def test_unconfirmed_power_off_latches_fault_and_does_not_recycle(self):
        process, log = self.launch(fail_power_off=True)
        self.begin_then_cancel(process, log)
        observed = {}

        def faulted():
            observed.update(exchange(self.socket_path, "status", process.pid))
            return observed["stage"] == "fault"

        self.wait_until(faulted, process, log, timeout=20)
        self.assertTrue(observed["hardware_active"], "power off was not confirmed")
        self.assertFalse(observed["restart_required"])
        self.assertTrue(observed["error"])
        self.assertEqual(observed["owner"], "none")
        self.assertIn("FAKE_BXI_POWER_OFF_FAILED", self.log_text(log))
        self.assertIn("FAKE_BXI_POWER_GET 1", self.log_text(log))
        rejected = exchange(self.socket_path, "start", process.pid)
        self.assertFalse(rejected["ok"])
        with self.assertRaisesRegex(client.ControlError, "hardware shutdown was not confirmed"):
            client.run_action(str(self.socket_path), "stop", timeout=0.3)
        time.sleep(1.5)
        self.assertIsNone(process.poll(), "fault must remain queryable, without automatic restart")
        status = exchange(self.socket_path, "status", process.pid)
        self.assertEqual(status["stage"], "fault")
        self.assertTrue(status["hardware_active"])
        self.assertFalse(status["restart_required"])
        self.assertEqual(self.log_text(log).count("FAKE_BXI_INIT"), 1)
        self.assertNotIn("SHARED_CONTROL restarting after clean stop", self.log_text(log))


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--binary", required=True, type=Path)
    arguments, extra = parser.parse_known_args()
    try:
        SharedDaemonRecycle.binary = verified_fake_binary(arguments.binary)
    except (OSError, ValueError) as error:
        parser.error(str(error))
    unittest.main(argv=[__file__, *extra])
