#!/usr/bin/env python3
"""Real daemon lifecycle with startup rejected by parameter validation before hardware.

The deliberately negative post_calibration_rpm is mandatory in this test: it
prevents constructing HardwareOwner or any CAN/EtherCAT/power objects.
"""
import argparse
import importlib.util
import os
from pathlib import Path
import signal
import subprocess
import tempfile
import time
import unittest

ROOT = Path(__file__).resolve().parents[2]
spec = importlib.util.spec_from_file_location("shared_client", ROOT / "scripts/shared_control.py")
client = importlib.util.module_from_spec(spec)
spec.loader.exec_module(client)


class SharedDaemon(unittest.TestCase):
    binary = None

    def test_waiting_and_fault_are_not_hardware_ready(self):
        with tempfile.TemporaryDirectory(prefix="car2-shared-daemon-") as directory:
            socket_path = str(Path(directory) / "control.sock")
            environment = dict(os.environ, ROS_LOCALHOST_ONLY="1", ROS_DOMAIN_ID="199",
                               ROS_LOG_DIR=directory, CHASSIS_CAN_LOG_DIR=directory)
            command = [str(self.binary), "--ros-args", "-p", "shared_control:=true",
                       "-p", f"shared_control_socket:={socket_path}",
                       "-p", "post_calibration_rpm:=-1.0",  # Never remove this hardware guard.
                       "-p", f"device_path:={directory}/missing-joystick"]
            with open(Path(directory) / "daemon.log", "w+") as output:
                process = subprocess.Popen(command, cwd=directory, env=environment,
                                           stdout=output, stderr=subprocess.STDOUT)
                try:
                    deadline = time.monotonic() + 8
                    while not Path(socket_path).exists() and process.poll() is None:
                        if time.monotonic() > deadline:
                            self.fail("daemon did not expose waiting endpoint")
                        time.sleep(0.02)
                    status = client.exchange(socket_path, "status")
                    self.assertEqual(status["pid"], process.pid)
                    self.assertEqual(status["stage"], "waiting")
                    self.assertFalse(status["hardware_active"])
                    self.assertFalse(status["arm_enabled"])
                    self.assertEqual(client.run_action(socket_path, "stop", 2)["stage"], "waiting")
                    client.exchange(socket_path, "start")
                    while time.monotonic() < deadline:
                        status = client.exchange(socket_path, "status")
                        if status["stage"] == "fault" and not status["hardware_active"]:
                            break
                        time.sleep(0.02)
                    self.assertEqual(status["stage"], "fault")
                    self.assertFalse(status["hardware_active"])
                    self.assertIn("post_calibration_rpm must be", status["error"])
                    with self.assertRaisesRegex(client.ControlError, "faulted"):
                        client.exchange(socket_path, "start")
                    stopped = client.run_action(socket_path, "stop", 2)
                    self.assertEqual(stopped["stage"], "fault")
                    self.assertEqual(stopped["owner"], "none")
                    self.assertIsNone(process.poll(), "fault remains queryable without restarting")
                finally:
                    process.send_signal(signal.SIGINT)
                    try:
                        process.wait(timeout=5)
                    except subprocess.TimeoutExpired:
                        process.kill()
                        process.wait()
                    output.seek(0)
                    log = output.read()
                self.assertIn("SHARED_CONTROL waiting", log)
                self.assertIn("SHARED_CONTROL stopped: stage=fault", log)
                self.assertNotIn("EtherCAT arm IK ready", log)
                self.assertFalse(Path(socket_path).exists(), "clean shutdown removes endpoint")


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("--binary", required=True, type=Path)
    arguments, extra = parser.parse_known_args()
    SharedDaemon.binary = arguments.binary
    unittest.main(argv=[__file__, *extra])
