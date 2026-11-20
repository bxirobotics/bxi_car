#!/usr/bin/env python3
"""Offline lifecycle client tests; all endpoints are temporary local sockets."""

import importlib.util
import contextlib
import io
import json
import os
from pathlib import Path
import socket
import subprocess
import sys
import tempfile
import threading
import time
import unittest
from unittest import mock


SCRIPT = Path(__file__).resolve().parents[2] / "scripts" / "shared_control.py"
SPEC = importlib.util.spec_from_file_location("shared_control", SCRIPT)
client = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(client)


def response(**overrides):
    value = dict(protocol=1, ok=True, pid=os.getpid(), stage="waiting", owner="none",
                 generation=0, hardware_active=False, arm_enabled=True, error="")
    value.update(overrides)
    return value


class FakeController:
    def __init__(self, answer=lambda action, number: response()):
        self.answer = answer
        self.actions = []
        self.errors = []
        self.done = threading.Event()

    def __enter__(self):
        self.directory = tempfile.TemporaryDirectory(prefix="car2-shared-client-")
        self.path = str(Path(self.directory.name) / "control.sock")
        self.listener = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM)
        self.listener.bind(self.path)
        self.listener.listen(4)
        self.listener.settimeout(0.05)
        self.thread = threading.Thread(target=self.serve, daemon=True)
        self.thread.start()
        return self

    def serve(self):
        while not self.done.is_set():
            try:
                connection, _ = self.listener.accept()
            except socket.timeout:
                continue
            except OSError:
                break
            try:
                with connection:
                    connection.settimeout(1)
                    line = bytearray()
                    while b"\n" not in line:
                        part = connection.recv(256)
                        if not part:
                            break
                        line.extend(part)
                    if not line:
                        continue
                    version, action, deadline = line.decode("ascii").strip().split()
                    assert version == "v1"
                    assert time.monotonic() * 1000 < int(deadline) <= time.monotonic() * 1000 + 5100
                    self.actions.append(action)
                    answer = self.answer(action, len(self.actions))
                    if answer is not None:
                        connection.sendall(json.dumps(answer).encode() + b"\n")
            except (BrokenPipeError, ConnectionResetError):
                pass
            except Exception as error:
                self.errors.append(error)

    def __exit__(self, *unused):
        self.done.set()
        self.listener.close()
        self.thread.join(2)
        self.directory.cleanup()
        if self.errors:
            raise self.errors[0]


class SharedControlClientTests(unittest.TestCase):
    def test_status_uses_real_peer_identity_without_starting(self):
        with FakeController() as server:
            state = client.run_action(server.path, "status", 1)
        self.assertEqual(state["pid"], os.getpid())
        self.assertEqual(server.actions, ["status"])
        self.assertFalse(state["hardware_active"])

    def test_mismatched_peer_is_rejected_before_sending_a_command(self):
        with FakeController() as server:
            with self.assertRaisesRegex(client.ControlError, "instance changed") as raised:
                client.exchange(server.path, "start", 1, os.getpid() + 1)
        self.assertEqual(server.actions, [])
        self.assertFalse(raised.exception.request_sent)

    def test_invalid_identity_or_state_is_not_success(self):
        for invalid in [response(pid=os.getpid() + 1), response(protocol=True),
                        response(stage="unknown"), response(stage=[]), response(owner={}),
                        response(generation=-1),
                        response(stage="running", hardware_active=False),
                        response(ok=False, error="fault latched")]:
            with self.subTest(invalid=invalid), FakeController(lambda action, number: invalid) as server:
                with self.assertRaises(client.ControlError) as raised:
                    client.exchange(server.path, "status", 1)
                self.assertFalse(raised.exception.request_sent)

    def test_start_preflights_then_returns_acceptance(self):
        def answer(action, number):
            return response(stage="requested", owner="none", generation=1) if action == "start" else response()
        with FakeController(answer) as server, mock.patch.object(client, "other_controllers", return_value=[]):
            state = client.run_action(server.path, "start", 1)
        self.assertEqual(server.actions, ["status", "start"])
        self.assertEqual(state["stage"], "requested")
        self.assertFalse(state["hardware_active"])

    def test_other_controller_blocks_start_without_killing_it(self):
        conflict = [{"pid": 123, "executable": "/legacy/build/chassis/robot_control"}]
        with FakeController() as server, mock.patch.object(client, "other_controllers", return_value=conflict):
            with self.assertRaisesRegex(client.ControlError, "another hardware controller") as raised:
                client.run_action(server.path, "start", 1)
        self.assertEqual(server.actions, ["status"])
        self.assertFalse(raised.exception.request_sent)

    def test_preflight_disconnect_does_not_mark_start_sent(self):
        with FakeController(lambda action, number: None) as server:
            with self.assertRaisesRegex(client.ControlError, "closed without") as raised:
                client.run_action(server.path, "start", 1)
        self.assertEqual(server.actions, ["status"])
        self.assertFalse(raised.exception.request_sent)

    def test_start_disconnect_after_send_is_explicit_in_cli_error_json(self):
        def answer(action, number):
            return response() if action == "status" else None
        output = io.StringIO()
        with FakeController(answer) as server, mock.patch.object(client, "other_controllers", return_value=[]):
            with contextlib.redirect_stdout(output):
                code = client.main(["start", "--socket", server.path, "--timeout", "1"])
        self.assertEqual(code, 1)
        self.assertEqual(server.actions, ["status", "start"])
        failure = json.loads(output.getvalue())
        self.assertFalse(failure["ok"])
        self.assertIs(failure["request_sent"], True)
        self.assertIn("closed without", failure["error"])

    def test_preflight_conflict_error_json_does_not_authorize_cleanup(self):
        conflict = [{"pid": 123, "executable": "/legacy/build/chassis/robot_control"}]
        output = io.StringIO()
        with FakeController() as server, mock.patch.object(client, "other_controllers", return_value=conflict):
            with contextlib.redirect_stdout(output):
                code = client.main(["start", "--socket", server.path, "--timeout", "1"])
        self.assertEqual(code, 1)
        self.assertEqual(server.actions, ["status"])
        failure = json.loads(output.getvalue())
        self.assertIs(failure["request_sent"], False)
        self.assertIn("another hardware controller", failure["error"])

    def test_failed_start_write_conservatively_counts_as_sent(self):
        with mock.patch.object(client.socket, "socket") as factory:
            connection = factory.return_value.__enter__.return_value
            connection.getsockopt.return_value = client.struct.pack("3i", os.getpid(), os.geteuid(), 0)
            connection.sendall.side_effect = BrokenPipeError("partial write outcome unknown")
            with self.assertRaisesRegex(client.ControlError, "partial write") as raised:
                client.exchange("/unused-fake-socket", "start", 1)
        self.assertTrue(raised.exception.request_sent)

    def test_explicit_start_rejection_reports_that_request_was_sent(self):
        with FakeController(lambda action, number: response(ok=False, error="fault latched")) as server:
            with self.assertRaisesRegex(client.ControlError, "fault latched") as raised:
                client.exchange(server.path, "start", 1)
        self.assertEqual(server.actions, ["start"])
        self.assertTrue(raised.exception.request_sent)

    def test_proc_scan_excludes_only_the_verified_controller(self):
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            for pid, executable in [(10, "/build/chassis/robot_control"),
                                    (11, "/other/lib/chassis/robot_control (deleted)"),
                                    (12, "/opt/lib/chassis/chassis_only"),
                                    (13, "/usr/bin/python3")]:
                (root / str(pid)).mkdir()
                (root / str(pid) / "exe").symlink_to(executable)
            conflicts = client.other_controllers(10, root)
        self.assertEqual({item["pid"] for item in conflicts}, {11, 12})

    def test_uninspectable_possible_controller_is_an_error(self):
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            (root / "123").mkdir()
            (root / "123" / "comm").write_text("robot_control\n")
            with mock.patch.object(client.os, "readlink", side_effect=PermissionError(13, "denied")):
                with self.assertRaisesRegex(client.ControlError, "cannot inspect possible controller"):
                    client.other_controllers(10, root)

    def test_stop_waits_for_hardware_cleanup(self):
        def answer(action, number):
            return response(stage="stopping", hardware_active=True) if number < 3 else response()
        with FakeController(answer) as server:
            result = client.run_action(server.path, "stop", 1)
        self.assertEqual(server.actions, ["stop", "status", "status"])
        self.assertEqual(result["stage"], "waiting")
        self.assertFalse(result["hardware_active"])

    def test_accepted_stop_retries_status_transport_timeout(self):
        def answer(action, number):
            if number == 1:
                return response(stage="stopping", hardware_active=True)
            if number == 2:
                time.sleep(0.12)  # Cleanup exceeds one bounded IPC read.
            return response()

        exchange = client.exchange

        def short_exchange(path, action, timeout, expected_peer_pid=None):
            return exchange(path, action, min(timeout, 0.08), expected_peer_pid)

        with FakeController(answer) as server, mock.patch.object(client, "exchange", side_effect=short_exchange):
            result = client.run_action(server.path, "stop", 1)
        self.assertEqual(server.actions, ["stop", "status", "status"])
        self.assertEqual(result["pid"], os.getpid())
        self.assertFalse(result["hardware_active"])

    def test_accepted_stop_retries_expired_status_request(self):
        def answer(action, number):
            if number == 1:
                return response(stage="stopping", hardware_active=True)
            if number == 2:
                return response(ok=False, error="expired or invalid shared control request deadline")
            return response()

        with FakeController(answer) as server:
            result = client.run_action(server.path, "stop", 1)
        self.assertEqual(server.actions, ["stop", "status", "status"])
        self.assertFalse(result["hardware_active"])

    def test_stop_timeout_retries_remain_bounded_by_total_deadline(self):
        def answer(action, number):
            if number == 1:
                return response(stage="stopping", hardware_active=True)
            return response(ok=False, error="expired or invalid shared control request deadline")

        began = time.monotonic()
        with FakeController(answer) as server:
            with self.assertRaisesRegex(client.ControlError, "shutdown was not confirmed"):
                client.run_action(server.path, "stop", 0.12)
        self.assertLess(time.monotonic() - began, 0.5)
        self.assertGreaterEqual(len(server.actions), 2)
        self.assertEqual(server.actions[0], "stop")
        self.assertTrue(all(action == "status" for action in server.actions[1:]))

    def test_stop_retry_rejects_a_replacement_pid_before_sending(self):
        with mock.patch.object(client.socket, "socket") as factory:
            connection = factory.return_value.__enter__.return_value
            original = client.struct.pack("3i", os.getpid(), os.geteuid(), 0)
            replacement = client.struct.pack("3i", os.getpid() + 1, os.geteuid(), 0)
            connection.getsockopt.side_effect = [original, original, replacement]
            connection.recv.side_effect = [
                json.dumps(response(stage="stopping", hardware_active=True)).encode() + b"\n",
                socket.timeout("first status timed out"),
            ]
            with self.assertRaisesRegex(client.ControlError, "instance changed"):
                client.run_action("/unused-fake-socket", "stop", 1)
            sent = [call.args[0].decode().split()[1] for call in connection.sendall.call_args_list]
        self.assertEqual(sent, ["stop", "status"])

    def test_stop_does_not_retry_other_rejections_or_disconnects(self):
        for rejected in [response(ok=False, error="controller rejected status"),
                         response(pid=os.getpid() + 1), None]:
            def answer(action, number):
                return response(stage="stopping", hardware_active=True) if number == 1 else rejected

            with self.subTest(rejected=rejected), FakeController(answer) as server:
                with self.assertRaises(client.ControlError):
                    client.run_action(server.path, "stop", 1)
            self.assertEqual(server.actions, ["stop", "status"])

    def test_unaccepted_stop_timeout_is_not_retried(self):
        with mock.patch.object(client.socket, "socket") as factory:
            connection = factory.return_value.__enter__.return_value
            connection.getsockopt.return_value = client.struct.pack("3i", os.getpid(), os.geteuid(), 0)
            connection.recv.side_effect = socket.timeout("stop acceptance unknown")
            with self.assertRaises(client.ControlTimeout):
                client.run_action("/unused-fake-socket", "stop", 1)
            sent = [call.args[0].decode().split()[1] for call in connection.sendall.call_args_list]
        self.assertEqual(sent, ["stop"])

    def test_fault_with_hardware_off_counts_as_confirmed_stop(self):
        with FakeController(lambda action, number: response(stage="fault", error="latched")) as server:
            result = client.run_action(server.path, "stop", 1)
        self.assertEqual(result["stage"], "fault")
        self.assertEqual(server.actions, ["stop"])

    def test_stop_deadline_does_not_report_false_success(self):
        with FakeController(lambda action, number: response(stage="stopping", hardware_active=True)) as server:
            with self.assertRaises(client.ControlError):
                client.run_action(server.path, "stop", 0.12)
        self.assertGreaterEqual(len(server.actions), 2)

    def test_cli_outputs_json_and_failure_exit_code(self):
        with FakeController() as server:
            completed = subprocess.run(
                [sys.executable, "-B", str(SCRIPT), "status", "--socket", server.path],
                text=True, capture_output=True, timeout=3)
        self.assertEqual(completed.returncode, 0, completed.stderr)
        self.assertTrue(json.loads(completed.stdout)["ok"])
        with tempfile.TemporaryDirectory() as temporary:
            missing = str(Path(temporary) / "missing.sock")
            failed = subprocess.run(
                [sys.executable, "-B", str(SCRIPT), "start", "--socket", missing],
                text=True, capture_output=True, timeout=3)
        self.assertEqual(failed.returncode, 1)
        self.assertFalse(json.loads(failed.stdout)["ok"])
        self.assertIs(json.loads(failed.stdout)["request_sent"], False)


if __name__ == "__main__":
    unittest.main()
