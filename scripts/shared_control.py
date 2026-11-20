#!/usr/bin/env python3
"""Request lifecycle actions from the single local car2 hardware owner.

This client never starts a controller process, kills processes, or accesses motors.
Successful start means accepted; hardware readiness still comes from /car2/ready.
"""

import argparse
import errno
import json
import math
import os
from pathlib import Path
import socket
import struct
import sys
import time


CONTROLLERS = frozenset({
    "chassis", "chassis_only", "robot_control", "key_control", "chassis_only_remote",
})
STAGES = frozenset({"waiting", "requested", "starting", "running", "stopping", "fault"})
SOURCES = frozenset({"none", "app", "remote"})
MAX_RESPONSE_BYTES = 4096


class ControlError(RuntimeError):
    """A lifecycle request could not be safely verified."""

    def __init__(self, message, *, request_sent=False):
        super().__init__(message)
        # Only a start request may have created work requiring cancellation.
        # A failed status/preflight read does not authorize stopping an owner.
        self.request_sent = request_sent


class ControlTimeout(ControlError):
    """A bounded transport/deadline failure; a stop status read can be retried."""


def _validate_response(response, peer_pid):
    if not isinstance(response, dict):
        raise ControlError("shared controller returned a non-object response")
    if (type(response.get("protocol")) is not int or response["protocol"] != 1
            or type(response.get("pid")) is not int or response["pid"] != peer_pid):
        raise ControlError("shared controller protocol or peer PID did not match")
    if (type(response.get("ok")) is not bool
            or type(response.get("hardware_active")) is not bool
            or type(response.get("arm_enabled")) is not bool
            or type(response.get("generation")) is not int or response["generation"] < 0
            or not isinstance(response.get("stage"), str) or response["stage"] not in STAGES
            or not isinstance(response.get("owner"), str) or response["owner"] not in SOURCES
            or not isinstance(response.get("error"), str)):
        raise ControlError("shared controller returned invalid lifecycle state")
    if ((response["stage"] == "running" and not response["hardware_active"])
            or (response["stage"] == "waiting" and response["hardware_active"])):
        raise ControlError("shared controller returned inconsistent hardware state")
    if not response["ok"]:
        if response["error"] == "expired or invalid shared control request deadline":
            raise ControlTimeout(response["error"])
        raise ControlError(response["error"] or "shared controller rejected the request")
    return response


def exchange(socket_path, action, timeout=5.0, expected_peer_pid=None):
    """One bounded request; SO_PEERCRED ties the reply to the connected process."""
    if action not in {"status", "start", "stop", "claim-app"}:
        raise ControlError("unknown shared control action")
    timeout = min(timeout, 5.0)
    if not math.isfinite(timeout):
        raise ControlError("shared control request timeout must be finite")
    if timeout <= 0:
        raise ControlTimeout("shared control request deadline expired")
    deadline = time.monotonic() + timeout
    start_sent = False
    try:
        with socket.socket(socket.AF_UNIX, socket.SOCK_STREAM) as connection:
            connection.settimeout(timeout)
            connection.connect(socket_path)
            peer_pid, peer_uid, _ = struct.unpack(
                "3i", connection.getsockopt(socket.SOL_SOCKET, socket.SO_PEERCRED, 12))
            if peer_pid <= 0 or peer_uid not in {0, os.geteuid()}:
                raise ControlError("shared controller has an unexpected peer identity")
            if expected_peer_pid is not None and peer_pid != expected_peer_pid:
                raise ControlError("shared controller instance changed before the request")
            request = f"v1 {action} {int(deadline * 1000)}\n".encode("ascii")
            # sendall may fail after partially delivering the request. Mark it
            # before sending so an ambiguous write/read failure stays explicit.
            start_sent = action == "start"
            connection.sendall(request)
            response = bytearray()
            while b"\n" not in response:
                remaining = deadline - time.monotonic()
                if remaining <= 0:
                    raise ControlTimeout("shared control request timed out")
                connection.settimeout(remaining)
                chunk = connection.recv(min(1024, MAX_RESPONSE_BYTES + 1 - len(response)))
                if not chunk:
                    raise ControlError("shared controller closed without a complete response")
                response.extend(chunk)
                if len(response) > MAX_RESPONSE_BYTES:
                    raise ControlError("shared controller response is too large")
            line, trailing = bytes(response).split(b"\n", 1)
            if trailing:
                raise ControlError("unexpected data after shared controller response")
            return _validate_response(json.loads(line), peer_pid)
    except ControlError as error:
        error.request_sent = start_sent
        raise
    except socket.timeout as error:
        raise ControlTimeout("shared control request timed out",
                             request_sent=start_sent) from error
    except (OSError, ValueError, UnicodeError) as error:
        raise ControlError(f"shared controller request failed: {error}",
                           request_sent=start_sent) from error


def other_controllers(peer_pid, proc_root=Path("/proc")):
    """Exclude only the verified endpoint process, never every robot_control."""
    conflicts = []
    for entry in proc_root.iterdir():
        if not entry.name.isdecimal() or int(entry.name) == peer_pid:
            continue
        try:
            executable = os.readlink(entry / "exe").removesuffix(" (deleted)")
        except OSError as error:
            if error.errno in (errno.ENOENT, errno.ESRCH):
                continue  # Exited processes and kernel threads have no executable.
            if error.errno in (errno.EACCES, errno.EPERM):
                try:
                    comm = (entry / "comm").read_text().strip()
                except OSError as detail:
                    raise ControlError(f"cannot inspect process {entry.name}: {detail}") from detail
                if comm not in {name[:15] for name in CONTROLLERS}:
                    continue
            raise ControlError(f"cannot inspect possible controller {entry.name}: {error}") from error
        name = Path(executable).name
        if name in CONTROLLERS:
            conflicts.append({"pid": int(entry.name), "executable": executable})
    return conflicts


def preflight(socket_path, timeout=5.0):
    status = exchange(socket_path, "status", timeout)
    conflicts = other_controllers(status["pid"])
    if conflicts:
        detail = ", ".join(f"{item['executable']} (PID {item['pid']})" for item in conflicts)
        raise ControlError("another hardware controller is running: " + detail)
    return status


def run_action(socket_path, action, timeout):
    deadline = time.monotonic() + timeout
    if action == "preflight":
        return preflight(socket_path, timeout)
    if action == "start":
        previous = preflight(socket_path, min(timeout, 5.0))
        return exchange(socket_path, action, deadline - time.monotonic(), previous["pid"])
    result = exchange(socket_path, action, min(timeout, 5.0))
    if action != "stop":
        return result
    peer_pid = result["pid"]
    while result["hardware_active"] or result["stage"] not in {"waiting", "fault"}:
        remaining = deadline - time.monotonic()
        if remaining <= 0:
            raise ControlError("stop was accepted but hardware shutdown was not confirmed")
        time.sleep(min(0.05, remaining))
        try:
            result = exchange(socket_path, "status", deadline - time.monotonic(), peer_pid)
        except ControlTimeout:
            # Cleanup can block the executor beyond one five-second IPC request.
            # Retry reads only, within the total deadline and bound to the same PID.
            continue
    return result


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("action", choices=("start", "stop", "status", "claim-app", "preflight"))
    parser.add_argument("--socket", default=os.environ.get(
        "BXI_CAR2_SHARED_SOCKET", "/run/car2/control.sock"))
    parser.add_argument("--timeout", type=float, default=None,
                        help="overall deadline in seconds; default 30 for stop, otherwise 5")
    arguments = parser.parse_args(argv)
    timeout = arguments.timeout if arguments.timeout is not None else (
        30.0 if arguments.action == "stop" else 5.0)
    try:
        if not math.isfinite(timeout) or timeout <= 0 or timeout > 3600:
            raise ControlError("timeout must be finite and in (0, 3600] seconds")
        if not arguments.socket.startswith("/"):
            raise ControlError("shared controller socket must use an absolute path")
        result = run_action(arguments.socket, arguments.action, timeout)
        print(json.dumps(result, ensure_ascii=False))
        return 0
    except (ControlError, OSError) as error:
        print(json.dumps({"ok": False, "error": str(error),
                          "request_sent": getattr(error, "request_sent", False)}, ensure_ascii=False))
        return 1


if __name__ == "__main__":
    sys.exit(main())
