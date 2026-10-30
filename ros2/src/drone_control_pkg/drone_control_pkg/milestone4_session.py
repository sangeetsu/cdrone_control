"""One-shot SSH/ROS startup operations; never exchanges flight data between hosts."""

import argparse
import base64
from concurrent.futures import ThreadPoolExecutor
from datetime import datetime, timezone
import fcntl
import json
import os
from pathlib import Path
import re
import shlex
import signal
import subprocess
import sys
import threading
import time


RESULT_PREFIX = "MILESTONE4_RESULT="
LOCK_ENV = "CDRONE_MILESTONE4_LOCK_FD"
_launch_locks = []


def session_dir():
    return Path(os.environ.get("ROS_HOME", str(Path.home() / ".ros"))) / "milestone4"


def acquire_launch_lock():
    """Hold one exclusive Milestone 4 camera/FCU owner on this host."""
    directory = session_dir()
    directory.mkdir(parents=True, exist_ok=True)
    path = directory / "launch.lock"
    inherited = os.environ.pop(LOCK_ENV, "")
    if inherited:
        descriptor = int(inherited)
        held, expected = os.fstat(descriptor), path.stat()
        if (held.st_dev, held.st_ino) != (expected.st_dev, expected.st_ino):
            raise RuntimeError("inherited Milestone 4 lock does not match this host")
        lock = os.fdopen(descriptor, "a")
    else:
        lock = path.open("a")
    try:
        fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
    except BlockingIOError:
        lock.close()
        raise RuntimeError("Milestone 4 is already launched on this host; stop it before relaunching")
    return lock


def claim_launch_lock(context, *args, **kwargs):
    """ROS launch action; lock lifetime is the launch process lifetime."""
    del context, args, kwargs
    _launch_locks.append(acquire_launch_lock())
    return []


def validate_drone_id(value):
    if not re.fullmatch(r"[A-Za-z][A-Za-z0-9_]*", value):
        raise ValueError(f"invalid drone ID: {value!r}")
    return value


def peer_launch_arguments(arguments, peer_id, mount_config, workspace):
    """Copy mission settings but resolve every topic and identity on the peer."""
    validate_drone_id(peer_id)
    result = {key: value for key, value in arguments.items()
              if not key.startswith("peer_") and key != "launch_peer"}
    local_ns = f"/cdrone/{arguments['drone_id']}"
    peer_ns = f"/cdrone/{peer_id}"
    # Package/config/model paths on these Jetsons use the same workspace layout.
    for key, value in result.items():
        result[key] = str(value).replace(local_ns + "/", peer_ns + "/")
        if result[key].startswith("/home/jetson/cdrone_control/"):
            result[key] = workspace.rstrip("/") + result[key][len("/home/jetson/cdrone_control"):]
    result.update(
        launch_peer="false", drone_id=peer_id, drone_namespace=peer_ns,
        mavros_namespace=peer_ns + "/mavros", mocap_namespace=peer_ns + "/vrpn_mocap",
        engagement_state_topic=peer_ns + "/engagement/state",
        zed_mount_config=mount_config, start_service="",
    )
    if not result.get("model_path"):
        result["model_path"] = str(Path(workspace) / "models/best.engine")
    return result


def helper_command(operation, payload):
    encoded = base64.urlsafe_b64encode(json.dumps(payload).encode()).decode()
    return [sys.executable, "-m", "drone_control_pkg.milestone4_session",
            operation, "--payload", encoded]


def ssh_command(host, user, workspace, command):
    if not re.fullmatch(r"[A-Za-z0-9][A-Za-z0-9.:-]*", host):
        raise ValueError("invalid peer_host")
    if not re.fullmatch(r"[A-Za-z_][A-Za-z0-9_-]*", user):
        raise ValueError("invalid peer_user")
    if not Path(workspace).is_absolute():
        raise ValueError("peer_workspace must be absolute")
    # OpenSSH executes a shell on the peer: quote each argument, including paths.
    command = ["python3", *command[1:]]
    script = ("source /opt/ros/humble/setup.bash && source " +
              shlex.quote(str(Path(workspace) / "ros2/install/setup.bash")) +
              " && export ROS_LOCALHOST_ONLY=1 && export ROS_DOMAIN_ID=" +
              shlex.quote(os.environ.get("ROS_DOMAIN_ID", "0")) +
              " && exec " + shlex.join(command))
    return ["ssh", "-o", "BatchMode=yes", "-o", "StrictHostKeyChecking=yes",
            "-o", "ConnectTimeout=5", f"{user}@{host}",
            "bash -lc " + shlex.quote(script)]


def invoke(operation, payload, peer=None):
    command = helper_command(operation, payload)
    if peer:
        command = ssh_command(*peer, command)
    try:
        completed = subprocess.run(command, capture_output=True, text=True, timeout=20)
        for line in reversed(completed.stdout.splitlines()):
            if line.startswith(RESULT_PREFIX):
                result = json.loads(line[len(RESULT_PREFIX):])
                if (isinstance(result, dict) and isinstance(result.get("success"), bool)
                        and isinstance(result.get("message"), str)):
                    return result
        detail = completed.stderr.strip() or completed.stdout.strip() or f"exit {completed.returncode}"
        return dict(success=False, message=detail[-2000:], uncertain=operation == "start")
    except (OSError, subprocess.TimeoutExpired, ValueError) as exc:
        return dict(success=False, message=str(exc), uncertain=operation == "start")


class StartDispatcher:
    """Serialize explicit start attempts; only contact peers inside this call."""

    def __init__(self, local_id, peer_id, peer, invoke_fn=invoke):
        self.local_id, self.peer_id, self.peer = local_id, peer_id, peer
        self.invoke = invoke_fn
        self.lock = threading.Lock()
        self.uncertain = False

    def start(self):
        if not self.lock.acquire(blocking=False):
            return dict(success=False, message="a paired start request is already in progress")
        try:
            if self.uncertain:
                return dict(success=False, message="previous start outcome uncertain; inspect both drones "
                            "before restarting the launcher; no start was retried")
            endpoints = [(self.local_id, None), (self.peer_id, self.peer)]
            with ThreadPoolExecutor(max_workers=2) as executor:
                def run(operation):
                    pending = []
                    for drone_id, peer in endpoints:
                        payload = {"drone_id": drone_id}
                        if operation == "start" and peer is None:
                            payload["service"] = f"/cdrone/{drone_id}/demo/milestone3_start_local"
                        pending.append(executor.submit(self.invoke, operation, payload, peer))
                    return [future.result() for future in pending]

                checks = run("check")
                if not all(result["success"] for result in checks):
                    return dict(success=False, message="No start requests sent. " +
                                self.describe(endpoints, checks))
                starts = run("start")
                self.uncertain = any(result.get("uncertain", False) for result in starts)
                return dict(success=all(result["success"] for result in starts),
                            message=self.describe(endpoints, starts) +
                            "; accepted missions continue independently; no automatic retry or abort")
        finally:
            self.lock.release()

    @staticmethod
    def describe(endpoints, results):
        return "; ".join(f"{drone_id}: " +
                         ("UNCERTAIN" if result.get("uncertain") else
                          "OK" if result["success"] else "FAILED") +
                         f" ({result['message']})"
                         for (drone_id, _), result in zip(endpoints, results))


def call_trigger(operation, payload):
    import rclpy
    from std_srvs.srv import Trigger

    drone_id = validate_drone_id(payload["drone_id"])
    leaf = {"check": "milestone4_start_check", "start": "milestone3_start",
            "abort": "milestone3_abort"}[operation]
    service = payload.get("service", f"/cdrone/{drone_id}/demo/{leaf}")
    rclpy.init(args=[])
    node = rclpy.create_node(f"milestone4_once_{os.getpid()}")
    sent = False
    try:
        client = node.create_client(Trigger, service)
        if not client.wait_for_service(timeout_sec=5.0):
            return dict(success=False, message=f"service unavailable: {service}")
        future = client.call_async(Trigger.Request())
        sent = True
        rclpy.spin_until_future_complete(node, future, timeout_sec=8.0)
        if not future.done():
            return dict(success=False, message=f"response timed out: {service}",
                        uncertain=operation == "start")
        response = future.result()
        return dict(success=response.success, message=response.message)
    except Exception as exc:
        return dict(success=False, message=str(exc), uncertain=sent and operation == "start")
    finally:
        node.destroy_node()
        rclpy.shutdown()


def process_start_ticks(pid):
    # Field 22; comm (field 2) may contain spaces or parentheses.
    return Path(f"/proc/{pid}/stat").read_text().rsplit(")", 1)[1].split()[19]


def launch_detached(payload):
    arguments = payload["arguments"]
    drone_id = validate_drone_id(arguments["drone_id"])
    if arguments.get("launch_peer") != "false":
        raise ValueError("remote launch must disable launch_peer")
    if not Path(arguments["zed_mount_config"]).is_file():
        raise ValueError("peer ZED mount configuration is missing")
    lock = acquire_launch_lock()
    try:
        stamp = datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%S_%fZ")
        log = session_dir() / f"{drone_id}_{stamp}.log"
        command = ["ros2", "launch", "drone_bringup", "milestone4_demo.launch.py"]
        command.extend(f"{key}:={value}" for key, value in arguments.items())
        environment = dict(os.environ, ROS_LOCALHOST_ONLY="1", **{LOCK_ENV: str(lock.fileno())})
        with log.open("x") as output:
            child = subprocess.Popen(command, stdin=subprocess.DEVNULL, stdout=output,
                                     stderr=subprocess.STDOUT, start_new_session=True,
                                     pass_fds=(lock.fileno(),), env=environment)
        metadata = {"pid": child.pid, "start_ticks": process_start_ticks(child.pid),
                    "drone_id": drone_id, "log": str(log)}
        # Wait only for an immediate launch failure, never for flight readiness.
        time.sleep(0.5)
        if child.poll() is not None:
            raise RuntimeError(f"peer launch exited {child.returncode}; inspect {log}")
        (session_dir() / "session.json").write_text(json.dumps(metadata))
        return dict(success=True, message=f"peer stack launched; readiness pending; log={log}",
                    **metadata)
    finally:
        # The detached ros2 launch inherited the lock. Never unlock it here.
        lock.close()


def stop_detached():
    """Stop only this recorded session, after a fresh connected/disarmed FCU state."""
    import rclpy
    from mavros_msgs.msg import State
    from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy

    metadata = json.loads((session_dir() / "session.json").read_text())
    pid = int(metadata["pid"])
    if process_start_ticks(pid) != metadata["start_ticks"] or os.getpgid(pid) != pid:
        raise RuntimeError("recorded session is no longer running; refusing to signal a reused PID")
    rclpy.init(args=[])
    node = rclpy.create_node(f"milestone4_stop_{os.getpid()}")
    states = []
    qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                     reliability=ReliabilityPolicy.RELIABLE)
    node.create_subscription(State, f"/cdrone/{metadata['drone_id']}/mavros/state", states.append, qos)
    try:
        deadline = time.monotonic() + 5.0
        while time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.1)
            if states:
                state = states[-1]
                stamp = state.header.stamp.sec + state.header.stamp.nanosec / 1e9
                age = node.get_clock().now().nanoseconds / 1e9 - stamp
                if 0 <= age <= 3.0 and state.connected:
                    if state.armed:
                        raise RuntimeError("drone is armed; call its abort service and wait for disarm")
                    if process_start_ticks(pid) != metadata["start_ticks"]:
                        raise RuntimeError("session changed during stop check")
                    os.killpg(pid, signal.SIGINT)
                    return dict(success=True, message="sent SIGINT to disarmed remote launch")
        raise RuntimeError("no fresh connected FCU state; remote launch was not stopped")
    finally:
        node.destroy_node()
        rclpy.shutdown()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("operation", choices=["launch", "check", "start", "abort", "stop"])
    parser.add_argument("--payload", help="base64-encoded internal launch/RPC configuration")
    parser.add_argument("--drone-id", default="cdrone4")
    args = parser.parse_args()
    try:
        payload = (json.loads(base64.urlsafe_b64decode(args.payload)) if args.payload
                   else {"drone_id": args.drone_id})
        if args.operation == "launch":
            result = launch_detached(payload)
        elif args.operation == "stop":
            result = stop_detached()
        else:
            result = call_trigger(args.operation, payload)
    except Exception as exc:
        result = dict(success=False, message=str(exc))
    print(RESULT_PREFIX + json.dumps(result), flush=True)
    return 0 if result["success"] else 1


if __name__ == "__main__":
    sys.exit(main())
