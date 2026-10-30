"""Exercise one-shot startup without connecting to SSH, cameras, or flight hardware."""

from concurrent.futures import ThreadPoolExecutor
import json
import os
from pathlib import Path
import shlex
import signal
import subprocess
import sys
import threading
from types import SimpleNamespace

import pytest

from drone_control_pkg import milestone4_session as session


PEER = ("192.168.0.121", "jetson", "/home/jetson/cdrone_control")


def test_both_checks_precede_concurrent_starts_and_then_communication_ends():
    calls = []
    barrier = threading.Barrier(2)

    def invoke(operation, payload, peer):
        calls.append((operation, payload, peer))
        if operation == "start":
            assert sum(call[0] == "check" for call in calls) == 2
            barrier.wait(timeout=2)
        return dict(success=True, message="accepted")

    dispatcher = session.StartDispatcher("cdrone3", "cdrone4", PEER, invoke)
    result = dispatcher.start()
    assert result["success"]
    assert [call[0] for call in calls] == ["check", "check", "start", "start"]
    local_start = next(payload for operation, payload, peer in calls
                       if operation == "start" and peer is None)
    assert local_start["service"] == "/cdrone/cdrone3/demo/milestone3_start_local"
    assert "continue independently" in result["message"]
    # All executor tasks have joined; there is no retained timer or connection.
    assert len(calls) == 4


@pytest.mark.parametrize("failed_id", ["cdrone3", "cdrone4"])
def test_one_failed_readiness_check_sends_neither_start(failed_id):
    calls = []

    def invoke(operation, payload, peer):
        calls.append(operation)
        return dict(success=payload["drone_id"] != failed_id, message="readiness result")

    result = session.StartDispatcher("cdrone3", "cdrone4", PEER, invoke).start()
    assert not result["success"]
    assert calls == ["check", "check"]
    assert "No start requests sent" in result["message"]


def test_partial_start_is_reported_without_aborting_the_running_drone():
    calls = []

    def invoke(operation, payload, peer):
        calls.append(operation)
        return dict(success=operation == "check" or peer is None, message="result")

    result = session.StartDispatcher("cdrone3", "cdrone4", PEER, invoke).start()
    assert not result["success"]
    assert "cdrone3: OK" in result["message"]
    assert "cdrone4: FAILED" in result["message"]
    assert calls.count("start") == 2 and "abort" not in calls


def test_uncertain_response_does_not_retry_start():
    calls = []

    def invoke(operation, payload, peer):
        calls.append(operation)
        if operation == "start" and peer:
            return dict(success=False, message="SSH response lost", uncertain=True)
        return dict(success=True, message="accepted")

    dispatcher = session.StartDispatcher("cdrone3", "cdrone4", PEER, invoke)
    assert "UNCERTAIN" in dispatcher.start()["message"]
    assert not dispatcher.start()["success"]
    assert len(calls) == 4


def test_overlapping_start_requests_are_rejected():
    entered, release = threading.Event(), threading.Event()

    def invoke(operation, payload, peer):
        entered.set()
        assert release.wait(2)
        return dict(success=True, message="ready")

    dispatcher = session.StartDispatcher("cdrone3", "cdrone4", PEER, invoke)
    with ThreadPoolExecutor(max_workers=1) as executor:
        running = executor.submit(dispatcher.start)
        assert entered.wait(2)
        assert "already in progress" in dispatcher.start()["message"]
        release.set()
        assert running.result()["success"]


def test_remote_command_is_quoted_noninteractive_and_localhost_only(monkeypatch):
    monkeypatch.setenv("ROS_DOMAIN_ID", "3")
    workspace = "/home/jetson/work space/$(touch should_not_exist)"
    command = session.ssh_command(PEER[0], PEER[1], workspace,
                                  ["python3", "-m", "example", "a'b; literal"])
    assert "BatchMode=yes" in command
    assert "StrictHostKeyChecking=yes" in command
    script = shlex.split(command[-1])[2]
    assert "export ROS_LOCALHOST_ONLY=1" in script
    assert "export ROS_DOMAIN_ID=3" in script
    assert shlex.split(script.split(" && exec ")[1])[-1] == "a'b; literal"
    assert shlex.quote(workspace + "/ros2/install/setup.bash") in script
    with pytest.raises(ValueError):
        session.ssh_command("-oProxyCommand=bad", PEER[1], PEER[2], ["python3"])


def test_ssh_failure_and_timeout_are_reported(monkeypatch):
    monkeypatch.setattr(session.subprocess, "run", lambda *a, **k: SimpleNamespace(
        stdout="", stderr="Permission denied", returncode=255))
    result = session.invoke("launch", {}, PEER)
    assert not result["success"] and "Permission denied" in result["message"]

    def timeout(*args, **kwargs):
        raise subprocess.TimeoutExpired(args[0], kwargs["timeout"])

    monkeypatch.setattr(session.subprocess, "run", timeout)
    assert session.invoke("start", {}, PEER)["uncertain"]


def test_remote_session_survives_helper_exit_and_holds_exclusive_lock(tmp_path, monkeypatch):
    """Use a dummy ros2 executable: never run the actual ROS launch or hardware."""
    monkeypatch.setenv("ROS_HOME", str(tmp_path / "ros_home"))
    monkeypatch.delenv(session.LOCK_ENV, raising=False)
    mount = tmp_path / "mount.yaml"
    mount.write_text("calibrated: true\n")
    fake = tmp_path / "ros2"
    fake.write_text(
        f"#!{sys.executable}\n"
        "import os, time\n"
        "from drone_control_pkg.milestone4_session import acquire_launch_lock\n"
        "lock = acquire_launch_lock()\n"
        "assert os.environ['ROS_LOCALHOST_ONLY'] == '1'\n"
        "print('dummy stack running', flush=True)\n"
        "time.sleep(20)\n")
    fake.chmod(0o755)
    monkeypatch.setenv("PATH", str(tmp_path) + os.pathsep + os.environ["PATH"])
    payload = {"arguments": dict(drone_id="cdrone4", launch_peer="false",
                                 zed_mount_config=str(mount))}
    completed = subprocess.run(session.helper_command("launch", payload),
                               capture_output=True, text=True, timeout=5)
    assert completed.returncode == 0, completed.stdout + completed.stderr
    result = json.loads(completed.stdout.split(session.RESULT_PREFIX)[-1])
    pid = result["pid"]
    try:
        assert os.getsid(pid) == pid  # detached from the exited SSH-equivalent helper
        assert session.process_start_ticks(pid) == result["start_ticks"]
        with pytest.raises(RuntimeError, match="already launched"):
            session.acquire_launch_lock()
        assert "dummy stack running" in Path(result["log"]).read_text()
        assert json.loads((session.session_dir() / "session.json").read_text())["pid"] == pid
    finally:
        os.killpg(pid, signal.SIGTERM)


def test_launch_lock_prevents_different_ids_sharing_the_same_camera(tmp_path, monkeypatch):
    monkeypatch.setenv("ROS_HOME", str(tmp_path))
    monkeypatch.delenv(session.LOCK_ENV, raising=False)
    first = session.acquire_launch_lock()
    try:
        with pytest.raises(RuntimeError, match="already launched"):
            session.acquire_launch_lock()
    finally:
        first.close()
    session.acquire_launch_lock().close()


def test_existing_public_service_dispatches_both_starts(monkeypatch):
    import rclpy
    from rclpy.executors import MultiThreadedExecutor
    from std_srvs.srv import Trigger
    from drone_control_pkg import milestone4_start_dispatcher as server

    monkeypatch.setenv("ROS_DOMAIN_ID", "213")
    monkeypatch.setenv("ROS_LOCALHOST_ONLY", "1")
    calls = []

    def invoke(operation, payload, peer):
        calls.append((operation, payload))
        return dict(success=True, message="accepted")

    monkeypatch.setattr(server, "invoke", invoke)
    rclpy.init(args=[])
    node = server.Milestone4StartDispatcher()
    node.dispatcher.invoke = invoke
    client_node = rclpy.create_node("test_milestone4_start_client")
    executor = MultiThreadedExecutor(num_threads=2)
    executor.add_node(node)
    executor.add_node(client_node)
    try:
        client = client_node.create_client(Trigger, "/cdrone/cdrone3/demo/milestone3_start")
        assert client.wait_for_service(timeout_sec=3.0)
        future = client.call_async(Trigger.Request())
        executor.spin_until_future_complete(future, timeout_sec=5.0)
        assert future.done() and future.result().success
        assert [operation for operation, _ in calls] == ["launch", "check", "check", "start", "start"]
        assert {payload["drone_id"] for operation, payload in calls
                if operation == "start"} == {"cdrone3", "cdrone4"}
    finally:
        executor.shutdown()
        client_node.destroy_node()
        node.destroy_node()
        rclpy.shutdown()


@pytest.mark.parametrize("armed,age,connected,allowed", [
    (False, 0.1, True, True), (True, 0.1, True, False),
    (False, 4.0, True, False), (False, 0.1, False, False),
])
def test_remote_stop_requires_fresh_disarmed_connected_state(
        tmp_path, monkeypatch, armed, age, connected, allowed):
    import rclpy
    from mavros_msgs.msg import State

    monkeypatch.setenv("ROS_HOME", str(tmp_path))
    session.session_dir().mkdir()
    (session.session_dir() / "session.json").write_text(json.dumps(
        dict(pid=123, start_ticks="456", drone_id="cdrone4")))
    monkeypatch.setattr(session, "process_start_ticks", lambda pid: "456")
    monkeypatch.setattr(session.os, "getpgid", lambda pid: pid)
    signals = []
    monkeypatch.setattr(session.os, "killpg", lambda *args: signals.append(args))
    ticks = iter(range(20))
    monkeypatch.setattr(session.time, "monotonic", lambda: next(ticks))
    state = State(connected=connected, armed=armed)
    state.header.stamp.sec = 100
    node = SimpleNamespace(
        create_subscription=lambda msg, topic, callback, qos: callback(state),
        get_clock=lambda: SimpleNamespace(now=lambda: SimpleNamespace(nanoseconds=int((100 + age) * 1e9))),
        destroy_node=lambda: None,
    )
    monkeypatch.setattr(rclpy, "init", lambda **kwargs: None)
    monkeypatch.setattr(rclpy, "shutdown", lambda: None)
    monkeypatch.setattr(rclpy, "create_node", lambda name: node)
    monkeypatch.setattr(rclpy, "spin_once", lambda *args, **kwargs: None)
    if allowed:
        assert session.stop_detached()["success"]
        assert signals == [(123, signal.SIGINT)]
    else:
        with pytest.raises(RuntimeError):
            session.stop_detached()
        assert not signals
