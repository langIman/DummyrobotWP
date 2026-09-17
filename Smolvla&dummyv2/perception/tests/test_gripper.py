import threading
import time
from types import SimpleNamespace

import pytest

from app import create_app
from perception_service.common import ServiceError
from perception_service.robot import RobotClient


class FakeBridge:
    def __init__(self):
        self.calls = []
        self.replies = {}
        self.disabled = threading.Event()
        self.events = []

    def request(self, method, path, body=None, timeout=4):
        command = body.get("command") if body else "!STOP"
        self.calls.append(command)
        if command in self.replies:
            reply = self.replies[command]
            if isinstance(reply, Exception):
                raise reply
            return reply
        if command.startswith("!HAND_I "):
            raw = f"ok hand current {float(command.split()[1]):.6f}\r\n"
        else:
            raw = {"!HAND_EN": "ok hand enable\r\nok hand enable/disable is 1 real_angle:-248.97\r\n",
                   "!HAND_DIS": "ok hand disable\r\nok hand enable/disable is 0 real_angle:-248.97\r\n",
                   "!HAND_O": "ok hand open\r\n", "!HAND_C": "ok hand close\r\n",
                   "!START": "Started ok\r\n", "!DISABLE": "Disabled ok\r\n", "!STOP": "Stopped ok\r\n"}[command]
        if command == "!HAND_DIS":
            self.disabled.set()
        return {"raw_response": raw, "sent": True, "stop": {"acknowledged": True}}


@pytest.fixture
def system(monkeypatch):
    bridge = FakeBridge()
    robot = RobotClient(start_sampler=False, event_callback=bridge.events.append)
    monkeypatch.setattr(robot, "_request", bridge.request)
    yield robot, bridge
    bridge.replies.clear()
    robot.close()


def test_startup_does_not_send_and_actions_require_local_enable(system):
    robot, bridge = system
    assert robot.status()["gripper"]["enabled_latch"] == "unknown"
    for action in ["open", "close"]:
        with pytest.raises(ServiceError, match="先在本页面使能"):
            robot.gripper.execute(action, {})
    assert bridge.calls == []


def test_enable_applies_current_and_stays_enabled_until_explicit_disable(system):
    robot, bridge = system
    result = robot.gripper.execute("enable", {})
    assert result["acknowledged"]
    assert not bridge.disabled.wait(0.1)
    assert robot.gripper.status()["enabled_latch"] == "confirmed"
    assert robot.gripper.status()["remaining_ms"] is None
    assert robot.gripper.status()["applied_current_a"] == 0.6
    assert robot.gripper.status()["control_ready"] is True
    assert robot.gripper.enabled()
    assert not robot.gripper.active()
    assert bridge.calls == ["!HAND_I 0.6", "!HAND_EN"]
    assert robot.status()["gripper"]["enabled_latch"] == "confirmed"
    assert robot.gripper.status()["reported_motor_angle"] == -248.97


@pytest.mark.parametrize("action,command", [("open", "!HAND_O"), ("close", "!HAND_C")])
def test_pulse_auto_stop_and_repeated_click_cannot_extend_it(system, action, command):
    robot, bridge = system
    robot.gripper.execute("enable", {})
    result = robot.gripper.execute(action, {"duration_ms": 100})
    assert result["transaction"]["raw_response"].startswith("ok hand")
    with pytest.raises(ServiceError) as exc:
        robot.gripper.execute(action, {"duration_ms": 1000})
    assert exc.value.code == "gripper_pulse_active"
    assert bridge.disabled.wait(1)
    with robot._io_lock:
        assert robot.gripper.status()["enabled_latch"] == "disabled"
    assert bridge.calls == ["!HAND_I 0.6", "!HAND_EN", command, "!HAND_DIS"]
    assert bridge.events[-1]["source"] == "watchdog"


def test_jog_requires_enable_and_uses_direction_commands(system):
    robot, bridge = system
    with pytest.raises(ServiceError, match="先在本页面使能"):
        robot.gripper.execute("jog-start", {"direction": "close"})
    robot.gripper.execute("enable", {})
    result = robot.gripper.execute("jog-start", {"direction": "close"})
    assert result["acknowledged"]
    assert robot.gripper.status()["jog_direction"] == "close"
    assert bridge.calls == ["!HAND_I 0.6", "!HAND_EN", "!HAND_I 0.6", "!HAND_C"]
    with pytest.raises(ServiceError, match="另一方向"):
        robot.gripper.execute("jog-start", {"direction": "open"})
    heartbeat = robot.gripper.execute("jog-heartbeat", {"direction": "close"})
    assert heartbeat["acknowledged"]
    assert bridge.calls == ["!HAND_I 0.6", "!HAND_EN", "!HAND_I 0.6", "!HAND_C"]
    stopped = robot.gripper.execute("jog-stop", {})
    assert stopped["acknowledged"]
    assert bridge.calls[-1] == "!HAND_DIS"
    assert "!HAND_I 0" not in bridge.calls
    assert robot.gripper.status()["jog_direction"] is None
    assert robot.gripper.status()["enabled_latch"] == "disabled"
    assert robot.gripper.status()["control_ready"] is True
    assert robot.gripper.status()["holding"] is True
    # A second press resumes without another HAND_EN.
    robot.gripper.execute("jog-start", {"direction": "open"})
    assert bridge.calls[-3:] == ["!HAND_I 0.6", "!HAND_EN", "!HAND_O"]


def test_jog_watchdog_disables_output_but_keeps_control_ready(system, monkeypatch):
    robot, bridge = system
    monkeypatch.setattr(robot.gripper, "JOG_WATCHDOG_SECONDS", 0.05)
    robot.gripper.execute("enable", {})
    robot.gripper.execute("jog-start", {"direction": "open"})
    deadline = time.monotonic() + 1
    while time.monotonic() < deadline and bridge.calls[-1] != "!HAND_DIS":
        time.sleep(0.01)
    assert bridge.calls[-1] == "!HAND_DIS"
    with robot._io_lock:
        assert robot.gripper.status()["enabled_latch"] == "disabled"
        assert robot.gripper.status()["control_ready"] is True
        assert robot.gripper.status()["holding"] is True
    assert bridge.calls[-1] == "!HAND_DIS"
    assert bridge.events[-1]["source"] == "watchdog"


def test_jog_and_short_pulse_cannot_overlap(system):
    robot, bridge = system
    robot.gripper.execute("enable", {})
    robot.gripper.execute("jog-start", {"direction": "close"})
    with pytest.raises(ServiceError, match="点动控制正在运行"):
        robot.gripper.execute("open", {"duration_ms": 100})
    robot.gripper.execute("jog-stop", {})
    with pytest.raises(ServiceError, match="先在本页面使能"):
        robot.gripper.execute("open", {"duration_ms": 100})
    robot.gripper.execute("jog-start", {"direction": "open"})
    with pytest.raises(ServiceError, match="点动控制正在运行"):
        robot.gripper.execute("open", {"duration_ms": 100})
    robot.gripper.disable()


def test_jog_http_routes_keep_exact_payload(system):
    robot, bridge = system
    app = create_app(SimpleNamespace(robot=robot))
    app.testing = True
    client = app.test_client()
    assert client.post("/api/robot/actions/gripper-jog-start", json={"direction": "close"}).status_code == 409
    robot.gripper.execute("enable", {})
    response = client.post("/api/robot/actions/gripper-jog-start", json={"direction": "close"})
    assert response.status_code == 200
    assert client.post("/api/robot/actions/gripper-jog-heartbeat", json={"direction": "close"}).status_code == 200
    assert client.post("/api/robot/actions/gripper-jog-stop", json={}).status_code == 200
    assert client.post("/api/robot/actions/gripper-jog-start", json={"direction": "sideways"}).status_code == 422
    assert bridge.calls[-1] == "!HAND_DIS"


@pytest.mark.parametrize("action,body", [
    ("current", {"current": 0}), ("current", {"current": 2}),
    ("current", {"current": True}), ("current", {"current": "0.2"}),
    ("current", {"current": float("nan")}), ("current", {"current": float("inf")}),
    ("current", {}), ("current", {"current": 0.2, "command": "!START"}),
    ("open", {"duration_ms": True}), ("open", {"duration_ms": 5000}),
    ("open", {"duration_ms": 1}), ("open", {"duration_ms": "500"}),
    ("enable", {"command": "!START"}),
])
def test_parameter_errors_send_nothing(system, action, body):
    robot, bridge = system
    with pytest.raises(ServiceError) as exc:
        robot.gripper.execute(action, body)
    assert exc.value.status == 422
    assert not bridge.calls


def test_current_confirmation_is_numeric_and_does_not_enable(system):
    robot, bridge = system
    result = robot.gripper.execute("current", {"current": 0.1})
    assert result["acknowledged"]
    assert robot.gripper.status()["current_a"] == 0.1
    assert robot.gripper.status()["enabled_latch"] == "unknown"
    bridge.replies["!HAND_I 0.2"] = {"sent": True, "raw_response": "ok hand current 0.1"}
    assert not robot.gripper.execute("current", {"current": 0.2})["acknowledged"]
    assert robot.gripper.status()["current_a"] == 0.1
    assert bridge.calls == ["!HAND_I 0.1", "!HAND_I 0.2"]


@pytest.mark.parametrize("reply", [{"sent": True, "raw_response": "ok\r\n"},
                                    {"sent": False, "raw_response": "ok hand open"},
                                    TimeoutError("bridge timeout")])
def test_motion_failure_preserves_reply_and_always_attempts_disable(system, reply):
    robot, bridge = system
    robot.gripper.execute("enable", {})
    bridge.replies["!HAND_O"] = reply
    result = robot.gripper.execute("open", {})
    assert not result["acknowledged"]
    assert result["cleanup"]["acknowledged"]
    assert bridge.calls[-1] == "!HAND_DIS"
    assert robot.gripper.status()["enabled_latch"] == "disabled"


def test_unconfirmed_disable_blocks_rearming_until_explicit_stop_succeeds(system):
    robot, bridge = system
    robot.gripper.execute("enable", {})
    bridge.replies["!HAND_DIS"] = {"sent": True, "raw_response": ""}
    assert not robot.gripper.disable()["acknowledged"]
    assert robot.gripper.status()["faulted"]
    with pytest.raises(ServiceError):
        robot.gripper.execute("enable", {})
    bridge.replies.clear()
    assert robot.gripper.disable()["acknowledged"]
    assert not robot.gripper.status()["faulted"]


def test_persistent_gripper_enable_does_not_block_six_axis_enable(system):
    robot, bridge = system
    robot.gripper.execute("enable", {})
    for action, body in [("current", {"current": 0.1}), ("enable", {})]:
        with pytest.raises(ServiceError):
            robot.gripper.execute(action, body)
    result = robot.enable()
    assert result["enabled_latch"] == "confirmed"
    assert bridge.calls == ["!HAND_I 0.6", "!HAND_EN", "!START"]


def test_preset_blocks_gripper_but_never_disable(system):
    robot, bridge = system
    robot._motion = {"status": "running"}
    with pytest.raises(ServiceError):
        robot.gripper.execute("enable", {})
    assert robot.gripper.disable()["acknowledged"]
    robot._motion = None


def test_stop_queued_during_send_prevents_a_new_action(system, monkeypatch):
    robot, bridge = system
    entered, release = threading.Event(), threading.Event()
    robot.gripper.execute("enable", {})
    original = bridge.request
    def delayed(method, path, body=None, timeout=4):
        if body and body.get("command") == "!HAND_O":
            entered.set()
            assert release.wait(2)
        return original(method, path, body, timeout)
    monkeypatch.setattr(robot, "_request", delayed)
    action_thread = threading.Thread(target=lambda: robot.gripper.execute("open", {"duration_ms": 100}))
    action_thread.start()
    assert entered.wait(1)
    # Timer is armed before the stalled HTTP request. It can mark stop-pending
    # while waiting for serialized IO, and new commands cannot enter.
    stop_thread = threading.Thread(target=robot.gripper.disable)
    stop_thread.start()
    with pytest.raises(ServiceError):
        robot.gripper.execute("close", {})
    release.set()
    action_thread.join(2)
    stop_thread.join(2)
    assert not action_thread.is_alive() and not stop_thread.is_alive()
    assert bridge.calls[-1] == "!HAND_DIS"
    assert "!HAND_C" not in bridge.calls


def test_shutdown_and_global_emergency_cover_gripper(system):
    robot, bridge = system
    robot.gripper.execute("enable", {})
    result = robot.emergency_stop()
    assert result["overall_acknowledged"]
    assert bridge.calls[-3:] == ["!STOP", "!DISABLE", "!HAND_DIS"]
    robot.gripper.execute("enable", {})
    robot.gripper.close()
    assert bridge.calls[-1] == "!HAND_DIS"


def test_http_whitelist_and_failure_reply_are_visible(system):
    robot, bridge = system
    app = create_app(SimpleNamespace(robot=robot))
    app.testing = True
    client = app.test_client()
    for action in ["position", "zero", "calibrate"]:
        response = client.post(f"/api/robot/actions/gripper-{action}", json={})
        assert response.status_code == 409
    assert not bridge.calls
    assert client.post("/api/robot/actions/gripper-raw", json={}).status_code == 404
    assert client.post("/api/robot/actions/gripper-enable", json=[]).status_code == 400
    assert client.post("/api/robot/actions/gripper-enable", json={}).status_code == 200
    bridge.replies["!HAND_O"] = {"sent": True, "raw_response": "ok\r\n"}
    response = client.post("/api/robot/actions/gripper-open", json={})
    assert response.status_code == 503
    assert response.json["error"]["code"] == "gripper_command_unconfirmed"
    assert response.json["transaction"]["raw_response"] == "ok\r\n"


def test_emergency_rejects_gripper_enable_without_queuing(system, monkeypatch):
    robot, bridge = system
    entered, release = threading.Event(), threading.Event()
    original = bridge.request
    def delayed(method, path, body=None, timeout=4):
        if path == "/stop":
            entered.set()
            assert release.wait(2)
        return original(method, path, body, timeout)
    monkeypatch.setattr(robot, "_request", delayed)
    thread = threading.Thread(target=robot.emergency_stop)
    thread.start()
    try:
        assert entered.wait(1)
        with pytest.raises(ServiceError) as exc:
            robot.gripper.execute("enable", {})
        assert exc.value.code == "motion_active"
    finally:
        release.set()
        thread.join(2)
    assert "!HAND_EN" not in bridge.calls
