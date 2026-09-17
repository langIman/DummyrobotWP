import time
from types import SimpleNamespace

import pytest

from app import create_app
from perception_service.common import ServiceError, wall_time_ms
from perception_service.live_control import LiveControl, MoveItFailure


class FakeGripper:
    def __init__(self):
        self.disable_count = 0

    def disable(self, **kwargs):
        self.disable_count += 1
        return {'acknowledged': True}

    def active(self):
        return False


class FakeRobot:
    def __init__(self):
        self.gripper = FakeGripper()
        self.positions = [0.0, 0.0, 90.0, 0.0, 0.0, 0.0]
        self.enabled = "confirmed"
        self.targets = []
        self.emergencies = []
        self.disable_count = 0
        self.holds = 0
        self.hold_modes = []

    def prepare_stream(self):
        pass

    def hold_stream(self, *, predictive=False):
        self.holds += 1
        self.hold_modes.append(predictive)
        return {'accepted':True}

    def status(self):
        return {
            "bridge_status": "ready", "enabled_latch": self.enabled, "motion": None,
            "state": {"positions": list(self.positions), "sampled_at_ms": wall_time_ms()},
        }

    def read_state(self):
        return self.status()["state"]

    def send_stream_target(self, positions, speed=4):
        self.targets.append((positions, speed))
        return {"accepted": True}

    def emergency_stop(self, source="operator"):
        self.emergencies.append(source)
        self.enabled = "disabled"
        return {"overall_acknowledged": True}

    def disable(self):
        self.disable_count += 1
        self.enabled = "disabled"
        return self.status()


class FakeMoveIt:
    def __init__(self):
        self.calls = []
        self.sequence = -1
        self.positions = None
        self.servo_status = None
        self.guard_warning = None
        self.age_ms = 1

    def request(self, method, path, body, timeout):
        self.calls.append((method, path, body))
        if path == "/health":
            return {"service": "dummy-moveit-safe-gateway", "protocol_version": 7,
                    "command_frame": "gripper_tool",
                    "status": "ready"}
        if path == "/arm":
            return {"armed": True}
        if path == "/disarm":
            return {"armed": False}
        if path in {"/feedback", "/command"}:
            return {"accepted": True}
        if path == "/trajectory":
            return {"sequence": self.sequence, "positions": self.positions, "age_ms": self.age_ms,
                    "servo_status": self.servo_status, "guard_warning": self.guard_warning}
        raise AssertionError(path)


def make_control():
    robot = FakeRobot()
    moveit = FakeMoveIt()
    events = []
    control = LiveControl(robot, request=moveit.request, event_callback=events.append,
                          start_worker=False)
    return control, robot, moveit, events


@pytest.mark.parametrize('reason', ['operator', 'page_hidden', 'service_shutdown'])
def test_exit_disables_arm_and_gripper_together(reason):
    control, robot, _, events = make_control()
    control.arm({'confirmation': 'ENABLE_LIVE_CONTROL'})
    result = control.disarm(reason)
    assert robot.disable_count == robot.gripper.disable_count == 1
    assert result['overall_acknowledged'] is True
    assert not control.is_armed()
    assert events[-1]['overall_acknowledged'] is True


@pytest.mark.parametrize('failed_part', ['arm', 'gripper', 'gripper_unconfirmed'])
def test_exit_attempts_both_disables_despite_failure(failed_part):
    control, robot, _, _ = make_control()
    control.arm({'confirmation': 'ENABLE_LIVE_CONTROL'})
    def fail(**kwargs):
        raise RuntimeError('USB timeout')
    if failed_part == 'arm':
        robot.disable = fail
    elif failed_part == 'gripper':
        robot.gripper.disable = fail
    else:
        robot.gripper.disable = lambda **kwargs: {'acknowledged': False}
    result = control.disarm()
    assert result['overall_acknowledged'] is False
    if failed_part == 'arm':
        assert robot.gripper.disable_count == 1
    else:
        assert robot.disable_count == 1


def test_gateway_only_disarm_does_not_touch_hardware():
    control, robot, _, _ = make_control()
    control.arm({'confirmation': 'ENABLE_LIVE_CONTROL'})
    control.disarm(disable_robot=False)
    assert robot.disable_count == robot.gripper.disable_count == 0


@pytest.mark.parametrize('version,frame', [(3, 'gripper_tool'), (5, 'base_translation_tool_rotation'), (6, 'base_translation_base_rotation'), (6, 'gripper_tool'), (7, 'base_translation_base_rotation')])
def test_wrong_coordinate_contract_cannot_arm(version, frame):
    control, robot, moveit, _ = make_control()
    def request(method, path, body, timeout):
        result = moveit.request(method, path, body, timeout)
        if path == '/health':
            result.update(protocol_version=version, command_frame=frame)
        return result
    control._request_override = request
    with pytest.raises(ServiceError) as exc:
        control.arm({'confirmation': 'ENABLE_LIVE_CONTROL'})
    assert exc.value.code == 'moveit_unavailable'
    assert all(call[1] == '/health' for call in moveit.calls)
    assert not robot.targets


def update(control, body):
    return control.update({**body, 'session_id':control._session_id})


def test_40hz_output_counter_does_not_saturate():
    control, _, _, _ = make_control()
    now = time.monotonic()
    control._output_times.extend(now - n*.025 for n in range(200))
    status = control.status()
    assert status['measured_output_hz'] == 40
    assert status['output_rate_hz'] == 40
    assert status['feedback_rate_hz'] == 5


def test_worker_includes_io_time_in_25ms_period(monkeypatch):
    control, _, _, _ = make_control()
    clock, starts = [0.], []
    monkeypatch.setattr('perception_service.live_control.time.monotonic', lambda: clock[0])
    class Stop:
        def wait(self, delay):
            clock[0] += delay
            return len(starts) == 5
    control._shutdown = Stop()
    control._armed = True
    def cycle():
        starts.append(clock[0])
        clock[0] += .015  # Time spent in HTTP/serial calls.
    control._cycle = cycle
    control._run()
    assert starts == pytest.approx([0, .025, .05, .075, .1])


def test_arm_refreshes_feedback_after_slow_mode_switch(monkeypatch):
    control, robot, moveit, _ = make_control()
    clock, sampled, order = [1000], [1000], []
    monkeypatch.setattr('perception_service.live_control.wall_time_ms', lambda: clock[0])
    original_status = robot.status
    def status():
        state = original_status()
        state['state']['sampled_at_ms'] = sampled[0]
        return state
    robot.status = status
    def prepare():
        order.append('prepare')
        clock[0] += 501  # Actual #CMDMODE transaction observed in bridge logs.
    robot.prepare_stream = prepare
    def read():
        order.append('read')
        sampled[0] = clock[0]
        robot.positions[1] = 12.
        return status()['state']
    robot.read_state = read
    original_request = moveit.request
    def request(method, path, body, timeout):
        if path == '/feedback':
            order.append('publish')
            assert body['age_ms'] == 0
            assert body['positions'][1] == 12.
        return original_request(method, path, body, timeout)
    control._request_override = request
    assert control.arm({'confirmation': 'ENABLE_LIVE_CONTROL'})['armed']
    assert order == ['prepare', 'read', 'publish']
    assert not robot.targets  # Entering control does not generate joint motion.


def test_arm_never_uses_old_feedback_if_refresh_fails():
    control, robot, moveit, _ = make_control()
    robot.read_state = lambda: {'positions': None, 'reason': 'unavailable'}
    original_status = robot.status
    prepared = [False]
    robot.prepare_stream = lambda: prepared.__setitem__(0, True)
    def status():
        value = original_status()
        if prepared[0]:
            value['state'] = {'positions': None, 'reason': 'unavailable'}
        return value
    robot.status = status
    with pytest.raises(ServiceError) as error:
        control.arm({'confirmation': 'ENABLE_LIVE_CONTROL'})
    assert error.value.code == 'joint_feedback_unavailable'
    assert not control.is_armed()
    assert not any(path == '/arm' for _, path, _ in moveit.calls)
    assert not robot.targets


def test_other_tab_cannot_override_the_owner_input():
    control, robot, moveit, _ = make_control()
    armed = control.arm({'confirmation':'ENABLE_LIVE_CONTROL'})
    update(control, {'keys':['w']})
    count = len(moveit.calls)
    assert 'session_id' not in control.status()
    assert armed['session_id']
    with pytest.raises(ServiceError) as exc:
        control.update({'keys':[], 'session_id':'other-tab'})
    assert exc.value.code == 'control_session_mismatch'
    assert len(moveit.calls) == count
    assert control.status()['input_command']['linear_x'] == .05


def test_release_holds_once_without_disabling_and_can_continue():
    control, robot, moveit, _ = make_control()
    control.arm({'confirmation':'ENABLE_LIVE_CONTROL'})
    update(control, {'keys':['w']})
    moveit.sequence = 1
    moveit.positions = [.2,0,90,0,0,0]
    control._cycle()
    update(control, {'keys':[]})
    update(control, {'keys':[]})
    assert robot.holds == 1
    assert robot.enabled == 'confirmed'
    assert not robot.emergencies
    control._last_cycle_monotonic = 0
    moveit.sequence = 2
    update(control, {'keys':['a']})
    control._cycle()
    assert len(robot.targets) == 2


def test_release_sends_no_target_and_immediate_stop_can_cancel_remaining_motion():
    control, robot, moveit, events = make_control()
    control.arm({'confirmation': 'ENABLE_LIVE_CONTROL'})
    update(control, {'keys': ['w']})
    moveit.sequence, moveit.positions = 1, [.2, 0, 90, 0, 0, 0]
    moveit.servo_status = {'code': 0, 'message': 'No warnings'}
    control._cycle()
    update(control, {'keys': [], 'stop_mode': 'release'})
    update(control, {'keys': [], 'stop_mode': 'release'})
    control._last_cycle_monotonic = 0
    moveit.sequence += 1
    moveit.positions = [4., 0, 90, 0, 0, 0]  # A late trajectory must not be sent.
    control._cycle()
    assert robot.hold_modes == []
    assert len(robot.targets) == 1
    assert control.status()['diagnostics']['output'] == 'released'
    assert len([e for e in events if e.get('action') == 'release']) == 1
    update(control, {'keys': [], 'stop_mode': 'immediate'})
    assert robot.hold_modes == [False]
    update(control, {'keys': [], 'stop_mode': 'immediate'})
    assert robot.hold_modes == [False]
    assert robot.enabled == 'confirmed'


@pytest.mark.parametrize('reason', ['timeout', 'collision', 'resume_then_stop'])
def test_release_does_not_bypass_later_stop_conditions(reason):
    control, robot, moveit, _ = make_control()
    control.arm({'confirmation': 'ENABLE_LIVE_CONTROL'})
    control._motion_active = True
    update(control, {'keys': [], 'stop_mode': 'release'})
    assert not robot.hold_modes
    if reason == 'resume_then_stop':
        # Stop before the first new trajectory arrives must cancel the old goal.
        update(control, {'keys': ['w'], 'stop_mode': 'release'})
        update(control, {'keys': [], 'stop_mode': 'immediate'})
    else:
        if reason == 'timeout':
            control._last_input_monotonic = time.monotonic() - 2
        else:
            moveit.servo_status = {'code': 5, 'message': 'Collision'}
        control._cycle()
    assert robot.hold_modes == [False]


def test_movement_resumes_after_release_without_extra_hold():
    control, robot, moveit, _ = make_control()
    control.arm({'confirmation': 'ENABLE_LIVE_CONTROL'})
    update(control, {'keys': ['w'], 'stop_mode': 'release'})
    moveit.sequence, moveit.positions = 1, [.2, 0, 90, 0, 0, 0]
    control._cycle()
    update(control, {'keys': [], 'stop_mode': 'release'})
    update(control, {'keys': ['s'], 'stop_mode': 'release'})
    moveit.sequence, moveit.positions = 2, [-.2, 0, 90, 0, 0, 0]
    control._last_cycle_monotonic = 0
    control._cycle()
    assert len(robot.targets) == 2
    assert not robot.holds
    assert not control._release_pending


def test_fault_pause_bypasses_release_prediction():
    control, robot, moveit, _ = make_control()
    control.arm({'confirmation': 'ENABLE_LIVE_CONTROL'})
    control._motion_active = True
    control._pause_motion('moveit_servo_5: collision')
    assert robot.hold_modes == [False]
    with pytest.raises(ServiceError) as exc:
        update(control, {'stop_mode': 'anything'})
    assert exc.value.code == 'control_stop_mode_invalid'


@pytest.mark.parametrize('code', [None, 0, 4])
def test_normal_release_never_replaces_target_for_prediction_fallback(code):
    control, robot, moveit, _ = make_control()
    control.arm({'confirmation': 'ENABLE_LIVE_CONTROL'})
    control._motion_active = True
    control._last_output_monotonic = time.monotonic()
    control._diagnostics['servo_status'] = {'code': code}
    moveit.servo_status = {'code': code}
    update(control, {'keys': [], 'stop_mode': 'release'})
    control._cycle()
    assert robot.hold_modes == []
    assert not robot.targets


def test_failsafe_clears_stale_input_and_sending_status():
    control, robot, moveit, _ = make_control()
    control.arm({'confirmation':'ENABLE_LIVE_CONTROL'})
    update(control, {'keys':['w']})
    control._failsafe('simulated_failure')
    assert control.status()['diagnostics']['output'] == 'stopped'
    assert control.status()['diagnostics']['emergency_acknowledged'] is True
    assert not any(control.status()['input_command'].values())
    assert control._session_id is None


def test_emergency_failure_still_disarms_gateway_and_records_failure():
    control, robot, moveit, events = make_control()
    control.arm({'confirmation':'ENABLE_LIVE_CONTROL'})
    def unavailable(*args):
        raise RuntimeError('transport disconnected')
    robot.emergency_stop = unavailable
    control._failsafe('test_disconnect')
    assert not control.is_armed()
    assert moveit.calls[-1][1] == '/disarm'
    assert events[-1]['emergency']['overall_acknowledged'] is False
    assert control.status()['diagnostics']['emergency_acknowledged'] is False


def test_emergency_during_arm_cannot_reenable_control():
    control, robot, moveit, _ = make_control()
    original = moveit.request
    def interrupt(method, path, body, timeout):
        result = original(method, path, body, timeout)
        if path == '/arm':
            control.emergency_reset()
        return result
    control._request_override = interrupt
    with pytest.raises(ServiceError) as exc:
        control.arm({'confirmation':'ENABLE_LIVE_CONTROL'})
    assert exc.value.code == 'live_control_cancelled'
    assert not control.is_armed()
    assert not robot.targets
    assert moveit.calls[-1][1] == '/disarm'


def test_disarm_still_sent_when_zero_command_is_rejected():
    control, robot, moveit, _ = make_control()
    control.arm({'confirmation':'ENABLE_LIVE_CONTROL'})
    original = moveit.request
    def unavailable(method, path, body, timeout):
        if path == '/command':
            raise MoveItFailure('already disarmed')
        return original(method, path, body, timeout)
    control._request_override = unavailable
    control.disarm()
    assert moveit.calls[-1][1] == '/disarm'
    assert robot.disable_count == 1


def test_guard_requires_release_before_resuming():
    control, robot, moveit, _ = make_control()
    control.arm({'confirmation':'ENABLE_LIVE_CONTROL'})
    update(control, {'keys':['w']})
    control._pause_motion('motion_no_progress')
    count = len(moveit.calls)
    assert update(control, {'keys':['w']})['paused']
    assert len(moveit.calls) == count
    update(control, {'keys':[]})
    assert not control.status()['release_required']
    update(control, {'keys':['w']})
    assert moveit.calls[-1][2]['linear_x'] == .05


def test_release_during_trajectory_query_drops_old_target():
    control, robot, moveit, _ = make_control()
    control.arm({'confirmation':'ENABLE_LIVE_CONTROL'})
    update(control, {'keys':['w']})
    original = moveit.request
    def query(method,path,body,timeout):
        if path == '/trajectory':
            update(control, {'keys':[], 'stop_mode':'release'})
            return {'sequence':1,'positions':[.2,0,90,0,0,0],'age_ms':1}
        return original(method,path,body,timeout)
    control._request_override = query
    control._cycle()
    assert not robot.targets


def test_arm_requires_confirmation_and_never_enables_robot_implicitly():
    control, robot, moveit, events = make_control()
    with pytest.raises(ServiceError) as exc:
        control.arm({"confirmation": "yes"})
    assert exc.value.code == "live_control_confirmation_required"
    assert not moveit.calls
    result = control.arm({"confirmation": "ENABLE_LIVE_CONTROL"})
    assert result["armed"] is True
    assert robot.enabled == "confirmed"
    assert [call[1] for call in moveit.calls] == ["/health", "/feedback", "/arm", "/command"]
    assert events[-1]["action"] == "armed"


def test_wasd_and_turn_keys_map_to_bounded_tool_velocity():
    control, _, moveit, _ = make_control()
    control.arm({"confirmation": "ENABLE_LIVE_CONTROL"})
    result = update(control, {"yaw": 0.5, "pitch": -0.25, "keys": ["w", "a"]})
    command = result["command"]
    assert command["linear_x"] == pytest.approx(0.05)
    assert command["linear_y"] == pytest.approx(0.05)
    assert command["angular_y"] == pytest.approx(0.06)
    assert command["angular_z"] == pytest.approx(-0.12)
    assert moveit.calls[-1][1] == "/command"
    with pytest.raises(ServiceError):
        update(control, {"yaw": float("nan"), "keys": []})


@pytest.mark.parametrize('keys,expected', [
    (['arrowup'], [0, 0, .05]),
    (['arrowdown'], [0, 0, -.05]),
    (['arrowup', 'arrowdown'], [0, 0, 0]),
    (['w', 'arrowdown'], [.05, 0, -.05]),
    (['w', 's', 'a', 'd', 'arrowup', 'arrowdown'], [0, 0, 0]),
    (['w', 'a', 'arrowup'], [.05] * 3),
    (['s', 'd', 'arrowdown'], [-.05] * 3),
])
def test_arrow_translation_in_tool_z_preserves_tool_rotation(keys, expected):
    control, robot, moveit, events = make_control()
    control.arm({'confirmation': 'ENABLE_LIVE_CONTROL'})
    command = update(control, {'keys': keys, 'pitch': .5, 'yaw': .25})['command']
    assert [command[name] for name in ('linear_x', 'linear_y', 'linear_z')] == pytest.approx(expected)
    assert command['angular_y'] == -.12 and command['angular_z'] == -.06
    assert moveit.calls[-1][2] == command
    assert events[-1]['command_frame'] == 'gripper_tool'
    released = update(control, {'keys': [], 'pitch': .5, 'yaw': .25})['command']
    assert released['linear_x'] == released['linear_y'] == released['linear_z'] == 0
    assert released['angular_y'] == -.12  # Release only the keyboard input.
    assert not robot.emergencies


def test_new_moveit_trajectory_is_forwarded_once():
    control, robot, moveit, _ = make_control()
    control.arm({"confirmation": "ENABLE_LIVE_CONTROL"})
    update(control, {"yaw": 0, "pitch": 0, "keys": ["w"]})
    moveit.sequence = 0
    moveit.positions = [0.2, 0.0, 90.0, 0.0, 0.0, 0.0]
    control._cycle()
    control._cycle()
    assert robot.targets == [(moveit.positions, 50)]


def test_browser_idle_commands_zero_without_disabling():
    control, robot, moveit, events = make_control()
    control.arm({"confirmation": "ENABLE_LIVE_CONTROL"})
    update(control, {"yaw": 0, "pitch": 0, "keys": ["w"]})
    control._last_input_monotonic = time.monotonic() - 2
    control._cycle()
    assert control.status()["armed"] is True
    assert control.status()["idle_auto_disarm"] is False
    assert control.status()["paused_reason"] == "browser_input_idle"
    assert robot.emergencies == []
    assert moveit.calls[-1] == ("POST", "/command", control._zero_command())
    assert events[-1]["action"] == "paused"


def test_servo_singularity_pauses_without_disabling():
    control, robot, moveit, _ = make_control()
    control.arm({"confirmation": "ENABLE_LIVE_CONTROL"})
    update(control, {"yaw": 0.2, "pitch": 0, "keys": []})
    moveit.servo_status = {"code": 2, "message": "Very close to a singularity"}
    control._cycle()
    assert control.status()["armed"] is True
    assert control.status()["paused_reason"].startswith("moveit_servo_2")
    assert robot.emergencies == []


def test_disarm_without_an_active_session_does_not_disable_robot():
    control, robot, _, _ = make_control()
    result = control.disarm("page_hidden")
    assert result["armed"] is False
    assert robot.disable_count == 0


def test_control_http_routes_delegate_to_guard():
    class FakeControl:
        def status(self): return {"armed": False}
        def arm(self, body): return {"armed": body["confirmation"] == "ENABLE_LIVE_CONTROL"}
        def update(self, body): return {"accepted": body["yaw"] == 0.2}
        def disarm(self, reason): return {"armed": False, "reason": reason}

    runtime = SimpleNamespace(robot=SimpleNamespace(), live_control=FakeControl())
    app = create_app(runtime)
    app.testing = True
    client = app.test_client()
    assert client.get("/api/control/status").json == {"armed": False}
    assert client.post("/api/control/arm", json={"confirmation": "ENABLE_LIVE_CONTROL"}).json["armed"]
    assert client.post("/api/control/input", json={"yaw": 0.2}).json["accepted"]
    assert client.post("/api/control/disarm", json={"reason": "test"}).json["reason"] == "test"


def test_idle_keeps_feedback_current_without_sending_hardware_targets():
    control, robot, moveit, _ = make_control()
    control.arm({"confirmation": "ENABLE_LIVE_CONTROL"})
    moveit.calls.clear()
    robot.positions[1] = 12.0
    control._cycle()
    feedback = next(body for method, path, body in moveit.calls if path == '/feedback')
    assert feedback['positions'] == robot.positions
    assert 0 <= feedback['age_ms'] < 500
    assert not robot.targets
    assert control.status()["diagnostics"]["output"] == "idle"


def test_noop_target_and_collision_deceleration_are_visible():
    control, robot, moveit, _ = make_control()
    control.arm({"confirmation": "ENABLE_LIVE_CONTROL"})
    update(control, {"keys": ["w"]})
    moveit.sequence = 0
    moveit.positions = robot.positions
    moveit.servo_status = {"code": 4, "message": "Close to a collision, decelerating"}
    robot.send_stream_target = lambda *args, **kwargs: {"accepted": True, "skipped": "target_matches_feedback"}
    control._cycle()
    status = control.status()
    assert status["input_command"]["linear_x"] == 0.05
    assert status["diagnostics"]["output"] == "target_matches_feedback"
    assert status["diagnostics"]["servo_status"]["code"] == 4
    assert not robot.emergencies


def test_singularity_warning_not_hidden_by_collision_status():
    control, robot, moveit, _ = make_control()
    control.arm({"confirmation": "ENABLE_LIVE_CONTROL"})
    update(control, {"keys": ["w"]})
    moveit.servo_status = {"code": 4, "message": "Close to a collision, decelerating"}
    moveit.guard_warning = {"code": "singularity_stop", "age_ms": 10}
    control._cycle()
    assert control.status()["paused_reason"].startswith("moveit_servo_2")
    assert control.status()["diagnostics"]["output"] == "paused"
    control._last_cycle_monotonic = 0
    control._cycle()
    assert control.status()["diagnostics"]["output"] == "paused"
    assert not robot.targets
    assert not robot.emergencies


def test_unchanged_stale_trajectory_is_not_silently_waited_on():
    control, robot, moveit, _ = make_control()
    control.arm({"confirmation": "ENABLE_LIVE_CONTROL"})
    update(control, {"keys": ["w"]})
    moveit.sequence = 7
    control._last_trajectory_seq = 7
    moveit.age_ms = 800
    control._cycle()
    assert control.status()["paused_reason"] == "moveit_trajectory_stale"
    assert not robot.targets


def test_restart_waits_for_first_velocity_but_missing_output_times_out():
    control, robot, moveit, _ = make_control()
    control.arm({'confirmation': 'ENABLE_LIVE_CONTROL'})
    update(control, {'keys': ['w']})
    moveit.sequence = 7
    moveit.positions = [0.2, 0, 90, 0, 0, 0]
    control._cycle()
    update(control, {'keys': []})
    update(control, {'keys': ['s']})
    missing_age = 10
    def request(method, path, body, timeout):
        if path == '/feedback':
            return {'accepted': True, 'trajectory': {
                'positions': None, 'sequence': 8, 'age_ms': None,
                'missing_age_ms': missing_age}}
        return moveit.request(method, path, body, timeout)
    control._request_override = request
    control._last_cycle_monotonic = 0
    control._cycle()
    assert control.status()['paused_reason'] is None
    assert control.status()['diagnostics']['output'] == 'waiting_trajectory'
    assert len(robot.targets) == 1
    missing_age = 251
    control._last_cycle_monotonic = 0
    control._cycle()
    assert control.status()['paused_reason'] == 'moveit_trajectory_stale'
    assert len(robot.targets) == 1


def test_sending_target_is_not_reported_as_idle():
    control, robot, moveit, _ = make_control()
    control.arm({"confirmation": "ENABLE_LIVE_CONTROL"})
    update(control, {"keys": ["w"]})
    moveit.sequence = 0
    moveit.positions = [0.1, 0, 90, 0, 0, 0]
    def send(*args, **kwargs):
        assert control.status()["diagnostics"]["output"] == "sending_target"
        return {"accepted": True}
    robot.send_stream_target = send
    control._cycle()
    assert control.status()["diagnostics"]["output"] == "target_sent"
