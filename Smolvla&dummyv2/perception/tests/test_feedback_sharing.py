"""No hardware: exercise multiple consumers at 40 Hz against shared 5 Hz reads."""
import ast
from pathlib import Path
import threading
from types import SimpleNamespace

import pytest

from perception_service.robot import RobotClient, BridgeFailure
from perception_service.live_control import LiveControl
from perception_service.common import ServiceError


def test_control_dashboard_and_recording_share_five_real_samples(monkeypatch):
    clock = [1_000_000_000]
    monkeypatch.setattr('perception_service.robot.monotonic_ns', lambda: clock[0])
    calls, recorded = [], []
    robot = RobotClient(start_sampler=False, state_callback=recorded.append)
    robot._request = lambda *args, **kwargs: calls.append(clock[0]) or {
        'raw_response': 'ok 0 0 90 0 45 0\r\n'}
    try:
        for tick in range(40):
            clock[0] = 1_000_000_000 + tick * 25_000_000
            sampler = robot.read_state(automatic=True)
            dashboard = robot.read_state()
            control = robot.read_state()
            assert sampler['monotonic_ns'] == dashboard['monotonic_ns'] == control['monotonic_ns']
        assert calls == [1_000_000_000 + n * 200_000_000 for n in range(5)]
        assert len(recorded) == 5  # Cached reads must not create fake recording samples.
    finally:
        robot.close()


def test_concurrent_consumers_do_not_duplicate_reads():
    robot = RobotClient(start_sampler=False)
    entered, release = threading.Event(), threading.Event()
    calls, states = [], []
    def request(*args, **kwargs):
        calls.append(1)
        entered.set()
        assert release.wait(2)
        return {'raw_response': 'ok 0 0 90 0 45 0\r\n'}
    robot._request = request
    threads = [threading.Thread(target=lambda: states.append(robot.read_state())) for _ in range(3)]
    try:
        threads[0].start()
        assert entered.wait(2)
        for thread in threads[1:]:
            thread.start()
        release.set()
        for thread in threads:
            thread.join(2)
            assert not thread.is_alive()
        assert len(calls) == 1
        assert len({s['monotonic_ns'] for s in states}) == 1
    finally:
        release.set()
        robot.close()


def test_failed_reads_are_shared_without_returning_last_good_angles(monkeypatch):
    clock = [1_000_000_000]
    monkeypatch.setattr('perception_service.robot.monotonic_ns', lambda: clock[0])
    robot = RobotClient(start_sampler=False)
    calls = []
    def request(*args, **kwargs):
        calls.append(clock[0])
        if len(calls) > 1:
            raise BridgeFailure('worker_unavailable')
        return {'raw_response': 'ok 0 0 90 0 45 0\r\n'}
    robot._request = request
    try:
        assert robot.read_state()['positions'] is not None
        clock[0] += 200_000_000
        failed = robot.read_state()
        for _ in range(40):
            state = robot.read_state()
            assert state['positions'] is None
            assert state['monotonic_ns'] == failed['monotonic_ns']
        assert len(calls) == 2
    finally:
        robot.close()


def test_forwarded_cached_sample_retains_age_and_expires(monkeypatch):
    clock = [1000]
    monkeypatch.setattr('perception_service.live_control.wall_time_ms', lambda: clock[0])
    state = {'positions': [0., 0., 90., 0., 45., 0.], 'sampled_at_ms': 1000}
    calls = []
    robot = SimpleNamespace(status=lambda: {'state': state})
    control = LiveControl(robot, start_worker=False,
                          request=lambda method, path, body, timeout: calls.append(body) or {})
    try:
        for age in [0, 25, 175, 200, 499]:
            clock[0] = 1000 + age
            control._publish_feedback(control._feedback())
            assert calls[-1]['age_ms'] == age
        clock[0] = 1501
        with pytest.raises(ServiceError) as error:
            control._feedback()
        assert error.value.code == 'joint_feedback_stale'
    finally:
        control.close()


def test_gateway_watchdog_does_not_refresh_replayed_feedback():
    # Load the actual two methods without importing ROS or starting any process.
    path = Path(__file__).parents[1] / 'motion/moveit_servo_gateway.py'
    tree = ast.parse(path.read_text(encoding='utf-8'))
    node = next(n for n in tree.body if isinstance(n, ast.ClassDef) and n.name == 'GatewayNode')
    methods = [n for n in node.body if isinstance(n, ast.FunctionDef)
               and n.name in {'set_feedback', 'feedback_fresh'}]
    clock = [10.]
    scope = {'time': SimpleNamespace(monotonic=lambda: clock[0]), 'hardware_to_model': list}
    exec(compile(ast.Module(body=methods, type_ignores=[]), str(path), 'exec'), scope)
    gateway = SimpleNamespace(lock=threading.RLock(), feedback=None, feedback_at=0.)
    for age in [0., .025, .2, .499, .501]:
        clock[0] = 10. + age
        scope['set_feedback'](gateway, [0]*6, age*1000)
        assert gateway.feedback_at == pytest.approx(10.)
        assert scope['feedback_fresh'](gateway) is (age <= .5)
