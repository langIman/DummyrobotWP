import threading

import pytest

from perception_service.common import ServiceError
from perception_service.live_control import LiveControl
from perception_service.robot import RobotClient, BridgeFailure


GOOD = 'ok -4.53 0.04 90.17 9.53 45.87 -7.81\r\n'
BAD = 'ok -4.53 0.04 10.17 9.53 45.87 -7.81\r\n'
RECOVERED = 'ok -4.61 0.04 90.16 9.69 45.88 -7.93\r\n'


@pytest.mark.parametrize('bad_reply', [BAD, 'ok -4.53 0.04', 'garbled\r\n',
    'ok 2.31 0.05 \x00\x00.\x004 -2.30 45.09 3.18'])
def test_one_bad_frame_is_not_published_or_used_for_motion(bad_reply):
    states, events, writes = [], [], []
    client = RobotClient(start_sampler=False, state_callback=states.append,
                         event_callback=events.append)
    replies = iter([GOOD, bad_reply, RECOVERED])
    def request(method, path, body=None, timeout=4):
        writes.append((path, body))
        if path == '/command':
            assert body['command'] == '#GETJPOS'
            return {'raw_response': next(replies)}
        assert path == '/motion'
        return {'accepted': True}
    client._request = request
    try:
        client._enabled_latch = 'confirmed'
        initial = client.read_state()
        state = client.read_state(force=True)
        assert state['positions'][2] == 90.16
        assert state['verification_retries'] == 1
        assert state['sampled_at_ms'] >= initial['sampled_at_ms']
        assert len(states) == 2 and all(s['positions'][2] > 90 for s in states)
        assert client.send_stream_target([-4.6, .04, 90.2, 9.7, 45.9, -8])['accepted']
        assert client.status()['enabled_latch'] == 'confirmed'
        assert [e['action'] for e in events if e['type'] == 'joint_feedback'] == ['rejected', 'recovered']
        assert all(body.get('command', '#GETJPOS') == '#GETJPOS' for _, body in writes)
    finally:
        client.close()


def test_stop_input_with_corrupt_reply_then_transport_failure_keeps_real_cause():
    client = RobotClient(start_sampler=False)
    calls, events = [], []
    replies = iter([GOOD, 'ok 2.31 0.05 \x00\x00.\x004 -2.30 45.09 3.18'])
    def request(method, path, body=None, timeout=4):
        calls.append((path, body))
        assert path == '/command' and body['command'] == '#GETJPOS'
        try:
            return {'raw_response': next(replies)}
        except StopIteration:
            raise BridgeFailure('worker_unavailable: local_worker_deadline')
    client._request = request
    gateway_calls = []
    control = LiveControl(client, request=lambda method, path, body, timeout:
                          gateway_calls.append((path, body)) or {},
                          start_worker=False, event_callback=events.append)
    emergency = []
    client.emergency_stop = lambda source: emergency.append(source) or {'overall_acknowledged': False}
    try:
        client._enabled_latch = 'confirmed'
        client.read_state()  # An old valid angle must never be reused to hold.
        control._armed = True
        control._session_id = 'test'
        control._motion_active = True
        with pytest.raises(ServiceError) as exc:
            control.update({'session_id': 'test', 'keys': []})
        assert exc.value.status == 503 and exc.value.code == 'hold_feedback_unavailable'
        assert 'local_worker_deadline' in control.status()['last_stop_reason']
        assert 'MoveIt' not in str(exc.value)
        assert len(calls) == 3 and all(path == '/command' for path, _ in calls)
        assert not control.is_armed()
        assert emergency == ['live_control_guard']
        assert gateway_calls[-1][0] == '/disarm'
        assert control.status()['diagnostics']['emergency_acknowledged'] is False
    finally:
        control.close()
        client.close()


def test_hold_recovers_from_one_corrupt_reply_using_fresh_angles_only():
    client = RobotClient(start_sampler=False)
    calls = []
    replies = iter([GOOD, 'ok 2.31 0.05 \x00\x00.\x004 -2.30 45.09 3.18', RECOVERED])
    def request(method, path, body=None, timeout=4):
        calls.append((path, body))
        return {'raw_response': next(replies)} if path == '/command' else {'accepted': True}
    client._request = request
    try:
        client._enabled_latch = 'confirmed'
        client.read_state()
        assert client.hold_stream()['accepted']
        assert calls[-1][0] == '/motion'
        assert calls[-1][1]['positions'] == [-4.61, .04, 90.16, 9.69, 45.88, -7.93]
        assert len(calls) == 4
        assert client.status()['enabled_latch'] == 'confirmed'
    finally:
        client.close()


def test_persistent_jump_cannot_age_into_a_valid_sample(monkeypatch):
    clock = [1_000_000_000]
    monkeypatch.setattr('perception_service.robot.monotonic_ns', lambda: clock[0])
    client = RobotClient(start_sampler=False)
    replies = iter([GOOD] + [BAD]*6 + [RECOVERED])
    calls = []
    def request(method, path, body=None, timeout=4):
        calls.append(body['command'])
        return {'raw_response': next(replies)}
    client._request = request
    try:
        client._enabled_latch = 'confirmed'
        first = client.read_state()
        clock[0] += 40_000_000
        assert client.read_state(force=True)['reason'] == 'joint_feedback_unreliable'
        assert client.status()['state']['positions'] is None
        with pytest.raises(ServiceError, match='关节反馈异常') as error:
            client.send_stream_target(first['positions'])
        assert error.value.code == 'joint_feedback_unreliable'
        clock[0] += 10_000_000_000
        assert client.read_state()['reason'] == 'joint_feedback_unreliable'
        assert client.read_state(force=True)['positions'][2] == 90.16
        assert calls == ['#GETJPOS'] * 8
    finally:
        client.close()


def test_slow_manual_reposition_is_not_a_short_sample_jump(monkeypatch):
    clock = [1_000_000_000]
    monkeypatch.setattr('perception_service.robot.monotonic_ns', lambda: clock[0])
    client = RobotClient(start_sampler=False)
    replies = iter([GOOD, BAD])
    client._request = lambda *args, **kwargs: {'raw_response': next(replies)}
    try:
        client.read_state()
        clock[0] += 2_000_000_000
        assert client.read_state()['positions'][2] == 10.17
    finally:
        client.close()


def test_control_waits_during_verification_then_resumes_without_disable():
    client = RobotClient(start_sampler=False)
    entered, release = threading.Event(), threading.Event()
    replies = iter([GOOD, BAD, RECOVERED])
    count = [0]
    def request(method, path, body=None, timeout=4):
        assert path == '/command' and body['command'] == '#GETJPOS'
        count[0] += 1
        if count[0] == 3:
            entered.set()
            assert release.wait(2)
        return {'raw_response': next(replies)}
    client._request = request
    control = LiveControl(client, start_worker=False)
    reader = None
    try:
        client._enabled_latch = 'confirmed'
        client.read_state()
        reader = threading.Thread(target=lambda: client.read_state(force=True))
        reader.start()
        assert entered.wait(2)
        with pytest.raises(ServiceError) as error:
            control._feedback()
        assert error.value.code == 'joint_feedback_verifying'
        assert client.status()['state']['positions'] is None
        assert client.status()['state']['sampled_at_ms'] is None
        control._armed = True
        class OneCycle:
            def __init__(self): self.n = 0
            def wait(self, delay):
                self.n += 1
                return self.n > 1
        original_shutdown = control._shutdown
        control._shutdown = OneCycle()
        control._run()
        control._shutdown = original_shutdown
        assert control.is_armed()
        assert control.status()['diagnostics']['output'] == 'feedback_verifying'
        assert client.status()['enabled_latch'] == 'confirmed'
        release.set()
        reader.join(2)
        assert not reader.is_alive()
        assert control._feedback()[2] == 90.16
    finally:
        release.set()
        if reader: reader.join(2)
        control._armed = False
        control.close()
        client.close()


def test_persistent_unreliable_feedback_runs_control_failsafe():
    client = RobotClient(start_sampler=False)
    client._last_state = client._empty_state('joint_feedback_unreliable')
    client._enabled_latch = 'confirmed'
    emergency = []
    client.emergency_stop = lambda source: emergency.append(source) or {'overall_acknowledged': True}
    control = LiveControl(client, request=lambda *args: {}, start_worker=False)
    original_shutdown = control._shutdown
    try:
        control._armed = True
        class OneCycle:
            def __init__(self): self.n = 0
            def wait(self, delay):
                self.n += 1
                return self.n > 1
        control._shutdown = OneCycle()
        control._run()
        assert not control.is_armed()
        assert emergency == ['live_control_guard']
        assert 'joint_feedback_unreliable' in control.status()['last_stop_reason']
    finally:
        control._shutdown = original_shutdown
        control.close()
        client.close()
