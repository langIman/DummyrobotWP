import pytest

from perception_service.common import ServiceError, wall_time_ms
from perception_service.robot import RobotClient


def test_ready_preset_requires_enable_confirmation_and_monitors_completion(monkeypatch):
    import threading
    client = RobotClient(start_sampler=False)
    target = [0., 0., 90., 0., 45., 0.]
    sent = []
    try:
        with pytest.raises(ServiceError) as exc:
            client.start_preset('ready', 'MOVE_READY')
        assert exc.value.code == 'robot_not_armed'
        client._enabled_latch = 'confirmed'
        with pytest.raises(ServiceError) as exc:
            client.start_preset('ready', 'MOVE_HOME')
        assert exc.value.code == 'motion_confirmation_required'
        monkeypatch.setattr(threading.Thread, 'start', lambda self: None)
        monkeypatch.setattr(client, 'read_state', lambda **kwargs: {'positions': target})
        monkeypatch.setattr(client._motion_cancel, 'wait', lambda timeout: False)
        def request(method, path, body=None, timeout=4):
            sent.append((path, body))
            return {'accepted': True}
        monkeypatch.setattr(client, '_request', request)
        result = client.start_preset('ready', 'MOVE_READY')
        assert result['motion']['status'] == 'running'
        assert result['motion']['speed'] == 12
        assert result['preset_speeds'] == {'home': 4, 'rest': 4, 'ready': 12}
        client._run_preset(result['motion']['id'], 'ready')
        assert client.status()['motion']['status'] == 'complete'
        assert sent == [('/motion', {'positions': target, 'speed': 12,
                                   'unit': 'degree', 'coordinate_space': 'hardware_joint'})]
    finally:
        client.close()


def test_ready_route_rejects_live_control_and_accepts_confirmed_preset():
    from types import SimpleNamespace
    from app import create_app
    armed = [True]
    calls = []
    def preset(name, confirmation):
        calls.append((name, confirmation))
        return {'motion': {'name': name, 'status': 'running'}}
    runtime = SimpleNamespace(robot=SimpleNamespace(start_preset=preset),
                              live_control=SimpleNamespace(is_armed=lambda: armed[0]))
    app = create_app(runtime)
    app.testing = True
    client = app.test_client()
    assert client.post('/api/robot/actions/ready', json={'confirmation':'MOVE_READY'}).status_code == 409
    assert not calls
    armed[0] = False
    assert client.post('/api/robot/actions/ready', json={'confirmation':'MOVE_READY'}).status_code == 202
    assert calls == [('ready','MOVE_READY')]


def test_motion_requires_service_enable_latch():
    client = RobotClient(start_sampler=False)
    try:
        with pytest.raises(ServiceError) as error:
            client.start_preset("home", "MOVE_HOME")
        assert error.value.code == "robot_not_armed"
    finally:
        client.close()


def test_enable_disable_and_emergency_are_whitelisted(monkeypatch):
    events = []
    client = RobotClient(start_sampler=False, event_callback=events.append)

    def fake_request(method, path, body=None, timeout=4):
        if path == "/stop":
            return {"stop": {"acknowledged": True}}
        command = body["command"]
        replies = {"!START": "Started ok\r\n", "!DISABLE": "Disabled ok\r\n", "!HAND_DIS": "ok hand disable\r\n"}
        return {"raw_response": replies[command], "sent": True}

    monkeypatch.setattr(client, "_request", fake_request)
    try:
        assert client.enable()["enabled_latch"] == "confirmed"
        assert client.disable()["enabled_latch"] == "disabled"
        result = client.emergency_stop()
        assert result["stop"]["ok"] and result["disable"]["ok"]
        assert result["overall_acknowledged"] is True
        assert [event["action"] for event in events if event["type"] == "robot_action"] == ["enable", "disable", "emergency_stop"]
    finally:
        client.close()


def test_emergency_transport_success_is_not_device_acknowledgement(monkeypatch):
    client = RobotClient(start_sampler=False)

    def fake_request(method, path, body=None, timeout=4):
        if path == "/stop":
            return {"stop": {"acknowledged": False}, "sent": True}
        return {"raw_response": "", "sent": True}

    monkeypatch.setattr(client, "_request", fake_request)
    try:
        result = client.emergency_stop()
        assert result["stop"]["ok"] and result["disable"]["ok"]
        assert result["overall_acknowledged"] is False
        assert client.status()["enabled_latch"] == "unknown"
    finally:
        client.close()


@pytest.mark.parametrize('transport_failed', [False, True])
def test_arm_emergency_commands_precede_gripper_even_on_transport_failure(monkeypatch, transport_failed):
    from perception_service.robot import BridgeFailure
    client = RobotClient(start_sampler=False)
    commands = []
    def request(method, path, body=None, timeout=4):
        commands.append('!STOP' if path == '/stop' else body['command'])
        if transport_failed:
            raise BridgeFailure('device_unreachable')
        return {'stop': {'acknowledged': True}, 'sent': True,
                'raw_response': 'Disabled ok\r\nok hand disable\r\n'}
    monkeypatch.setattr(client, '_request', request)
    try:
        result = client.emergency_stop()
        assert commands == ['!STOP', '!DISABLE', '!HAND_DIS']
        assert result['overall_acknowledged'] is (not transport_failed)
        assert client.status()['enabled_latch'] == ('unknown' if transport_failed else 'disabled')
    finally:
        client.close()


def test_firmware_termination_is_explicit_fault_and_never_retried(monkeypatch, caplog):
    from perception_service.robot import BridgeFailure
    client = RobotClient(start_sampler=False)
    calls = []
    def request(*args, **kwargs):
        calls.append(args)
        return {'sent': True, 'accepted': False,
                'raw_response': "terminate called after throwing an instance of '"}
    monkeypatch.setattr(client, '_request', request)
    try:
        client._enabled_latch = 'confirmed'
        client._last_state = {'positions': [0, 0, 90, 0, 45, 0], 'sampled_at_ms': wall_time_ms()}
        with pytest.raises(BridgeFailure, match='controller_firmware_exception'):
            client.send_stream_target([.2, 0, 90, 0, 45, 0])
        assert len(calls) == 1
        assert client.status()['enabled_latch'] == 'unknown'
        assert 'terminate called' in caplog.text
    finally:
        client.close()


def test_stream_gap_is_displayed_without_rejecting_large_difference(monkeypatch):
    client = RobotClient(start_sampler=False)
    sent = []
    monkeypatch.setattr(client, '_request', lambda *args, **kwargs: sent.append(args) or {'accepted': True})
    try:
        assert client.status()['stream_target_gap'] is None
        client._enabled_latch = 'confirmed'
        sampled_at = wall_time_ms()
        client._last_state = {'positions': [0, 0, 90, 0, 45, -41.19], 'sampled_at_ms': sampled_at}
        assert client.send_stream_target([1, 0, 90, 0, 45, -32.96])['accepted']
        sample = client.status()['stream_target_gap']
        assert sample['joint'] == 'J6'
        assert sample['gap_deg'] == pytest.approx(8.23)
        assert sample['target_deg'] == -32.96
        assert sample['actual_deg'] == -41.19
        assert sample['sampled_at_ms'] == sampled_at
        assert sample['compared_at_ms'] >= sampled_at
        assert sample['over_limit'] is False
        assert sample['limit_enabled'] is False
        assert len(sent) == 1
        # Reading status later does not silently substitute a different sample.
        client._last_state = {'positions': [0, 0, 90, 0, 45, -33], 'sampled_at_ms': wall_time_ms()}
        assert client.status()['stream_target_gap'] == sample
        client.send_stream_target([0, 0, 90, 0, 45, -33])
        assert client.status()['stream_target_gap']['gap_deg'] == 0
        assert client.status()['stream_target_gap']['over_limit'] is False
    finally:
        client.close()


def test_stream_target_retains_joint_limits_without_gap_cap(monkeypatch):
    client = RobotClient(start_sampler=False)
    sent = []

    def request(method, path, body=None, timeout=4):
        sent.append((method, path, body))
        return {"accepted": True}

    monkeypatch.setattr(client, "_request", request)
    try:
        client._enabled_latch = "confirmed"
        client._last_state = {
            "positions": [0.0, 0.0, 90.0, 0.0, 0.0, 0.0],
            "sampled_at_ms": wall_time_ms(),
        }
        assert client.send_stream_target([0.5, 0.0, 90.0, 0.0, 0.0, 0.0])["accepted"]
        assert sent[-1][1] == "/motion"
        sent.clear()
        client._last_state = {
            "positions": [0.0, -75.2, 90.0, 0.0, 0.0, 0.0],
            "sampled_at_ms": wall_time_ms(),
        }
        with pytest.raises(ServiceError) as exc:
            client.send_stream_target([0.0, -75.2, 90.0, 0.0, 0.0, 0.0])
        assert exc.value.code == 'stream_target_outside_limit'
        assert sent == []
        client._last_state = {
            "positions": [0.0, 0.0, 90.0, 0.0, 0.0, 0.0],
            "sampled_at_ms": wall_time_ms(),
        }
        assert client.send_stream_target([28.01, 0.0, 90.0, 0.0, 0.0, 0.0])['accepted']
        assert client.send_stream_target([7.99, 0.0, 90.0, 0.0, 0.0, 0.0])['accepted']
        assert sent[-1][2]['speed'] == 50
        for invalid_speed in (0, 50.01, float('nan'), float('inf'), True):
            count_before = len(sent)
            with pytest.raises(ServiceError) as exc:
                client.send_stream_target([7.99, 0.0, 90.0, 0.0, 0.0, 0.0], speed=invalid_speed)
            assert exc.value.code == 'stream_speed_invalid'
            assert len(sent) == count_before
        with pytest.raises(ServiceError) as exc:
            client.send_stream_target([0.0, -80.0, 90.0, 0.0, 0.0, 0.0])
        assert exc.value.code == "stream_target_outside_limit"
        client._last_state = {
            "positions": [0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
            "sampled_at_ms": wall_time_ms(),
        }
        assert client.send_stream_target([0.0, 0.0, 35.0, 0.0, 0.0, 0.0])['accepted']
        # Firmware rejects even slightly out-of-range motion destinations.
        with pytest.raises(ServiceError) as exc:
            client.send_stream_target([0.0, 0.0, 34.999, 0.0, 0.0, 0.0])
        assert exc.value.code == "stream_target_outside_limit"
    finally:
        client.close()


def test_prepare_stream_requires_explicit_firmware_ack(monkeypatch):
    client = RobotClient(start_sampler=False)
    try:
        client._enabled_latch = 'confirmed'
        monkeypatch.setattr(client,'_request',lambda *args,**kwargs:{'raw_response':'ok Set command mode to [2]\r\n'})
        client.prepare_stream()
        monkeypatch.setattr(client,'_request',lambda *args,**kwargs:{'raw_response':'ok\r\n'})
        with pytest.raises(ServiceError) as exc:
            client.prepare_stream()
        assert exc.value.code == 'stream_mode_unconfirmed'
    finally:
        client.close()
