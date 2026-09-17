import threading
from types import SimpleNamespace

import pytest
import requests

from app import create_app
from perception_service.common import ServiceError
from perception_service.robot import RobotClient


@pytest.fixture
def system(monkeypatch):
    calls, events = [], []
    robot = RobotClient(start_sampler=False, event_callback=events.append)
    def request(method, url, json=None, timeout=None):
        calls.append((method, url, json))
        command = (json or {}).get('command')
        raw = {'!HAND_DIS': 'ok hand disable', '!DISABLE': 'Disabled ok'}.get(command, 'ok\r\n')
        return SimpleNamespace(status_code=200, json=lambda: {
            'sent': True, 'bytes_written': len(command or '') + 1,
            'raw_response': raw, 'pending_response': 'earlier reply\r\n',
            'stop': {'acknowledged': True}})
    monkeypatch.setattr(robot._session, 'request', request)
    yield robot, calls, events
    robot.close()


@pytest.mark.parametrize('command', ['#GETJPOS', '!HAND_ZERO', '!HAND_POS 42',
                                    '!HAND_I 1.5', '!START', '>0,0,90,0,0,0,4',
                                    'unknown command  ', 'X' * 62])
def test_arbitrary_commands_are_exact_and_never_claim_execution(system, command):
    robot, calls, events = system
    assert calls == []
    robot._enabled_latch = 'confirmed'
    result = robot.raw_command({'command': command})
    assert calls == [('POST', 'http://127.0.0.1:8765/command',
                      {'command': command, 'read_timeout_ms': 500})]
    assert result['sent'] is True
    assert result['execution_status'] == 'unknown'
    assert result['raw_response'] == 'ok\r\n'
    assert result['pending_response'] == 'earlier reply\r\n'
    assert 'acknowledged' not in result
    assert robot.status()['enabled_latch'] == 'unknown'
    assert robot.status()['raw_console']['active']
    assert events[-1]['type'] == 'robot_raw_command'
    assert events[-1]['command'] == command
    before = len(calls)
    robot.read_state(automatic=True)
    assert len(calls) == before


@pytest.mark.parametrize('body', [[], {}, {'command': ''}, {'command': '   '},
    {'command': '!START\n!STOP'}, {'command': 'X\r'}, {'command': '\x00'},
    {'command': '\tX'}, {'command': '\x7f'}, {'command': '打开'},
    {'command': 'X'*63}, {'command': 123}, {'command':'X', 'extra':True},
    {'command':'X', 'read_timeout_ms':True}, {'command':'X', 'read_timeout_ms':-1},
    {'command':'X', 'read_timeout_ms':2001}, {'command':'X', 'read_timeout_ms':'500'}])
def test_invalid_payload_never_reaches_bridge(system, body):
    robot, calls, _ = system
    with pytest.raises(ServiceError) as exc:
        robot.raw_command(body)
    assert exc.value.status == 422
    assert calls == []
    assert not robot.status()['raw_console']['active']


def test_raw_mode_blocks_structured_actions_until_emergency_confirmed(system):
    robot, calls, _ = system
    robot.raw_command({'command': '!HAND_EN'})
    for operation in [robot.enable, lambda:robot.start_preset('home','MOVE_HOME'),
                      lambda:robot.gripper.execute('enable', {})]:
        with pytest.raises(ServiceError):
            operation()
    assert len(calls) == 1
    result = robot.emergency_stop()
    assert result['overall_acknowledged']
    assert [entry[2].get('command', '!STOP') for entry in calls[1:]] == ['!STOP', '!DISABLE', '!HAND_DIS']
    assert not robot.status()['raw_console']['active']


def test_active_preset_gripper_and_emergency_block_raw(system):
    robot, calls, _ = system
    for attribute, value in [('_motion', {'status':'running'}), ('_emergency_count',1)]:
        old = getattr(robot, attribute)
        setattr(robot, attribute, value)
        with pytest.raises(ServiceError):
            robot.raw_command({'command':'X'})
        setattr(robot, attribute, old)
    robot.gripper._busy = True
    with pytest.raises(ServiceError):
        robot.raw_command({'command':'X'})
    robot.gripper._busy = False
    assert calls == []


def test_concurrent_commands_do_not_queue_and_emergency_rejects_new_raw(system, monkeypatch):
    robot, calls, _ = system
    entered, release = threading.Event(), threading.Event()
    original = robot._session.request
    def delayed(*args, **kwargs):
        if kwargs['json'].get('command') == 'X':
            entered.set()
            assert release.wait(2)
        return original(*args, **kwargs)
    monkeypatch.setattr(robot._session, 'request', delayed)
    worker = threading.Thread(target=lambda: robot.raw_command({'command':'X'}))
    worker.start()
    try:
        assert entered.wait(1)
        with pytest.raises(ServiceError) as exc:
            robot.raw_command({'command':'Y'})
        assert exc.value.code == 'robot_io_busy'
        with pytest.raises(ServiceError):
            robot.enable()
        assert robot.status()['raw_console']['busy']
    finally:
        release.set()
        worker.join(2)
    assert len(calls) == 1
    assert not robot.status()['raw_console']['busy']


@pytest.mark.parametrize('http_status', [200, 503])
def test_bridge_failure_preserves_partial_response_and_unknown_state(system, monkeypatch, http_status):
    robot, _, _ = system
    bridge = {'sent':True, 'raw_response':'partial response', 'bytes_written':2,
              'error':{'code':'serial_read_failed','message':'read failed'}}
    monkeypatch.setattr(robot._session, 'request', lambda *a, **kw:
                        SimpleNamespace(status_code=http_status,json=lambda:bridge))
    app = create_app(SimpleNamespace(robot=robot))
    app.testing = True
    response = app.test_client().post('/api/robot/command', json={'command':'X'})
    assert response.status_code == 503
    assert response.json['bridge'] == bridge
    assert response.json['raw_response'] == 'partial response'
    assert response.json['execution_status'] == 'unknown'
    assert response.json['error']['code'] == 'raw_transport_unconfirmed'


def test_timeout_never_retries_or_claims_not_sent(system, monkeypatch):
    robot, _, _ = system
    attempts = []
    def timeout(*a, **kw):
        attempts.append(kw)
        raise requests.Timeout('read timeout after write')
    monkeypatch.setattr(robot._session, 'request', timeout)
    result = robot.raw_command({'command':'X'})
    assert len(attempts) == 1
    assert result['sent'] is None
    assert result['execution_status'] == 'unknown'
    assert result['error']['code'] == 'raw_transport_unavailable'
    assert not robot.status()['raw_console']['busy']


def test_http_origin_host_and_json_protections_apply_to_raw(system):
    robot, calls, _ = system
    client = create_app(SimpleNamespace(robot=robot)).test_client()
    url = 'http://127.0.0.1:8770/api/robot/command'
    assert client.post(url, json={'command':'X'}, headers={'Origin':'https://evil.example'}).status_code == 403
    assert client.post('http://evil.example/api/robot/command', json={'command':'X'}).status_code == 403
    assert client.post(url, data='X', content_type='text/plain').status_code == 415
    assert calls == []
    assert client.post(url, json={'command':'X', 'read_timeout_ms':0}).status_code == 200
    assert len(calls) == 1


def test_history_is_bounded(system):
    robot, _, _ = system
    for index in range(32):
        robot.raw_command({'command':f'X{index}'})
    history = robot.status()['raw_console']['history']
    assert len(history) == 30
    assert history[0]['command'] == 'X2'
