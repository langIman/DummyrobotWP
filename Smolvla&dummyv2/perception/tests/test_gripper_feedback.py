import importlib.util
from pathlib import Path
import struct
import subprocess
import threading
from types import SimpleNamespace

import pytest

from app import create_app
from perception_service.common import monotonic_ns, wall_time_ms
from perception_service.robot import RobotClient

spec = importlib.util.spec_from_file_location('probe_feedback', Path(__file__).resolve().parents[3] / 'windows_dummy_bridge/gripper_feedback.py')
probe = importlib.util.module_from_spec(spec)
spec.loader.exec_module(probe)


class FakeProbe:
    pointer = 0x20007138

    def __init__(self):
        self.calls = []
        self.memory = {}
        for address, data in probe.SIGNATURES.items():
            self.put(address, data)
        self.put(probe.POINTER, struct.pack('<I', self.pointer))
        obj = bytearray(0x50)
        obj[4], obj[0x19] = 7, 8
        struct.pack_into('<f', obj, 8, 22.099218368530273)
        self.put(self.pointer, obj)

    def put(self, address, data):
        self.memory.update({address + i: value for i, value in enumerate(data)})

    def run(self, args, **kwargs):
        self.calls.append(args)
        assert args[1:5] == ['-c', 'SN=' + probe.PROBE_SN, 'SWD', 'HOTPLUG']
        output = f'ST-LINK SN: {probe.PROBE_SN}\nConnection mode: HotPlug\nDevice ID: 0x413\n'
        for i in range(5, len(args), 3):
            assert args[i] == '-r32'
            address, count = int(args[i+1], 16), int(args[i+2], 16)
            for row in range(0, count, 16):
                data = bytes(self.memory[address+j] for j in range(row, min(count,row+16)))
                words = '  '.join(f'{word[0]:08X}' for word in struct.iter_unpack('<I', data))
                output += f'0x{address+row:08X} : {words} \r\n'
        return SimpleNamespace(returncode=0, stdout=output.encode())


def test_probe_reads_only_known_firmware_validated_dynamic_object(tmp_path):
    fake = FakeProbe()
    cli = tmp_path / 'cli.exe'
    cli.touch()
    reader = probe.GripperFeedback(cli=cli, run=fake.run)
    value = reader.read()
    assert value['status'] == 'available'
    assert value['angle_deg'] == pytest.approx(22.099218)
    assert value['device_sample_at_ms'] is None
    assert len(fake.calls) == 2
    assert reader.read() == value
    assert len(fake.calls) == 2


@pytest.mark.parametrize('failure', ['firmware','pointer','node','nan','disconnect','timeout'])
def test_probe_failures_do_not_publish_old_or_invalid_angles(tmp_path, failure):
    fake = FakeProbe()
    cli = tmp_path / 'cli.exe'
    cli.touch()
    reader = probe.GripperFeedback(cli=cli, run=fake.run)
    assert reader.read()['status'] == 'available'
    reader._last_attempt = float('-inf')
    if failure == 'firmware': fake.put(next(iter(probe.SIGNATURES)), b'\x00')
    if failure == 'pointer': fake.put(probe.POINTER, struct.pack('<I',0x08000000))
    if failure == 'node': fake.put(fake.pointer+4, b'\x06')
    if failure == 'nan': fake.put(fake.pointer+8, struct.pack('<f',float('nan')))
    if failure == 'disconnect': reader._run = lambda *a, **kw: SimpleNamespace(returncode=1,stdout=b'No STLINK')
    if failure == 'timeout':
        def timeout(*args, **kwargs):
            raise subprocess.TimeoutExpired(args[0],1.5)
        reader._run = timeout
    value = reader.read()
    assert value['status'] == 'unavailable'
    assert value['angle_deg'] is None
    assert value['sampled_at_ms'] is None


@pytest.fixture
def robot_system(monkeypatch):
    samples, calls = [], []
    robot = RobotClient(start_sampler=False, state_callback=samples.append)
    def request(method, path, body=None, timeout=4):
        calls.append((method,path,body))
        if path == '/gripper/state':
            return {'status':'available','angle_deg':22.099218,'sampled_at_ms':wall_time_ms(),
                    'monotonic_ns':monotonic_ns(),'source':'stlink_mainboard_cache'}
        assert body['command'] == '#GETJPOS'
        return {'raw_response':'ok -1.37 -75.24 180.66 0.04 -0.27 0.17\r\n'}
    monkeypatch.setattr(robot, '_request', request)
    yield robot, samples, calls
    robot.close()


def test_read_button_updates_both_and_recording_keeps_separate_time(robot_system):
    robot, samples, calls = robot_system
    app = create_app(SimpleNamespace(robot=robot))
    app.testing = True
    response = app.test_client().get('/api/robot/state')
    assert response.status_code == 200
    assert len(response.json['state']['positions']) == 6
    assert response.json['state']['gripper']['angle_deg'] == pytest.approx(22.099218)
    assert response.json['state']['gripper']['logical_angle_deg'] == pytest.approx(115.0)
    assert response.json['state']['gripper']['reference_source'] == 'startup_assumed_closed'
    assert response.json['state']['gripper']['stale'] is False
    assert samples[-1]['gripper']['source'] == 'stlink_mainboard_cache'
    assert samples[-1]['gripper']['monotonic_ns'] >= samples[-1]['monotonic_ns']
    assert calls == [('POST','/command',{'command':'#GETJPOS','read_timeout_ms':100}),
                     ('GET','/gripper/state',None)]


def test_software_reference_tracks_raw_delta_and_can_be_rebased(robot_system):
    robot, _, _ = robot_system
    first = robot.read_gripper_state()
    assert first['logical_angle_deg'] == pytest.approx(115.0)
    original = robot._request
    def moved(method, path, body=None, timeout=4):
        if path == '/gripper/state':
            return {'status':'available','angle_deg':-50.0,'sampled_at_ms':wall_time_ms(),
                    'monotonic_ns':monotonic_ns(),'source':'stlink_mainboard_cache'}
        return original(method, path, body, timeout)
    robot._request = moved
    moved_state = robot.read_gripper_state()
    assert moved_state['raw_angle_deg'] == pytest.approx(-50.0)
    assert moved_state['logical_angle_deg'] == pytest.approx(42.900782)
    assert robot.capture_gripper_reference()['logical_angle_deg'] == pytest.approx(115.0)
    assert robot.gripper_feedback()['reference_source'] == 'operator_confirmed_closed'


def test_reference_endpoint_is_read_only(robot_system):
    robot, _, calls = robot_system
    app = create_app(SimpleNamespace(robot=robot))
    app.testing = True
    response = app.test_client().post('/api/robot/gripper/reference', json={})
    assert response.status_code == 200
    assert response.json['logical_angle_deg'] == pytest.approx(115.0)
    assert calls == [('GET','/gripper/state',None)]
    assert app.test_client().post('/api/robot/gripper/reference', json={'angle': 115}).status_code == 422


def test_missing_probe_does_not_block_six_axes(robot_system, monkeypatch):
    robot, _, _ = robot_system
    original = robot._request
    def request(method,path,body=None,timeout=4):
        if path == '/gripper/state': return {'status':'unavailable','angle_deg':None,'message':'探针断开'}
        return original(method,path,body,timeout)
    monkeypatch.setattr(robot,'_request',request)
    state = robot.read_state(include_gripper=True)
    assert len(state['positions']) == 6
    assert state['gripper']['angle_deg'] is None


def test_slow_probe_does_not_hold_serial_lock(robot_system, monkeypatch):
    robot, _, calls = robot_system
    entered, release = threading.Event(), threading.Event()
    original = robot._request
    def request(method,path,body=None,timeout=4):
        if path == '/gripper/state':
            entered.set()
            assert release.wait(2)
        return original(method,path,body,timeout)
    monkeypatch.setattr(robot,'_request',request)
    thread = threading.Thread(target=robot.read_gripper_state)
    thread.start()
    try:
        assert entered.wait(1)
        assert robot._io_lock.acquire(blocking=False)
        robot._io_lock.release()
        assert robot.read_state(automatic=True)['positions'] is not None
        assert calls[0][1] == '/command'
    finally:
        release.set()
        thread.join(2)


def test_old_cache_is_marked_stale(robot_system):
    robot, _, _ = robot_system
    robot.read_gripper_state()
    robot._gripper_feedback['monotonic_ns'] -= 3_000_000_000
    assert robot.status()['state']['gripper']['stale']
