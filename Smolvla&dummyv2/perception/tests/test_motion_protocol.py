import importlib.util
from pathlib import Path
import pytest

path = Path(__file__).resolve().parents[3] / 'windows_dummy_bridge/motion_protocol.py'
spec = importlib.util.spec_from_file_location('tested_motion_protocol', path)
protocol = importlib.util.module_from_spec(spec)
spec.loader.exec_module(protocol)


def test_observed_long_motion_fits_fifo_without_losing_fields():
    values = [.00145206719156076,3.4949822967455e-05,89.9998840153899,
              .0729183885998403,45.0301376331569,.000985305735152206]
    command = protocol.format_motion(values,4)
    assert len(command.encode()) + 2 <= 64
    assert 'e' not in command
    decoded = [float(v) for v in command[1:].split(',')]
    assert len(decoded) == 7
    assert decoded[:6] == pytest.approx(values, abs=.0005)
    assert decoded[6] == 4
    assert len(protocol.format_motion([-170,-75,180,-180,-120,-720],20)) + 2 <= 64
    with pytest.raises(ValueError):
        protocol.format_motion([1e25]*6,4)


@pytest.mark.parametrize('raw', ['ok\r\n','15ok\r\n\r\n','15\r\nok\r\n','0ok\n'])
def test_valid_motion_ack(raw):
    assert protocol.motion_acknowledged({'sent':True,'raw_response':raw})


@pytest.mark.parametrize('raw', ['', '15', '255ok', '255\nok', 'error\nok', 'not ok',
                                 'ok hand enable', 'ok 0 0 90 0 0 0', '16ok'])
def test_uncertain_or_failed_motion_never_accepted(raw):
    assert not protocol.motion_acknowledged({'sent':True,'raw_response':raw})


def test_transport_failure_overrides_ack():
    assert not protocol.motion_acknowledged({'sent':False,'raw_response':'ok'})
    assert not protocol.motion_acknowledged({'sent':True,'raw_response':'ok','error':'write_failed'})


@pytest.mark.parametrize('raw', [b'ok 0 -75.2 180 0 0 0\r\n', b'okok 0 0 90 0 45 0\n',
                                  b'15ok\r\nok 0 0 90 0 45 0\r\n'])
def test_joint_reply_requires_complete_six_axis_line(raw):
    assert protocol.joint_reply_complete(raw)


@pytest.mark.parametrize('raw', [b'ok', b'ok\n', b'ok 0 0 90 0 45 0',
    b'ok 0 0 90 0 45\n', b'ok 0 0 90 0 45 0 1\n', b'ok 0 0 nan 0 45 0\n',
    b'error 0 0 90 0 45 0\n', b'15ok\n'])
def test_partial_or_other_reply_does_not_release_joint_read(raw):
    assert not protocol.joint_reply_complete(raw)


@pytest.mark.parametrize('payload,reply,early', [
    (b'#GETJPOS\n', [b'15ok\r\n', b'ok 0 0 90 ', b'0 45 0\r', b'\n'], True),
    (b'#GETJPOS\n', [b'ok 0 0 90 0 45 0'], False),
    (b'!START\n', [b'ok 0 0 90 0 45 0\n'], False),
])
def test_joint_read_releases_lock_early_only_for_exact_complete_query(monkeypatch, payload, reply, early):
    monkeypatch.syspath_prepend(str(path.parent))
    spec = importlib.util.spec_from_file_location('tested_joint_backend', path.parent/'motion_backend.py')
    backend = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(backend)
    clock = [0.]
    monkeypatch.setattr(backend.time, 'monotonic', lambda: clock[0])
    monkeypatch.setattr(backend.time, 'sleep', lambda seconds: clock.__setitem__(0, clock[0] + seconds))
    class Serial:
        written = False
        @property
        def in_waiting(self): return len(reply[0]) if self.written and reply else 0
        def write(self, payload):
            self.written = True
            return len(payload)
        def read(self, n):
            clock[0] += .001
            return reply.pop(0) if reply else b''
    serial = Serial()
    class Reader:
        identity = ('FAKE', 'NO-HARDWARE')
        def open(self): return serial
        def close(self): pass
    result = backend.exchange(Reader(), payload, 100)
    # Run the real HTTP bridge transaction wrapper without constructing its
    # device/camera workers; metadata must also survive its logging contract.
    import ast
    from types import SimpleNamespace
    tree = ast.parse((path.parent/'bridge.py').read_text(encoding='utf-8'))
    bridge_class = next(n for n in tree.body if isinstance(n, ast.ClassDef) and n.name == 'Bridge')
    transaction = next(n for n in bridge_class.body if isinstance(n, ast.FunctionDef) and n.name == 'transact')
    scope = {'time': backend.time, 'BridgeError': RuntimeError}
    exec(compile(ast.Module(body=[transaction], type_ignores=[]), '<bridge transact>', 'exec'), scope)
    logs = []
    bridge = SimpleNamespace(usb=SimpleNamespace(request=lambda *_: result),
                             log=lambda event, **fields: logs.append((event, fields)))
    assert scope['transact'](bridge, payload.decode().strip(), 100, '/command') is result
    assert 'elapsed_ms' in logs[-1][1] and 'serial_elapsed_ms' in logs[-1][1]
    assert bool(result.get('read_ended_on_joint_reply')) == early
    if early:
        assert result['serial_elapsed_ms'] < 10
    else:
        assert result['serial_elapsed_ms'] >= 100


@pytest.mark.parametrize('chunks,expected_reads', [([b'15',b'ok\r',b'\n'],3),([b'15\r\n',b'ok\r\n'],2)])
def test_serial_transaction_ends_only_after_full_ack(monkeypatch,chunks,expected_reads):
    monkeypatch.syspath_prepend(str(path.parent))
    spec = importlib.util.spec_from_file_location('tested_motion_backend',path.parent/'motion_backend.py')
    backend = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(backend)
    clock = [0.]
    monkeypatch.setattr(backend.time,'monotonic',lambda:clock[0])
    monkeypatch.setattr(backend.time, 'sleep', lambda seconds: clock.__setitem__(0, clock[0] + seconds))
    class Serial:
        written = False
        @property
        def in_waiting(self): return len(chunks[0]) if self.written and chunks else 0
        reads = 0
        def write(self,payload):
            self.written = True
            return len(payload)
        def read(self,n):
            self.reads += 1
            clock[0] += .01
            return chunks.pop(0) if chunks else b''
    serial = Serial()
    class Reader:
        identity = ('FAKE','NO-HARDWARE')
        def open(self): return serial
        def close(self): pass
    result = backend.exchange(Reader(),b'>0,0,90,0,45,0,4\n',500)
    assert result['read_ended_on_ack']
    assert protocol.motion_acknowledged(result)
    assert serial.reads == expected_reads
    assert clock[0] < .1
