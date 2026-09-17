"""Offline regressions for the captured Windows serial reconfiguration stall."""
import importlib.util
from pathlib import Path
from types import SimpleNamespace
import sys

import pytest

BRIDGE = Path(__file__).resolve().parents[3] / 'windows_dummy_bridge'


def load(monkeypatch, name):
    monkeypatch.syspath_prepend(str(BRIDGE))
    spec = importlib.util.spec_from_file_location('tested_poll_' + name, BRIDGE / (name + '.py'))
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


class Port:
    def __init__(self, clock, chunks=(), pending=b'', partial=False):
        self.clock, self.chunks = clock, list(chunks)
        self.pending, self.partial = pending, partial
        self.writes, self.reads = [], 0

    @property
    def timeout(self):
        return 0

    @timeout.setter
    def timeout(self, value):
        pytest.fail('Open-port timeout assignment reconfigures the Windows driver')

    @property
    def in_waiting(self):
        if self.pending:
            return len(self.pending)
        if self.writes and self.chunks and self.clock[0] >= self.chunks[0][0]:
            return len(self.chunks[0][1])
        return 0

    def write(self, payload):
        self.writes.append(payload)
        return len(payload) - int(self.partial)

    def read(self, count):
        assert count > 0
        self.reads += 1
        if self.pending:
            data, self.pending = self.pending[:count], self.pending[count:]
            return data
        _, data = self.chunks.pop(0)
        assert len(data) <= count
        return data


@pytest.fixture
def rig(monkeypatch):
    backend = load(monkeypatch, 'motion_backend')
    clock, sleeps = [0.], []
    monkeypatch.setattr(backend.time, 'monotonic', lambda: clock[0])
    def sleep(seconds):
        assert 0 < seconds <= .001
        sleeps.append(seconds)
        clock[0] += seconds
    monkeypatch.setattr(backend.time, 'sleep', sleep)
    def run(chunks=(), pending=b'', partial=False, timeout=100):
        port = Port(clock, chunks, pending, partial)
        closed = []
        reader = SimpleNamespace(identity=('FAKE', 'NO-HARDWARE'),
                                 open=lambda: port, close=lambda: closed.append(True))
        result = backend.exchange(reader, b'>0,0,90,0,45,0,4\n', timeout)
        assert len(port.writes) == 1  # No retry, including silence/partial write.
        return result, port, closed
    return run, clock, sleeps


def test_delayed_fragmented_ack_without_reconfiguration(rig):
    run, clock, sleeps = rig
    result, port, closed = run([(.003, b'15'), (.008, b'ok\r'), (.012, b'\n')])
    assert result['read_ended_on_ack']
    assert result['raw_response'] == '15ok\r\n'
    assert .012 <= clock[0] < .014
    assert sleeps and port.reads == 3 and not closed


@pytest.mark.parametrize('chunks,pending', [([], b''), ([], b'15ok\r\n'), ([(.005, b'15')], b'')])
def test_deadline_preserves_partial_and_stale_replies_without_replay(rig, chunks, pending):
    run, clock, sleeps = rig
    result, port, closed = run(chunks, pending)
    assert not result.get('read_ended_on_ack')
    assert result['pending_response'] == pending.decode()
    assert result['raw_response'] == ('15' if chunks else '')
    assert result['read_timed_out'] == (not chunks)
    assert .100 <= clock[0] < .102
    assert 99 <= len(sleeps) <= 102
    assert not closed


def test_partial_write_closes_without_read_or_retry(rig):
    run, clock, sleeps = rig
    result, port, closed = run(partial=True)
    assert result['error']['code'] == 'partial_write'
    assert result['write_status'] == 'partial'
    assert closed and not sleeps and port.reads == 0


def test_zero_read_wait_does_not_poll(rig):
    run, clock, sleeps = rig
    result, port, _ = run(timeout=0)
    assert result['read_wait_skipped'] and not result['read_timed_out']
    assert not sleeps and port.reads == 0


def test_device_opens_nonblocking_once_without_initialization_writes(monkeypatch):
    device = load(monkeypatch, 'device')
    opened, options = [], []
    class Serial:
        is_open = False
        def __init__(self, **kwargs):
            options.append(kwargs)
        def open(self):
            opened.append(self)
            self.is_open = True
        def close(self):
            self.is_open = False
    ports = SimpleNamespace(comports=lambda: [SimpleNamespace(
        vid=0x1209, pid=0x0D32, serial_number='347933853335', device='FAKE')])
    monkeypatch.setitem(sys.modules, 'serial', SimpleNamespace(Serial=Serial))
    monkeypatch.setitem(sys.modules, 'serial.tools', SimpleNamespace(list_ports=ports))
    reader = device.SerialReader()
    port = reader.open()
    assert reader.open() is port and len(opened) == 1
    assert options[0]['timeout'] == 0 and options[0]['write_timeout'] == .25
    assert port.dtr is False and port.rts is False
    reader.close()
    assert not port.is_open and reader.identity is None
