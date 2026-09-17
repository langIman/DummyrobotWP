import math
import xml.etree.ElementTree as ET

import pytest

from motion.joint_contract import HARDWARE_LIMITS, apply_model_limits, model_limits
from motion.continuous_target import ContinuousTarget
from perception_service.common import ServiceError, wall_time_ms
from perception_service.robot import COMMAND_LIMITS, RobotClient


EXPECTED = ((-170., 170.), (-75., 90.), (35., 180.),
            (-180., 180.), (-100., 100.), (-180., 180.))


def test_firmware_bounds_are_shared_and_model_offsets_are_applied():
    assert COMMAND_LIMITS == HARDWARE_LIMITS == EXPECTED
    # J3 hardware 35..180 becomes model -55..90, not 35..180 radians/degrees.
    assert model_limits()[2] == pytest.approx([math.radians(-55), math.radians(90)])
    assert model_limits()[4] == pytest.approx([math.radians(-100), math.radians(100)])
    assert model_limits()[5] == pytest.approx([-math.pi, math.pi])
    source = '<robot name="test">' + ''.join(
        f'<joint name="Joint{i}" type="revolute"><limit lower="-99" upper="99" velocity="2" effort="3"/></joint>'
        for i in range(1, 7)) + '</robot>'
    result = ET.fromstring(apply_model_limits(source))
    for joint, (low, high) in zip(result.findall('joint'), model_limits()):
        limit = joint.find('limit')
        assert [float(limit.get('lower')), float(limit.get('upper'))] == pytest.approx([low, high])
        assert limit.get('velocity') == '2' and limit.get('effort') == '3'
    with pytest.raises(ValueError):
        apply_model_limits('<robot/>')


@pytest.mark.parametrize('axis', range(6))
@pytest.mark.parametrize('sign', [-1, 1])
def test_each_firmware_boundary_stops_target_and_allows_inward_recovery(axis, sign):
    target = ContinuousTarget()
    actual = [0., 0., 90., 0., 45., 0.]
    low, high = EXPECTED[axis]
    actual[axis] = low + .1 if sign < 0 else high - .1
    velocity = [0.] * 6
    velocity[axis] = sign * 20.
    for n in range(20):
        result = target.advance(velocity, actual, n * .025)
        assert all(a <= q <= b for q, (a, b) in zip(result, EXPECTED))
    assert result[axis] == (low if sign < 0 else high)
    assert target.joint_limit_axes == [{'joint': f'J{axis+1}', 'bound': 'lower' if sign < 0 else 'upper', 'limit_deg': low if sign < 0 else high}]
    velocity[axis] *= -1
    assert target.advance(velocity, actual, .5)[axis] == pytest.approx(result[axis] - sign * .5)


@pytest.mark.parametrize('axis', range(6))
@pytest.mark.parametrize('sign', [-1, 1])
def test_serial_sender_rejects_outside_firmware_even_with_previous_tolerance(monkeypatch, axis, sign):
    client = RobotClient(start_sampler=False)
    sent = []
    monkeypatch.setattr(client, '_request', lambda *args, **kwargs: sent.append((args, kwargs)))
    try:
        client._enabled_latch = 'confirmed'
        client._last_state = {'positions': [0., 0., 90., 0., 45., 0.], 'sampled_at_ms': wall_time_ms()}
        target = client._last_state['positions'].copy()
        target[axis] = EXPECTED[axis][0 if sign < 0 else 1] + sign * .001
        with pytest.raises(ServiceError) as exc:
            client.send_stream_target(target)
        assert exc.value.code == 'stream_target_outside_limit'
        assert not sent
    finally:
        client.close()


def test_explicit_hold_clamps_small_feedback_noise_but_does_not_hide_large_invalid_pose(monkeypatch):
    client = RobotClient(start_sampler=False)
    sent = []
    positions = [0., -75.2, 180.6, 0., 0., 0.]
    monkeypatch.setattr(client, 'read_state', lambda **kwargs: {'positions': positions})
    monkeypatch.setattr(client, '_request', lambda method, path, body, **kwargs: sent.append(body) or {'accepted': True})
    try:
        client._enabled_latch = 'confirmed'
        client._last_state = {'positions': positions, 'sampled_at_ms': wall_time_ms()}
        client.hold_stream()
        assert sent[-1]['positions'] == [0., -75., 180., 0., 0., 0.]
        positions[2] = 20.
        with pytest.raises(ServiceError):
            client.hold_stream()
        assert len(sent) == 1
    finally:
        client.close()
