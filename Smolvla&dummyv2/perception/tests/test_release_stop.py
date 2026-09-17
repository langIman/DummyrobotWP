import math

import pytest

from perception_service.release_stop import release_destination
from perception_service.robot import COMMAND_LIMITS, RobotClient


def sample(t, q, **extras):
    return {'positions': [q, 0., 90., 0., 45., 0.],
            'monotonic_ns': int((10 + t) * 1e9), 'sampled_at_ms': int(t * 1000),
            'verification_retries': 0, **extras}


def plan(points, age=0):
    return release_destination(points[-1], points, COMMAND_LIMITS,
                               points[-1]['monotonic_ns'] + int(age * 1e9))


@pytest.mark.parametrize('direction', [1, -1])
def test_stop_target_is_ahead_in_measured_direction(direction):
    points = [sample(t, direction * (100 + 20 * t)) for t in (0., .2, .4)]
    result = plan(points)
    assert result['applied']
    assert result['velocities_deg_s'][0] == pytest.approx(direction * 20)
    assert result['positions'][0] == pytest.approx(direction * 110)
    assert result['offsets_deg'][0] == pytest.approx(direction * 2)
    assert points[-1]['positions'][0] == direction * 108  # No feedback mutation.


def test_noise_reversal_retries_and_duplicate_samples_do_not_predict():
    assert not plan([sample(0, 0), sample(.2, 2)])['applied']
    assert not plan([sample(0, 0), sample(0, 0), sample(0, 0)])['applied']
    assert not plan([sample(0, 0), sample(.2, 2), sample(.4, 0)])['applied']
    assert not plan([sample(0, 0), sample(.2, .01), sample(.4, .02)])['applied']
    assert not plan([sample(0, 0), sample(.2, 2), sample(.4, 4, verification_retries=1)])['applied']
    assert not plan([sample(0, 0), sample(.2, 50), sample(.4, 100)])['applied']


def test_old_feedback_or_gap_never_extrapolates():
    points = [sample(0, 0), sample(.2, 2), sample(.4, 4)]
    assert not plan(points, age=.151)['applied']
    assert not plan(points, age=-.001)['applied']
    assert not plan([sample(0, 0), sample(.8, 4), sample(1, 8)])['applied']


def test_prediction_caps_only_release_extension_and_respects_joint_limits():
    result = plan([sample(0, 0), sample(.2, 12), sample(.4, 24)], age=.1)
    assert result['positions'][0] == pytest.approx(29)
    near_bound = plan([sample(0, 165), sample(.2, 167), sample(.4, 169.9)])
    assert near_bound['positions'][0] == 170
    already_outside = plan([sample(0, 167), sample(.2, 169), sample(.4, 170.5)])
    assert already_outside['positions'][0] == 170.5  # Do not extend beyond an existing violation.


def test_simulated_decelerating_driver_returns_less_with_forward_stop_target():
    # Simplified position driver with finite acceleration, not a hardware claim.
    # Same initial position/velocity and speed ceiling in both runs.
    def run(target):
        q, v, peak = 108., 20., 108.
        for _ in range(1600):
            distance = target - q
            desired = math.copysign(min(24., math.sqrt(200 * abs(distance))), distance)
            next_v = v + max(-.2, min(.2, desired - v))
            q += (v + next_v) * .001
            v = next_v
            peak = max(peak, q)
        return peak, q
    target = plan([sample(0, 100), sample(.2, 104), sample(.4, 108)])['positions'][0]
    old_peak, old_final = run(108.)
    new_peak, new_final = run(target)
    assert old_peak - old_final > 1.5
    assert new_peak - new_final < .15
    assert abs(new_final - target) < .03


def test_robot_replaces_old_goal_once_with_same_speed_and_logs_prediction(monkeypatch):
    client = RobotClient(start_sampler=False)
    events, calls = [], []
    client._event_callback = events.append
    client._enabled_latch = 'confirmed'
    clock = [10_000_000_000]
    replies = iter(['ok 100 0 90 0 45 0', 'ok 104 0 90 0 45 0', 'ok 108 0 90 0 45 0'])
    monkeypatch.setattr('perception_service.robot.monotonic_ns', lambda: clock[0])
    def request(method, path, body=None, **kwargs):
        calls.append((path, body))
        return {'raw_response': next(replies)} if path == '/command' else {'accepted': True}
    monkeypatch.setattr(client, '_request', request)
    try:
        client.read_state(force=True)
        clock[0] += 200_000_000
        client.read_state(force=True)
        clock[0] += 200_000_000
        result = client.hold_stream(predictive=True)
        motions = [body for path, body in calls if path == '/motion']
        assert len(motions) == 1
        assert motions[0]['positions'][0] == pytest.approx(110)
        assert motions[0]['speed'] == 50
        assert result['release_stop']['applied'] is True
        assert events[-1]['type'] == 'robot_release_stop'
        assert client.status()['state']['positions'][0] == 108
    finally:
        client.close()
