import math

import pytest

from motion.continuous_target import ContinuousTarget, HARDWARE_LIMITS
from perception_service.robot import COMMAND_LIMITS


def test_targets_advance_between_feedback_samples():
    target = ContinuousTarget()
    actual = [0., 0., 90., 0., 45., 0.]
    positions = [target.advance([10., 0., 0., 0., 0., 0.], actual, n*.025)
                 for n in range(8)]
    # One unchanged 200 ms hardware sample still produces eight distinct steps.
    assert [p[0] for p in positions] == pytest.approx([.25*(n+1) for n in range(8)])
    updated = positions[-1].copy()
    assert target.advance([10., 0., 0., 0., 0., 0.], updated, .2)[0] == pytest.approx(2.25)


def test_feedback_lead_is_disabled_by_default_in_both_directions():
    assert ContinuousTarget.MAX_LEAD_DEG is None
    target = ContinuousTarget()
    actual = [0., 0., 90., 0., 45., 0.]
    for n in range(40):
        result = target.advance([20., -20., 0., 0., 0., 0.], actual, n*.025)
    assert result[:2] == pytest.approx([20., -20.])
    assert not target.lead_limited
    assert not target.limited


def test_feedback_lead_saturates_at_ten_degrees_and_resumes_without_backlog(monkeypatch):
    monkeypatch.setattr(ContinuousTarget, 'MAX_LEAD_DEG', 10.)
    target = ContinuousTarget()
    actual = [0., 0., 90., 0., 45., 0.]
    for n in range(40):
        result = target.advance([20., 10., 0., 0., 0., 0.], actual, n*.025)
    assert result[:2] == pytest.approx([10., 5.])
    assert target.lead_limited
    assert not target.joint_limited
    actual[0] = 1.
    assert target.advance([20., 10., 0., 0., 0., 0.], actual, 1.)[:2] == pytest.approx([10.5, 5.25])


def test_joint_limit_saturates_without_windup_and_reversal_is_immediate():
    target = ContinuousTarget()
    actual = [165., 0., 90., 0., 45., 0.]
    for n in range(400):
        result = target.advance([20., 10., 0., 0., 0., 0.], actual, n*.025)
        assert result[0] <= 170.000001
    assert result[:2] == pytest.approx([170., 2.5])
    assert target.limited
    result = target.advance([-20., -10., 0., 0., 0., 0.], actual, 10.)
    assert result[:2] == pytest.approx([169.5, 2.25])


def test_negative_lead_limit_and_reversal_do_not_snap_to_feedback(monkeypatch):
    monkeypatch.setattr(ContinuousTarget, 'MAX_LEAD_DEG', 10.)
    target = ContinuousTarget()
    actual = [0., 0., 90., 0., 45., 0.]
    for n in range(40):
        result = target.advance([-20., 10., 0., 0., 0., 0.], actual, n*.025)
    assert result[:2] == pytest.approx([-10., 5.])
    assert target.advance([20., -10., 0., 0., 0., 0.], actual, 1.)[:2] == pytest.approx([-9.5, 4.75])


def test_feedback_change_outside_envelope_blocks_only_outward_progress(monkeypatch):
    monkeypatch.setattr(ContinuousTarget, 'MAX_LEAD_DEG', 10.)
    target = ContinuousTarget()
    actual = [0., 0., 90., 0., 45., 0.]
    target.advance([0.]*6, actual, 0.)
    actual[0] = 17.
    assert target.advance([-20., 0., 0., 0., 0., 0.], actual, .025)[0] == 0.
    assert target.lead_limited
    assert target.advance([20., 0., 0., 0., 0., 0.], actual, .05)[0] == pytest.approx(.5)


def test_release_reset_and_restart_have_no_leftover_target():
    target = ContinuousTarget()
    actual = [0., 0., 90., 0., 45., 0.]
    target.advance([20., 0., 0., 0., 0., 0.], actual, 0.)
    target.reset()
    result = target.advance([-20., 0., 0., 0., 0., 0.], actual, .1)
    assert result[0] == pytest.approx(-.5)


def test_long_gap_never_turns_into_catchup_motion():
    target = ContinuousTarget()
    actual = [0., 0., 90., 0., 45., 0.]
    target.advance([20., 0., 0., 0., 0., 0.], actual, 0.)
    assert target.advance([20., 0., 0., 0., 0., 0.], actual, 2.) is None
    assert target.advance([20., 0., 0., 0., 0., 0.], actual, 2.025)[0] == pytest.approx(.5)


def test_feedback_difference_alone_does_not_invalidate_target():
    target = ContinuousTarget()
    actual = [0., 0., 90., 0., 45., 0.]
    target.advance([0.]*6, actual, 0.)
    actual[0] = 17.
    assert target.advance([20., 0., 0., 0., 0., 0.], actual, .025)[0] == pytest.approx(.5)


def test_joint_limits_scale_all_axes_and_match_server_contract():
    assert HARDWARE_LIMITS == COMMAND_LIMITS
    target = ContinuousTarget()
    result = target.advance([20., 10., 0., 0., 0., 0.], [169.9, 0., 90., 0., 45., 0.], 0.)
    assert result[:2] == pytest.approx([170., .05])


@pytest.mark.parametrize('bad', [float('nan'), float('inf')])
def test_nonfinite_velocity_cannot_be_integrated(bad):
    with pytest.raises(ValueError):
        ContinuousTarget().advance([bad]+[0.]*5, [0.]*6, 0.)
