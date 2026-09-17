import json
import math

import numpy as np
import pytest

from motion.tool_frame import ToolFrame, TOOL_IN_LINK6, rotation


def command(**values):
    return {**{f'{kind}_{axis}': 0. for kind in ('linear', 'angular') for axis in 'xyz'}, **values}


@pytest.fixture
def frame(tmp_path):
    config = tmp_path / 'tool.json'
    # Nonzero hypothetical TCP for compensation tests, not a real measurement.
    config.write_text(json.dumps({'tcp_offset_in_link6_m': [.003, -.10, .004],
                                 'tcp_calibrated': True}))
    return ToolFrame(config_path=config)


def test_forward_is_along_gripper_shaft_and_left_is_across_it(frame):
    flange_rotation = frame.flange_pose([0.] * 6)[:3, :3]
    moved = frame.to_base(command(linear_x=.1), [0.] * 6)
    assert [moved[f'linear_{a}'] for a in 'xyz'] == pytest.approx(flange_rotation @ [0., -.1, 0.])
    left = frame.to_base(command(linear_y=.1), [0.] * 6)
    assert [left[f'linear_{a}'] for a in 'xyz'] == pytest.approx(flange_rotation @ [.1, 0., 0.])
    assert np.linalg.det(TOOL_IN_LINK6) == pytest.approx(1)


@pytest.mark.parametrize('q', [[0., 0., 0., 0., -math.pi/4, 0.],
                              [math.pi/2, 0., 0., 0., -math.pi/4, math.pi/2]])
@pytest.mark.parametrize('velocity', [[.1, 0., 0.], [-.1, 0., 0.], [0., .1, 0.], [0., -.1, 0.], [0., 0., .1], [0., 0., -.1], [.1, .1, -.1]])
def test_translation_follows_current_yaw_tilt_and_roll(frame, q, velocity):
    moved = frame.to_base(command(**dict(zip(('linear_x', 'linear_y', 'linear_z'), velocity))), q)
    _, orient = frame.pose(q)
    assert orient.T @ [moved[f'linear_{a}'] for a in 'xyz'] == pytest.approx(velocity)


def test_combined_input_moves_tcp_in_tool_frame_while_turning(frame):
    q = [.2, -.5, .4, .7, -.6, .8]
    flange = frame.flange_pose(q)
    moved = frame.to_base(command(linear_x=.1, linear_z=-.1, angular_y=-.24), q)
    v = np.array([moved[f'linear_{a}'] for a in 'xyz'])
    omega = np.array([moved[f'angular_{a}'] for a in 'xyz'])
    dt = 1e-5
    after_rotation = rotation(omega, np.linalg.norm(omega)*dt) @ flange[:3, :3]
    displacement = v*dt + (after_rotation-flange[:3, :3]) @ frame.offset
    _, orient = frame.pose(q)
    assert orient.T @ (displacement/dt) == pytest.approx([.1, 0., -.1], abs=1e-7)


@pytest.mark.parametrize('q', [[0.]*6, [.2, -.5, .4, .7, -.6, .8], [1.5, -.3, .5, -1.2, .7, 2.4]])
@pytest.mark.parametrize('turn', [command(angular_y=-.24), command(angular_y=.24),
                                  command(angular_z=-.24), command(angular_z=.24),
                                  command(angular_y=-.12, angular_z=.12)])
def test_turn_keeps_tcp_stationary_and_uses_current_tool_axes(frame, q, turn):
    flange = frame.flange_pose(q)
    _, orient = frame.pose(q)
    result = frame.to_base(turn, q)
    velocity = np.array([result[f'linear_{a}'] for a in 'xyz'])
    omega = np.array([result[f'angular_{a}'] for a in 'xyz'])
    dt = 1e-5
    after_rotation = rotation(omega, np.linalg.norm(omega)*dt) @ flange[:3, :3]
    before_tcp = flange[:3, 3] + flange[:3, :3] @ frame.offset
    after_tcp = flange[:3, 3] + velocity*dt + after_rotation @ frame.offset
    assert np.linalg.norm(after_tcp-before_tcp) < 1e-10
    assert orient.T @ omega == pytest.approx([turn[f'angular_{a}'] for a in 'xyz'])
    # Tool forward always tips toward tool up on 8 and tool right on 6.
    forward_velocity = orient.T @ np.cross(omega, orient[:, 0])
    assert forward_velocity == pytest.approx([0., turn['angular_z'], -turn['angular_y']])


def test_status_declares_both_inputs_in_tool(frame):
    assert frame.status()['translation_frame'] == 'gripper_tool'
    assert frame.status()['rotation_frame'] == 'gripper_tool'
    assert frame.status()['command_frame'] == 'gripper_tool'


def test_release_is_zero_regardless_of_feedback_and_tcp(frame):
    assert frame.to_base(command(), [.1, .2, -.3, .4, .5, -.6]) == command()


def test_bad_tcp_configuration_fails_closed(tmp_path):
    config = tmp_path / 'bad.json'
    config.write_text(json.dumps({'tcp_offset_in_link6_m': [0., -4., 0.],
                                 'tcp_calibrated': True}))
    with pytest.raises(ValueError):
        ToolFrame(config_path=config)
