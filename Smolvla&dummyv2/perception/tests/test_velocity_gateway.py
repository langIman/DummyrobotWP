"""Exercise real gateway callbacks without ROS, USB, or wall-clock waits."""
import ast
import __future__
import math
from pathlib import Path
import threading
from types import MethodType, SimpleNamespace

import pytest

from motion.continuous_target import ContinuousTarget
from motion.joint_contract import MODEL_DIRECTIONS, MODEL_OFFSETS_RAD, near_limit_axes


@pytest.fixture
def gateway():
    path = Path(__file__).parents[1] / 'motion/moveit_servo_gateway.py'
    tree = ast.parse(path.read_text(encoding='utf-8'))
    cls = next(n for n in tree.body if isinstance(n, ast.ClassDef) and n.name == 'GatewayNode')
    names = {'feedback_fresh', 'set_feedback', 'set_command', 'on_velocities',
             'on_servo_status', 'latest_trajectory'}
    definitions = [n for n in tree.body if (
        isinstance(n, ast.FunctionDef) and n.name in {'hardware_to_model', 'model_to_hardware'}
    ) or (isinstance(n, ast.Assign) and any(
        isinstance(t, ast.Name) and t.id in {'DIRECTION', 'OFFSET_RAD'} for t in n.targets))]
    definitions += [n for n in cls.body if isinstance(n, ast.FunctionDef) and n.name in names]
    clock = [10.]
    scope = {'math': math, 'time': SimpleNamespace(monotonic=lambda: clock[0]),
             'MODEL_DIRECTIONS': MODEL_DIRECTIONS, 'MODEL_OFFSETS_RAD': MODEL_OFFSETS_RAD,
             'near_limit_axes': near_limit_axes}
    exec(compile(ast.Module(body=definitions, type_ignores=[]), str(path), 'exec',
                 flags=__future__.annotations.compiler_flag), scope)
    node = SimpleNamespace(lock=threading.RLock(), armed=True, feedback=None, feedback_at=0.,
                           command={'linear_x': 0.}, command_at=0., trajectory=None,
                           trajectory_at=0., trajectory_sequence=-1,
                           continuous_target=ContinuousTarget(),
                           servo_status={'code': 0}, servo_status_at=0.,
                           guard_warning=None, guard_warning_at=0.)
    for name in names:
        setattr(node, name, MethodType(scope[name], node))
    node.set_feedback([0., 0., 90., 0., 45., 0.])
    node.set_command({'linear_x': .1})
    return node, clock


def test_velocity_conversion_has_sign_but_no_position_offset(gateway):
    node, clock = gateway
    measured = node.feedback.copy()
    for n in range(8):
        clock[0] = 10.+n*.025
        node.on_velocities(SimpleNamespace(data=[math.radians(10.)]*6))
    assert node.trajectory == pytest.approx([2., 2., 92., 2., 43., -2.])
    assert node.feedback == measured  # Never publish desired values as measured.


@pytest.mark.parametrize('reason', ['release', 'input_timeout', 'feedback_timeout', 'disarmed', 'collision'])
def test_stop_conditions_discard_pending_target(gateway, reason):
    node, clock = gateway
    velocity = SimpleNamespace(data=[.1]*6)
    node.on_velocities(velocity)
    assert node.trajectory is not None
    clock[0] += .025
    if reason == 'release':
        node.set_command({'linear_x': 0.})
    elif reason == 'input_timeout':
        node.command_at -= 1.
    elif reason == 'feedback_timeout':
        node.feedback_at -= 1.
    elif reason == 'disarmed':
        node.armed = False
    else:
        node.on_servo_status(SimpleNamespace(code=5, message='collision'))
    node.on_velocities(velocity)
    assert node.trajectory is None
    assert node.continuous_target.target is None


def test_first_frame_wait_is_bounded_and_not_refreshed_by_heartbeat(gateway):
    node, clock = gateway
    clock[0] += .1
    node.set_command({'linear_x': .1})
    assert node.latest_trajectory()['missing_age_ms'] == pytest.approx(100.)
    clock[0] += .151
    node.set_command({'linear_x': .1})
    assert node.latest_trajectory()['missing_age_ms'] == pytest.approx(251.)
    node.on_velocities(SimpleNamespace(data=[.1]*6))
    assert node.latest_trajectory()['positions'] is not None
    assert node.latest_trajectory()['missing_age_ms'] is None
    node.set_command({'linear_x': 0.})
    node.set_command({'linear_x': -.1})
    assert node.latest_trajectory()['positions'] is None
    assert node.latest_trajectory()['missing_age_ms'] == 0.


def test_invalid_velocity_discards_previous_target(gateway):
    node, _ = gateway
    node.on_velocities(SimpleNamespace(data=[.1]*6))
    node.on_velocities(SimpleNamespace(data=[float('nan')]*6))
    assert node.trajectory is None
    assert node.continuous_target.target is None
