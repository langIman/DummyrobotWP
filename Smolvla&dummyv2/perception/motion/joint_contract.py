"""Joint bounds read from the installed mainboard snapshot (2026-09-17)."""
import math
import xml.etree.ElementTree as ET


HARDWARE_LIMITS = ((-170., 170.), (-75., 90.), (35., 180.),
                   (-180., 180.), (-100., 100.), (-180., 180.))
MODEL_DIRECTIONS = (1., 1., 1., 1., -1., -1.)
MODEL_OFFSETS_RAD = (0., 0., math.pi / 2, 0., 0., 0.)


def model_limits():
    return tuple(tuple(sorted(math.radians(q) * direction - offset for q in bounds))
                 for bounds, direction, offset in zip(
                     HARDWARE_LIMITS, MODEL_DIRECTIONS, MODEL_OFFSETS_RAD))


def apply_model_limits(description):
    """Overlay only position bounds on expanded URDF, never edit the source repo."""
    robot = ET.fromstring(description)
    for axis, (low, high) in enumerate(model_limits(), 1):
        joint = robot.find(f"joint[@name='Joint{axis}']")
        if joint is None or joint.get('type') != 'revolute' or joint.find('limit') is None:
            raise ValueError(f'missing_revolute_joint_limit_J{axis}')
        joint.find('limit').set('lower', repr(low))
        joint.find('limit').set('upper', repr(high))
    return ET.tostring(robot, encoding='unicode')


def near_limit_axes(actual, margin_deg):
    result = []
    for axis, (value, (low, high)) in enumerate(zip(actual, HARDWARE_LIMITS), 1):
        if value <= low + margin_deg:
            result.append({'joint': f'J{axis}', 'bound': 'lower', 'limit_deg': low})
        elif value >= high - margin_deg:
            result.append({'joint': f'J{axis}', 'bound': 'upper', 'limit_deg': high})
    return result
