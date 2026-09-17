"""Tool-frame translation and rotation, using MoveIt's copied URDF.

No robot I/O. Velocities are expressed at the tool point, then shifted to
link6's origin, the point controlled by the existing Servo chain.
"""
from __future__ import annotations

import json
import math
from pathlib import Path
import xml.etree.ElementTree as ET

import numpy as np

ROOT = Path(__file__).resolve().parent
MODEL = ROOT / '.ros/shadow/dummy-ros2_description/urdf/dummy-ros2.xacro'
COMMAND_FRAME = 'gripper_tool'
# Columns are forward, left, up expressed in link6. J6's shaft/approach axis
# is -Y (the CAD serial chain extends toward negative Y), not base +X.
TOOL_IN_LINK6 = np.array([[0., 1., 0.], [-1., 0., 0.], [0., 0., 1.]])


def rotation(axis, angle):
    axis = np.asarray(axis, dtype=float)
    axis = axis / np.linalg.norm(axis)
    x, y, z = axis
    skew = np.array([[0., -z, y], [z, 0., -x], [-y, x, 0.]])
    return np.eye(3) + math.sin(angle) * skew + (1 - math.cos(angle)) * (skew @ skew)


class ToolFrame:
    def __init__(self, model_path=MODEL, config_path=ROOT / 'tool_frame.json'):
        config = json.loads(Path(config_path).read_text(encoding='utf-8'))
        offset = config['tcp_offset_in_link6_m']
        if (not isinstance(offset, list) or len(offset) != 3
                or not all(type(v) in (int, float) and math.isfinite(v) for v in offset)
                or np.linalg.norm(offset) > .3):
            raise ValueError('TCP offset must be three finite metres, length <= 0.3 m')
        if type(config['tcp_calibrated']) is not bool:
            raise ValueError('tcp_calibrated must be boolean')
        self.offset = np.asarray(offset, dtype=float)
        self.calibrated = config['tcp_calibrated']
        robot = ET.parse(model_path).getroot()
        self.chain = []
        parent = 'base_link'
        for index in range(1, 7):
            joint = robot.find(f"joint[@name='Joint{index}']")
            if joint is None or joint.find('parent').get('link') != parent:
                raise ValueError('Unexpected six-axis URDF chain')
            parent = joint.find('child').get('link')
            origin = joint.find('origin')
            xyz = np.fromstring(origin.get('xyz'), sep=' ')
            roll, pitch, yaw = np.fromstring(origin.get('rpy', '0 0 0'), sep=' ')
            fixed = np.eye(4)
            fixed[:3, 3] = xyz
            fixed[:3, :3] = (rotation([0, 0, 1], yaw) @ rotation([0, 1, 0], pitch)
                            @ rotation([1, 0, 0], roll))
            axis = np.fromstring(joint.find('axis').get('xyz'), sep=' ')
            self.chain.append((fixed, axis))
        if parent != 'link6_1_1':
            raise ValueError('Unexpected Servo tip')

    def flange_pose(self, model_radians):
        if len(model_radians) != 6 or not all(math.isfinite(q) for q in model_radians):
            raise ValueError('Six finite model joint angles required')
        transform = np.eye(4)
        for (fixed, axis), angle in zip(self.chain, model_radians):
            moving = np.eye(4)
            moving[:3, :3] = rotation(axis, angle)
            transform = transform @ fixed @ moving
        return transform

    def pose(self, model_radians):
        flange = self.flange_pose(model_radians)
        return (flange[:3, 3] + flange[:3, :3] @ self.offset,
                flange[:3, :3] @ TOOL_IN_LINK6)

    def to_base(self, command, model_radians):
        flange = self.flange_pose(model_radians)
        tool_rotation = flange[:3, :3] @ TOOL_IN_LINK6
        # Re-evaluate the axes from current feedback on every gateway tick.
        linear = tool_rotation @ np.array([command[f'linear_{a}'] for a in 'xyz'], dtype=float)
        angular = tool_rotation @ np.array([command[f'angular_{a}'] for a in 'xyz'], dtype=float)
        # v_tcp = v_flange + omega x (p_tcp - p_flange).
        # Compensate this lever arm so rotation-only input holds the TCP.
        linear -= np.cross(angular, flange[:3, :3] @ self.offset)
        return {**{f'linear_{a}': float(v) for a, v in zip('xyz', linear)},
                **{f'angular_{a}': float(v) for a, v in zip('xyz', angular)}}

    def status(self):
        return {'command_frame': COMMAND_FRAME,
                'translation_frame': 'gripper_tool', 'rotation_frame': 'gripper_tool',
                'forward_axis_in_link6': [0, -1, 0],
                'left_axis_in_link6': [1, 0, 0], 'up_axis_in_link6': [0, 0, 1],
                'tcp_offset_in_link6_m': self.offset.tolist(),
                'tcp_configured': bool(np.linalg.norm(self.offset) > 0 or self.calibrated),
                'tcp_reference_approximate': not self.calibrated,
                'tcp_calibrated': self.calibrated}
