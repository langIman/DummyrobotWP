"""Read-only kinematic audit. Never imports a robot driver or sends commands."""
import ast
import hashlib
import json
import math
from pathlib import Path
import xml.etree.ElementTree as ET
import numpy as np

ROOT = Path(__file__).resolve().parent
WORKSPACE = ROOT.parents[2]
ORIGINAL = WORKSPACE / 'dummy_moveit_ws'
MODEL = ROOT / '.ros/shadow/dummy-ros2_description/urdf/dummy-ros2.xacro'

def rotation(axis, angle):
    axis = np.asarray(axis, dtype=float)
    axis /= np.linalg.norm(axis)
    x,y,z = axis
    skew = np.array([[0,-z,y],[z,0,-x],[-y,x,0]])
    return np.eye(3) + math.sin(angle)*skew + (1-math.cos(angle))*(skew@skew)

def audit(pose):
    model = np.radians(pose)*[1,1,1,1,-1,-1] - [0,0,math.pi/2,0,0,0]
    robot = ET.parse(MODEL).getroot()
    transform = np.eye(4)
    axes, origins = [], []
    for index, angle in enumerate(model, 1):
        joint = robot.find(f"joint[@name='Joint{index}']")
        origin = joint.find('origin')
        offset = np.eye(4)
        offset[:3,3] = np.fromstring(origin.get('xyz'), sep=' ')
        roll,pitch,yaw = np.fromstring(origin.get('rpy', '0 0 0'), sep=' ')
        offset[:3,:3] = rotation([0,0,1],yaw) @ rotation([0,1,0],pitch) @ rotation([1,0,0],roll)
        transform = transform @ offset
        axis = np.fromstring(joint.find('axis').get('xyz'), sep=' ')
        axes.append(transform[:3,:3] @ axis)
        origins.append(transform[:3,3].copy())
        offset = np.eye(4)
        offset[:3,:3] = rotation(axis, angle)
        transform = transform @ offset
    tip = transform[:3,3]
    jacobian = np.array([np.r_[np.cross(a,tip-o), a] for a,o in zip(axes,origins)]).T
    singular_values = np.linalg.svd(jacobian, compute_uv=False)
    return dict(hardware_degrees=pose, model_degrees=np.degrees(model).tolist(),
                condition=float(singular_values[0]/singular_values[-1]),
                wrist_axis_alignment_abs=float(abs(np.dot(axes[3],axes[5]))))

if __name__ == '__main__':
    original_model = ORIGINAL / 'dummy-ros2_description/urdf/dummy-ros2.xacro'
    print(json.dumps({'model_copy_identical': hashlib.sha256(MODEL.read_bytes()).digest() == hashlib.sha256(original_model.read_bytes()).digest()}))
    # Parse only the gateway's constants/functions, never initialize ROS/HTTP.
    tree = ast.parse((ROOT / 'moveit_servo_gateway.py').read_text(encoding='utf-8'))
    nodes = [n for n in tree.body if isinstance(n, ast.FunctionDef) and n.name in {'hardware_to_model','model_to_hardware'} or isinstance(n, ast.Assign) and any(isinstance(t,ast.Name) and t.id in {'OFFSET_RAD','DIRECTION'} for t in n.targets)]
    from joint_contract import MODEL_DIRECTIONS, MODEL_OFFSETS_RAD
    scope = {'math': math, 'MODEL_DIRECTIONS': MODEL_DIRECTIONS, 'MODEL_OFFSETS_RAD': MODEL_OFFSETS_RAD}
    exec(compile(ast.Module(body=nodes,type_ignores=[]), '<gateway mapping>', 'exec'),scope)
    for pose in [[.71,-64.6,157.05,-.01,.17,.09], [20,-30,100,20,.17,40],
                 [.71,-64.6,157.05,-.01,15,.09], [.71,-64.6,157.05,-.01,30,.09],
                 [0,0,90,0,45,0]]:
        assert np.allclose(scope['model_to_hardware'](scope['hardware_to_model'](pose)), pose)
        print(json.dumps(audit(pose)))
