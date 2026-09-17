"""Real MoveIt direction/TCP checks in ROS domain 197; no hardware or HTTP I/O."""
import os
os.environ['ROS_DOMAIN_ID'] = '197'
import argparse
import json
from pathlib import Path
import signal
import subprocess
import sys
import threading
import time

import numpy as np
import rclpy
from tf2_ros import Buffer, TransformListener
from geometry_msgs.msg import TwistStamped

from moveit_servo_gateway import GatewayNode, hardware_to_model

ROOT = Path(__file__).resolve().parent


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--configured-tcp', action='store_true',
                        help='Validate the configured TCP in isolated ROS, without hardware I/O')
    args = parser.parse_args()
    log = open(ROOT / '../runtime/tool-frame-offline.log', 'w')
    process = subprocess.Popen([sys.executable, str(ROOT / 'run_safe_launch.py')], stdout=log, stderr=log)
    rclpy.init()
    node = GatewayNode()
    buffer = Buffer()
    listener = TransformListener(buffer, node)
    twists = []
    subscription = node.create_subscription(TwistStamped, '/servo_node/delta_twist_cmds', twists.append, 10)
    spinner = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    spinner.start()
    reports = []
    try:
        deadline = time.monotonic() + 25
        while node.health()['status'] != 'ready' and time.monotonic() < deadline:
            time.sleep(.1)
        # A hypothetical 100 mm TCP checks compensation without assuming the
        # physical gripper's dimensions. Production configuration is untouched.
        if not args.configured_tcp:
            node.tool_frame.offset = np.array([0., -.10, 0.])
        for pose in ([0., 0., 90., 0., 45., 0.], [40., 0., 90., 0., 45., 35.]):
            q = hardware_to_model(pose)
            start, rotation = node.tool_frame.pose(q)
            for name, fields, direction in (
                ('w', {'linear_x': .02}, [1., 0., 0.]),
                ('s', {'linear_x': -.02}, [-1., 0., 0.]),
                ('a', {'linear_y': .02}, [0., 1., 0.]),
                ('d', {'linear_y': -.02}, [0., -1., 0.]),
                ('up', {'linear_z': .02}, [0., 0., 1.]),
                ('down', {'linear_z': -.02}, [0., 0., -1.]),
                ('w_down', {'linear_x': .02, 'linear_z': -.02}, [1., 0., -1.]),
                ('8', {'angular_y': -.08}, [0., -1., 0.]),
                ('2', {'angular_y': .08}, [0., 1., 0.]),
                ('4', {'angular_z': .08}, [0., 0., 1.]),
                ('6', {'angular_z': -.08}, [0., 0., -1.]),
            ):
                node.set_feedback(pose)
                okay, reason = node.arm()
                assert okay, reason
                command = {f'{kind}_{a}': 0. for kind in ('linear', 'angular') for a in 'xyz'}
                command.update(fields)
                for _ in range(30):
                    node.set_feedback(pose)
                    node.set_command(command)
                    time.sleep(.04)
                sample = node.latest_trajectory()
                assert sample['positions'] is not None, sample
                end, end_rotation = node.tool_frame.pose(hardware_to_model(sample['positions']))
                delta = end - start
                forward_delta = rotation.T @ (end_rotation[:, 0] - rotation[:, 0])
                expected = np.array(direction)
                twist = twists[-1].twist
                linear = np.array([twist.linear.x, twist.linear.y, twist.linear.z])
                angular = np.array([twist.angular.x, twist.angular.y, twist.angular.z])
                flange = node.tool_frame.flange_pose(q)
                tcp_velocity = linear + np.cross(angular, flange[:3, :3] @ node.tool_frame.offset)
                # Check published ROS velocity, not accumulated Euler targets
                # against a pose deliberately held fixed by this offline probe.
                if name in ('w', 's', 'a', 'd', 'up', 'down', 'w_down'):
                    assert np.allclose(rotation.T @ tcp_velocity, .02*expected, atol=1e-9), (name, tcp_velocity)
                    assert np.allclose(angular, 0., atol=1e-9)
                else:
                    assert np.allclose(tcp_velocity, 0., atol=1e-9), (name, tcp_velocity)
                    assert np.allclose(rotation.T @ angular, .08*expected, atol=1e-9), (name, angular)
                # Independently validate our FK against robot_state_publisher's TF.
                tf = buffer.lookup_transform('base_link', 'link6_1_1', rclpy.time.Time()).transform
                flange = node.tool_frame.flange_pose(q)
                assert np.allclose([tf.translation.x, tf.translation.y, tf.translation.z], flange[:3, 3], atol=1e-7)
                quat = tf.rotation
                xyz = np.array([quat.x, quat.y, quat.z])
                skew = np.array([[0, -xyz[2], xyz[1]], [xyz[2], 0, -xyz[0]], [-xyz[1], xyz[0], 0]])
                tf_rotation = np.eye(3) + 2*quat.w*skew + 2*skew@skew
                assert np.allclose(tf_rotation, flange[:3, :3], atol=1e-7)
                assert twists[-1].header.frame_id == 'base_link'
                reports.append({'pose': pose, 'input': name, 'tcp_delta_in_base_mm': (delta*1000).tolist(),
                                'tcp_velocity_in_base_m_s': tcp_velocity.tolist(),
                                'angular_velocity_in_base_rad_s': angular.tolist(),
                                'forward_change_in_tool': forward_delta.tolist(), 'tf_matches': True})
                node.disarm()
                time.sleep(.15)
        report = {'hardware_io': False, 'command_frame': node.tool_frame.status()['command_frame'], 'configured_tcp': args.configured_tcp,
                  'tcp_offset_m': node.tool_frame.offset.tolist(),
                  'checks': reports, 'result': 'passed'}
        (ROOT / '../runtime/tool-frame-offline.json').write_text(json.dumps(report, indent=2), encoding='utf-8')
        print(json.dumps(report), flush=True)
    finally:
        node.disarm()
        process.send_signal(signal.SIGINT)
        try:
            process.wait(timeout=8)
        except subprocess.TimeoutExpired:
            process.terminate()
        rclpy.shutdown()
        spinner.join(timeout=2)
        log.close()


if __name__ == '__main__':
    main()
