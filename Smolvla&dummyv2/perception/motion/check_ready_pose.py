"""Offline self-collision sampling, isolated from the real robot ROS domain."""
import os
os.environ['ROS_DOMAIN_ID'] = '195'
import json
from pathlib import Path
import subprocess
import sys
import time

ROOT = Path(__file__).resolve().parent

def server():
    from launch import LaunchService, LaunchDescription
    from launch_ros.actions import Node
    from moveit_configs_utils import MoveItConfigsBuilder
    config = (MoveItConfigsBuilder('dummy-ros2', package_name='dummy_moveit_config')
              .robot_description(file_path='config/dummy-ros2.urdf.xacro')
              .robot_description_semantic(file_path='config/dummy-ros2.srdf')
              .to_moveit_configs())
    launch = LaunchService()
    launch.include_launch_description(LaunchDescription([Node(
        package='moveit_ros_move_group', executable='move_group',
        parameters=[config.to_dict()], output='screen')]))
    launch.run()

def check():
    import rclpy
    from moveit_msgs.srv import GetStateValidity
    from moveit_servo_gateway import hardware_to_model, JOINT_NAMES
    report = []
    with open(ROOT / '../runtime/ready-pose-check.log', 'w') as log:
        process = subprocess.Popen([sys.executable, __file__, '--server'], stdout=log, stderr=log)
        rclpy.init()
        node = rclpy.create_node('offline_ready_pose_check')
        client = node.create_client(GetStateValidity, '/check_state_validity')
        try:
            if not client.wait_for_service(timeout_sec=25):
                raise RuntimeError('state validity service unavailable')
            start = [0,-75,180,0,0,0]
            end = [0,0,90,0,45,0]
            for index in range(91):
                t = index/90
                pose = [a+(b-a)*t for a,b in zip(start,end)]
                request = GetStateValidity.Request()
                request.group_name = 'dummy_arm'
                request.robot_state.joint_state.name = JOINT_NAMES
                request.robot_state.joint_state.position = hardware_to_model(pose)
                future = client.call_async(request)
                rclpy.spin_until_future_complete(node,future,timeout_sec=3)
                result = future.result()
                if result is None:
                    raise RuntimeError('validity query timeout')
                report.append({'fraction':t,'valid':result.valid,'contacts':[
                    [c.contact_body_1,c.contact_body_2] for c in result.contacts]})
            summary = {'start':start,'target':end,'samples':len(report),
                       'invalid':[r for r in report if not r['valid']],
                       'scope':'joint-linear samples; model self-collision only, no real-world obstacles'}
            print(json.dumps(summary),flush=True)
            (ROOT / '../runtime/ready-pose-check.json').write_text(json.dumps(summary,indent=2),encoding='utf-8')
        finally:
            process.send_signal(2)
            try:
                process.wait(timeout=8)
            except subprocess.TimeoutExpired:
                process.terminate()
            node.destroy_node()
            rclpy.shutdown()

if __name__ == '__main__':
    server() if '--server' in sys.argv else check()
