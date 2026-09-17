"""MoveIt Servo launch without the repository's hardware/auto-motion node."""
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch_param_builder import ParameterBuilder
from moveit_configs_utils import MoveItConfigsBuilder
from joint_contract import apply_model_limits


def generate_launch_description():
    moveit_config = (
        MoveItConfigsBuilder("dummy-ros2", package_name="dummy_moveit_config")
        .robot_description(file_path="config/dummy-ros2.urdf.xacro")
        .robot_description_semantic(file_path="config/dummy-ros2.srdf")
        .robot_description_kinematics(file_path="config/kinematics.yaml")
        .to_moveit_configs()
    )
    moveit_config.robot_description['robot_description'] = apply_model_limits(
        moveit_config.robot_description['robot_description'])
    safe_yaml = str(Path(__file__).with_name("servo_safe.yaml"))
    servo_params = (
        ParameterBuilder("dummy_moveit_config")
        .yaml(parameter_namespace="moveit_servo", file_path=safe_yaml)
        .to_dict()
    )
    static_tf = Node(
        package="tf2_ros", executable="static_transform_publisher",
        name="static_transform_publisher", output="log",
        arguments=["0", "0", "0", "0", "0", "0", "world", "base_link"],
    )
    servo = Node(
        package="moveit_servo", executable="servo_node", name="servo_node",
        output="screen", parameters=[servo_params, moveit_config.robot_description,
                                     moveit_config.robot_description_semantic,
                                     moveit_config.robot_description_kinematics],
    )
    state_publisher = Node(
        package="robot_state_publisher", executable="robot_state_publisher",
        output="screen", parameters=[moveit_config.robot_description],
    )
    return LaunchDescription([static_tf, servo, state_publisher])
