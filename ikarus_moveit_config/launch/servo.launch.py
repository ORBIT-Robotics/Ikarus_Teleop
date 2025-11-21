"""MoveIt Servo bringup for ikarus_simplified (structured like upstream examples)."""

import os
import yaml
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder


def load_yaml(package_name, file_path):
    package_path = get_package_share_directory(package_name)
    absolute_file_path = os.path.join(package_path, file_path)

    try:
        with open(absolute_file_path, "r") as file:
            return yaml.safe_load(file)
    except EnvironmentError:
        return None


def generate_launch_description():
    moveit_config = (
        MoveItConfigsBuilder("ikarus_simplified", package_name="ikarus_moveit_config")
        .to_moveit_configs()
    )

    ros2_controllers_path = os.path.join(
        get_package_share_directory("ikarus_moveit_config"),
        "config",
        "ros2_controllers.yaml",
    )

    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="screen",
        parameters=[moveit_config.robot_description],
    )

    ros2_control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[moveit_config.robot_description, ros2_controllers_path],
        output="screen",
    )

    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster"],
        output="screen",
    )

    arm_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["arm_controller"],
        output="screen",
    )

    joint_state_to_array = Node(
        package="ikarus_moveit_config",
        executable="joint_state_to_array.py",
        output="screen",
        parameters=[
            {
                "input_topic": "/joint_states",
                "output_topic": "/ikarus_ik",
                # "joint_order": ["Shoulder1R", "Shoulder2R", "Shoulder3R", "ElbowR", "uaR"],
            }
        ],
    )

    # Load Servo params
    servo_yaml = load_yaml("ikarus_moveit_config", "config/servo.yaml")
    servo_params = {"moveit_servo": servo_yaml}

    servo_node = Node(
        package="moveit_servo",
        executable="servo_node_main",
        parameters=[
            servo_params,
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.robot_description_kinematics,
        ],
        output="screen",
    )

    return LaunchDescription(
        [
            robot_state_publisher,
            ros2_control_node,
            joint_state_broadcaster_spawner,
            arm_controller_spawner,
            joint_state_to_array,
            servo_node,
        ]
    )
