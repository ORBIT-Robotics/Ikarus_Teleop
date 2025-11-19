"""Launch MoveIt Servo plus joint-state bridge for ikarus_simplified.

This launch spins up:
- moveit_servo/servo_node_main: consumes Twist commands per config/servo.yaml and
  publishes trajectory_msgs/JointTrajectory to /arm_controller/joint_trajectory.
- joint_state_to_array bridge: subscribes to sensor_msgs/JointState on /joint_states
  and republishes a std_msgs/Float64MultiArray named /ikarus_ik containing
  [Shoulder1R, Shoulder2R, Shoulder3R, ElbowR, uaR] positions.
"""

from pathlib import Path

import yaml
from launch import LaunchDescription
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder


def generate_launch_description():
    # Load your MoveIt config for ikarus_simplified
    moveit_config = (
        MoveItConfigsBuilder("ikarus_simplified", package_name="ikarus_moveit_config")
        .to_moveit_configs()
    )

    # Point to *your* servo.yaml in your package (note: no extra 'o' here)
    servo_config_path = Path(moveit_config.package_path) / "config" / "servo.yaml"
    with servo_config_path.open("r") as servo_file:
        servo_params = yaml.safe_load(servo_file)

    if not isinstance(servo_params, dict):
        servo_params = {}

    # Normalize to { "moveit_servo": { ... actual parameters ... } }
    if "moveit_servo" in servo_params:
        nested = servo_params["moveit_servo"]
        if isinstance(nested, dict) and "ros__parameters" in nested:
            servo_param_dict = nested["ros__parameters"]
        else:
            servo_param_dict = nested
    else:
        servo_param_dict = servo_params

    servo_param_block = {"moveit_servo": servo_param_dict}

    servo_node = Node(
        package="moveit_servo",
        executable="servo_node_main",
        name="moveit_servo",
        output="screen",
        parameters=[
            servo_param_block,
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.robot_description_kinematics,
            moveit_config.joint_limits,
            moveit_config.planning_pipelines,
        ],
        # optional but recommended to remove the warning:
        # extra_arguments=[{"use_intra_process_comms": True}],
    )

    joint_state_bridge = Node(
        package="ikarus_moveit_config",
        executable="joint_state_to_array.py",  # keep .py if that's how you installed it
        name="joint_state_to_array",
        output="screen",
        parameters=[
            {
                "input_topic": "/joint_states",
                "output_topic": "/ikarus_ik",
                "joint_order": [
                    "Shoulder1R",
                    "Shoulder2R",
                    "Shoulder3R",
                    "ElbowR",
                    "uaR",
                ],
            }
        ],
    )

    ros2_controllers_path = Path(moveit_config.package_path) / "config" / "ros2_controllers.yaml"

    ros2_control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[moveit_config.robot_description, str(ros2_controllers_path)],
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

    return LaunchDescription(
        [
            ros2_control_node,
            joint_state_broadcaster_spawner,
            arm_controller_spawner,
            servo_node,
            joint_state_bridge,
        ]
    )
