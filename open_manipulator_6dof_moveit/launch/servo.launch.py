"""MoveIt Servo (realtime Cartesian jog) on the 6dof description.

Single-kinematics setup: Servo shares the same URDF + SRDF as move_group
(open_manipulator_6dof), so IK, collision model, and joint limits all come
from one place.

Requires the ros2_control controllers (om_chain_bringup hardware.launch.py).
Easiest all-in-one: go2w_remote_arm demo_servo.launch.py. Standalone:

    ros2 launch om_chain_bringup hardware.launch.py
    ros2 launch open_manipulator_6dof_moveit servo.launch.py

Then publish geometry_msgs/TwistStamped on /servo_node/delta_twist_cmds
(or use the go2w_remote_arm remote_servo_bridge).
"""
import os
from pathlib import Path

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch.substitutions import Command, FindExecutable


def _load_yaml(share, rel):
    with open(Path(share) / rel) as f:
        return yaml.safe_load(f) or {}


def _read(share, rel):
    with open(Path(share) / rel) as f:
        return f.read()


def generate_launch_description():
    moveit_share = get_package_share_directory("open_manipulator_6dof_moveit")
    description_share = get_package_share_directory(
        "open_manipulator_6dof_description")

    robot_description = {
        "robot_description": ParameterValue(
            Command([
                FindExecutable(name="xacro"), " ",
                os.path.join(description_share, "urdf",
                             "open_manipulator_6dof.urdf.xacro"),
            ]),
            value_type=str,
        )
    }
    robot_description_semantic = {
        "robot_description_semantic": _read(
            moveit_share, "config/open_manipulator_6dof.srdf"),
    }
    kinematics = {
        "robot_description_kinematics": _load_yaml(
            moveit_share, "config/kinematics.yaml"),
    }

    servo_params = {
        "moveit_servo": _load_yaml(moveit_share, "config/moveit_servo.yaml"),
    }

    servo_node = Node(
        package="moveit_servo",
        executable="servo_node_main",
        output="screen",
        parameters=[
            servo_params,
            robot_description,
            robot_description_semantic,
            kinematics,
        ],
    )

    return LaunchDescription([servo_node])
