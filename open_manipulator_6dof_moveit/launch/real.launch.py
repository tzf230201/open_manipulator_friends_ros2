"""Real-hardware launch: OpenManipulator 6DOF MoveIt on the OM-Chain rig.

  1. om_chain_bringup hardware.launch.py   (ros2_control + custom-ID
     Dynamixels + robot_state_publisher + controllers — the proven stack)
  2. move_group with THIS package's URDF/SRDF (tip = end_effector_link,
     so the RViz marker sits at the gripper grip point)
  3. RViz with the MotionPlanning plugin

Works because both descriptions use identical joint names (joint1..6,
gripper) and identical link geometry, and config/fake_controllers.yaml
here actually targets the real ros2_control actions
(/arm_controller/follow_joint_trajectory + /gripper_controller/gripper_cmd).

Only one process can hold /dev/ttyUSB0. Stop the teleop first:

    sudo systemctl stop go2w-arm-launcher.service

⚠ SRDF pose "init_pose" is all-zeros = arm pointing straight UP.
  Prefer "home_pose" as the parking pose.
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from moveit_configs_utils import MoveItConfigsBuilder


def generate_launch_description():
    description_pkg = get_package_share_directory("open_manipulator_6dof_description")
    moveit_pkg = get_package_share_directory("open_manipulator_6dof_moveit")

    moveit_config = (
        MoveItConfigsBuilder(
            "open_manipulator_6dof",
            package_name="open_manipulator_6dof_moveit",
        )
        .robot_description(
            file_path=os.path.join(
                description_pkg, "urdf", "open_manipulator_6dof.urdf.xacro"
            )
        )
        .robot_description_semantic(file_path="config/open_manipulator_6dof.srdf")
        .robot_description_kinematics(file_path="config/kinematics.yaml")
        .joint_limits(file_path="config/joint_limits.yaml")
        .trajectory_execution(file_path="config/fake_controllers.yaml")
        .planning_pipelines(default_planning_pipeline="ompl", pipelines=["ompl"])
        .to_moveit_configs()
    )

    # 1. hardware: ros2_control + RSP + controller spawners (om_chain stack)
    hardware = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution([
            FindPackageShare("om_chain_bringup"), "launch", "hardware.launch.py",
        ])),
    )

    # 2. move_group (delayed until controllers are up)
    move_group = TimerAction(
        period=4.0,
        actions=[Node(
            package="moveit_ros_move_group",
            executable="move_group",
            output="screen",
            parameters=[moveit_config.to_dict()],
        )],
    )

    # 3. RViz + MotionPlanning — start well after move_group is ready, or
    # the MotionPlanning plugin's initial planning-scene fetch fails and the
    # gray scene robot never syncs with the real arm.
    rviz = TimerAction(
        period=12.0,
        actions=[Node(
            package="rviz2",
            executable="rviz2",
            name="rviz2",
            output="screen",
            arguments=["-d", os.path.join(moveit_pkg, "launch", "moveit.rviz")],
            parameters=[
                moveit_config.robot_description,
                moveit_config.robot_description_semantic,
                moveit_config.robot_description_kinematics,
                moveit_config.planning_pipelines,
            ],
        )],
    )

    return LaunchDescription([hardware, move_group, rviz])
