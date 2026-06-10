import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder


def generate_launch_description():
    open_rviz = LaunchConfiguration("open_rviz")
    use_gui = LaunchConfiguration("use_gui")

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

    move_group = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=[moveit_config.to_dict()],
    )

    joint_state_publisher = Node(
        package="joint_state_publisher",
        executable="joint_state_publisher",
        name="joint_state_publisher",
        parameters=[
            moveit_config.robot_description,
            {"source_list": ["move_group/fake_controller_joint_states"]},
        ],
        condition=UnlessCondition(use_gui),
    )

    joint_state_publisher_gui = Node(
        package="joint_state_publisher_gui",
        executable="joint_state_publisher_gui",
        name="joint_state_publisher_gui",
        parameters=[moveit_config.robot_description],
        condition=IfCondition(use_gui),
    )

    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="screen",
        parameters=[moveit_config.robot_description],
    )

    rviz = Node(
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
        condition=IfCondition(open_rviz),
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("open_rviz", default_value="true"),
            DeclareLaunchArgument("use_gui", default_value="false"),
            move_group,
            joint_state_publisher,
            joint_state_publisher_gui,
            robot_state_publisher,
            rviz,
        ]
    )
