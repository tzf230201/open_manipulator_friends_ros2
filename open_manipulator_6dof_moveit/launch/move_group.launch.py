import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder


def generate_launch_description():
    log_level = LaunchConfiguration("log_level")

    description_pkg = get_package_share_directory("open_manipulator_6dof_description")
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
        arguments=["--ros-args", "--log-level", log_level],
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("log_level", default_value="info"),
            move_group,
        ]
    )
