import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from moveit_configs_utils import MoveItConfigsBuilder
from launch_ros.actions import Node


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
        .trajectory_execution(file_path="config/fake_controllers.yaml")
        .planning_pipelines(default_planning_pipeline="ompl", pipelines=["ompl"])
        .to_moveit_configs()
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
    )

    return LaunchDescription([rviz])
