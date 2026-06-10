from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    open_rviz = LaunchConfiguration("open_rviz")

    move_group = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [
                    FindPackageShare("open_manipulator_6dof_moveit"),
                    "launch",
                    "move_group.launch.py",
                ]
            )
        )
    )

    rviz = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [
                    FindPackageShare("open_manipulator_6dof_moveit"),
                    "launch",
                    "moveit_rviz.launch.py",
                ]
            )
        ),
        condition=IfCondition(open_rviz),
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("open_rviz", default_value="true"),
            move_group,
            rviz,
        ]
    )
