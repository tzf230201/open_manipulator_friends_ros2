from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    robot_name = LaunchConfiguration("robot_name")
    dynamixel_usb_port = LaunchConfiguration("dynamixel_usb_port")
    dynamixel_baud_rate = LaunchConfiguration("dynamixel_baud_rate")
    control_period = LaunchConfiguration("control_period")
    use_platform = LaunchConfiguration("use_platform")
    use_moveit = LaunchConfiguration("use_moveit")
    planning_group_name = LaunchConfiguration("planning_group_name")
    moveit_plan_only = LaunchConfiguration("moveit_plan_only")

    moveit_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [
                    FindPackageShare("open_manipulator_6dof_controller"),
                    "launch",
                    "open_manipulator_6dof_moveit.launch.py",
                ]
            )
        ),
        condition=IfCondition(use_moveit),
    )

    controller = Node(
        package="open_manipulator_6dof_controller",
        executable="open_manipulator_6dof_controller",
        name=robot_name,
        output="screen",
        arguments=[dynamixel_usb_port, dynamixel_baud_rate],
        parameters=[
            {
                "using_platform": use_platform,
                "using_moveit": use_moveit,
                "planning_group_name": planning_group_name,
                "control_period": control_period,
                "moveit_plan_only": moveit_plan_only,
            }
        ],
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("robot_name", default_value="open_manipulator_6dof"),
            DeclareLaunchArgument("dynamixel_usb_port", default_value="/dev/ttyACM0"),
            DeclareLaunchArgument("dynamixel_baud_rate", default_value="1000000"),
            DeclareLaunchArgument("control_period", default_value="0.010"),
            DeclareLaunchArgument("use_platform", default_value="true"),
            DeclareLaunchArgument("use_moveit", default_value="false"),
            DeclareLaunchArgument("planning_group_name", default_value="arm"),
            DeclareLaunchArgument("moveit_plan_only", default_value="true"),
            moveit_launch,
            controller,
        ]
    )
