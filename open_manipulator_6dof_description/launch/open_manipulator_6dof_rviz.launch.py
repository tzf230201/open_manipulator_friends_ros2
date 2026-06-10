from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    open_rviz = LaunchConfiguration("open_rviz")
    use_gui = LaunchConfiguration("use_gui")

    description_pkg = FindPackageShare("open_manipulator_6dof_description")
    robot_description_content = Command(
        [
            FindExecutable(name="xacro"),
            " ",
            PathJoinSubstitution(
                [description_pkg, "urdf", "open_manipulator_6dof.urdf.xacro"]
            ),
        ]
    )
    robot_description = {
        "robot_description": ParameterValue(robot_description_content, value_type=str)
    }

    joint_state_publisher = Node(
        package="joint_state_publisher",
        executable="joint_state_publisher",
        name="joint_state_publisher",
        parameters=[
            {"source_list": ["/open_manipulator_6dof/joint_states"]},
        ],
        condition=UnlessCondition(use_gui),
    )

    joint_state_publisher_gui = Node(
        package="joint_state_publisher_gui",
        executable="joint_state_publisher_gui",
        name="joint_state_publisher_gui",
        condition=IfCondition(use_gui),
    )

    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="screen",
        parameters=[robot_description],
    )

    rviz = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
        arguments=[
            "-d",
            PathJoinSubstitution(
                [description_pkg, "rviz", "open_manipulator_6dof.rviz"]
            ),
        ],
        condition=IfCondition(open_rviz),
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("open_rviz", default_value="true"),
            DeclareLaunchArgument("use_gui", default_value="false"),
            joint_state_publisher,
            joint_state_publisher_gui,
            robot_state_publisher,
            rviz,
        ]
    )
