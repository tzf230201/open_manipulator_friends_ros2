from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node, PushRosNamespace


def generate_launch_description():
    robot_name = LaunchConfiguration("robot_name")
    end_effector = LaunchConfiguration("end_effector")

    teleop = GroupAction(
        [
            PushRosNamespace(robot_name),
            Node(
                package="open_manipulator_6dof_teleop",
                executable="open_manipulator_6dof_teleop_keyboard",
                name="teleop_keyboard",
                output="screen",
                emulate_tty=True,
                parameters=[{"end_effector_name": end_effector}],
                remappings=[
                    (
                        "kinematics_pose",
                        PathJoinSubstitution([end_effector, "kinematics_pose"]),
                    )
                ],
            ),
        ]
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("robot_name", default_value="open_manipulator_6dof"),
            DeclareLaunchArgument("end_effector", default_value="gripper"),
            teleop,
        ]
    )
