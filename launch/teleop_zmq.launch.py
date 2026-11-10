"""Backward-compatible alias for the native Go2-X5 OCS2 teleop launch."""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    share = FindPackageShare("teleop_iphone")
    rviz = DeclareLaunchArgument("rviz", default_value="false")
    teleop = Node(
        package="teleop_iphone", executable="teleop_iphone", name="teleop_iphone", output="screen",
        parameters=[PathJoinSubstitution([share, "config", "teleop_iphone.yaml"])],
    )
    view = Node(
        package="rviz2", executable="rviz2", name="teleop_iphone_rviz", output="screen",
        arguments=["-d", PathJoinSubstitution([share, "config", "default.rviz"])],
        condition=IfCondition(LaunchConfiguration("rviz")),
    )
    return LaunchDescription([rviz, teleop, view])
