from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    pkg_share = FindPackageShare("teleop_iphone")
    osc_config = PathJoinSubstitution([pkg_share, "config", "osc_params.yaml"])
    teleop_config = PathJoinSubstitution([pkg_share, "config", "teleop_iphone.yaml"])
    rviz_config = PathJoinSubstitution([pkg_share, "config", "default.rviz"])
    use_rviz = DeclareLaunchArgument("rviz", default_value="false")

    osc_node = Node(
        package="teleop_iphone", executable="osc_node", name="osc_node",
        output="screen", parameters=[osc_config], emulate_tty=True,
    )
    teleop_node = Node(
        package="teleop_iphone", executable="teleop_iphone", name="teleop_iphone",
        output="screen", parameters=[teleop_config], emulate_tty=True,
    )
    rviz_node = Node(
        package="rviz2", executable="rviz2", name="teleop_iphone_rviz",
        output="screen", arguments=["-d", rviz_config], emulate_tty=True,
        condition=IfCondition(LaunchConfiguration("rviz")),
    )
    return LaunchDescription([use_rviz, osc_node, teleop_node, rviz_node])
