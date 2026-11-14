"""Start the iPhone OSC receiver and EE-goal teleop node.

Run the OCS2 graph separately with go2_x5_ocs2/go2_x5_ros_mpc.launch.py. This
node publishes /mobile_manipulator_mpc_target directly; the planner in that
graph may also consume /go2_x5/ee/goal when it is running.
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    share = FindPackageShare("teleop_iphone")
    rviz = DeclareLaunchArgument("rviz", default_value="false")
    osc = Node(package="teleop_iphone", executable="osc_node", name="osc_node", output="screen",
               parameters=[PathJoinSubstitution([share, "config", "osc_params.yaml"])])
    teleop = Node(package="teleop_iphone", executable="teleop_iphone", name="teleop_iphone", output="screen",
                  parameters=[PathJoinSubstitution([share, "config", "teleop_iphone.yaml"])])
    view = Node(package="rviz2", executable="rviz2", name="teleop_iphone_rviz", output="screen",
                arguments=["-d", PathJoinSubstitution([share, "config", "default.rviz"])],
                condition=IfCondition(LaunchConfiguration("rviz")))
    feedback_arg = DeclareLaunchArgument("feedback", default_value="true")
    feedback_host_arg = DeclareLaunchArgument("feedback_host", default_value="")
    feedback_node = Node(
        package="teleop_iphone", executable="robot_state_feedback.py", name="robot_state_feedback",
        output="screen", condition=IfCondition(LaunchConfiguration("feedback")),
        parameters=[PathJoinSubstitution([share, "config", "robot_state_feedback.yaml"]),
                    {"host": ParameterValue(LaunchConfiguration("feedback_host"), value_type=str)}],
    )
    return LaunchDescription([feedback_arg, feedback_host_arg, feedback_node, rviz, osc, teleop, view])
