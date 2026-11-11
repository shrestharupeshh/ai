"""Publish the robot description and static TF tree."""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    xacro_file = PathJoinSubstitution([FindPackageShare("agrirover_description"), "urdf", "agrirover.urdf.xacro"])
    description = ParameterValue(Command(["xacro ", xacro_file, " use_gazebo:=", LaunchConfiguration("use_gazebo")]), value_type=str)
    return LaunchDescription([
        DeclareLaunchArgument("use_gazebo", default_value="false"),
        DeclareLaunchArgument("use_sim_time", default_value="false"),
        Node(package="robot_state_publisher", executable="robot_state_publisher",
             parameters=[{"robot_description": description, "use_sim_time": LaunchConfiguration("use_sim_time")}]),
    ])
