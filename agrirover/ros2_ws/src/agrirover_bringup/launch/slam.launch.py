"""Site survey: build a map with slam_toolbox while driving manually (teleop)."""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    share = get_package_share_directory("agrirover_bringup")
    sim = {"use_sim_time": LaunchConfiguration("use_sim_time")}
    return LaunchDescription([
        DeclareLaunchArgument("use_sim_time", default_value="false"),
        Node(package="robot_localization", executable="ekf_node", name="ekf_filter_node", parameters=[os.path.join(share, "config", "ekf.yaml"), sim]),
        Node(package="slam_toolbox", executable="async_slam_toolbox_node", name="slam_toolbox",
             parameters=[os.path.join(share, "config", "slam_toolbox_mapping.yaml"), sim], output="screen"),
    ])
