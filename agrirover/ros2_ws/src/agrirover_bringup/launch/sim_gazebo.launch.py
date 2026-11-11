"""Gazebo (gz-sim Harmonic) greenhouse simulation + robot + bridge. Add navigation with:
    ros2 launch agrirover_bringup navigation.launch.py use_sim_time:=true
NOT executed in the authoring environment (no Gazebo); the headless simulator in software/agrirover/sim is the tested path."""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():
    share = get_package_share_directory("agrirover_bringup")
    desc = get_package_share_directory("agrirover_description")
    world = os.path.join(share, "worlds", "greenhouse.sdf")
    return LaunchDescription([
        IncludeLaunchDescription(PythonLaunchDescriptionSource(os.path.join(get_package_share_directory("ros_gz_sim"), "launch", "gz_sim.launch.py")),
                                 launch_arguments={"gz_args": f"-r {world}"}.items()),
        IncludeLaunchDescription(PythonLaunchDescriptionSource(os.path.join(desc, "launch", "description.launch.py")),
                                 launch_arguments={"use_gazebo": "true", "use_sim_time": "true"}.items()),
        Node(package="ros_gz_sim", executable="create", arguments=["-name", "agrirover", "-topic", "robot_description", "-x", "1.0", "-y", "0.4", "-z", "0.02"]),
        Node(package="ros_gz_bridge", executable="parameter_bridge", parameters=[{"config_file": os.path.join(share, "config", "gz_bridge.yaml")}]),
    ])
