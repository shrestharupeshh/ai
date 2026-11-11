"""Physical robot bring-up: description, base controller bridge, lidar, cameras, environment sensors.
Navigation and the mission node are launched separately (navigation.launch.py, agrirover_apps mission_node)."""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    desc = get_package_share_directory("agrirover_description")
    cams = [Node(package="v4l2_camera", executable="v4l2_camera_node", name=f"camera_{side}", namespace=f"camera_{side}",
                 parameters=[{"video_device": LaunchConfiguration(f"{side}_camera_device"), "image_size": [1280, 720],
                              "camera_frame_id": f"camera_{side}_optical_frame"}]) for side in ("left", "right")]
    return LaunchDescription([
        DeclareLaunchArgument("base_port", default_value="/dev/ttyAMA0"),
        DeclareLaunchArgument("lidar_port", default_value="/dev/ttyUSB0"),
        DeclareLaunchArgument("left_camera_device", default_value="/dev/video0"),
        DeclareLaunchArgument("right_camera_device", default_value="/dev/video2"),
        IncludeLaunchDescription(PythonLaunchDescriptionSource(os.path.join(desc, "launch", "description.launch.py"))),
        Node(package="agrirover_base", executable="base_node", parameters=[{"port": LaunchConfiguration("base_port")}], output="screen"),
        Node(package="sllidar_ros2", executable="sllidar_node", name="sllidar_node",
             parameters=[{"channel_type": "serial", "serial_port": LaunchConfiguration("lidar_port"), "serial_baudrate": 460800,
                          "frame_id": "lidar_link", "inverted": False, "angle_compensate": True, "scan_mode": "Standard"}]),
        *cams,
        Node(package="agrirover_apps", executable="env_sensor_node", output="screen"),
    ])
