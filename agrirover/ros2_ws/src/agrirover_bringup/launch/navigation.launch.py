"""Localisation (map_server + AMCL), keep-out filter, EKF and the Nav2 servers with a collision monitor in the cmd_vel path."""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    share = get_package_share_directory("agrirover_bringup")
    params = os.path.join(share, "config", "nav2_params.yaml")
    sim = {"use_sim_time": LaunchConfiguration("use_sim_time")}
    remap_cmd = [("cmd_vel", "cmd_vel_nav")]
    nodes = [
        ("nav2_map_server", "map_server", "map_server", [params, sim, {"yaml_filename": os.path.join(share, "maps", "greenhouse.yaml")}], []),
        ("nav2_map_server", "map_server", "filter_mask_server", [params, sim, {"yaml_filename": os.path.join(share, "maps", "keepout_mask.yaml")}], []),
        ("nav2_map_server", "costmap_filter_info_server", "costmap_filter_info_server", [params, sim], []),
        ("nav2_amcl", "amcl", "amcl", [params, sim], []),
        ("nav2_controller", "controller_server", "controller_server", [params, sim], remap_cmd),
        ("nav2_smoother", "smoother_server", "smoother_server", [params, sim], []),
        ("nav2_planner", "planner_server", "planner_server", [params, sim], []),
        ("nav2_behaviors", "behavior_server", "behavior_server", [params, sim], remap_cmd),
        ("nav2_bt_navigator", "bt_navigator", "bt_navigator", [params, sim], []),
        ("nav2_waypoint_follower", "waypoint_follower", "waypoint_follower", [params, sim], []),
        ("nav2_velocity_smoother", "velocity_smoother", "velocity_smoother", [params, sim], [("cmd_vel", "cmd_vel_nav")]),
        ("nav2_collision_monitor", "collision_monitor", "collision_monitor", [params, sim], []),
    ]
    names = [n[2] for n in nodes]
    return LaunchDescription([
        DeclareLaunchArgument("use_sim_time", default_value="false"),
        Node(package="robot_localization", executable="ekf_node", name="ekf_filter_node",
             parameters=[os.path.join(share, "config", "ekf.yaml"), sim], remappings=[("odometry/filtered", "/odometry/filtered")]),
        *[Node(package=p, executable=e, name=n, parameters=prm, remappings=rm, output="screen") for p, e, n, prm, rm in nodes],
        Node(package="nav2_lifecycle_manager", executable="lifecycle_manager", name="lifecycle_manager_navigation",
             parameters=[sim, {"autostart": True, "node_names": names}]),
    ])
