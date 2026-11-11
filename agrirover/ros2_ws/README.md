# ROS 2 workspace (Jazzy, Ubuntu 24.04)

**Status: statically checked, never executed.** The package repository for ROS 2 was not reachable where this
workspace was written, so nothing here has been built with `colcon` or launched. Expect to fix small things on the
first build (parameter names, launch arguments). The shared Python core it depends on *is* tested (`software/tests`).

## Install on the Raspberry Pi 5 / a development PC

```bash
# ROS 2 Jazzy: follow https://docs.ros.org/en/jazzy/Installation/Ubuntu-Install-Debs.html, then
sudo apt install ros-jazzy-navigation2 ros-jazzy-nav2-bringup ros-jazzy-slam-toolbox ros-jazzy-robot-localization \
  ros-jazzy-xacro ros-jazzy-robot-state-publisher ros-jazzy-v4l2-camera ros-jazzy-cv-bridge \
  ros-jazzy-ros-gz python3-colcon-common-extensions python3-serial

cd agrirover
pip install --break-system-packages -e "software[sensors]"      # core library used by the nodes (or use a venv with system-site-packages)
python3 tools/gen_ros_assets.py                                  # map, keep-out mask, world, stations

cd ros2_ws/src && git clone https://github.com/Slamtec/sllidar_ros2.git && cd ..
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install && source install/setup.bash
```

## Packages

| Package | Contents |
|---|---|
| `agrirover_description` | `urdf/agrirover.urdf.xacro` (frames per REP-105, Gazebo plugins behind `use_gazebo`), `description.launch.py` |
| `agrirover_bringup` | `robot.launch.py` (hardware), `sim_gazebo.launch.py`, `navigation.launch.py` (map server, AMCL, keep-out filter, EKF, Nav2 servers, collision monitor), `slam.launch.py`; `config/` (Nav2, EKF, slam_toolbox, gz bridge, stations); `maps/`; `worlds/greenhouse.sdf` |
| `agrirover_base` | `base_node`: serial bridge to the STM32 — `/cmd_vel` in; `/wheel/odometry`, `/imu/data_raw`, `/battery_state`, `/base/state` out; `~/enable`, `~/disable`, `~/clear_fault` |
| `agrirover_apps` | `mission_node` (Nav2 client, capture, vision, SQLite), `env_sensor_node` |

## Run

```bash
# simulation
ros2 launch agrirover_bringup sim_gazebo.launch.py
ros2 launch agrirover_bringup navigation.launch.py use_sim_time:=true
ros2 run agrirover_apps mission_node --ros-args -p stations_file:=$(ros2 pkg prefix agrirover_bringup)/share/agrirover_bringup/config/stations.yaml
ros2 service call /agrirover_mission/start std_srvs/srv/Trigger

# robot
ros2 launch agrirover_bringup robot.launch.py base_port:=/dev/ttyAMA0 lidar_port:=/dev/ttyUSB0
ros2 launch agrirover_bringup navigation.launch.py
```

Velocity path: Nav2 controller → `cmd_vel_nav` → velocity smoother → `cmd_vel_smoothed` → collision monitor →
`cmd_vel` → `base_node` → firmware (which applies its own limits and time-outs).

## First-build checklist

1. `colcon build` clean; `ros2 launch agrirover_description description.launch.py` and inspect TF in RViz.
2. Gazebo: robot spawns on the dock, `/scan` shows walls and bench legs, teleop drives it.
3. Navigation: AMCL converges at the dock; a goal in an aisle is reached; the keep-out mask covers the benches in the costmap.
4. Repeat the scenarios of `software/agrirover/sim/scenarios.py` (test T-ROS-01) and record the results in `docs/10`.
