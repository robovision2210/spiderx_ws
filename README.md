# 🤖 SpiderX WS — Autonomous Quadruped (ROS 2 Humble)

SpiderX workspace for a legged quadruped with simulation, lidar-enabled mapping/localization, and Nav2 scaffolding.

## 📦 Workspace Layout

```text
spiderx_ws/src/
├── spiderx_description/      # URDF xacro, meshes, Gazebo world/launch, ros2_control config
├── spiderx_bringup/          # Top-level simulation bringup launch
├── spiderx_mapping/          # slam_toolbox config and launch
├── spiderx_localization/     # AMCL + EKF localization launch/config
├── spiderx_navigation/       # Nav2 launch/config + behavior tree
└── rplidar_ros-24cc9b6dea97e045bda1408eaa867ce730fd3fc3/  # Vendored lidar driver
```

## ✅ Recommended Simulator Path (Gazebo Fortress)

### Install (Ubuntu 22.04 + ROS 2 Humble)

```bash
sudo apt update
sudo apt install -y \
  ros-humble-ros-gz-sim \
  ros-humble-ros-gz-bridge
```

### Build

```bash
cd ~/spiderx_ws
colcon build
source install/setup.bash
```

### Launch SpiderX in Fortress

```bash
ros2 launch spiderx_bringup fortress.launch.py
```

## 🕰️ Legacy Simulator Path (Gazebo Classic 11)

Legacy Gazebo Classic files are retained for reference/backward compatibility.

```bash
ros2 launch spiderx_bringup gazebo.launch.py
```

## 🧭 Navigation Stack

```bash
ros2 launch spiderx_navigation navigation.launch.py
```

> Current Fortress migration intentionally spawns a passive SpiderX model. Full leg control and Nav2-in-sim execution require future validated gait-controller + `gz_ros2_control` integration.

## 📘 Migration Notes

See `MIGRATION_FORTRESS.md` for detailed plugin/topic migration and limitations.
