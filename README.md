# 🤖 SpiderX WS — Autonomous Quadruped (ROS 2 Humble + Gazebo Classic)

SpiderX workspace for a legged quadruped with Gazebo Classic simulation, lidar-enabled mapping/localization, and Nav2 navigation scaffolding.

## 📦 Workspace Layout

```
spiderx_ws/src/
├── spiderx_description/      # URDF xacro, meshes, Gazebo world/launch, ros2_control config
├── spiderx_bringup/          # Top-level Gazebo + state + lidar bringup launch
├── spiderx_mapping/          # slam_toolbox config and launch
├── spiderx_localization/     # AMCL + EKF localization launch/config
├── spiderx_navigation/       # Nav2 launch/config + behavior tree
└── rplidar_ros-24cc9b6dea97e045bda1408eaa867ce730fd3fc3/  # Vendored lidar driver
```

## 🚀 Build

```bash
cd ~/spiderx_ws
colcon build
source install/setup.bash
```

## ▶️ Run Gazebo Simulation

```bash
ros2 launch spiderx_bringup gazebo.launch.py
```

## 🧭 Run Navigation Stack

```bash
ros2 launch spiderx_navigation navigation.launch.py
```
