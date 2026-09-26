# SpiderX Testing Guide

New to Ubuntu or ROS 2? Follow the step-by-step
[`SPIDERX_FORTRESS_UBUNTU_TEST_GUIDE.md`](SPIDERX_FORTRESS_UBUNTU_TEST_GUIDE.md) first.
This page lists every test layer and the command for each.

## Setup (every terminal)

```bash
source /opt/ros/humble/setup.bash
source ~/spiderx_ws/install/setup.bash
```

## Test layers

| # | Layer | Command | Pass criterion |
|---|---|---|---|
| 1 | Build | `cd ~/spiderx_ws && colcon build --symlink-install` | `Summary: 8 packages finished` |
| 2 | Static validation | `./scripts/validate_fortress.sh` | `All checks passed.` |
| 3 | Unit tests (including M1 safety rejections, M2 config/metrics and M3 FK/IK negative tests) | `colcon test --packages-select spiderx_controller spiderx_scripts && colcon test-result --verbose` | 0 failures |
| 4 | Controller config vs URDF | `ros2 run spiderx_controller validate_controller_config` | `Controller configuration: valid` |
| 5 | Runtime validation (starts Gazebo) | `./scripts/validate_fortress.sh --runtime` | `All checks passed.` (topics, `lidar_link`, 12 joints, no hardware node) |
| 6 | Manual simulation | `ros2 launch spiderx_bringup fortress.launch.py` | SpiderX visible in the walled world |
| 7 | Lidar diagnostics (sim running) | `ros2 run spiderx_scripts read_lidar` | About 3 m in all four sectors at the spawn point |
| 8 | Joint diagnostics (sim running) | `ros2 run spiderx_scripts read_joint_states` | 12 angles. The feet settle at ±25° (their limits) because the model is passive |
| 9 | RViz | `ros2 launch spiderx_bringup fortress.launch.py rviz:=true` | Global Status OK; the scan outlines the walls |
| 10 | TF | `ros2 run tf2_ros tf2_echo base_link lidar_link` | Translation `[0.051, -0.045, 0.141]` |
| 11 | M1 static | `./scripts/validate_m1_control.sh` | `All M1 checks passed.` |
| 12 | M1 runtime (starts Gazebo with control) | `./scripts/validate_m1_control.sh --runtime` | Controllers active, one joint and neutral pose PASS |
| 13 | M1 manual | `ros2 launch spiderx_bringup fortress_control.launch.py`, then `test_one_joint.py` / `test_neutral_pose.py` | See [M1 guide](M1_JOINT_POSITION_CONTROL_GUIDE.md) |
| 14 | M2 static (simulation only) | `./scripts/validate_m2_posture.sh` | `All M2 checks passed.` |
| 15 | M2 runtime (starts Gazebo with control and the ground-truth pose bridge) | `./scripts/validate_m2_posture.sh --runtime` | `Simulation posture hold verified.` and `All M2 checks passed.` |
| 16 | M2 manual | `ros2 launch spiderx_bringup fortress_posture_hold.launch.py`, then `ros2 run spiderx_controller run_posture_hold_test.py` | See [M2 guide](M2_SIMULATION_POSTURE_GUIDE.md) |
| 17 | M3 static (single-leg kinematics, simulation only) | `./scripts/validate_m3_kinematics.sh` | `All M3 checks passed.` (geometry from URDF, FK/IK unit tests, refusals) |
| 18 | M3 runtime (starts Gazebo with control and the ground-truth pose bridge) | `./scripts/validate_m3_kinematics.sh --runtime` | `Single-leg FK verified ...`, `Single-leg IK verified ...`, `All M3 checks passed.` |
| 19 | M3 manual | `ros2 launch spiderx_bringup fortress_posture_hold.launch.py`, then `ros2 run spiderx_controller validate_leg_kinematics` | See [FK guide](M3_FORWARD_KINEMATICS_GUIDE.md) and [IK guide](M3_INVERSE_KINEMATICS_GUIDE.md) |

## Blocked stacks (optional, expected to wait)

These launch files load their parameters but cannot do their job yet (see `STATUS.md`). Run them
only to check that the configuration loads. Each prints a `BLOCKED:` message at start.

| Command | Expected behaviour today |
|---|---|
| `ros2 launch spiderx_mapping slam.launch.py` | Nodes start; SLAM waits for `odom` TF |
| `ros2 launch spiderx_localization local_localization.launch.py` | EKF starts; no inputs |
| `ros2 launch spiderx_localization global_localization.launch.py map:=<map.yaml>` | Map server and AMCL activate; no pose without odometry |
| `ros2 launch spiderx_navigation navigation.launch.py` | Plugins load; activation stops at `frame "odom" does not exist` |

## What the cloud validation covered

These layers were run on a cloud VM with Gazebo Fortress 6.16, ros_gz 0.244.20, Nav2, SLAM
Toolbox and robot_localization from RoboStack, with no GPU, using Xvfb and Mesa:

- **Layers 1–10**, including the blocked-stack checks.
- **Layer 1 build:** checked with Ubuntu 22.04's setuptools 59.6. Newer setuptools (≥ 80) breaks `colcon build --symlink-install` for `ament_python` packages; that is a known upstream incompatibility, not a SpiderX issue.
- **Layer 3 unit tests:** run with `PYTEST_DISABLE_PLUGIN_AUTOLOAD=1`, because that environment's pytest 9 cannot load Humble's `launch_testing` plugin. Ubuntu's pytest 6.2 does not have this problem.

**Local verification:** the owner ran layers 1–5, 7–8 and 11–13 on Ubuntu 22.04 (M1 branch), and later layers 1–3 and 11–16 (M2 branch). All passed.
