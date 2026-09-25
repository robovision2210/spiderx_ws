# SpiderX System Architecture

The status of every component is tracked in [`../STATUS.md`](../STATUS.md).

## 1. Packages

| Package | Build type | Role | Launch files |
|---|---|---|---|
| `spiderx_description` | ament_cmake | CAD URDF/xacro (source of truth), meshes, Fortress world, Gazebo↔ROS bridge config, legacy Classic config | `fortress.launch.py`, `display.launch.py` |
| `spiderx_bringup` | ament_cmake | Top-level entry points only; composes other packages | `fortress.launch.py` (simulation), `real_robot.launch.py` (hardware) |
| `spiderx_controller` | ament_cmake + python | Leg/joint groups, named poses, soft limits, ros2_control scaffold, URDF consistency checker | none yet (nothing to run until M1) |
| `spiderx_mapping` | ament_cmake | SLAM Toolbox config and launch | `slam.launch.py` (blocked) |
| `spiderx_localization` | ament_cmake | EKF and AMCL configs and launches | `local_localization.launch.py`, `global_localization.launch.py` (blocked) |
| `spiderx_navigation` | ament_cmake | Nav2 per-server configs, behavior tree, launch | `navigation.launch.py` (blocked) |
| `spiderx_firmware` | ament_cmake | Real-hardware layer: RPLidar launch; actuator and IMU templates | `lidar.launch.py` (hardware only) |
| `spiderx_scripts` | ament_python | Diagnostics: `read_lidar`, `read_joint_states` | `ros2 run` |

Dependency direction:

- `bringup` → `description` + `firmware`
- `scripts` → `controller` (it reads the leg configuration)
- Mapping, localization and navigation are independent and are never started by `bringup`.

## 2. Simulation data flow (verified)

```
 spiderx.urdf.xacro ──xacro sim_backend:=fortress──► robot_state_publisher ──► /robot_description
                                                            ▲         │
                                                            │         └──► /tf, /tf_static
                                                  /joint_states
                                                            │
 ┌──────── Gazebo Fortress (ign gazebo via ros_gz_sim) ────────┐      ┌──── ros_gz_bridge ────┐
 │ world: spiderx_fortress.sdf  (Physics, Sensors/ogre2, ...)  │      │ fortress_bridge.yaml  │
 │ model: spiderx (spawned by ros_gz_sim create)               │─gz──►│ /clock                │──► ROS 2
 │   gpu_lidar on lidar_link   → /spiderx/scan                 │      │ /spiderx/scan → /scan │
 │   JointStatePublisher       → /spiderx/joint_states         │      │ → /joint_states       │
 └─────────────────────────────────────────────────────────────┘      └───────────────────────┘
```

## 3. TF tree

```
[odom] ···(future: EKF / leg odometry publishes odom → dummy_link)···
  └─ dummy_link                  URDF root (identical pose to base_link)
      └─ base_link               CAD body frame: +y = front, +x = right, +z = up (not REP-103)
          ├─ lidar_link          (0.051, −0.045, 0.141), yaw +90° → +x = robot front
          ├─ top_1
          └─ b_s{1..4}_1 → hip → holders → thigh → foot   (4 legs, 3 revolute joints each)
```

- `map → odom` would come from SLAM or AMCL, and `odom → dummy_link` from odometry. **Neither exists yet.**
- The odometry must target `dummy_link`, the URDF root. Targeting `base_link` would give it two parents.

## 4. Topics

| Topic | Type | Producer | Status |
|---|---|---|---|
| `/scan` | `sensor_msgs/LaserScan` | Gazebo `gpu_lidar` (sim) / RPLidar driver (hardware) | Verified in sim |
| `/joint_states` | `sensor_msgs/JointState` | Gazebo `JointStatePublisher` | Verified in sim |
| `/clock` | `rosgraph_msgs/Clock` | Gazebo | Verified in sim |
| `/tf`, `/tf_static`, `/robot_description` | | `robot_state_publisher` | Verified |
| `/cmd_vel` | `geometry_msgs/Twist` | Nav2 / teleop (future) | **No consumer** |
| `/spiderx/leg_odometry` | `nav_msgs/Odometry` | future gait / leg odometry | Does not exist |
| `/imu/data` | `sensor_msgs/Imu` | future IMU | Does not exist |
| `/odometry/filtered` | `nav_msgs/Odometry` | EKF (future) | Blocked |

## 5. Future control architecture (not implemented)

```
/cmd_vel ─► cmd_vel→gait bridge ─► gait generator ─► foot targets (base_link)
                                                          │
                                           leg IK (per leg, sign table)
                                                          │
                                  joint targets ─► leg_trajectory_controller (ros2_control)
                                                          │
                          gz_ros2_control (sim)  /  SpiderX actuator interface (hardware)
                                                          │
                                   /joint_states ─► leg odometry ─► EKF ─► odom → dummy_link
```

The joint names, groups, limits and the controller scaffold for the lower layers already exist in
`spiderx_controller`. See [`SPIDERX_DEVELOPMENT_ROADMAP.md`](SPIDERX_DEVELOPMENT_ROADMAP.md).
