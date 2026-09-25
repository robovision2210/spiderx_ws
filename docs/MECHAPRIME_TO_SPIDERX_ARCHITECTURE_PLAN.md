# Mechaprime → SpiderX Architecture Plan

Reference: [`robovision2210/mechaprime_ws`](https://github.com/robovision2210/mechaprime_ws) at commit
`041d3f8`. It was inspected read-only and was not modified.

Baseline: SpiderX branch `claude/stoic-shannon-ur2mes`, which was validated on Ubuntu 22.04 with
ROS 2 Humble and Gazebo Fortress.

This document was written **before** any SpiderX implementation change on this branch.

---

## 1. Mechaprime package tree (as inspected)

```
mechaprime_ws/
├── README.md                    # badges, demo GIFs, screenshot tables, features, usage per subsystem
├── media/                       # GIF/MP4 demos + PNG screenshots referenced by README
└── src/
    ├── mechaprime_description/  # urdf/ (xacro split: base, gazebo, ros2control, sensor), meshes/,
    │                            # launch/{display,gazebo}.launch.py, rviz/, worlds/, models/
    ├── mechaprime_bringup/      # launch/{simulated_robot,real_robot}.launch.py (top-level entry points)
    ├── mechaprime_controller/   # config/{mechaprime_controllers,joy_*,twist_mux_*}.yaml,
    │                            # launch/{controller,joystick}.launch.py  (DiffDriveController + twist_mux)
    ├── mechaprime_mapping/      # config/slam_toolbox.yaml, launch/slam.launch.py, maps/, rviz/
    ├── mechaprime_localization/ # config/amcl.yaml, launch/global_localization.launch.py, rviz/
    ├── mechaprime_navigation/   # config/ (one YAML per Nav2 server), behavior_tree/, launch/,
    │                            # python waypoint_following node
    ├── mechaprime_firmware/     # C++ ros2_control SystemInterface over serial (ESP32),
    │                            # launch/hardware_interface.launch.py
    ├── mechaprime_scripts/      # ament_python: sensor readers + autonomy nodes (console_scripts)
    └── rplidar_ros-<hash>/      # vendored Slamtec driver
```

Conventions observed:

- Every package is named `mechaprime_*`.
- Each package keeps its own `launch/`, `config/` and `rviz/` folders.
- `bringup` only composes other packages' launch files.
- Nav2 parameters are split into one YAML file per server.
- Python nodes live in a separate `*_scripts` package and are exposed as `console_scripts`.
- The README is the showcase: badges, demo GIFs, screenshot grids, a features list, a spec table, a topic table, and per-subsystem commands.

Issues observed in Mechaprime that are **not** carried over:

- `mechaprime_mapping/CMakeLists.txt` installs its files only inside `if(BUILD_TESTING)`.
- `mechaprime_firmware/CMakeLists.txt` never builds its `.cpp`, and its plugin XML names "mechabot".
- Every `package.xml` still has TODO descriptions and licenses.
- A stray `.swp` file is committed.
- `simulated_robot.launch.py` starts localization and Nav2 unconditionally.

## 2. SpiderX package tree (current, validated baseline)

```
spiderx_ws/
├── README.md, MIGRATION_FORTRESS.md
├── docs/            # URDF audit, inertia audit, beginner Ubuntu test guide
├── media/           # cloud validation captures + media/README.md capture list
├── scripts/validate_fortress.sh
└── src/
    ├── spiderx_description/  # CAD xacro (sim_backend arg), meshes, Fortress world,
    │                         # launch/{fortress,display}.launch.py, config/{fortress_bridge,controllers}.yaml
    └── spiderx_bringup/      # launch/{fortress,hardware_lidar}.launch.py, rviz/
```

## 3. Target SpiderX package tree

```
spiderx_ws/
├── README.md                 # Mechaprime-style showcase, verified claims only
├── STATUS.md                 # Verified / Configured but blocked / Future work
├── docs/                     # architecture, roadmap, hardware, sim-vs-hw, testing, audits
├── media/                    # captures + capture list
├── scripts/validate_fortress.sh   # static checks + optional --runtime topic checks
└── src/
    ├── spiderx_description/  # unchanged source of truth (CAD URDF, meshes, Fortress world)
    ├── spiderx_bringup/      # fortress.launch.py (sim entry), real_robot.launch.py (hardware entry)
    ├── spiderx_controller/   # leg/joint groups, neutral pose, safety limits, ros2_control scaffold,
    │                         # config validator (no motion)
    ├── spiderx_localization/ # EKF + AMCL configs and launch (configured, blocked)
    ├── spiderx_mapping/      # SLAM Toolbox config and launch (configured, blocked)
    ├── spiderx_navigation/   # Nav2 configs, BT, launch (configured, blocked)
    ├── spiderx_firmware/     # real-hardware layer: RPLidar launch + actuator/IMU interface templates
    └── spiderx_scripts/      # ament_python tools that work today: read_lidar, read_joint_states
```

**Vendor packages.** `rplidar_ros` exists as a Humble binary (`ros-humble-rplidar-ros`), so it is
declared as a dependency instead of being vendored. This avoids Mechaprime's hash-named copy.

## 4. What transfers directly

| From Mechaprime | To SpiderX |
|---|---|
| Package split: description / bringup / controller / mapping / localization / navigation / firmware / scripts | Same split with `spiderx_*` names |
| Per-package `launch/`, `config/`, `rviz/` folders; bringup only composes launches | Same |
| `real_robot.launch.py` as the hardware entry point, separate from simulation | `spiderx_bringup/launch/real_robot.launch.py` |
| Nav2 parameters split one YAML per server + custom BT XML | Same file layout, SpiderX values |
| `slam.launch.py` (SLAM Toolbox + map saver + lifecycle manager), `global_localization.launch.py` (map_server + AMCL) | Same structure, SpiderX frames and topics |
| `*_scripts` as `ament_python` with `console_scripts` | `spiderx_scripts` |
| README showcase layout: badges, demos, screenshots, features, specs, topics, commands, author | Same layout; features only where verified |
| `ros_gz_sim` / `ros_gz_bridge` Fortress simulation pattern | Already present (validated) |

## 5. What must be adapted for a quadruped

| Mechaprime concept | SpiderX adaptation |
|---|---|
| 2 wheel joints, velocity command | 12 revolute leg joints (4 legs × hip / thigh / foot), position targets |
| `diff_drive_controller` turns `/cmd_vel` into wheel speeds | **A gait generator plus leg IK** must turn `/cmd_vel` into foot trajectories, then joint targets (future) |
| Wheel odometry from the drive controller | Leg odometry and/or IMU fusion (future). Nothing publishes `/odom` today |
| `base_footprint` frame (REP-103, x forward) | SpiderX has `dummy_link` → `base_link`, with **+y forward** (CAD). The configs use the real frames and document the REP-103 blocker |
| Rectangular wheeled footprint | Footprint from measured leg extents in `base_link` coordinates |
| `DifferentialMotionModel` (AMCL) | `OmniMotionModel`: a legged gait can sidestep. This is an assumption, to be confirmed by the gait |
| ESP32 serial `SystemInterface` for wheel motors | Documented **template** for a 12-servo actuator interface. No firmware exists yet |
| twist_mux / joystick teleop | Deferred: there is no locomotion to teleoperate |

## 6. What must not be copied

- `diff_drive_controller`, `wheel_controller`, and wheel radius or separation.
- Wheel velocity or acceleration limits and `twist_mux` priorities.
- Wheel odometry assumptions, `enable_odom_tf`, and the `base_footprint`, `lidar_sensor_link` and wheel link frames.
- Any fake `/odom` source.
- The ESP32 serial port defaults (`/dev/ttyUSB1`) and the `MechaprimeInterface` C++ code.
- Mechaprime worlds, AWS models, maps, meshes, spawn poses and dimensions.
- Nav2 velocity values and footprints.
- Launching SLAM, AMCL or Nav2 from the simulation entry point.
- QR, docking, battery and maze scripts: their behaviour depends on a driving robot.

## 7. Package-by-package migration plan

| Package | Action | Verification |
|---|---|---|
| `spiderx_description` | **Unchanged.** It remains the source of truth for URDF, meshes, the Fortress world and the bridge | `validate_fortress.sh` static + runtime |
| `spiderx_bringup` | Keep `fortress.launch.py` untouched. Rename `hardware_lidar.launch.py` → `real_robot.launch.py`, which composes `spiderx_firmware/lidar.launch.py` | Launch parsing; sim launch must not start any serial node |
| `spiderx_controller` (new) | Joint-group YAML (from the URDF), neutral pose, soft limits, and a `ros2_control` controller scaffold (not loaded). Python validator checks the configs against the URDF | Validator run in CI-style check |
| `spiderx_localization` (new) | `ekf.yaml` (future `/odom` + `/imu`), `amcl.yaml`, `local_localization.launch.py`, `global_localization.launch.py` | Launch parsing only. **Blocked**: no odometry |
| `spiderx_mapping` (new) | `slam_toolbox.yaml` using `/scan` and `lidar_link`/`base_link`; `slam.launch.py`; `maps/` | Launch parsing only. **Blocked**: no odometry or motion |
| `spiderx_navigation` (new) | Nav2 YAML per server with the measured footprint and placeholder legged velocities; BT; `navigation.launch.py` | Launch parsing only. **Blocked**: no `/cmd_vel` consumer |
| `spiderx_firmware` (new) | `lidar.launch.py` (real RPLidar A1); actuator, IMU and embedded-computer interface templates (labelled placeholders) | Launch parsing; driver run on hardware only |
| `spiderx_scripts` (new) | `read_lidar` and `read_joint_states`: diagnostics that work against today's simulation | Runtime check in simulation |

## 8. Why a quadruped needs gait control and leg kinematics, not diff-drive

A differential-drive robot has two wheels in continuous ground contact. Its motion is fully
defined by two wheel speeds, so `diff_drive_controller` can map `/cmd_vel (v, ω)` to wheel
velocities in closed form, and integrate those same wheel speeds into odometry.

SpiderX has **no wheels**. It moves by lifting and placing four feet, and every step requires:

1. **A gait schedule.** It decides which legs are in stance (supporting the body) and which are in swing, and for how long. Examples are a walk, which keeps 3 feet down, and a trot, which moves diagonal pairs.
2. **Foot trajectories.** Swing feet follow a lift–move–place path. Stance feet move backwards relative to the body to push it forwards.
3. **Inverse kinematics.** Each foot target in `base_link` is converted into the three joint angles of that leg (hip abduction about ±y, thigh and knee about ±x). The per-joint axis signs are documented in `docs/SPIDERX_URDF_AUDIT.md` §5.
4. **Balance.** The centre of mass must stay inside the support polygon of the stance feet.
5. **Odometry.** Body motion is estimated from stance-foot kinematics and/or an IMU, not from wheel rotation.

None of this can be expressed as wheel radius and separation. The correct build order is:

**joint control → stand → FK/IK → gait → `/cmd_vel` bridge → odometry → SLAM/Nav2.**

See `docs/SPIDERX_DEVELOPMENT_ROADMAP.md`.
