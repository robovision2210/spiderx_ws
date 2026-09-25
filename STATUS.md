# SpiderX Feature Status

Last updated on branch `claude/spiderx-m2-posture-hold` (M2, stacked on M1 / PR #6).

**M2 was verified locally** by the owner on Ubuntu 22.04: every step of the M2 test sequence passed, including `run_posture_hold_test.py`, `validate_m2_posture.sh --runtime` and the M1 and passive runtime regressions.

**M1 was verified locally** by the owner on Ubuntu 22.04: all 26 steps of the M1 test sequence passed, including `validate_m1_control.sh --runtime` and `validate_fortress.sh --runtime`.

The architecture branch was verified **locally** by the owner: 8 packages built, `validate_fortress.sh` and `--runtime` passed, 6 `colcon test` results, `read_lidar` and `read_joint_states` working.

| Status | Meaning |
|---|---|
| ✅ **Verified** | Implemented and tested. "Local" means on the owner's Ubuntu 22.04 PC; "cloud" means on Fortress 6.16 in a headless cloud VM |
| 🟡 **Configured but blocked** | Configuration and launch files exist and load in the real ROS nodes, but the feature cannot work until a missing prerequisite exists |
| ⚪ **Future work** | Not implemented |

## Simulation and description

| Feature | Status | Evidence / blocker |
|---|---|---|
| Build (`colcon build --symlink-install`), 8 packages | ✅ Verified (local + cloud) | 8 packages; locally on the owner's Ubuntu 22.04 PC (M1 and M2 branches) |
| Gazebo Fortress simulation (`spiderx_bringup fortress.launch.py`) | ✅ Verified (local + cloud) | Robot spawns in `spiderx_fortress.sdf` |
| URDF / xacro, audited CAD model | ✅ Verified | `docs/SPIDERX_URDF_AUDIT.md`, `check_urdf`, sdformat |
| TF (`/tf`, `/tf_static`, `robot_state_publisher`) | ✅ Verified (local + cloud) | `base_link → lidar_link` = (0.051, −0.045, 0.141) |
| Simulated 2D lidar → `/scan` (`lidar_link`) | ✅ Verified (local + cloud) | Wall ranges within ±8 mm of the world geometry |
| `/joint_states` from Gazebo (12 joints) | ✅ Verified (local + cloud) | `validate_fortress.sh --runtime` |
| `/clock` | ✅ Verified (local + cloud) | |
| RViz view (`rviz:=true`) | ✅ Verified (cloud) | Global Status OK |
| `validate_fortress.sh` static checks | ✅ Verified (local: previous version; cloud: current version) | |
| `validate_fortress.sh --runtime` | ✅ Verified (local + cloud) | Passive-path regression during M1 |
| `spiderx_scripts read_lidar`, `read_joint_states` | ✅ Verified (cloud, against the live simulation) | |
| Controller config vs URDF (`validate_controller_config`) | ✅ Verified (cloud) | Plus 3 unit tests |
| Passive physics (legs fold to their joint limits, stable rest) | ✅ Verified (local + cloud) | Expected: no actuation |
| Real-time factor on the owner's PC ≈ 21–25 % | ⚠️ Open question | Check the GPU renderer (`glxinfo -B`) |
| `headless:=true` (EGL) | ⚪ Untested | The cloud ogre-next build lacks EGL |

## Control and locomotion

| Feature | Status | Evidence / blocker |
|---|---|---|
| Leg / joint groups, chain order, soft limits | ✅ Verified against the URDF | `spiderx_controller/config/spiderx_legs.yaml` |
| Named pose `cad_neutral` | ✅ Reached by joint position control (local + cloud) | 12-joint trajectory, max error 0.0000 rad (`test_neutral_pose.py`). **Not** a standing controller |
| M2 CAD neutral posture hold (simulation only) | ✅ **Simulation posture hold verified** (local + cloud) | `run_posture_hold_test.py`: 10 s hold (simulation time); 3 runs; body height min 0.05448 m; \|roll\|, \|pitch\| ≤ 0.00022 rad; max joint error ≤ 0.00028 rad. Idealised by the 100 N·m placeholder effort; foot contact **not measured**. [M2 results](docs/M2_TEST_RESULTS.md), [limitations](docs/M2_SIMULATION_LIMITATIONS.md). **Not** balance, walking, IK or hardware validation |
| ros2_control controllers (`joint_state_broadcaster`, `leg_trajectory_controller`) | ✅ Verified active (local + cloud) | `fortress_control.launch.py`; [M1 results](docs/M1_TEST_RESULTS.md) |
| Joint position control in Fortress (`gz_ros2_control` 0.7.x, 12 joints) | ✅ Verified (local + cloud) | `lf_hip` → +0.2 rad and back, error 0.0000; invalid joints and out-of-limit targets refused. Tracking is idealised by the 100 N·m placeholder effort limit |
| Balance / standing controller (body feedback) | ⚪ Future work | Not scheduled. M1/M2 only hold joint angles; there is no balance feedback, IMU or disturbance test |
| Forward / inverse kinematics | ⚪ Future work | M3 |
| Gait generator | ⚪ Future work | M4 |
| `/cmd_vel` → gait bridge | ⚪ Future work | M5. Nothing consumes `/cmd_vel` today |
| Odometry (`/spiderx/leg_odometry`, `/odom`) | ⚪ Future work | M6. **Not faked** |

## Mapping, localization, navigation

| Feature | Status | Evidence / blocker |
|---|---|---|
| SLAM Toolbox (`spiderx_mapping slam.launch.py`) | 🟡 Configured but blocked | Nodes start and parameters load (cloud). Needs `odom → dummy_link` |
| EKF (`spiderx_localization local_localization.launch.py`) | 🟡 Configured but blocked | Its inputs `/spiderx/leg_odometry` and `/imu/data` do not exist |
| AMCL + map server (`global_localization.launch.py`) | 🟡 Configured but blocked | Configures and activates with a test map (cloud). Needs odometry and a real map |
| Nav2 (`spiderx_navigation navigation.launch.py`) | 🟡 Configured but blocked | All plugins are created (cloud); it stops at "frame odom does not exist". Also needs a `/cmd_vel` consumer and a REP-103 base frame |
| Autonomous navigation | ⚪ Blocked until locomotion and odometry exist | |

## Hardware

| Feature | Status | Evidence / blocker |
|---|---|---|
| RPLidar A1 launch (`spiderx_firmware lidar.launch.py`, `real_robot.launch.py`) | 🟡 Configured | Parses; the driver fails without a device, as expected. Not yet run with the real sensor |
| Actuator (servo) interface | ⚪ Future work | Template only: `spiderx_firmware/config/actuator_interface_template.yaml` |
| IMU | ⚪ Future work | Template only. No `imu_link` in the URDF |
| Embedded computer setup | ⚪ Future work | `docs/SPIDERX_HARDWARE_INTERFACE.md` |
| Microcontroller firmware | ⚪ Future work | None exists in this repository |
