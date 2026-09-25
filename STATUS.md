# SpiderX Feature Status

Last updated on branch `claude/spiderx-mechaprime-architecture`.

| Status | Meaning |
|---|---|
| ✅ **Verified** | Implemented and tested. "Local" means on the owner's Ubuntu 22.04 PC; "cloud" means on Fortress 6.16 in a headless cloud VM |
| 🟡 **Configured but blocked** | Configuration and launch files exist and load in the real ROS nodes, but the feature cannot work until a missing prerequisite exists |
| ⚪ **Future work** | Not implemented |

## Simulation and description

| Feature | Status | Evidence / blocker |
|---|---|---|
| Build (`colcon build --symlink-install`), 8 packages | ✅ Verified (cloud; local for the 2 original packages) | Cloud: all 8 packages with Ubuntu's setuptools 59.6 |
| Gazebo Fortress simulation (`spiderx_bringup fortress.launch.py`) | ✅ Verified (local + cloud) | Robot spawns in `spiderx_fortress.sdf` |
| URDF / xacro, audited CAD model | ✅ Verified | `docs/SPIDERX_URDF_AUDIT.md`, `check_urdf`, sdformat |
| TF (`/tf`, `/tf_static`, `robot_state_publisher`) | ✅ Verified (local + cloud) | `base_link → lidar_link` = (0.051, −0.045, 0.141) |
| Simulated 2D lidar → `/scan` (`lidar_link`) | ✅ Verified (local + cloud) | Wall ranges within ±8 mm of the world geometry |
| `/joint_states` from Gazebo (12 joints) | ✅ Verified (local + cloud) | `validate_fortress.sh --runtime` |
| `/clock` | ✅ Verified (local + cloud) | |
| RViz view (`rviz:=true`) | ✅ Verified (cloud) | Global Status OK |
| `validate_fortress.sh` static checks | ✅ Verified (local: previous version; cloud: current version) | |
| `validate_fortress.sh --runtime` | ✅ Verified (cloud, GUI under Xvfb) | Needs a local run |
| `spiderx_scripts read_lidar`, `read_joint_states` | ✅ Verified (cloud, against the live simulation) | |
| Controller config vs URDF (`validate_controller_config`) | ✅ Verified (cloud) | Plus 3 unit tests |
| Passive physics (legs fold to their joint limits, stable rest) | ✅ Verified (local + cloud) | Expected: no actuation |
| Real-time factor on the owner's PC ≈ 21–25 % | ⚠️ Open question | Check the GPU renderer (`glxinfo -B`) |
| `headless:=true` (EGL) | ⚪ Untested | The cloud ogre-next build lacks EGL |

## Control and locomotion

| Feature | Status | Evidence / blocker |
|---|---|---|
| Leg / joint groups, chain order, soft limits | ✅ Verified against the URDF | `spiderx_controller/config/spiderx_legs.yaml` |
| Named pose `cad_neutral` | 🟡 Configured (geometric only) | Within limits and feet coplanar at q = 0; never held by a controller |
| ros2_control controllers (`spiderx_ros2_controllers.yaml`) | 🟡 Configured but blocked | The robot description has no Fortress `<ros2_control>` block yet (M1) |
| Joint position control in Fortress (`gz_ros2_control`) | ⚪ Future work | Roadmap M1 |
| Standing controller | ⚪ Future work | M2, needs M1 |
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
