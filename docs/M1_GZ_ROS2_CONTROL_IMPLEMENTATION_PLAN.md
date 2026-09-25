# M1 Implementation Plan: joint position control with gz_ros2_control

Branch `claude/spiderx-m1-joint-control`, started from `claude/spiderx-mechaprime-architecture`
(validated locally: 8 packages, runtime checks, 6 tests). This plan was written **before** any
implementation change.

## 1. Audit of the starting point

| Item | Current state |
|---|---|
| Robot description | `spiderx.urdf.xacro` with `sim_backend:=classic\|fortress\|none`. The Fortress extras are in `spiderx_fortress.gazebo.xacro`: `gpu_lidar`, the Gazebo `JointStatePublisher` system, and contact parameters |
| Existing ros2_control tags | Only legacy Classic ones: `spiderx.trans` (`gazebo_ros2_control/GazeboSystem`) and `spiderx.gazebo` (`libgazebo_ros2_control.so`). They are included **only** for `sim_backend:=classic`, and stay isolated there |
| Fortress launch | `spiderx_description/launch/fortress.launch.py` (world, robot_state_publisher, `ros_gz_sim create`, bridge for `/clock`, `/scan`, `/joint_states`), included by `spiderx_bringup/launch/fortress.launch.py` |
| `/joint_states` source today | Gazebo `JointStatePublisher` → `/spiderx/joint_states` → bridge → `/joint_states` |
| Controller package | `spiderx_controller`: legs config (validated against the URDF), `cad_neutral` pose, `spiderx_ros2_controllers.yaml` scaffold (JointTrajectoryController, not loaded), `validate_controller_config` |
| Tests / validation | `validate_fortress.sh` (static + `--runtime`), 3 controller tests, 2 scripts tests |

## 2. Packages and plugin names (Humble + Fortress)

These were confirmed in the installed Humble environment (RoboStack builds of the same packages
that Humble apt ships). The `gz_ros2_control` build links `libignition-gazebo6`, which is Fortress.

| Item | Exact name |
|---|---|
| apt package | `ros-humble-gz-ros2-control` (0.7.x; 0.7.15 in the cloud) |
| Gazebo system plugin | `filename="gz_ros2_control-system"` (library `libgz_ros2_control-system.so`), `name="gz_ros2_control::GazeboSimROS2ControlPlugin"` |
| ros2_control hardware plugin | `gz_ros2_control/GazeboSimSystem` |
| Controllers | `joint_state_broadcaster/JointStateBroadcaster`, `joint_trajectory_controller/JointTrajectoryController` |
| Other apt packages | `ros-humble-ros2-control`, `ros-humble-ros2-controllers`, `ros-humble-controller-manager`, `ros-humble-joint-state-broadcaster`, `ros-humble-joint-trajectory-controller`, `ros-humble-control-msgs`, `ros-humble-ros2controlcli` |

The Humble 0.7.x package also ships the `ign_ros2_control*` names as compatibility aliases. They are
**not** used here. Jazzy/Rolling names (for example `gz_ros2_control::GazeboSimROS2ControlPlugin`
loaded from Harmonic) do not apply.

`forward_command_controller` is **not** used. One JointTrajectoryController handles both the
single-joint and the 12-joint tests (see §5), so a second controller would only compete for the
same command interfaces.

## 3. Controlled joints (exact URDF names) and interfaces

| Leg | Hip | Thigh | Foot (knee) |
|---|---|---|---|
| front_left | `lf_hip` | `lf_thigh_joint` | `lf_foot_joint` |
| front_right | `rf_hip` | `rf_thigh_joint` | `rf_foot_joint` |
| rear_left | `lr_hip` | `lr_thigh_joint` | `lr_foot_joint` |
| rear_right | `rr_hip` | `rr_thigh_joint` | `rr_foot_joint` |

Interfaces, identical for all 12 joints:

- command interface: `position`
- state interfaces: `position` and `velocity`

The URDF limits (lower / upper, rad) are copied unchanged from the URDF:

| Joint | Lower | Upper | Joint | Lower | Upper |
|---|---|---|---|---|---|
| lf_hip | −0.610865 | 0.698132 | lr_hip | −0.698132 | 0.610865 |
| rf_hip | −0.698132 | 0.610865 | rr_hip | −0.610865 | 0.698132 |
| lf_thigh_joint | −0.610865 | 0.785398 | lr_thigh_joint | −0.610865 | 0.785398 |
| rf_thigh_joint | −0.785398 | 0.610865 | rr_thigh_joint | −0.785398 | 0.610865 |
| lf_foot_joint | −0.698132 | 0.436332 | lr_foot_joint | −0.436332 | 0.698132 |
| rf_foot_joint | −0.436332 | 0.698132 | rr_foot_joint | −0.436332 | 0.698132 |

No CAD geometry, joint axis, origin, limit, mass or inertia is changed.

## 4. Design

1. **New `urdf/spiderx_fortress_control.xacro`.** It holds the `<ros2_control>` block for the 12 joints and the `gz_ros2_control-system` plugin. It is included only when `sim_backend:=fortress enable_control:=true`.
2. **New xacro args.**
   - `enable_control` (default `false`) and `controllers_file` (the path to the controller YAML, passed in by the launch file).
   - The path is passed as an argument so that `spiderx_description` does not depend on `spiderx_controller`, which would create a package cycle.
   - `enable_control:=true` without `controllers_file`, or with a backend other than fortress, fails with `xacro.fatal`.
3. **One `/joint_states` publisher.**
   - With control enabled, the Gazebo `JointStatePublisher` system is left out, and the bridge uses `config/fortress_bridge_control.yaml` (`/clock` and `/scan` only).
   - `/joint_states` then comes from `joint_state_broadcaster`, reading the same Gazebo joint positions through `gz_ros2_control`.
4. **`spiderx_description/launch/fortress.launch.py` gains `enable_control` (default `false`).** With the default, the robot description, bridge config and processes are **identical** to the verified passive path. This is checked by comparing the expanded URDF byte for byte.
5. **New `spiderx_bringup/launch/fortress_control.launch.py`.**
   - Arguments: `enable_control` (default `true`), `headless`, `rviz`.
   - It includes the Fortress launch with control, then `spiderx_controller/launch/controller.launch.py`.
   - That file spawns `joint_state_broadcaster` first, and starts `leg_trajectory_controller` only when the first spawner **exits**, using an `OnProcessExit` event handler.
   - Each spawner waits for the `controller_manager` services itself (`--controller-manager-timeout`), so no fixed sleeps are needed.
   - It never includes `spiderx_firmware`.
6. **`ros2 launch spiderx_bringup fortress.launch.py` stays passive** and unchanged.

## 5. Test tools and safety rules

Both tools live in `spiderx_controller`. They share `spiderx_controller/joint_safety.py`, which
loads the validated joint list and limits from `config/spiderx_legs.yaml`.

**`test_one_joint.py --joint <name> [--target 0.2] [--duration 3.0] [--margin 0.05] [--tolerance 0.05] [--return-to-initial]`**

- **Before commanding:** it prints the controller list from `/controller_manager/list_controllers`.
- **Rejects:**
  - joint names that are not among the 12 validated joints;
  - targets outside `[lower + margin, upper − margin]`. With the default margin this range equals the soft limits in `spiderx_legs.yaml`.
- **Sends:** one `FollowJointTrajectory` goal to `leg_trajectory_controller`. The selected joint goes to the target over `--duration` seconds; the other 11 joints are held at their measured positions.
- **Reports:** the requested joint and target, the position before and after (from `/joint_states`), the absolute error, and PASS/FAIL against `--tolerance`.
- **Stops cleanly:** it sends exactly one goal and never loops. `--return-to-initial` sends one more goal back to the starting position. Ctrl-C while waiting sends that return goal and exits.

**`test_neutral_pose.py [--pose cad_neutral] [--duration 6.0] [--tolerance 0.05]`**

- Uses only the documented `cad_neutral` pose (all 0 rad).
- Validates all 12 targets against the limits before commanding.
- Sends one slow trajectory from the current positions.
- Logs the per-joint final error.
- Prints **"joint-position test only – not a standing controller"**.

**Why the chosen test joint and angle are safe (all in simulation):**

| Choice | Reason |
|---|---|
| Default single-joint test: `lf_hip` → **+0.20 rad** over 3 s | A hip abduction joint moves one leg sideways and does not fold the body. 0.20 rad (11.5°) lies inside the soft range [−0.5609, 0.6481], 0.448 rad from the nearest soft limit. The mean speed of 0.067 rad/s is far below the 0.5 rad/s placeholder cap. 0.20 rad also lies inside every joint's soft range, so the same default is valid for any selected joint |
| Neutral pose `cad_neutral` = 0 rad over 6 s | Zero lies inside every joint's limits. It is the CAD configuration in which all four feet share one plane (audit §3). The largest move, the knees from their passive fold at ±0.436 rad, gives a mean of 0.073 rad/s |

## 6. Where each value comes from

| Value | Source |
|---|---|
| Joint names, axes, lower/upper limits | **URDF** (checked by `validate_controller_config`) |
| Soft-limit margin 0.05 rad | Project safety choice (existing `spiderx_legs.yaml`) |
| Test targets (0.2 rad, `cad_neutral`), durations (3 s, 6 s), error tolerance 0.05 rad | **Simulation-only test parameters** |
| Controller update rate 100 Hz, JTC settings | Simulation-only tuning |
| gz_ros2_control position tracking (built-in proportional gain) | Package default, unchanged |
| URDF effort 100 N·m, velocity 100 rad/s | **Exporter placeholders**, unchanged. Real servo limits are M9 |

## 7. Acceptance criteria (M1)

1. `ros2 launch spiderx_bringup fortress_control.launch.py` spawns SpiderX in Fortress.
2. `/controller_manager` is available.
3. `joint_state_broadcaster` and `leg_trajectory_controller` report **active**.
4. `/joint_states` has exactly **one** publisher, carrying 12 joints.
5. `test_one_joint.py --joint lf_hip --target 0.2` gives |error| ≤ 0.05 rad (PASS).
6. The invalid joint name and the out-of-limit target are rejected **before** anything is sent (unit tests and a live check).
7. `test_neutral_pose.py` brings all 12 joints within 0.05 rad of `cad_neutral`.
8. The passive `fortress.launch.py` path still passes `validate_fortress.sh --runtime`.
9. **No claim** of standing, balance or walking is made.

## 8. Rollback

- **Runtime:** use `ros2 launch spiderx_bringup fortress.launch.py`, which is passive and the default, or pass `fortress_control.launch.py enable_control:=false`.
- **Code:** `git checkout claude/spiderx-mechaprime-architecture && rm -rf build install log && colcon build --symlink-install`. M1 is contained in new files plus additive, default-off arguments.
