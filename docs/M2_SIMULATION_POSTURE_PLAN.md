# M2 Plan: Simulation-Only CAD Neutral Posture Hold Validation (Gazebo Fortress)

```text
Simulation-only posture hold.
Not dynamic balance control.
Not walking or gait control.
Not inverse kinematics.
Not hardware validation.
Not real-servo torque validation.
Not battery/current validation.
Not proof of real-world stability.
```

This plan was written and committed **before** any M2 implementation or configuration change.

- **Branch:** `claude/spiderx-m2-posture-hold`.
- **Base:** `claude/spiderx-m1-joint-control`, the head of PR #6. On the owner's instruction, M2 is stacked on the unmerged chain PR #4 → PR #5 → PR #6. `main` still holds only the initial commit.

## 0. Audit summary (what exists today)

| Area | Finding | Source |
|---|---|---|
| Robot description | `spiderx.urdf.xacro`. The root is `dummy_link`, joined to `base_link` by the fixed `dummy_joint` with **no origin offset**, so the two frames coincide. `base_link` convention: +y front, +x right, +z up (not REP-103) | URDF, `SPIDERX_URDF_AUDIT.md` §3 |
| Leg joints | 12 revolute joints, 3 per leg. All limits in §3 | URDF `<limit>` |
| Actuator limits | Every joint has `effort="100" velocity="100"`. These are **exporter placeholders** (audit item K4) | URDF |
| Joint dynamics | **No `<dynamics>` tags**: no joint damping or friction in the URDF | URDF |
| Contact | `mu1 = mu2 = 0.2` and `self_collide = false` on 30 CAD links. These are fusion2urdf template defaults (audit P4), not measured | `spiderx_fortress.gazebo.xacro` |
| Mass and inertia | CAD values at steel density (7,850 kg/m³): total **7.29 kg**, almost certainly not the real robot | `SPIDERX_URDF_AUDIT.md` |
| World physics | `max_step_size 0.001` s and `real_time_factor 1.0`. Default Fortress physics plugin (DART, `ignition-physics5-dartsim`). Gravity −9.8 m/s². Flat ground plane | `worlds/spiderx_fortress.sdf` |
| Spawn | `spawn_z = 0.075` m | `spiderx_description/launch/fortress.launch.py` |
| M1 control | `gz_ros2_control` 0.7.15 with `gz_ros2_control/GazeboSimSystem`. Every joint has a position command interface and position + velocity state interfaces. Controllers `joint_state_broadcaster` and `leg_trajectory_controller` (JTC, 12 joints, `allow_partial_joints_goal: false`), `update_rate` 100 Hz. Position commands are applied as velocity commands with `position_proportional_gain` 0.1 (confirmed in the runtime log) | `spiderx_fortress_control.xacro`, `spiderx_ros2_controllers.yaml` |
| M1 safety | `joint_safety.check_pose()` checks exactly the 12 joints, with each target inside the URDF limits pulled in by 0.05 rad | `spiderx_controller/joint_safety.py` |
| Named pose | `cad_neutral`: all 12 joints at 0.0 rad, `status: GEOMETRIC_ONLY` | `spiderx_controller/config/spiderx_poses.yaml` |
| ROS topics (control mode) | `/clock`, `/joint_states` (1 publisher: `joint_state_broadcaster`), `/dynamic_joint_states`, `/leg_trajectory_controller/*`, `/robot_description`, `/scan`, `/tf`, `/tf_static` | Runtime `ros2 topic list` |
| TF | `robot_state_publisher` publishes `dummy_link → …` only. **There is no `world`/`odom` frame in TF**, so TF alone cannot give body height or tilt relative to the ground | Runtime |
| Gazebo transport | `/world/spiderx_fortress/pose/info` (`ignition.msgs.Pose_V`, about 50 Hz), from the world's SceneBroadcaster. Its entry named `spiderx` is the **model pose in the world frame**. Measured at rest in control mode: z = 0.0545 m, orientation ≈ identity | Runtime `ign topic -e` |
| Ground-truth bridge | A `ros_gz_bridge` `Pose_V → tf2_msgs/msg/TFMessage` bridge on this topic works. It must be remapped **away from `/tf`**. Per-pose header stamps are **0**, so samples must be timestamped from `/clock` | Runtime test |
| Contact data | **No contact sensor or contact system** exists in the model or world, and no contact topic is published. Direct foot-contact data is therefore **unavailable** | `ign topic -l` |
| Validation | `validate_fortress.sh [--runtime]`, `validate_m1_control.sh [--runtime]`, `validate_controller_config`; 20 pytest cases (colcon prints 22 including 2 CTest wrappers) | `scripts/`, `spiderx_controller/test` |

## 1. Verified baseline (M0 + M1)

- **M0 (local and cloud):**
  - The passive Fortress simulation spawns SpiderX.
  - `/clock`, `/scan` (on `lidar_link`), `/joint_states` (12 joints) and TF are verified.
  - Passively, the legs fold to their joint limits.
- **M1 (local and cloud):**
  - `fortress_control.launch.py` brings both controllers up **active**.
  - Exactly one `/joint_states` publisher.
  - `lf_hip` reaches +0.2 rad and returns.
  - Unknown joints and out-of-limit targets are refused before anything is sent.
  - An all-joint `cad_neutral` trajectory returns displaced joints to 0.
  - Observation from M1: in control mode the robot rests on its feet at z ≈ 0.054 m, because every joint is held stiffly with the 100 N·m placeholder effort limit.

M2 must keep all of this passing.

## 2. Command path and feedback path

```
run_posture_hold_test.py (rclpy, use_sim_time)
  └─ FollowJointTrajectory goal → /leg_trajectory_controller/follow_joint_trajectory
       └─ JointTrajectoryController (controller_manager inside gz_ros2_control, 100 Hz)
            └─ position command interfaces (12) → GazeboSimSystem → joint velocity commands → DART
Feedback:
  /joint_states                    ← joint_state_broadcaster (12 positions)
  /controller_manager/list_controllers (service) → controller states
  /clock                           ← ros_gz_bridge (simulation time)
  /spiderx/sim/world_poses         ← ros_gz_bridge of /world/spiderx_fortress/pose/info (NEW, M2 only)
```

The one new element is `/spiderx/sim/world_poses`. It is simulation **ground truth** taken from Gazebo, **not** odometry. It is:
- never published on `/tf`;
- used only by the M2 test;
- started only by the new `spiderx_bringup/launch/fortress_posture_hold.launch.py`.

That launch file includes the unchanged `fortress_control.launch.py`. The M1 and passive launch files are not modified.

## 3. The 12 joints (URDF limits, rad)

| Joint | Axis | Lower | Upper | Allowed with 0.05 margin |
|---|---|---|---|---|
| `lf_hip` | [0 1 0] | −0.610865 | 0.698132 | [−0.560865, 0.648132] |
| `lf_thigh_joint` | [−1 0 0] | −0.610865 | 0.785398 | [−0.560865, 0.735398] |
| `lf_foot_joint` | [1 0 0] | −0.698132 | 0.436332 | [−0.648132, 0.386332] |
| `rf_hip` | [0 1 0] | −0.698132 | 0.610865 | [−0.648132, 0.560865] |
| `rf_thigh_joint` | [1 0 0] | −0.785398 | 0.610865 | [−0.735398, 0.560865] |
| `rf_foot_joint` | [−1 0 0] | −0.436332 | 0.698132 | [−0.386332, 0.648132] |
| `lr_hip` | [0 −1 0] | −0.698132 | 0.610865 | [−0.648132, 0.560865] |
| `lr_thigh_joint` | [−1 0 0] | −0.610865 | 0.785398 | [−0.560865, 0.735398] |
| `lr_foot_joint` | [−1 0 0] | −0.436332 | 0.698132 | [−0.386332, 0.648132] |
| `rr_hip` | [0 −1 0] | −0.610865 | 0.698132 | [−0.560865, 0.648132] |
| `rr_thigh_joint` | [1 0 0] | −0.785398 | 0.610865 | [−0.735398, 0.560865] |
| `rr_foot_joint` | [−1 0 0] | −0.436332 | 0.698132 | [−0.386332, 0.648132] |

Every URDF value is also copied in `spiderx_controller/config/spiderx_legs.yaml`, which `validate_controller_config` checks against the URDF.

## 4. CAD neutral pose (the only M2 posture)

- **Source:** `src/spiderx_controller/config/spiderx_poses.yaml`, `poses.cad_neutral`.
- **Values:** all 12 joints at **0.0 rad**, the CAD zero configuration.
- **Evidence** (`SPIDERX_URDF_AUDIT.md` §3):
  - at q = 0, all four feet bottom out at z = −0.0545 m in `base_link`;
  - the whole-robot COM, (0.051, −0.058, 0.070), is inside the support polygon.
- Every target is inside its limit ± 0.05 range: 0 lies in every interval in §3.

**Owner decision.** M2 uses only this existing pose. No other posture will be searched for, tuned, altered, optimised or added. If `cad_neutral` fails the criteria, the result is recorded as **"Simulation posture hold not verified."**, the likely cause is explained with evidence, and work stops there.

## 5. Definitions of the outcome

**"Simulation posture hold verified."** Every criterion below holds in one run:
1. Both M1 controllers are `active` at the start, during the hold (polled) and at the end.
2. `/joint_states` has exactly 1 publisher, and every message carries all 12 joints.
3. The posture passes validation before it is sent: exactly the 12 joints, no duplicates, numeric values, inside URDF limits minus 0.05 rad, and identical to `cad_neutral`.
4. The command is one 12-joint trajectory with duration `command_duration_s`. Its peak joint speed (largest |target − start| ÷ duration) is ≤ `max_joint_velocity_rad_s` (0.5 rad/s, from `spiderx_legs.yaml`).
5. The action server is available, the goal is accepted, and the result is `SUCCESSFUL` (error_code 0) within its timeout.
6. Joint tracking stays within the thresholds for the whole hold window:
   - the largest absolute error over all 12 joints and all samples is ≤ `max_joint_error_rad`;
   - the RMS error over all joints and samples is ≤ `rms_joint_error_rad`.
7. During the hold, the body height stays ≥ `min_body_height_m`, and its range (max − min) is ≤ `max_body_height_range_m`.
8. During the hold, |roll| ≤ `max_abs_roll_rad` and |pitch| ≤ `max_abs_pitch_rad`.
9. The hold lasts at least `hold_duration_s` of **simulation** time. The number of body-pose samples and joint-state samples each meets a minimum (§7).
10. `/clock`, `/joint_states` and `/spiderx/sim/world_poses` are all received, and the world poses include the model `spiderx`.

**"Simulation posture hold not verified."** Any criterion fails, or the test cannot complete. The report names each failed criterion and its measured value. There is no third outcome.

## 6. Values and their provenance

| Value | Category | Where it comes from |
|---|---|---|
| 12 joint names, axes, limits | **URDF** | `spiderx.urdf.xacro` (copied in `spiderx_legs.yaml`, checked) |
| Targets = 0.0 rad | **Existing validated config** | `spiderx_poses.yaml` `cad_neutral` (CAD zero) |
| Feet 0.0545 m below `base_link` at q = 0 | **URDF/CAD-derived** | Audit §3 (mesh geometry) |
| Safety margin 0.05 rad | **Existing validated config** | `spiderx_legs.yaml` `soft_limit_margin_rad`, `joint_safety.DEFAULT_MARGIN_RAD` |
| Max joint speed 0.5 rad/s | **Existing config (simulation placeholder)** | `spiderx_legs.yaml` `motion_constraints` |
| `command_duration_s` = 6.0 | **Existing validated config** | M1 `test_neutral_pose.py` default, verified locally and in the cloud |
| `hold_duration_s` = 10.0 | **Simulation-only choice** | Owner's initial target |
| `max_joint_error_rad` = 0.05 | **Existing validated config** | M1 tracking tolerance |
| `rms_joint_error_rad` = 0.02 | **Simulation-only tuning** | Stricter aggregate bound |
| `min_body_height_m` = 0.045 | **Simulation-only threshold** | 9.5 mm below the CAD foot-plane height of 0.0545 m, and far above the passive folded height of about 0.012 m |
| `max_body_height_range_m` = 0.005 | **Simulation-only threshold** | Detects sagging or bouncing during the hold |
| `max_abs_roll_rad`, `max_abs_pitch_rad` = 0.10 | **Simulation-only threshold** | About 5.7° |
| Effort 100 N·m, velocity 100 rad/s | **Placeholder** | URDF exporter (audit K4) |
| µ = 0.2, no joint damping, steel density | **Placeholder / assumption** | fusion2urdf template and CAD default |
| Physics step 1 ms, DART | **Simulation assumption** | World SDF |
| Every measured height, angle and error | **Measured runtime result** | M2 report JSON |

The thresholds are written **once**, in `m2_simulation_postures.yaml`, before the first M2 runtime run. They will not be changed to turn a failing run into a pass.

## 7. Runtime metrics (report JSON)

The report is written to `--output`, by default `$ROS_HOME/spiderx_m2/posture_hold_report.json` (`~/.ros/...`). It contains:
- **Identification:** `simulation_only: true`, posture name, config version and date.
- **Command:** every commanded joint position; start positions; peak joint speed; action `error_code`.
- **Joint results:** final measured positions; per-joint final error and per-joint hold-max error; max and RMS error over the hold.
- **Controllers:** states at start, during the hold and at the end.
- **Timing:** simulation timestamps for command start, trajectory end, hold start and hold end; hold duration in simulation seconds; sample counts.
- **Body pose:**
  - **Frame:** the pose of Gazebo model `spiderx` in the Gazebo world frame of world `spiderx_fortress`. That is the `dummy_link` pose, which equals `base_link` because `dummy_joint` has no offset.
  - **Height:** z of the `base_link` origin above the ground plane (world z = 0) — minimum, maximum, range and final value.
  - **Roll and pitch:**
    - Computed as ZYX Euler angles of `base_link` in the world. "Roll" is rotation about `base_link` x, which points to the robot's right, so it is **nose up/down**. "Pitch" is rotation about `base_link` y, which points forward, so it is **side tilt**. This mapping follows from the non-REP-103 `base_link` convention.
    - Also reported: total tilt (angle between body z and world z), xy drift and yaw drift. These are informational, not criteria.
- **Contact:** `foot_contact: "unavailable"`, with the reason. No values are faked.
- **Geometric indicator (not a contact measurement):** body height minus 0.0545 m.
- **Outcome:** `outcome` is one of the two exact strings; `passed` is a boolean; `failures` lists every failed criterion.
- **Limitations:** the full list is copied into every report.

Minimum samples: at least 20 world-pose samples and 20 joint-state samples during the hold. This guards against a hold that "passes" with no data.

## 8. Contact evidence decision

Neither the model nor the world has a contact sensor or `Contact` system, and adding one would change the verified description. So, as instructed, M2 does **not** add a contact plugin. Foot contact is reported as **unavailable**.

The posture evidence is:
- body height and orientation (ground truth);
- joint tracking;
- the geometric consistency of body height with the CAD foot plane.

M2 will not claim that foot contact was verified.

## 9. Planned files

| File | Change |
|---|---|
| `spiderx_controller/config/m2_simulation_postures.yaml` | **New.** The single source of M2 posture targets, durations and thresholds |
| `spiderx_controller/spiderx_controller/posture_config.py` | **New.** Loads the YAML (rejecting duplicate keys) and validates it strictly. Reuses `joint_safety.check_pose` and cross-checks the targets against `cad_neutral` |
| `spiderx_controller/spiderx_controller/posture_metrics.py` | **New.** Pure metrics: joint errors, RMS, quaternion to roll/pitch/yaw, pose-sample extraction, evaluation against thresholds, report assembly |
| `spiderx_controller/scripts/run_posture_hold_test.py` | **New.** Runtime tool |
| `spiderx_controller/spiderx_controller/config_check.py` and `scripts/validate_controller_config` | Extended with `check_m2_posture` |
| `spiderx_controller/test/test_posture_config.py`, `test_posture_metrics.py` | **New** unit tests, including negative tests |
| `spiderx_controller/CMakeLists.txt`, `package.xml` | Install the new script and tests. Add the `tf2_msgs` and `geometry_msgs` exec dependencies |
| `spiderx_bringup/launch/fortress_posture_hold.launch.py` | **New.** Includes `fortress_control.launch.py` and adds the ground-truth pose bridge (not on `/tf`) |
| `spiderx_bringup/package.xml` | Add the `ros_gz_bridge` exec dependency if it is missing |
| `scripts/validate_m2_posture.sh` | **New.** Static checks, plus a `--runtime` mode |
| Docs | New `M2_SIMULATION_POSTURE_GUIDE.md`, `M2_TEST_RESULTS.md` and `M2_SIMULATION_LIMITATIONS.md`. Updates to `STATUS.md`, the README, the roadmap, the testing guide and the package READMEs |

Nothing in `spiderx_description` changes: the URDF, xacro, world, bridge YAML and launch files all stay the same.

## 10. Safety and rollback

- **Simulation only.** No hardware driver, serial device or firmware is touched. The static check greps the new launch and script files for hardware references.
- **Checks before sending.** Invalid configuration is refused with exit code 2, before `rclpy.init()` and so before any command is sent.
- **Motion bounds.** A single slow trajectory, with peak speed at most 0.5 rad/s. Test nodes use `use_sim_time`.
- **Ctrl-C.** The tool cancels its active goal. Nothing else commands the joints.
- **M1 unchanged.** The M1 and passive launch files are untouched; regression runs of `validate_m1_control.sh [--runtime]` and `validate_fortress.sh --runtime` confirm it.
- **Rollback.** Every M2 change is additive and lives in new files, apart from `validate_controller_config` gaining a check. Reverting the M2 commits restores M1 exactly.

## 11. Commands

```bash
# build and test
source /opt/ros/humble/setup.bash
cd ~/spiderx_ws && rm -rf build install log && colcon build --symlink-install
source install/setup.bash
colcon test && colcon test-result --verbose
ros2 run spiderx_controller validate_controller_config

# static validation
./scripts/validate_m1_control.sh
./scripts/validate_m2_posture.sh

# manual run: terminal 1
ros2 launch spiderx_bringup fortress_posture_hold.launch.py
# manual run: terminal 2
ros2 run spiderx_controller run_posture_hold_test.py

# automated runtime and regression
./scripts/validate_m2_posture.sh --runtime
./scripts/validate_m1_control.sh --runtime
./scripts/validate_fortress.sh --runtime
```

**Cloud-only caveat:** the cloud environment's setuptools (≥ 80) breaks `colcon build --symlink-install` for `ament_python` packages. It is pinned to Ubuntu's 59.6.0 during the build and restored to 84.0.0 afterwards. This is recorded in the test results.

## 12. Acceptance criteria (M2 done)

1. This plan is committed before any implementation.
2. The static checks pass:
   - `validate_controller_config`, including the M2 check;
   - `validate_m2_posture.sh`;
   - `validate_m1_control.sh`;
   - all unit tests, including the new negative tests.
3. `validate_m2_posture.sh --runtime`:
   - executes the full posture-hold test;
   - records a complete report JSON with every field in §7;
   - confirms one `/joint_states` publisher;
   - shuts down cleanly, with no leftover processes.
4. The outcome is stated honestly as exactly one of the two strings, with measured values. If the result is "not verified", the cause is documented and nothing is tuned.
5. The regression runs pass: `validate_m1_control.sh --runtime` and `validate_fortress.sh --runtime`.
6. Every M2 document and report states **simulation-only** and lists the limitations (placeholder effort, contact model, friction, damping, mass and inertia, physics timestep and solver, and no real actuator, battery, electronics, structural or IMU validation).
7. The PR is a draft, targets `claude/spiderx-m1-joint-control`, and nothing is merged. M3 is not started.
