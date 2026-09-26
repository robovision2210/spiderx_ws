# spiderx_controller

SpiderX quadruped joint configuration, **M1 joint position control** (Gazebo Fortress +
gz_ros2_control), the **M2 simulation-only CAD neutral posture-hold test** and **M3 simulation-only
single-leg (front-left) forward/inverse kinematics**. No balance control, gait, locomotion or
hardware support (see `../../STATUS.md`).

| File | Content |
|---|---|
| `config/spiderx_legs.yaml` | 4 legs × (hip, thigh, foot): joint names, parent/child links, axes, URDF limits, soft limits (−/+0.05 rad), motion constraints (placeholder / unknown) |
| `config/spiderx_poses.yaml` | Named poses. `cad_neutral` (all 0 rad) is the CAD zero configuration (the audit shows all four feet on one plane). Reached by M1 joint-position control; for the M2 simulation hold, see `docs/M2_TEST_RESULTS.md` |
| `config/m2_simulation_postures.yaml` | **M2, simulation only.** The single source of the posture-hold test: `cad_neutral_simulation_hold` (targets must equal `cad_neutral`), command and hold durations, pass/fail thresholds with their provenance |
| `config/spiderx_ros2_controllers.yaml` | `joint_state_broadcaster` + `leg_trajectory_controller` (12 joints, position). Loaded by `fortress_control.launch.py` |
| `launch/controller.launch.py` | Spawns the two controllers in order (event handler, no sleeps) |
| `scripts/test_one_joint.py`, `scripts/test_neutral_pose.py` | Safe M1 test tools: refuse unknown joints and out-of-limit targets; one slow trajectory; report errors |
| `scripts/run_posture_hold_test.py` | **M2.** Validates the posture and refuses before sending if it is invalid. Sends one slow 12-joint trajectory, then records joints, controllers and the Gazebo ground-truth body pose for 10 s of simulation time. Writes a JSON report; exit 0 = "Simulation posture hold verified." |
| `spiderx_controller/posture_config.py`, `posture_metrics.py` | M2 strict config loader and validator; metrics, thresholds and report (unit-tested) |
| `spiderx_controller/leg_kinematics.py` | **M3.** Extracts the front-left chain (axes, origins, limits) and a derived foot-tip point from the expanded URDF. FK uses the URDF transform chain and a product of exponentials; the IK is analytic and position-only, respects the URDF limits minus 0.05 rad and never clamps. No ROS; never commands anything |
| `config/m3_kinematics_targets.yaml`, `spiderx_controller/kinematics_targets.py` | **M3, simulation only.** Safe lifted targets written as offsets from FK(CAD neutral), negative-test-only targets, tolerances; strict validation |
| `scripts/validate_leg_kinematics`, `spiderx_controller/kinematics_validation.py` | **M3.** Compares FK with TF and Gazebo ground truth. Sends each safe IK target as one trajectory, holding the other 9 joints, and checks that negative targets are refused. Writes a JSON report. Exit 0 = FK and IK verified |
| `spiderx_controller/joint_safety.py` | Shared safety rules (unit-tested) |
| `spiderx_controller/config_check.py`, `scripts/validate_controller_config` | Checks every file above against the URDF |

```bash
ros2 run spiderx_controller validate_controller_config
```

```bash
ros2 launch spiderx_bringup fortress_control.launch.py
ros2 run spiderx_controller test_one_joint.py --joint lf_hip --target 0.2 --return-to-initial
ros2 run spiderx_controller test_neutral_pose.py
```

M2 (simulation only; not balance, walking, IK or hardware validation):

```bash
ros2 launch spiderx_bringup fortress_posture_hold.launch.py
ros2 run spiderx_controller run_posture_hold_test.py
```

M3 (simulation only; one leg; not walking, balance or hardware validation):

```bash
ros2 launch spiderx_bringup fortress_posture_hold.launch.py   # M1 + Gazebo ground-truth pose
ros2 run spiderx_controller validate_leg_kinematics
```

It is not diff-drive: a quadruped needs joint control (M1) → simulation posture hold (M2) → single-leg FK/IK (M3) → gait → `/cmd_vel` bridge.
See `docs/SPIDERX_DEVELOPMENT_ROADMAP.md`.
