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
| 3 | Unit tests (including M1 safety rejections, M2 config/metrics, M3 FK/IK negative tests, M4.1 controller start-up ordering and validator state checks, the M4.5 gait framework and regression pins, the M5 study, gate, records, tables and regression pins, and the M6.0 offline preflight, mock action-client and mocked live-preflight tests) | `colcon test --packages-select spiderx_controller spiderx_scripts && colcon test-result --verbose` | 0 failures |
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
| 20 | M4 static (all-leg kinematics, simulation only) | `./scripts/validate_m4_all_leg_ik.sh` | `All M4 checks passed.` |
| 21 | M4 runtime (Gazebo, 3 static poses) | `./scripts/validate_m4_all_leg_ik.sh --runtime` | The three M4 outcome lines, then `All M4 checks passed.` |
| 22 | M4.1 controller start-up (manual) | `ros2 launch spiderx_bringup fortress_control.launch.py`, then Ctrl+C in the same terminal | `Configured and activated joint_state_broadcaster` appears before the `leg_trajectory_controller` spawner starts. If the broadcaster fails, you see exactly one `[ERROR] [spiderx_controller]: joint_state_broadcaster startup failed; leg_trajectory_controller was not started; press Ctrl+C and relaunch.` and no trajectory controller. Stop with Ctrl+C in the launch terminal: signalling only the `ros2 launch` process can leave `ign gazebo server`/`gui` running (see [M4.1 results](M4_1_TEST_RESULTS.md)) |
| 23 | M4.5 offline gait analysis (no simulator; verified cloud + local) | `ros2 run spiderx_controller m4_5_gait_analysis --out log/m4_5_gait_analysis` (`--check-only` validates the YAML only; `--no-plots` for tables only) | Exit 0; 6 gaits evaluated; reports in `log/m4_5_gait_analysis/`. Expected verdicts: wave and tripod_crawl FAIL (`static_stability`; wave also `joint_speed`), the other four PASS. A FAIL verdict is a reported result, not a test failure. See [M4.5 results](M4_5_TEST_RESULTS.md) and [framework guide](SPIDERX_GAIT_FRAMEWORK.md) |
| 24 | M5 offline evaluation study (no simulator; cloud + local offline verification passed) | `ros2 run spiderx_controller m5_offline_evaluation --check-only`, then `--dry-run`, then a full run (about 12–25 min; `--out` to choose the output root) | `--check-only`: 285/264, 293/270, 6 speed checks. Full run: exit 0, all 7 Stage 0/1 gate checks PASS, results in `log/m5_offline_evaluation/<study_id>/`. Exit 3 = gate failed (Stages 2–4 blocked, evidence kept). See [M5 results](M5_TEST_RESULTS.md) and [M5 guide](SPIDERX_M5_EVALUATION_GUIDE.md) |
| 25 | M6.0-A offline trajectory preflight (no simulator; cloud + local verified) | `ros2 run spiderx_controller m6_offline_preflight --check-only` (without `--check-only` it writes `log/m6_playback/offline/<id>/`) | Exit 0, `Preflight: PASS`, 3 points, 9.0 s, maximum displacement 0.1222941360 rad within the 0.1223 rad cap. Exit 2 = REFUSED with named codes. See [M6 guide](SPIDERX_M6_PLAYBACK_GUIDE.md) |
| 26 | M6.0-B live READ-ONLY preflight (needs a running `fortress_control.launch.py`; sends no goal; local, owner-approved) | `ros2 run spiderx_controller m6_live_preflight` (`--interface-only` needs no running stack and creates no node) | Exit 0 `READY`; exit 1 `NOT READY` with named codes. READY proves no tracking, movement or walking. Cloud ran `--interface-only` only; graph mode passed locally once (layer 27). See [M6 results](M6_TEST_RESULTS.md) |
| 27 | M6.0-B manual GUI graph-mode observation (owner's Ubuntu PC only; GUI, not headless; never in the cloud; sends no goal or command) | Terminal A: `ros2 launch spiderx_bringup fortress_control.launch.py 2>&1 \| tee log/m6_graph_preflight/launch.log`. Terminal B, after both controllers are `active` plus 3 s: `timeout 60 ros2 run spiderx_controller m6_live_preflight --timeout 10 --window 2`, then the read-only evidence queries of [plan §9](M6_GRAPH_PREFLIGHT_PLAN.md#9-local-execution-checklist); stop with one Ctrl+C in terminal A, then `ros2 daemon stop` and the leftover `pgrep` | Exit 0 `READY`; all five evidence tiers pass (an unobservable tier is reported `unavailable`, never passed); classification by hand; no leftover process. **Limits:** observation only. It does not show goal acceptance, tracking, motion, contact, stability or walking. Passed locally at `b64217d`: [results](M6_GRAPH_PREFLIGHT_RESULTS.md) |

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

**Local verification:** the owner ran layers 1–5, 7–8 and 11–13 on Ubuntu 22.04 (M1 branch), later layers 1–3 and 11–16 (M2 branch). On the M3 branch they ran layers 1–3, 17 and 18, and the M2, M1 and Fortress regression scripts. On the M4 branch they ran layers 1–3, 20 and 21, plus the M3, M2, M1 and Fortress runtime regressions. All passed. On the M4.1 branch, layers 1–5, 11–12, 14–15, 17–18 and 20–22 passed in the cloud. Locally (`35b83b5`), the owner ran layers 1, 3 and 22, including the forced failure, plus the M4, M3, M2, M1 and Fortress validators. All passed. On the M6 branch (`f49a6e0`), the owner ran layers 1, 3, 25 and 26 in `--interface-only` mode only, plus the Fortress and M1–M4 static validators. All passed.
