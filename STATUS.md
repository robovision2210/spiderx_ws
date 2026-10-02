# SpiderX Feature Status

Last updated on branch `claude/spiderx-m6d-live-playback` (M6.0-D implementation, from `main` @ `9885057` with M6.0 and M6.0-B merged).

**M6.0-D implementation cloud + local verified in offline/mock/isolated-domain testing; live dispatch remains hard-disabled and local live playback remains pending separate approval.** Draft PR, not merged.
- **Local verification passed** on the owner's Ubuntu PC at `412eb45`, in offline/mock/isolated-domain modes only:
  - clean build, then 1116 tests with 0 failures and 180 M6.0-D tests passing;
  - the same trajectory ID and goal fingerprint as the cloud;
  - dry-run and six mock reports byte-identical to the cloud SHA-256 values;
  - `--live` refused with exit 3;
  - the M1–M4 and Fortress static validators pass;
  - no live graph, goal or Gazebo was used. See [results §12](docs/M6D_LIVE_PLAYBACK_RESULTS.md#12-local-verification-passed-owners-ubuntu-pc-local-result).
- **What it is.** The code path that would send exactly **one** approved goal, neutral → `crouch_10mm` → neutral (3 points at 3/6/9 s, ≤ 0.1223 rad, 0.05 rad tolerances, goal velocity tolerance 0.05 rad/s), with a goal fingerprint, same-process readiness (≤ 10 s, `compatible`/`warning`/`incompatible`), a typed confirmation word, one cancel then hold, and no retry, no automatic return and no second goal.
- **Cloud results.** 1116 tests, 0 failures, 0 skipped; 180 M6.0-D tests, including the fingerprint and state-machine mutation suites and rclpy tests against an in-process action test double on an isolated, non-default, localhost-only ROS domain; dry-run and six mock reports byte-identical across two runs; M1–M4 and Fortress static validators pass.
- **Live dispatch is hard-disabled** (`LIVE_DISPATCH_ENABLED = False`; `--live` exits 3 before importing `rclpy`). No trajectory goal was sent to a live controller; no Gazebo playback, movement, contact, balance, walking, navigation or hardware operation occurred. A separate owner approval and enabling change are required before any local live goal.
- See the [M6.0-D results](docs/M6D_LIVE_PLAYBACK_RESULTS.md), [guide](docs/SPIDERX_M6D_LIVE_PLAYBACK_GUIDE.md) and [plan](docs/M6D_LIVE_PLAYBACK_PLAN.md).

**M6.0-B is merged into `main` (`9885057`).** The notes below describe the M6.0/M6.0-B state.


**M6.0-B graph-mode read-only preflight passed locally; M6.0-D local live playback remains pending.** M6.0 was already cloud + local offline/mock verified and is merged into `main` (`f5a7252`).
- **M6.0-B (local GUI, owner's Ubuntu PC, `b64217d`).**
  - **Result:** verdict READY, `failure_codes: []`; all five zero-command evidence tiers passed; manual installed-stack classification `warning`, because 8 local packages are newer than the cloud references; clean Ctrl+C shutdown with no leftovers and no retry.
  - **Observed:** the controller manager, both controllers active, one FollowJointTrajectory action server, and one fresh 12-joint `/joint_states` publisher.
  - **Observation only:** no goal, cancel, command, controller switch or parameter change was sent by the preflight. It does **not** show goal acceptance, tracking, motion, contact, stability or locomotion.
  - See the [M6.0-B results](docs/M6_GRAPH_PREFLIGHT_RESULTS.md).
- **Local offline/mock verification passed** on the owner's Ubuntu PC at `f49a6e0`: clean build (8 packages), 932 tests with 0 failures, `trajectory_id 44f0a7ad52e5c330` identical to the cloud (portable provenance, `fd7a9de`), byte-identical offline outputs matching the cloud SHA-256 values, all 8 refusal cases exit 2, `--interface-only` with no node, and the M1–M4 static validators pass. (The installed stack was later classified `warning` by M6.0-B.)
- **What it is.** Safety layers for replaying **one** bounded trajectory, neutral → `crouch_10mm` → neutral, through the existing controller stack. It is a trajectory-execution and observability check only.
- **Implemented (cloud + local, 932 tests, 0 failures):**
  - an offline conversion and preflight with 32 machine-readable refusal codes (M6.0-A);
  - a single-goal action client tested only against a deterministic mock, including mutation tests (M6.0-C);
  - a live **read-only** preflight tool (M6.0-B). It is mock-tested; the cloud ran only its no-node `--interface-only` mode.
- **Three separate quantities:**
  - a 0.1223 rad maximum commanded displacement, for M6.0-D only (owner option (i));
  - a 0.05 rad tracking tolerance;
  - a 0.05 rad joint-limit soft margin.
- **Not done.** No valid or invalid trajectory was sent to a live controller. No Gazebo playback, contact, locomotion, walking, navigation or hardware operation. The owner-approved M6.0-D valid playback is pending (the M6.0-B graph-mode preflight was observation only). M6.1 is deferred.
- See the [M6 plan](docs/M6_GAIT_PLAYBACK_SAFETY_PLAN.md) §14, the [M6 results](docs/M6_TEST_RESULTS.md) and the [M6 playback guide](docs/SPIDERX_M6_PLAYBACK_GUIDE.md).
- **Naming (owner decision).** M6 is Gait playback safety: M6.0 is the safety implementation and M6.1 is future protected replay, not implemented. The former future "M6 – Odometry" is now **M7 – Odometry and state estimation (future work)**; SLAM, Nav2 and hardware move to M8, M9 and M10. See the [roadmap](docs/SPIDERX_DEVELOPMENT_ROADMAP.md).

**M5 is merged into `main` (`99c835a`).** The notes below describe its branch-time state.

**Cloud + local offline verification passed for the M5 offline evaluation study.** It is a draft PR and not merged. The local check ran two full studies on the owner's Ubuntu PC (exit 0, byte-identical between those two runs; not compared with the cloud outputs), with 700 tests passing. Offline model results only: no Gazebo or contact-dynamics, runtime, real-time or hardware validation.
- **What it is.** An offline, staged evaluation of gait configuration classes with the unchanged M4.5 evaluator. No Gazebo, no gait playback, no walking, no hardware.
- **Cloud results.**
  - 700 tests, 0 failures;
  - the Stage 0/1 gate passed (all 7 checks);
  - the full study (293 stage evaluations, 270 unique, plus 6 speed checks) ran twice with byte-identical artifacts;
  - the six M4.5 baselines and the negative controls (wave, tripod_crawl) were reproduced;
  - pace/trot static support is N/A (never zero);
  - pre-registered hypotheses H1–H3 are supported (model predictions only);
  - the M1–M4 static validators pass.
- **Finding fixed in M4.5.** The gate found a floating-point phase-boundary round-off in `gait_phase`, which made wave's support fractions 0.6 + 1/n. The owner approved fixing it (`ece1e23`); every M4.5 pin is unchanged. See the erratum in [M4.5 results](docs/M4_5_TEST_RESULTS.md).
- **Limits.** Offline model metrics only. Static support is an approximation, K is a model-derived joint-rate-per-distance indicator, 0.5 rad/s is a provisional screening flag, and the motion proxies are not energy.
- **Naming.** The former "M5 `/cmd_vel` bridge" is now **M5.5: Command-velocity bridge**. It is future work: not implemented and not scheduled.
- See the [M5 evaluation plan](docs/M5_EVALUATION_PLAN.md).

**M4.5 (offline multi-gait configuration and trajectory validation) was verified in the cloud and locally** by the owner on Ubuntu (`4a6553c`). M4.5 is merged into main at merge commit `3db1aec` and is cloud + locally verified.
- **What it is.** OFFLINE kinematic analysis: no Gazebo gait playback, no walking, no hardware.
- **Six YAML gaits** are evaluated through the unchanged M3/M4 IK: wave, tripod_crawl, ripple, amble, pace and trot.
- **Checks.** IK feasibility, joint-limit, singularity and continuity margins, a quasi-static stability **approximation**, joint speed against the 0.5 rad/s **placeholder**, and heuristic energy **proxies**.
- **Output.** Deterministic CSV, JSON and PNG under `log/m4_5_gait_analysis/`.
- **Reported results.** All six gaits are kinematically feasible.
  - wave and tripod_crawl **FAIL** static stability, because the URDF COM is ≈ 5.8 mm behind the foot centre.
  - wave also exceeds the joint-speed placeholder (0.541 rad/s).
  - ripple, amble, pace and trot PASS their kinematic checks. That is **not** a stability or walking claim.
- **Cloud test results.** 556 tests, 0 failures; the M1–M4 static validators pass; no URDF, launch, controller or simulation file changed.
- **Local test results.**
  - build: 8 packages in 11.2 s, no warnings;
  - 556 tests, 0 errors, 0 failures, 0 skipped;
  - two full analysis runs, each exit 0, gave 40 byte-identical files with the same verdicts and metrics as the cloud;
  - the M1–M4 static validators passed;
  - no Gazebo was started and no hardware was touched.
- **Limits.** Offline only. Stability is a quasi-static approximation, energy values are heuristic proxies and the joint-speed reference is a placeholder. Masses are CAD placeholders.
- See the [M4.5 results](docs/M4_5_TEST_RESULTS.md) and the [gait framework guide](docs/SPIDERX_GAIT_FRAMEWORK.md).

**M4.1 (controller start-up robustness) was verified in the cloud and locally** by the owner on Ubuntu (`35b83b5`) and merged into `main` (`c4a993c`).
- **Ordering.** `leg_trajectory_controller` now starts only after the `joint_state_broadcaster` spawner exited with code 0.
- **Failure.** A failed broadcaster spawner logs one error line (`joint_state_broadcaster startup failed; leg_trajectory_controller was not started; press Ctrl+C and relaunch.`) and nothing else is started.
- **Shutdown.** Nothing is started during Ctrl+C.
- **Cloud results.**
  - 385 tests, 0 failures;
  - a forced broadcaster failure never loaded the trajectory controller;
  - Ctrl+C during start-up left nothing behind;
  - the M4, M3, M2, M1 and Fortress runtime regressions passed.
- **Local results.**
  - clean build of 8 packages in 9.45 s;
  - 385 tests, 0 errors, 0 failures, 0 skipped;
  - the trajectory spawner started 4 ms after the broadcaster spawner exited 0, and the broadcaster activated in 0.59 s;
  - a forced broadcaster failure logged exactly one error line and never loaded the trajectory controller;
  - Ctrl+C during start-up left no orphaned Gazebo server or GUI;
  - the M4, M3, M2, M1 and Fortress validators all passed;
  - no environment hangs and no leftover processes.

  See the [M4.1 results](docs/M4_1_TEST_RESULTS.md). This fixes start-up ordering and robustness only; it adds no gait or walking.

**M4 was verified locally** by the owner on Ubuntu (`fd777fe`): clean build (8 packages), 311 tests with 0 failures, and `validate_m4_all_leg_ik.sh` static and `--runtime` passed. All three M4 outcome lines were verified. The M3, M2, M1 and Fortress runtime regressions passed. At that time the controller-spawner start-up race was unfixed; it did not occur in 5 local launches. M4.1 (above) addresses it.

**M3 was verified locally** by the owner on Ubuntu: build (8 packages), 172 tests with 0 failures, and `validate_m3_kinematics.sh` static and `--runtime` all passed. FK matched TF (≤ 5.4e-11 m) and Gazebo (≤ 6.2e-12 m); all 5 IK targets were reached and returned; both negative targets were rejected with 0 goals sent. The M2, M1 and Fortress regressions passed.

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
| Controller start-up order and failure handling (M4.1, simulation only) | ✅ Verified (cloud + local) | `controller.launch.py`: the trajectory spawner starts only after the broadcaster spawner exits 0; one error line on failure; nothing starts during Ctrl+C; `--switch-timeout 60`, `--service-call-timeout 75`. A forced broadcaster failure (0.001 s switch timeout) never loaded the trajectory controller. [M4.1 results](docs/M4_1_TEST_RESULTS.md) |
| Joint position control in Fortress (`gz_ros2_control` 0.7.x, 12 joints) | ✅ Verified (local + cloud) | `lf_hip` → +0.2 rad and back, error 0.0000; invalid joints and out-of-limit targets refused. Tracking is idealised by the 100 N·m placeholder effort limit |
| Balance / standing controller (body feedback) | ⚪ Future work | Not scheduled. M1/M2 only hold joint angles; there is no balance feedback, IMU or disturbance test |
| Single-leg forward kinematics (front-left, simulation only) | ✅ **Single-leg FK verified against the current URDF/TF/Gazebo model** (local + cloud) | `leg_kinematics.py` (URDF-derived chain; derived foot tip). FK vs TF ≤ 5.4e-11 m / 3.5e-10 rad and vs Gazebo ≤ 3.0e-11 m / 1.0e-10 rad, at CAD neutral and 5 IK configurations. [M3 results](docs/M3_TEST_RESULTS.md) |
| Single-leg inverse kinematics (front-left, simulation only) | ✅ **Single-leg IK verified for documented reachable, joint-safe simulation targets** (local + cloud) | Analytic IK. 5 lifted targets reached within ≤ 1.1e-10 m and returned; unreachable and out-of-limit targets refused with 0 goals sent. Idealised by the 100 N·m placeholder joints. **Not** walking, balance or hardware validation. [Limitations](docs/M3_SIMULATION_LIMITATIONS.md) |
| All-leg forward kinematics (simulation only) | ✅ **All-leg forward kinematics verified for the current URDF/TF/Gazebo model** (cloud + local) | Opt-in `allow_all_legs` in `leg_kinematics.py` (M3 default unchanged). FK vs TF ≤ 3.4e-11 m and vs Gazebo ≤ 1.2e-7 m on all four feet. [M4 results](docs/M4_TEST_RESULTS.md) |
| All-leg inverse kinematics (simulation only) | ✅ **All-leg inverse kinematics verified for documented, joint-safe, simulation-only static poses** (cloud + local) | 3 poses: `neutral_stance`, `crouch_10mm`, `lift_lf_15mm`. Foot tips ≤ 1.2e-10 m from target; negative poses refused and never commanded |
| Static multi-leg pose hold via IK (simulation only) | ✅ **Static multi-leg pose hold via IK validated in Gazebo** (cloud + local) | 5 s holds; \|roll\|, \|pitch\| ≤ 2.8e-5 rad; body height within 0.05 mm of the geometric expectation. **Not** walking, gait, balance or hardware validation; contact not measured |
| Offline gait configuration and trajectory validation (M4.5) | ✅ Verified (offline; cloud + local) | `m4_5_gait_analysis`: 6 YAML gaits checked against the URDF-derived IK, joint limits, a static-stability **approximation** and the joint-speed **placeholder**. wave and tripod_crawl FAIL static stability (reported, not tuned). [M4.5 results](docs/M4_5_TEST_RESULTS.md). **Not** walking, not a validated gait, not hardware |
| M6.0 gait-playback safety layers (one bounded crouch trajectory) | 🟡 Implemented; cloud + local offline/mock verified; M6.0-B graph-mode read-only preflight passed locally; M6.0-D valid playback pending | `m6_offline_preflight` (offline conversion + preflight), `m6_action_client` (single goal, mock-tested only), `m6_live_preflight` (read-only; sends no goal). No goal has been sent to a live controller; M6.0-D needs owner approval. [M6 results](docs/M6_TEST_RESULTS.md), [guide](docs/SPIDERX_M6_PLAYBACK_GUIDE.md). **Not** gait playback or walking |
| Gait playback / walking in simulation | ⚪ Future work | Unscheduled "Future — gait playback" item (owner decision). Nothing streams gait trajectories to the controllers |
| Offline evaluation study (M5) | ✅ Cloud + local offline verification passed | `m5_offline_evaluation`: staged study, Stage 0/1 gate, deterministic provenance-rich records. [M5 results](docs/M5_TEST_RESULTS.md), [guide](docs/SPIDERX_M5_EVALUATION_GUIDE.md). Offline model analysis only; **not** walking, dynamic stability, energy or hardware |
| `/cmd_vel` → gait bridge | ⚪ Future work | M5.5: Command-velocity bridge (renamed from M5). Not implemented. Nothing consumes `/cmd_vel` today |
| Odometry (`/spiderx/leg_odometry`, `/odom`) | ⚪ Future work | M7 – Odometry and state estimation (formerly M6). Not implemented. **Not faked** |

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
