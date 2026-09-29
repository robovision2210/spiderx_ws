# M4 Test Results – All-Leg Kinematics and Static Pose Hold via IK

```text
Four-leg static poses in Gazebo Fortress - simulation only.
Not walking, not a gait, not balance control, not locomotion, not hardware validation.
```

## Outcome (cloud + local verified)

- **All-leg forward kinematics verified for the current URDF/TF/Gazebo model.**
- **All-leg inverse kinematics verified for documented, joint-safe, simulation-only static poses.**
- **Static multi-leg pose hold via IK validated in Gazebo (no walking, no gait, no hardware).**

These results are bounded by four things:
- **One URDF.** Gazebo, TF and the M4 code share one URDF, so agreement shows *model consistency*, not physical truth.
- **Idealised joints.** They use the 100 N·m placeholders, so they are near-rigid.
- **Contact isn't measured.** Body height agreeing with the geometric expectation is consistent with the feet resting on the ground, but it is not a contact measurement.
- **Verified in the cloud and locally.** It passed on the owner's Ubuntu PC too (see [Local verification](#local-verification-owners-ubuntu-pc--passed)).

See the [plan](M4_PLAN.md) for the design and the full list of limitations.

## Environment

| Item | Value |
|---|---|
| Branch | `claude/spiderx-m4-all-leg-ik`, from `main` @ `ce5c27e` (M3 merged) |
| Stack | ROS 2 Humble, Ignition Gazebo 6.16 (Fortress, DART, 1 ms step), gz_ros2_control 0.7.15, ros_gz 0.244.20 |
| Machine | Cloud VM, no GPU, Xvfb and Mesa; real-time factor ≈ 0.25 |
| Launch | `fortress_posture_hold.launch.py`: M1 controllers plus the M2 ground-truth pose bridge (unchanged) |

## Runtime (`./scripts/validate_m4_all_leg_ik.sh --runtime`, cloud)

**Before motion:**
- both controllers were `active`, at the start and at the end;
- `/joint_states` had publishers [1, 1];
- FK at the start matched TF and Gazebo on all four legs.

**The run:** 6 trajectory goals (3 poses + 3 returns to neutral), all with `error_code` 0. Every pose was
**one 12-joint trajectory** of 3.0 s, followed by a 1 s settle and a **5.0 s hold**
(simulation time).

| Pose | FK vs TF (max, 4 legs, hold start and end) | FK vs Gazebo (max) | Foot tip vs target (max) | Body height observed / expected (deviation) | \|roll\| / \|pitch\| max | Samples | Result |
|---|---|---|---|---|---|---|---|
| `neutral_stance` | 3.4e-14 m / 6.6e-12 rad | 1.2e-15 m / 2.3e-13 rad | 6.4e-11 m | 0.054495–0.054504 / 0.054543 m (0.048 mm) | 1.5e-7 / 2.2e-12 rad | 1416 | ✅ PASS |
| `crouch_10mm` | 1.1e-14 m / 4.1e-13 rad | 1.2e-7 m / 7.0e-8 rad | 6.5e-11 m | 0.044562–0.044568 / 0.044543 m (0.025 mm) | 2.8e-5 / 3.4e-11 rad | 895 | ✅ PASS |
| `lift_lf_15mm` | 2.8e-11 m / 3.1e-10 rad | 3.0e-12 m / 3.3e-11 rad | 1.2e-10 m | 0.054495–0.054504 / 0.054543 m (0.048 mm) | 2.0e-6 / 2.4e-6 rad | 1072 | ✅ PASS |
| **Criterion** | ≤ 1e-6 m / 1e-6 rad | ≤ 1 mm / 0.005 rad | ≤ 2 mm | ± 3 mm | ≤ 0.10 rad each | ≥ 20 | |

- **Crouch height.** The crouch lowered the body by 9.93 mm, from 0.054500 m to 0.044565 m, against 10 mm predicted from IK geometry.
- **Lift stability.** With the front-left foot lifted, the robot stayed level on three feet: the largest tilt was 3.1e-6 rad.
- **Joint tracking.** The largest joint tracking error during any hold was 1.1e-9 rad.
- **Negative poses.** `rr_unreachable_300mm_below` (rear_right `unreachable_knee`) and `lr_thigh_outside_limits` (rear_left `joint_limits`) were refused by the loader and **never commanded**.
- **Shutdown.** Clean; no leftover processes.

The report was copied to `log/m4_all_leg_ik/latest_report.json`, which git ignores. It contains
`simulation_only: true`, `passed: true`, `fk_verified`, `ik_verified` and `hold_validated` all true, and
`failures: []`.

## Earlier failed run (not counted)

The first runtime run **failed before any motion**. `joint_state_broadcaster` was `inactive` while
`leg_trajectory_controller` was `active`: this is the intermittent **M1 spawner start-up race**
already seen once during M3. The M4 tool refused correctly, and nothing was commanded. That run
exposed two small M4 issues, both fixed in `ee42108`:
- the report lacked `joint_state_publishers` when the tool stopped early;
- the launch log was deleted, so the spawner error couldn't be confirmed. It is now kept as `log/m4_all_leg_ik/launch_log_<timestamp>.txt` whenever a runtime check fails.

The race itself is **not fixed in M4**, because that would change the M1 launch file. It stays a separate task.

## Unit tests and static validation (cloud)

| Check | Result |
|---|---|
| Clean `colcon build --symlink-install` | ✅ 8 packages |
| `colcon test` + `colcon test-result` | ✅ **311 tests, 0 failures**: 302 pytest cases + 9 CTest wrappers |
| M4 test files | `test_all_leg_kinematics` 70, `test_m4_pose_targets` 48, `test_m4_pose_validation` 18 |
| M3 test files (unchanged) | `test_leg_kinematics` 55, `test_kinematics_targets` 29 |
| `validate_m4_all_leg_ik.sh` (static) | ✅ All M4 checks passed: `--check-only`; 136 M4 unit tests; refusals of `simulation_only: false`, frame `odom`, a missing leg and an unreachable leg, each for its expected reason |
| `validate_m3_kinematics.sh`, `validate_m2_posture.sh`, `validate_m1_control.sh`, `validate_fortress.sh` (static) | ✅ all passed |

## Regression (runtime, cloud)

| Check | Result |
|---|---|
| `validate_m3_kinematics.sh --runtime` | ✅ All M3 checks passed: single-leg FK and IK verified, clean shutdown |
| `validate_m2_posture.sh --runtime` | ✅ All M2 checks passed: "Simulation posture hold verified.", clean shutdown |
| `validate_m1_control.sh --runtime` | ✅ All M1 checks passed |
| `validate_fortress.sh --runtime` (passive M0 path) | ✅ All checks passed |
| Leftover simulation/controller processes after all runs | ✅ none |

Each regression ran in its own simulation, with a 30 s pause in between so DDS could discard the
previous run's participants. The controller start-up race did not recur in these runs.

## Local verification (owner's Ubuntu PC) – passed

The owner reported these facts; they are recorded here exactly as given.

| Item | Local result |
|---|---|
| Branch and commit | `claude/spiderx-m4-all-leg-ik` @ `fd777fe`; the local SHA matched `origin/claude/spiderx-m4-all-leg-ik` exactly |
| Working tree | Clean before and after; no tracked files modified |
| Hardware | No hardware driver or hardware interface was started |
| Build | Clean build: 8 packages finished in 8.53 s; no warnings or errors |
| Tests | **311 tests, 0 errors, 0 failures, 0 skipped** (302 pytest cases + 9 CTest wrapper entries) |
| `./scripts/validate_m4_all_leg_ik.sh` | ✅ Passed: 136 M4 unit tests and 4 invalid-config refusals |
| `./scripts/validate_m4_all_leg_ik.sh --runtime` | ✅ Passed in 4 min 3 s |
| Controllers | `joint_state_broadcaster` and `leg_trajectory_controller` active at start and end |
| `/joint_states` | One publisher |
| Trajectory goals | All accepted with `error_code` 0; `goals_sent`: 6 |
| Negative poses (refused, never commanded) | `rr_unreachable_300mm_below`: rear_right `unreachable_knee`; `lr_thigh_outside_limits`: rear_left `joint_limits` |

**Outcome lines (local):**
- All-leg forward kinematics verified for the current URDF/TF/Gazebo model.
- All-leg inverse kinematics verified for documented, joint-safe, simulation-only static poses.
- Static multi-leg pose hold via IK validated in Gazebo (no walking, no gait, no hardware).

**Local per-pose results:**

| Pose | Samples | Tip error (all four legs) | Max roll / pitch | Expected / measured height | Max height deviation | Returned to neutral | Result |
|---|---|---|---|---|---|---|---|
| `neutral_stance` | 1356 | 0.0000 mm | 1.5e-07 / 2.2e-12 rad | 0.05454 / 0.05450 m | 0.048 mm | true | ✅ PASS |
| `crouch_10mm` | 914 | 0.0000 mm | 2.8e-05 / 3.4e-11 rad | 0.04454 / 0.04456–0.04457 m | 0.025 mm | true | ✅ PASS |
| `lift_lf_15mm` (front-left lifted 15.0 mm, three support legs) | 1023 | 0.0000 mm | 2.0e-06 / 2.4e-06 rad | 0.05454 / 0.05449–0.05450 m | 0.048 mm | true | ✅ PASS |

**Local FK extrema.** Every check passed:

| Comparison | Worst tip error | Worst orientation error |
|---|---|---|
| FK vs TF | 3.2e-11 m | 2.1e-10 rad |
| FK vs Gazebo | 4.0e-07 m | 9.1e-07 rad |

**Local report.** `log/m4_all_leg_ik/latest_report.json` (git-ignored, 117 KB) contains:
- `simulation_only: true`, `passed: true`, `fk_verified: true`, `ik_verified: true`, `hold_validated: true`, `final_return_ok: true`, `failures: []`;
- frame `base_link`; config version v1 (2026-09-26);
- hold criteria: 5.0 s, roll and pitch ≤ 0.1 rad, height tolerance 0.003 m, at least 20 samples.

**Local runtime regressions:**
- M3: All M3 checks passed.
- M2: All M2 checks passed.
- M1: All M1 checks passed.
- Fortress: All checks passed.

No Gazebo, controller-manager, bridge, `robot_state_publisher` or spawner process was left over.

**Known limitation (not fixed).** The unrelated controller-spawner start-up race remains unfixed on
this branch. It did not occur in any of the five local runtime launches. It stays documented as a
separate maintenance task.
