# M3 Test Results – Single-Leg (Front-Left) Forward and Inverse Kinematics

```text
Single-leg simulation kinematics validation (Gazebo Fortress).
Not walking or a gait. Not balance or standing control. Not locomotion.
Not real-world leg control. Not hardware validation.
```

## Outcome (cloud; local run pending)

- **Single-leg FK verified against the current URDF/TF/Gazebo model.**
- **Single-leg IK verified for documented reachable, joint-safe simulation targets.**

Two caveats bound these results:
- **Model, not robot.** Gazebo, TF and the M3 code all use the same URDF. Agreement shows that the kinematics *implementation* matches the *model*, not that the model matches the physical robot.
- **Idealised tracking.** The 100 N·m placeholder joints track the IK angles almost perfectly.

See [M3_SIMULATION_LIMITATIONS.md](M3_SIMULATION_LIMITATIONS.md).

## Environment

| Item | Value |
|---|---|
| Branch | `claude/spiderx-m3-leg-kinematics`, from `main` @ `d9426c9` (merged PR #4–#7) |
| Stack | ROS 2 Humble, Ignition Gazebo 6.16 (Fortress, DART, 1 ms step), gz_ros2_control 0.7.15, ros_gz 0.244.20 |
| Machine | Cloud VM, no GPU, Xvfb and Mesa; real-time factor ≈ 0.25 |
| Build | `colcon build --symlink-install`, **8 packages**; setuptools temporarily pinned to 59.6.0 (the known setuptools ≥ 80 issue), then restored |
| Launch | `fortress_posture_hold.launch.py`: M1 controllers plus the M2 Gazebo ground-truth pose bridge. No new launch file was needed |

## Geometry extracted from the URDF (front-left leg)

- **Joints:** `lf_hip` about [0 1 0], `lf_thigh_joint` about [−1 0 0], `lf_foot_joint` about [1 0 0].
- **Joint-axis points at q = 0:**

| Point | Position in base_link (m) |
|---|---|
| H (hip axis) | (0.010, 0.0161, 0.07725) |
| T (thigh axis) | (−0.0237, 0.03885, 0.08875) |
| K (knee axis) | (−0.0367, −0.038932, 0.010968) |

- **Derived foot tip** (not a URDF frame): (−0.033548, 0.042264, −0.054543) m at CAD neutral. In the `lf_foot_1` frame it is (0.003152, 0.081196, −0.065511).
- **Analytic-IK preconditions hold:** hip·thigh = 0, |thigh × knee| = 0, knee = −thigh.

See [frame conventions](M3_FRAME_CONVENTIONS.md).

## FK: comparison with independent observations (run `validate_m3_kinematics.sh --runtime`)

| Configuration [hip, thigh, foot] (rad) | vs TF: link origin / orientation / tip | vs Gazebo: link origin / orientation / tip |
|---|---|---|
| start = CAD neutral [0, 0, 0] | 5.5e-16 m / 7.6e-14 rad / 8.0e-15 m | 1.8e-15 m / 3.0e-13 rad / 3.2e-14 m |
| `lift_10mm` [0, 0.0555, 0.1223] | 9.6e-12 m / 1.0e-10 rad / 1.6e-11 m | 9.4e-12 m / 1.0e-10 rad / 1.6e-11 m |
| `lift_15mm_forward_10mm` [0, 0.0007, 0.1736] | 2.1e-17 m / 5.6e-17 rad / 3.2e-17 m | 3.0e-14 m / 6.2e-11 rad / 6.5e-12 m |
| `lift_15mm_rearward_10mm` [0, 0.1591, 0.1798] | 1.1e-11 m / 1.2e-11 rad / 1.1e-11 m | 2.9e-11 m / 3.4e-11 rad / 3.0e-11 m |
| `lift_15mm_outward_8mm` [0.0675, 0.0650, 0.1435] | 3.3e-11 m / 3.3e-10 rad / 5.4e-11 m | 5.1e-12 m / 5.0e-11 rad / 8.2e-12 m |
| `lift_15mm_inward_8mm` [−0.0693, 0.0960, 0.2130] | 2.9e-11 m / 3.5e-10 rad / 4.5e-11 m | 5.0e-12 m / 7.3e-11 rad / 9.4e-12 m |
| **Tolerance** | 1e-6 m / 1e-6 rad | 1 mm / 0.005 rad (simulation-only) |

**How each source was measured:**
- **TF** is robot_state_publisher (KDL), looked up at the exact `/joint_states` stamp used for FK.
- **Gazebo** is the ground-truth `lf_foot_1` link pose relative to the model, and the model frame is `base_link`.
- **The observed tip** is the observed link pose composed with the derived tip offset.

**Why the errors are so small.** Every source evaluates the same URDF: DART computes link poses from joint coordinates, and KDL uses the same numbers. The link-origin comparisons involve no derived quantity.

## IK: safe targets (the robot was commanded)

| Target | Target in base_link (m) | IK solution (rad) | Observed tip − target (m) | FK(IK) residual | Returned to start |
|---|---|---|---|---|---|
| `lift_10mm` | (−0.033548, 0.042264, −0.044543) | [0.0, 0.0555, 0.1223] | 5.5e-11 | 1.4e-17 m | yes |
| `lift_15mm_forward_10mm` | (−0.033548, 0.052264, −0.039543) | [0.0, 0.0007, 0.1736] | 1.1e-10 | 1.6e-17 m | yes |
| `lift_15mm_rearward_10mm` | (−0.033548, 0.032264, −0.039543) | [0.0, 0.1591, 0.1798] | 7.1e-11 | 1.6e-17 m | yes |
| `lift_15mm_outward_8mm` | (−0.041548, 0.042264, −0.039543) | [0.0675, 0.0650, 0.1435] | 9.8e-11 | 4.0e-17 m | yes |
| `lift_15mm_inward_8mm` | (−0.025548, 0.042264, −0.039543) | [−0.0693, 0.0960, 0.2130] | 9.6e-11 | 1.6e-17 m | yes |
| **Tolerance** | | inside the URDF limits minus 0.05 rad | ≤ 2 mm (simulation-only) | ≤ 1e-6 m | ≤ 0.01 rad |

**How the moves were run:**
- **One trajectory per move.** Each target was **one** 3.0 s trajectory: the 3 LF joints moved to the IK angles and the other 9 joints were held at their measured positions. Each return was another 3.0 s trajectory.
- **Checked before sending.** All 12 commanded values passed the M1 `check_pose` rule before sending.
- **Totals.** 10 goals in total (5 targets + 5 returns), all with `error_code` 0.
- **Controllers.** Both were `active` at the start and at the end, and `/joint_states` had publishers [1, 1].
- **Body tilt.** The maximum body tilt while the foot was lifted was 1.8e-5 rad (informational; three-foot stance).

## Rejected targets (the robot was NOT commanded)

| Test | How the target was made | IK result | Goals sent | Joint motion during the test |
|---|---|---|---|---|
| `unreachable_300mm_below` | reference tip − 0.30 m in z | `unreachable_knee` ("knee-plane distance 0.4203 m outside [0.0057, 0.2143] m") | 0 | 2.5e-13 rad |
| `outside_joint_limits` | FK of [0, 1.2, 0] (`lf_thigh_joint` 1.2 > 0.785) | `joint_limits` ("geometrically reachable, but every solution violates the URDF limits minus 0.05 rad") | 0 | 5.9e-12 rad |

**Refused before anything was sent.** `validate_m3_kinematics.sh` checks four invalid configs; each was refused before the tool started ROS (exit 2) and wrote no report. Each was refused for its expected reason:
- a safe target pushing 10 mm into the ground (`min_lift_m`);
- leg `rear_right` (unsupported leg);
- frame `odom`;
- `simulation_only: false`.

## Unit tests and static checks

| Check | Result |
|---|---|
| `validate_controller_config` (M1 + M2 + **M3** geometry and target checks) | ✅ valid |
| `test_leg_kinematics` (55): independent numpy URDF walk (300 random configurations), PoE, mesh tip, runtime anchor, signs, 500 IK round trips, candidate selection, limits and margin, unreachable / near-axis / singular, out-of-limit refused not clamped, malformed inputs, other frames, unsupported legs, URDF mutations | ✅ 55 passed |
| `test_kinematics_targets` (29): shipped config, 20 invalid-config cases, malformed and duplicate YAML, unsafe / wrongly accepted targets, FK/IK/negative decision logic, outcome strings, tool refusal before ROS | ✅ 29 passed |
| Mutation check: a sign bug injected into FK makes 6 tests fail | ✅ caught |
| `colcon test` (whole workspace) + `colcon test-result --verbose` | ✅ **172 tests, 0 failures**: 166 pytest cases (`spiderx_controller` 5 + 13 + 29 + 55 + 39 + 23 = 164, `spiderx_scripts` 2) plus 6 CTest wrapper entries |
| `validate_m3_kinematics.sh` (static) | ✅ All M3 checks passed |
| `validate_m3_kinematics.sh --runtime` | ✅ All M3 checks passed, clean shutdown |

## Regression (M0/M1/M2)

| Check | Result |
|---|---|
| `validate_fortress.sh` (static) and `--runtime` (passive M0 path) | ✅ All checks passed |
| `validate_m1_control.sh` (static) and `--runtime` | ✅ All M1 checks passed (`lf_hip` +0.2 rad and back, error 0.0000; `cad_neutral` trajectory moved 3 joints) |
| `validate_m2_posture.sh` (static) | ✅ All M2 checks passed |
| `validate_m2_posture.sh --runtime`, isolated run | ✅ All M2 checks passed: "Simulation posture hold verified.", publishers [1, 1], clean shutdown |
| Protected M0/M1/M2 files unchanged vs `origin/main` (all of `spiderx_description` and `spiderx_bringup`, controller/leg/pose/M2 configs, M1/M2 tools, `trajectory_client.py`, the M0/M1/M2 scripts) | ✅ `git diff` empty |
| Fresh clone of the pushed branch: clean build, `colcon test`, static M3/M2/M1/Fortress validation | ✅ 8 packages; 172 tests, 0 failures; all static checks passed |
| Leftover simulation/controller processes after all runs | ✅ none |

**Two M2 runtime artifacts in an earlier combined run.** Neither was an M2 regression; the isolated rerun above passed.
1. The leftover-process check matched **my own calling shell**, whose command line contained the searched words. The M2 script is unchanged.
2. `/joint_states publisher count = 2` appeared when M2 started seconds after M3's simulation had been killed. The DDS graph still listed the dead publisher; the M2 tool itself measured [1, 1] during the run.

## Problems found and fixed during M3

| Problem | Cause | Fix |
|---|---|---|
| The first automated runtime run failed with "TF base_link → lf_foot_1 unavailable at the /joint_states stamp" | On a fresh launch, the FK check ran before the TF buffer had the chain; the lookup waited only 2 s of wall time | The tool now waits for the transform as a precondition and allows 10 s for the stamped lookup. M3 then passed |
| Refusal checks passed for the **wrong** reason: every one failed on "cannot read spiderx_legs.yaml" | With a custom `--config`, the loader looked for `spiderx_legs.yaml` next to the custom file | `spiderx_legs.yaml` now always comes from the installed config directory, and each refusal check requires its expected reason |
| The tip centroid counted shared STL vertices more than once | STL triangles repeat corner vertices | Vertices are deduplicated before averaging (13 unique vertices) |
| Orientation errors printed as exactly 0 | `acos(trace)` cannot resolve angles below about 1e-8 rad | Replaced with the `atan2` form, plus a unit test down to 1e-9 rad |
| The runtime script's leftover-process check matched its own calling shell | `pgrep -f` matched a parent command line that contained the search words | M3 excludes its own ancestor processes |

## Observed, not fixed (out of M3 scope)

- **Controller start-up race.** On one cloud start-up, `joint_state_broadcaster` failed to activate (`Switch controller timed out after 5 s`) while `leg_trajectory_controller` became active; a relaunch worked. This is an M1 launch race (`--switch-timeout`, and an unconditional `OnProcessExit`). It is proposed as a separate task. M3 reports this condition as a failure.
- **Screenshot.** The 10–15 mm foot lifts are only a few pixels in a clean Gazebo frame, so no image clearly shows the test. Following the M3 rule, **no screenshot was added**. The JSON report is the evidence.
