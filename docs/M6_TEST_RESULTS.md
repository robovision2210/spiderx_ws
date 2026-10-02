# M6.0 Test Results – Gait-Playback Safety Layers (cloud offline/mock only)

```text
M6.0 implementation is cloud offline/mock verified only. No valid trajectory was sent to a live
controller; no Gazebo playback, contact, locomotion, walking, navigation, or hardware operation
was performed. Local live read-only preflight and owner-approved M6.0-D playback remain pending.
```

## Outcome

- **Implementation complete for the four bounded layers:**
  - **A:** offline conversion and preflight;
  - **B:** mock-only action client;
  - **C:** live read-only preflight tool;
  - **D:** these docs.
- **Cloud verification is offline, mocked and static only.**
  - Clean build: 8 packages.
  - **932 tests, 0 errors, 0 failures, 0 skipped** after the provenance portability fix
    (`fd7a9de`; 912 before it).
  - The offline preflight CLI is deterministic. It refuses every invalid trajectory with exit 2.
  - Mutation tests show the dispatch gates, the single-goal rule and the no-auto-return rule are
    all enforced.
  - The M1–M4 static validators pass.
- **Not done in the cloud, by design:**
  - no Gazebo;
  - no launch file;
  - no controller manager;
  - no action client against a live server;
  - no goal, valid or invalid, sent to any live controller;
  - no `/cmd_vel`;
  - no hardware.
- **Pending:**
  - the live read-only preflight (M6.0-B) on the owner's Ubuntu PC;
  - M6.0-D playback, which needs separate owner approval;
  - local verification.

**Labels used below:**
- **[MEASURED]**: a command run in this cloud environment.
- **[RESULT]**: an offline output.

The design and the owner decisions are in [M6_GAIT_PLAYBACK_SAFETY_PLAN.md](M6_GAIT_PLAYBACK_SAFETY_PLAN.md)
§14. Usage is in [SPIDERX_M6_PLAYBACK_GUIDE.md](SPIDERX_M6_PLAYBACK_GUIDE.md).

## Environment [MEASURED]

| Item | Value |
|---|---|
| Branch | `claude/spiderx-m6-gait-playback-safety-plan`, from `origin/main` @ `99c835a` |
| Stack | ROS 2 Humble (RoboStack). Python 3.11.16. Stack versions match the plan §2.2 reference (collected by `m6_live_preflight --interface-only`). setuptools was pinned to 59.6.0 for `colcon build` and restored to 84.0.0 afterwards (see the testing guide). Unit tests ran with `PYTEST_DISABLE_PLUGIN_AUTOLOAD=1` |
| Machine | Cloud VM, no GPU. No simulator, launch file or controller was started. No hardware |

## Commits and test evidence [MEASURED]

Each code batch was checked the same way:
- a **clean** `colcon build --symlink-install`, with `build/` and `install/` removed first, building 8 packages;
- then the full `colcon test`;
- each gate had 0 errors and 0 failures;
- `colcon test` and `colcon test-result` both exited 0.

| Commit | Step | Full-suite total | New tests (file) |
|---|---|---|---|
| `99c835a` | `main` (M5 merged; baseline) | 700 | – |
| `bef3c25` | Docs: owner decision option (i), 0.1223 rad cap for M6.0-D | – | – |
| `0a9d2db` | A: offline conversion and trajectory preflight | 805 | 89 (`test_m6_trajectory`), 14 (`test_m6_offline_preflight`) |
| `d6ebaad` | B: single-goal action client, mock-only | 869 | 63 (`test_m6_action_client`) |
| `e9b1565` | C: live read-only preflight tool, mock-tested | 912 | 42 (`test_m6_live_preflight`) |
| `fd7a9de` | Fix: expanded-URDF provenance made workspace-path independent | **932** | 20 (`test_m6_trajectory` 89 → 109) |

- **How totals add up.** Each total is the earlier total, plus the new pytest cases, plus one
  CTest entry per new test file.
- **Final gate.** The final gate re-ran the clean build and the full suite at `e9b1565`:
  912 tests, 0 errors, 0 failures, 0 skipped. After the fix at `fd7a9de` a clean build and
  the full suite gave **932 tests, 0 errors, 0 failures, 0 skipped**.

## The three separate quantities

The code keeps these three apart. Tests pin each one.

| Quantity | Value | Where | Test evidence |
|---|---|---|---|
| Maximum commanded displacement from neutral (M6.0-D only) | **0.1223 rad**, accepted iff \|Δ\| ≤ 0.1223 + **1e-9** rad | `m6_envelope.M6_0_D_MAX_DISPLACEMENT_RAD`, `DISPLACEMENT_EPSILON_RAD` | 0.1223 and 0.1223 + 0.5e-9 are accepted. 0.1223 + 2e-9, 0.1224, 0.2 and −0.13 are refused with exactly `displacement_exceeds_cap` |
| Tracking tolerance | **0.05 rad** | `m6_envelope.TRACKING_TOLERANCE_RAD`. Used for the `/joint_states` comparison and the goal's `path_tolerance` / `goal_tolerance` | The crouch (0.0555 / 0.1223 rad) passes preflight. Changing the tracking tolerance to 1e-6 does not change preflight. The preflight source never references it |
| Joint-limit soft margin | **0.05 rad**, read from `spiderx_legs.yaml` (= `joint_safety.DEFAULT_MARGIN_RAD`) | `Sources.soft_margin_rad` | A tightened limit fires only `joint_limit_violation`. The same limit without the margin passes. A changed margin is refused (`soft_margin_inconsistent`) |

**A fourth, separate value:** the start-pose tolerance of **0.05 rad**, the existing M4
`START_POSE_TOL_RAD`. The owner approved it for M6.0-D only ([plan §14.9](M6_GAIT_PLAYBACK_SAFETY_PLAN.md#149-final-owner-decisions-2026-10-02)).
- It is an observed `/joint_states` comparison against the expected neutral.
- A failure must block goal construction and dispatch.
- Today only the live read-only preflight evaluates it. Wiring it in front of goal construction is
  a requirement for the future M6.0-D runner.

## Offline preflight evidence (M6.0-A) [MEASURED]

> **Provenance-ID migration (`fd7a9de`).** The `trajectory_id` below, `b884584ed4aeddf0`, is the
> superseded pre-fix value. After the fix the same trajectory has `trajectory_id`
> **`44f0a7ad52e5c330`**; see [Provenance portability fix](#provenance-portability-fix-fd7a9de).
> Only the identity and provenance hash changed. The trajectory content (points, times,
> positions, displacement) is unchanged.

**Valid trajectory.**
- Command: `ros2 run spiderx_controller m6_offline_preflight --out log/m6_playback/offline_run1`,
  then again with `offline_run2`. Both exited 0.
- Result:
  - **PASS**, `trajectory_id = b884584ed4aeddf0`;
  - 3 points: neutral at 3.0 s, `crouch_10mm` at 6.0 s, neutral at 9.0 s;
  - start delay 3.0 s;
  - duration 9.0 s;
  - all 12 checks passed.
- The maximum commanded displacement is **0.1222941360 rad** (`rr_foot_joint`), within the
  0.1223 rad cap.
- The crouch waypoint is the exact M4 `pose_command(plan, 'crouch_10mm')`.
- **Determinism:** `diff -r` of the two runs is empty, so they are byte-identical.

  | File | SHA-256 |
  |---|---|
  | `trajectory.json` | `b298d86015de3d3df68e9ca3750ee7728bca25a14a29a9c2ddc304936361c8d5` |
  | `preflight.json` | `235d8ab49fbb37d7cd4fb5d871834db555fc1627d679ad77bce8c7cba806c614` |

- Re-running into an existing output directory is refused with exit 2. Nothing is overwritten.

**Rejections.** Each mutated copy of the trajectory, with its ID re-stamped, was given to
`--check-only --trajectory`:

| Mutation | Exit | Failure codes |
|---|---|---|
| crouch foot joint 0.1224 rad | 2 | `displacement_exceeds_cap`, `source_pose_mismatch` |
| non-monotonic time (last point 5.0 s) | 2 | `time_not_strictly_increasing` |
| NaN position | 2 | `non_finite_value` |
| duration 31 s | 2 | `duration_exceeds_max` |
| mode `cyclic` | 2 | `mode_not_single` |
| reversed joint order | 2 | `joint_order_not_canonical` |

**Unit-test coverage** (`test_m6_trajectory`, 89 tests; `test_m6_offline_preflight`, 14 tests):
- every one of the 32 documented failure codes is exercised;
- 17 single-condition mutations produce **only** their own code;
- other coverage includes:
  - source failures: missing, incomplete or inconsistent neutral; changed controller order; an M4-refused crouch; unreadable config;
  - staleness after a source edit;
  - tampered or missing `trajectory_id`;
  - JSON round-trip;
  - the offline modules import no ROS client library.

## Mock action-client evidence (M6.0-C) [MEASURED]

These are `test_m6_action_client`, 63 tests. They run against a deterministic test double
(`test/m6_mock_action.py`) only.

**Goal construction.**
- A goal is built only from a passing preflight; `build_goal` re-runs the preflight.
- The goal has:
  - canonical joint order;
  - header stamp 0;
  - 3 points at (3, 0), (6, 0) and (9, 0);
  - zero velocities;
  - no accelerations or effort.
- `path_tolerance` and `goal_tolerance` both have 12 entries, one per joint: `position` 0.05 and
  `velocity`/`acceleration` 0, which means "unspecified".
- `goal_time_tolerance` is 1.0 s.
- The controller YAML has no `constraints` block and was not changed.

**Invalid input never reaches construction or dispatch.** The invalid inputs are:
- malformed;
- incomplete;
- NaN;
- non-monotonic;
- out-of-limit;
- beyond the cap;
- stale;
- a tampered ID;
- cyclic;
- not a mapping.

For each one, the mock server records **0 calls**. The outcome is `refused`, every channel is
`unavailable` and `goals_sent` is 0.

**Structured failures.**

| Server behaviour | Outcome | Channels and other evidence |
|---|---|---|
| Rejection | `rejected` | `action` failed |
| Abort −1, −2 or −3 | `failed` | `goal_tolerances` unavailable; error name recorded |
| Abort −4 or −5 | `failed` | `goal_tolerances` failed |
| Result timeout | `timed_out` | One cancel request; holding position |
| No answer to the goal request | `timed_out` | – |
| `KeyboardInterrupt` | `canceled` | One cancel; holding position |
| Adapter error | `error` | One cancel; holding position |
| Action success with no `/joint_states` | `failed` | `tracking` is `unavailable`. Action success alone is not tracking evidence |
| Offset of 0.051 rad | `failed` | `tracking` failed |
| Offset of 0.049 rad | – | `tracking` passed |

**Single goal.**
- In every failure scenario, at most one goal is sent: the preflighted goal.
- A second `run()` raises `SecondGoalForbidden`, after success and after a refusal alike.

**Mutation tests.** Each deliberately broken client is detected:
- `test_mutation_removed_dispatch_gate_is_detected`: both gates removed, so invalid input reaches
  the server;
- `test_mutation_removed_session_gate_is_detected_by_spy`: with the session gate removed, the goal
  builder is reached;
- `test_mutation_removed_goal_construction_preflight_is_detected`: goals are built for invalid
  input;
- `test_mutation_retry_or_auto_return_is_detected` with three mutants:
  - `RetryingSession`;
  - `AutoReturnSession`, which sends a neutral goal after a timeout;
  - `AutoReturnOnCancelSession`, which sends the neutral waypoint after a cancel.

**Installed interface checks.**
- The action name is `/leg_trajectory_controller/follow_joint_trajectory`, and the controller type
  is a JointTrajectoryController.
- The installed `FollowJointTrajectory.action` has the fields used.
- The error codes 0 to −5 and the `GoalStatus` codes 4, 5 and 6 match the installed messages.
- The installed `tolerances.hpp` reads `goal.path_tolerance`, `goal.goal_tolerance` and
  `goal.goal_time_tolerance` (`get_segment_tolerances`). This test ran; it was not skipped.

## Live read-only preflight evidence (M6.0-B tool) [MEASURED]

**Mock-tested only in the cloud** (`test_m6_live_preflight`, 42 tests).

**Requirements.** Each of the 17 not-ready conditions alone gives exactly its own code:
- missing, ambiguous or wrongly typed action server;
- controller manager unavailable;
- controller missing, inactive or of the wrong type;
- `/joint_states` with no publisher, several publishers or the wrong type;
- no messages, or stale messages;
- name mismatch;
- incomplete or NaN positions;
- start pose outside 0.05 rad;
- interface contract mismatch;
- probe error.

The verdict is then `NOT READY` and the exit code 1.

**Versions.** A version difference is reported visibly (D6) but is not a failure.
- The owner-approved classification is in [plan §14.9](M6_GAIT_PLAYBACK_SAFETY_PLAN.md#149-final-owner-decisions-2026-10-02): `compatible`, `warning` or
  `incompatible`; only `incompatible` blocks.
- The tool does not yet emit these labels. It records the actual versions with
  `matches` / `differs` / `unavailable` and reports the blocking contract failures separately.

**No-dispatch guarantee.** A fake node raises on any method outside a read-only allow-list:
- graph queries;
- one `ListControllers` client;
- one `/joint_states` subscription;
- their destruction.

The probe completed with no violation. The module source contains none of these:
- `send_goal`;
- `ActionClient`;
- `create_publisher`;
- `.publish(`;
- `subprocess`, `Popen` or `os.system`;
- `switch_controller` or `load_controller`;
- `set_parameters`;
- `m6_action_client`;
- `cmd_vel`.

Importing the tool loads neither `rclpy` nor `control_msgs`.

**Timeouts.**
- A service that is unavailable or unanswered becomes `controller_manager_unavailable`.
- The sampling window is bounded by the clock.
- At most 200 messages are kept.

**Cloud run of the tool.** It ran only as `--interface-only`, which creates no ROS node.
- Result: exit 0, verdict `INTERFACE ONLY`, interface contract passed.
- Installed versions match the reference: `joint_trajectory_controller` 2.48.0,
  `control_msgs` 4.8.0, `trajectory_msgs` 4.9.0, `controller_manager` 2.51.0, `rclpy` 3.3.16.
- The report was written to `log/m6_playback/live_preflight/20261002T052138Z/live_preflight.json`
  (git-ignored).
- Afterwards, one `ros2 node list` was run to confirm that no node existed. It listed nothing. The
  `ros2` CLI daemon it started was then stopped.
- **The graph mode of the tool was not run in the cloud.**

## Static regression [MEASURED]

These are the M1–M4 static validators, run without `--runtime`, so they started no simulator.

| Validator | Result |
|---|---|
| `validate_fortress.sh` | 58 PASS, 0 FAIL, `All checks passed.` |
| `validate_m1_control.sh` | 14 PASS, 0 FAIL, `All M1 checks passed.` |
| `validate_m2_posture.sh` | 17 PASS, 0 FAIL, `All M2 checks passed.` |
| `validate_m3_kinematics.sh` | 17 PASS, 0 FAIL, `All M3 checks passed.` |
| `validate_m4_all_leg_ik.sh` | 18 PASS, 0 FAIL, `All M4 checks passed.` |

After the runs, `pgrep` found no Gazebo, bridge, `robot_state_publisher`, spawner or launch
process.

## Scope audit [MEASURED]

**Diff.** `git diff origin/main..HEAD` touches only:
- `docs/M6_GAIT_PLAYBACK_SAFETY_PLAN.md`;
- new `m6_*` modules, scripts and tests in `spiderx_controller`;
- 6 appended lines in `src/spiderx_controller/CMakeLists.txt`: 2 script installs and 4 test
  registrations.

**Unchanged:**
- URDF/xacro, meshes and worlds;
- controller YAML and the controller launch;
- bringup;
- `trajectory_client.py`;
- the M1–M5 modules, tools and scripts;
- `package.xml`;
- the M4.5/M5 configs.

**Generated output** exists only under the git-ignored `log/m6_playback/`. No `log/`, `build/` or
`install/` file is tracked.

## `goal_time_tolerance` (owner-approved 2026-10-02)

**`goal_time_tolerance` = 1.0 s**, approved for the bounded M6.0-D trajectory only
([plan §14.9](M6_GAIT_PLAYBACK_SAFETY_PLAN.md#149-final-owner-decisions-2026-10-02)). This is the existing settled-pose convention: `settle_s` in
`m3_kinematics_targets.yaml` and `m4_pose_validation.SETTLE_S`.
- It is a finite, M6.0-specific deadline set only in the goal message. It is not a controller-YAML
  change.
- In the installed controller, 0 would mean "unchecked". 1.0 s makes the controller abort with
  `GOAL_TOLERANCE_VIOLATED` if the 0.05 rad goal tolerance is not met within 1.0 s after the
  trajectory ends.
- It is **not** a claim that controller tracking is independently validated. Tracking evidence
  comes only from the `/joint_states` channel, and none has been gathered live.

## Provenance portability fix (`fd7a9de`)

**Finding (owner's local verification).** The same trajectory got different identities on the two
machines:
- cloud: `trajectory_id b884584ed4aeddf0`;
- owner's Ubuntu PC: `trajectory_id 1280770cae26aa54`.

Everything else was identical:
- the M4 pose hash;
- the 12 joints and their order;
- the three points and the 3.0 / 6.0 / 9.0 s times;
- the 0.1222941360 rad maximum displacement at `rr_foot_joint`.

**Cause.** The provenance included a SHA-256 of the xacro-expanded URDF. Its 60
`<mesh filename>` attributes are absolute, machine-specific URIs, such as
`file:///home/user/spiderx_ws/install/...` versus `file:///home/jagadeswar/spiderx_ws/install/...`.
The ID was therefore path-dependent.

**Fix (provenance hashing only).** `m6_trajectory.urdf_provenance_sha256` hashes a deep copy
of the URDF in which each `<mesh filename>` matching
`^file://(/<path>)?/share/spiderx_description/meshes/<rel>$` is rewritten to
`package://spiderx_description/meshes/<rel>`.
- Nothing else is touched:
  - no other element or attribute;
  - no other package;
  - no `src/` path;
  - no plain path;
  - no URI with a query or fragment.
- The URDF used for geometry, the xacro output, the install tree and the meshes are all unchanged.
- The rewrite is idempotent.

**Evidence [MEASURED].**

| Expanded URDF | Raw SHA-256 (old) | Canonical SHA-256 (new) | `trajectory_id` (new) |
|---|---|---|---|
| Cloud, actual `/home/user/...` install | `2af3fefeb0a1…` | `915125596b3d…` | `44f0a7ad52e5c330` |
| Cloud-like relocated prefix | `fc7785a66b3d…` | `915125596b3d…` | `44f0a7ad52e5c330` |
| Local-like `/home/jagadeswar/...` prefix | `6e6693a4344d…` | `915125596b3d…` | `44f0a7ad52e5c330` |

**Offline runs after the fix.**
- Two CLI runs into `log/m6_playback/fix_run1` and `fix_run2` both exited 0 with PASS,
  `trajectory_id 44f0a7ad52e5c330`.
- `diff -r` is empty, so the runs are byte-identical:
  - `trajectory.json`: `0ceff902410b38ceea255b880a544c043bad05b69f0288117737d4e14a20abab`;
  - `preflight.json`: `a3ed82168761ef522778a0304314c9451e6eba90c29756312dbcc65a9f2ae80c`.
- Refusals are unchanged, each exiting 2:
  - beyond the cap: `displacement_exceeds_cap` and `source_pose_mismatch`;
  - non-monotonic time: `time_not_strictly_increasing`;
  - NaN: `non_finite_value`;
  - 31 s: `duration_exceeds_max`;
  - cyclic: `mode_not_single`;
  - reversed order: `joint_order_not_canonical`;
  - a changed URDF hash: `source_stale`;
  - an edited but un-restamped file: `trajectory_id_mismatch`.
- The CLI loaded no ROS client library.

**Stale and tamper protection is unchanged.** Twenty new tests (`test_m6_trajectory`) show that:
- cloud-like and local-like prefixes give the same hash and the same `trajectory_id`;
- a trajectory from one prefix passes preflight against the other;
- a different mesh filename or mesh subpath changes the hash;
- a changed joint limit, inertia, mass or origin value changes the hash, and such a change is
  refused as `source_stale`;
- unrelated URIs and non-mesh attributes are left unchanged;
- the canonicalization is idempotent and never modifies its input.

No existing test was changed or weakened. No ID is pinned in the tests, so no expected value
needed migrating.

**Local verification remains pending** until the owner reruns it. With identical sources, the
owner's PC should now also report `trajectory_id 44f0a7ad52e5c330`.

## Pending (not claimed)

**1. Local verification on the owner's Ubuntu PC:**
- build;
- full suite;
- offline preflight;
- `--interface-only`;
- the M6.0-B live read-only preflight against a running `fortress_control.launch.py`.

**2. M6.0-D**, one neutral → `crouch_10mm` → neutral goal. It needs:
- a separate owner approval;
- a live adapter, which does not exist yet and is not part of this PR;
- a local run.

M6.1 remains deferred (D2).

```text
M6.0 implementation is cloud offline/mock verified only. No valid trajectory was sent to a live
controller; no Gazebo playback, contact, locomotion, walking, navigation, or hardware operation
was performed. Local live read-only preflight and owner-approved M6.0-D playback remain pending.
```
