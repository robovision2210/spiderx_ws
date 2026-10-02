# M6.0 Plan – Gazebo Gait-Playback Safety Plan

> **Phase 0 (read-only audit and plan only).** Written before any M6 code. Branch
> `claude/spiderx-m6-gait-playback-safety-plan`, created from `origin/main` @ `99c835a` (M5 merged;
> the tree of `99c835a` is identical to the cloud + locally verified M5 head `c344a2a`).

```text
M6.0 is NOT walking. It defines a safe, observable, bounded way to replay ONE precomputed,
offline-validated joint trajectory in Gazebo through the existing controller stack.
Nothing in SpiderX is verified as walking, dynamically stable, navigating, or running on hardware.
```

**Labels:**
- **[FACT]**: read in this repository or in the installed ROS 2 stack, with a path:line reference.
- **[RESULT]**: an existing verified result, or an audit probe run read-only and in memory.
- **[HYPOTHESIS]**: believed but not proven; it must be checked at the M6 step named.
- **[PLAN]**: a proposed M6 rule or implementation.
- **[OWNER DECISION]**: a choice for the owner (§12).

> **Owner decision addendum (2026-10-02).** The owner approved decisions D1–D7 and revised
> M6.0-C; see [§14](#14-owner-decision-addendum-2026-10-02). Where §1–§13 disagree with §14, **§14 governs**. Superseded passages are
> marked inline.

## Contents

1. [Status and scope boundary](#1-status-and-scope-boundary)
2. [Repository and installed-stack provenance](#2-repository-and-installed-stack-provenance)
3. [Audited action and controller interface contract](#3-audited-action-and-controller-interface-contract)
4. [Current command and data flow](#4-current-command-and-data-flow)
5. [Conservative first-replay safety envelope](#5-conservative-first-replay-safety-envelope)
6. [Preflight design and rejection behaviour](#6-preflight-design-and-rejection-behaviour)
7. [Conversion compatibility contract (M4.5/M5 → M6)](#7-conversion-compatibility-contract-m45m5--m6)
8. [Verification ladder and claims](#8-verification-ladder-and-claims)
9. [Logging, provenance, determinism and cleanup](#9-logging-provenance-determinism-and-cleanup)
10. [Test strategy](#10-test-strategy)
11. [Planned implementation batches](#11-planned-implementation-batches)
12. [Risk register and owner decisions](#12-risk-register-and-owner-decisions)
13. [Out of scope](#13-out-of-scope)
14. [Owner decision addendum (2026-10-02)](#14-owner-decision-addendum-2026-10-02)

---

## 1. Status and scope boundary

- **Status [PLAN].** M6.0 Phase 0 is complete when this document is committed. M6.0 Phase 1
  starts only after the owner reviews §12.
- **What M6.0 may eventually show:**
  - a validated trajectory can be converted, preflighted and **refused** without sending anything;
  - the controller's action interface is available and its controllers are active;
  - ~~an invalid goal is rejected with no motion~~ (superseded by [§14](#14-owner-decision-addendum-2026-10-02): invalid-goal handling is
    proven against a **mock** action server only; no invalid goal is sent to the live controller);
  - **one** bounded, non-cyclic, neutral-to-neutral, all-joint trajectory is executed in Gazebo,
    and the commanded and observed joint positions are logged and compared.
- **What M6.0 never shows:** walking, gait execution on the robot, body support, translation,
  contact behaviour, dynamic stability, balance, navigation, real-time behaviour, actuator
  capability, power or efficiency, or hardware readiness.
- **Phase 0 boundary [FACT].** During this audit:
  - no Gazebo, `ros2 launch`, controller, action client, goal, topic publication or `/cmd_vel`
    was used;
  - no code, YAML, URDF, launch, test or package file was changed;
  - only this file is added.
- **Separate milestone.** M5.5 (command-velocity bridge) remains separate future work.

## 2. Repository and installed-stack provenance

### 2.1 Repository [FACT]

| Item | Value |
|---|---|
| `origin/main` | `99c835a` (merge of `c344a2a` into `3db1aec`); `git diff c344a2a 99c835a` is empty |
| Milestones merged | M1–M5, documented as cloud and local verified (`STATUS.md`) |
| Artifact policy | `.gitignore:3` `log/`. Existing tools write under `log/` (for example `m4_pose_validation.py:74-75`; M5 `eval_records.DEFAULT_OUT`) |

**Files audited, with high-value lines:**
- **Controller config.** `spiderx_controller/config/spiderx_ros2_controllers.yaml`:
  - controller manager at :13-22;
  - joints at :26-38;
  - interfaces at :39-43;
  - rates at :44-45;
  - `allow_partial_joints_goal: false` at :48;
  - **no `constraints` block**.
- **Legacy Classic config.** `spiderx_description/config/controllers.yaml:14-31` declares a legacy
  `JointGroupPositionController` with a **different joint order** (rr, lr, rf, lf). It is
  unvalidated and must never feed M6.
- **Controller launch.** `spiderx_controller/launch/controller.launch.py:26-47` (spawner timeouts)
  and `:56-90` (success gate, no automatic shutdown).
- **Bringup launches.**
  - `spiderx_bringup/launch/fortress_control.launch.py:44-56` includes the simulation and
    `controller.launch.py`.
  - `fortress_posture_hold.launch.py:36-49` adds the ground-truth bridge.
- **gz_ros2_control config.** `spiderx_description/urdf/spiderx_fortress_control.xacro:21-23, 27-29,
  47-48`: position command, position and velocity state interfaces, the `gz_ros2_control` plugin.
- **URDF limits.** `spiderx_description/urdf/spiderx.urdf.xacro:725-885`: twelve `<limit>` tags,
  with effort 100 and velocity 100 (exporter placeholders).
- **Joint config.** `spiderx_controller/config/spiderx_legs.yaml`:
  - margin 0.05 at :17;
  - `max_joint_velocity_rad_s: 0.5` `SIMULATION_PLACEHOLDER` at :24;
  - joints and limits at :29-61.
- **Shared action client.** `spiderx_controller/spiderx_controller/trajectory_client.py:16, 23-29,
  65-101`.
- **Existing tools.**
  - `scripts/test_one_joint.py:31-45, 53-63, 76-82`;
  - `scripts/test_neutral_pose.py:6, 29-30`;
  - `spiderx_controller/m4_pose_validation.py`: constants at :54-58, pacing at :78-80, preflight at
    :297-336, Ctrl+C at :476-480.
- **Joint safety and joint order.**
  - `spiderx_controller/joint_safety.py:12, 19-32, 47, 62`;
  - `leg_kinematics.py:613-615` (`all_joint_names` = controller order).
- **M4.5 and M5.**
  - `gait_phase.py:61` (`u_k = k/n`);
  - `gait_trajectory.py:71-83` (`t = u·T`);
  - `gait_kinematics.py:56-83` (IK series; sample 0 reference = CAD neutral);
  - `gait_report.py:103-110` (`samples.csv` q columns);
  - `eval_runner.py:425-426` (**M5 discards q series after the gate**);
  - `eval_study.py:94-99` (`config_id` / `eval_id`);
  - `eval_records.py:237-238`.
- **Earlier pacing configs.** `m3_kinematics_targets.yaml:57-59` (min duration 3.0 s, speed factor
  2.0, settle 1.0 s) and `m2_simulation_postures.yaml:56` (M1 tracking tolerance).
- **Start-up and shutdown hazards.**
  - `docs/M4_1_TEST_RESULTS.md:174-178, 209, 376` (a launch-only shutdown can orphan the Gazebo
    server and GUI; SIGINT to the process group is the proven path);
  - `scripts/validate_m4_all_leg_ik.sh:124, 186, 192-195` (`setsid` launch, `kill -INT -- -pgid`,
    `pgrep` leftover check);
  - `docs/SPIDERX_TESTING_GUIDE.md:39`.

### 2.2 Installed ROS 2 Humble stack (cloud RoboStack environment `/opt/mm/root/envs/humble`) [FACT]

| Package | Version | Evidence used |
|---|---|---|
| `ros-humble-joint-trajectory-controller` / `ros2-controllers` | **2.48.0** | `include/joint_trajectory_controller/joint_trajectory_controller_parameters.hpp:71-119` (parameter defaults); `lib/libjoint_trajectory_controller.so` (log and rejection strings; the `.cpp` source is not installed) |
| `ros-humble-control-msgs` | **4.8.0** | `share/control_msgs/action/FollowJointTrajectory.action`, `msg/JointTolerance.msg` |
| `ros-humble-trajectory-msgs` | **4.9.0** | `share/trajectory_msgs/msg/JointTrajectory.msg`, `JointTrajectoryPoint.msg` |
| `ros-humble-controller-manager`, `hardware-interface`, `joint-limits`, `ros2-control` | **2.51.0** | (version only) |
| `ros-humble-gz-ros2-control` / `ign-ros2-control` | **0.7.15** | `lib/libgz_ros2_control-system.so` (no string evidence of command limit enforcement) |
| `ros-humble-rclpy` | **3.3.16** | used by `trajectory_client.py` |
| `ros-humble-realtime-tools` | 2.14.0 | (version only) |

**[OWNER DECISION D6 / PLAN]** The owner's Ubuntu PC uses apt packages, whose versions may differ.
M6 Batch B must record the versions on both machines and repeat the interface checks there.

## 3. Audited action and controller interface contract

| Item | Value | Source |
|---|---|---|
| Controller | `leg_trajectory_controller`, type `joint_trajectory_controller/JointTrajectoryController` | `spiderx_ros2_controllers.yaml:21-22` |
| Prerequisite controller | `joint_state_broadcaster` (publishes `/joint_states`), spawned first; the trajectory controller starts only after its spawner exits 0 | `controller.launch.py:56-90` |
| Action endpoint | `/leg_trajectory_controller/follow_joint_trajectory` | `trajectory_client.py:16` |
| Action type | `control_msgs/action/FollowJointTrajectory` (control_msgs 4.8.0) | installed `.action` |
| Joints (order) | `lf_hip, lf_thigh_joint, lf_foot_joint, rf_hip, rf_thigh_joint, rf_foot_joint, lr_hip, lr_thigh_joint, lr_foot_joint, rr_hip, rr_thigh_joint, rr_foot_joint` | `spiderx_ros2_controllers.yaml:26-38`; same as `leg_kinematics.all_joint_names` (`:613-615`) |
| Partial goals | refused (`allow_partial_joints_goal: false`); all 12 joints in every goal | `:48` |
| Interfaces | command: position; state: position, velocity | `:39-43`; `spiderx_fortress_control.xacro:21-23` |
| Rates | controller manager `update_rate` 100 Hz; JTC `state_publish_rate` 50 Hz; `action_monitor_rate` 20 Hz | `:15, 44, 45` |
| Clock | `use_sim_time: true`; the existing clients run on `/clock` | `:16`; `trajectory_client.py:24` |

**Goal fields** (`FollowJointTrajectory.action:2-27`):
- `trajectory` (`JointTrajectory`: `header`, `joint_names`, `points[]`);
- `multi_dof_trajectory` (unused);
- `path_tolerance`, `component_path_tolerance`;
- `goal_tolerance`, `component_goal_tolerance`;
- `goal_time_tolerance`.

`JointTrajectoryPoint` has `positions`, `velocities`, `accelerations`, `effort` and
`time_from_start`, all in `joint_names` order (`JointTrajectoryPoint.msg:1-21`).

**Result and feedback** (`.action:29-54`):
- The result carries `error_code` and `error_string`. The codes are `SUCCESSFUL 0`,
  `INVALID_GOAL −1`, `INVALID_JOINTS −2`, `OLD_HEADER_TIMESTAMP −3`, `PATH_TOLERANCE_VIOLATED −4`
  and `GOAL_TOLERANCE_VIOLATED −5`.
- Feedback carries `header`, `joint_names`, and `desired`, `actual` and `error` points.

**Installed controller defaults that apply because the YAML does not set them [FACT]**
(`joint_trajectory_controller_parameters.hpp:76-101`):

| Parameter | Default | Consequence for M6 |
|---|---|---|
| `constraints.goal_time` | 0.0 | No goal-time check. The library describes zero as "the controller will wait a potentially infinite amount of time" |
| `constraints.<joint>.trajectory`, `.goal` | 0.0 | Path and goal tolerances are **unchecked**, so **`SUCCESSFUL` does not prove tracking** |
| `constraints.stopped_velocity_tolerance` | 0.01 | – |
| `allow_nonzero_velocity_at_trajectory_end` | **true** | A non-zero final velocity is **accepted** by the controller, so M6 must refuse it client-side |
| `cmd_timeout` | 0.0 | Disabled |
| `interpolation_method` | `"splines"` | Exact behaviour for position-only points is [HYPOTHESIS]; it is observed through feedback `desired` in M6.0-D |
| `open_loop_control`, `allow_integration_in_goal_trajectories` | false, false | – |

**Rejection and abort behaviour of the installed controller** [FACT, from the strings in
`libjoint_trajectory_controller.so`; logic not re-read because the `.cpp` is not installed]:

| Case | Behaviour |
|---|---|
| Empty `joint_names` | "Empty joint names on incoming trajectory." |
| Empty trajectory | "Empty trajectory received." |
| Unknown joint | "Incoming joint %s doesn't match the controller's joints." |
| Per-point size mismatch | "Mismatch between joint_names size … at point #…" |
| Non-monotonic times | "Time between points %zu and %zu is not strictly increasing" |
| Stale stamp | "Received trajectory with non-zero start time … that ends in the past" |
| Controller inactive | "Can't accept new action goals. Controller is not running." |
| Second goal | **Preempts the first** ("Current goal cancelled due to new incoming action.") |
| Cancel | "Canceling active action goal because cancel callback received." |
| Aborts | "Aborted due to goal_time_tolerance …", "Aborted due to path tolerance violation", "Aborted due to state tolerance violation", "Aborted due to command timeout" |
| Joint order | The controller reorders the goal into its own order (`sort_to_local_joint_order`) |
| Not found | **No** string evidence of NaN/finite checks or joint-position-limit checks in the controller, and **no** command limit enforcement string in `gz_ros2_control` |

So M6 must reject non-finite values and limit violations **before** sending [PLAN]. Gazebo's
URDF joint stops act physically; that is not command validation.

**Existing client behaviour [FACT]** (`trajectory_client.py:65-101`):
- it sends **one point** with zero velocities and **no header stamp**, so the goal starts on
  receipt;
- it waits for the server 10 s (wall clock), and for the result `10 × duration + 30` s (wall clock);
- it returns `(accepted, error_code)` and has a `cancel_active_goal()`.

**Preflight practice already proven in M4** (`m4_pose_validation.py:297-336`):
- `/clock` is present;
- both controllers are exactly `active`;
- exactly one `/joint_states` publisher, at the start and at the end (:318-320, :390-392);
- all 12 joints are present;
- the start pose is within 0.05 rad of CAD neutral;
- an FK check passes.

## 4. Current command and data flow

```text
 (1) M5 offline result / M4.5 config ──► (2) M6 conversion boundary ──► (3) FollowJointTrajectory client
          EXISTS, offline-verified           FUTURE [PLAN]                   EXISTS for single-point goals
                                                                             (trajectory_client.py);
                                                                             multi-point: FUTURE
 ──► (4) leg_trajectory_controller ──► (5) controller_manager ──► (6) gz_ros2_control ──► (7) Gazebo joints
          EXISTS, proven for single-       EXISTS, proven (M1-M4.1)    EXISTS, proven (M1-M4)    EXISTS, proven
          point goals (M1, M2, M4)
 ──► (8) /joint_states, controller state, action feedback/result ──► (9) M6 verification logs
          /joint_states + result: EXISTS, proven (M1-M4); feedback: EXISTS, never consumed     FUTURE [PLAN]
```

| Arrow | Status |
|---|---|
| (1) | **Exists** offline. M5 records (`eval_id`, effective configuration) are deterministic and byte-reproducible. M5 does **not** store joint series (`eval_runner.py:425-426`); M4.5 `samples.csv` stores them at 10 significant digits under the ignored `log/` |
| (1)→(2) | **Future.** No converter exists |
| (2)→(3) | **Future** |
| (3)→(4)→…→(7) | **Proven** for single-point, all-12-joint goals (M1 one joint, M2 posture, M4 three poses, cloud + local). **Never exercised** with multi-point trajectories |
| (8) | `/joint_states` and the result are **proven**. Action feedback exists but has never been consumed or verified |
| (9) | **Future** |

## 5. Conservative first-replay safety envelope

Every value below is **derived from an existing repository source**, or marked as an
**[OWNER DECISION]**. No new numbers are invented. **[PLAN]**

| Rule | Value | Source |
|---|---|---|
| Start pose | All 12 joints within **0.05 rad** of CAD neutral (q = 0), observed on fresh `/joint_states`. Otherwise refuse and tell the user to run `test_neutral_pose.py` | `m4_pose_validation.py:58` (`START_POSE_TOL_RAD`) |
| Position limits | Every point inside **URDF limits −/+ 0.05 rad**. Refuse; **never clamp** | `spiderx_legs.yaml:17, 35-60`; `joint_safety.check_pose` |
| Per-joint delta from neutral | **[OWNER DECISION D1]** | Evidence: 0.05 rad start tolerance; 0.2 rad single-joint target verified in M1 (`test_one_joint.py:30`); M4 validated pose angles; shipped gait peak 0.18–0.20 rad [RESULT] |
| Joint speed | Every segment's peak \|Δq\|/Δt ≤ **0.5 rad/s** (a `SIMULATION_PLACEHOLDER`, not a servo value) | `spiderx_legs.yaml:24` |
| Segment and lead-in duration | ≥ **max(3.0 s, 2.0 × max\|Δq\| / 0.5 rad/s)** for every move between waypoints that start or end at rest | `m3_kinematics_targets.yaml:57-58`; `m4_pose_validation.py:78-80` |
| Settle after motion | **1.0 s** of simulation time before the post-motion observation | `m3_kinematics_targets.yaml:59` |
| Total trajectory duration | M6.0-D: the sum of the segments above, **≤ [OWNER DECISION D4]** | – |
| Point spacing | Strictly increasing `time_from_start`. Spacing ≥ **2 controller periods (0.02 s)**, with 100 Hz `update_rate` | `spiderx_ros2_controllers.yaml:15` (derived floor; tighter values are owner choices) |
| Point count | M6.0-D: **≤ [OWNER DECISION D4]**. For a later single gait cycle: n = 200 samples + lead-in and lead-out points (shipped spacing 0.047–0.080 s [RESULT]) | M4.5 `samples_per_cycle` |
| Header and start | `header.stamp = 0` (start on receipt, as today). The first point is never at t = 0 unless it equals the observed start (within 0.05 rad); otherwise its `time_from_start` = the lead-in duration above | `trajectory_client.py:71-78`; installed stale-stamp rule |
| Velocities | Positions **and** velocities given. The final point velocity must be exactly 0 (refused client-side, because the controller would accept non-zero). M6.0-D uses zero velocity at **every** waypoint (stop-and-go) | installed default `allow_nonzero_velocity_at_trajectory_end = true` |
| Accelerations, effort | Not sent | – |
| Goals | **One goal**, never cyclic or repeated, **no automatic retry**. A second goal would preempt the first, so it is forbidden while a goal is active | installed preemption behaviour |
| Tolerances in the goal | **[OWNER DECISION D3]** | Installed defaults are unchecked |
| Wall-clock watchdog | Result wait ≤ `10 × trajectory duration + 30` s (as today). On expiry: cancel, record `timeout`, exit non-zero | `trajectory_client.py:86-88` |
| Abort and Ctrl+C | **[OWNER DECISION D5]** | Today's tools cancel and then send a return-to-neutral trajectory (`m4_pose_validation.py:476-480`; `test_one_joint.py:79-81`) |
| Shutdown | SIGINT to the launch **process group** only, then a `pgrep` leftover check | `validate_m4_all_leg_ik.sh:186, 192-195`; `M4_1_TEST_RESULTS.md:376` |

**Dry run (no-motion validation mode) [PLAN].**
- `--validate-only` converts, preflights offline and prints the full planned goal. **No ROS node
  is created.**
- `--dry-run` additionally connects read-only: it checks the controllers, the action server,
  `/joint_states` and the start pose. It **creates no goal object that could be sent** and exits
  before any send call.
- A test proves that the action client's send function is never reached in either mode, using a
  mock that fails the test if it is called.

**Command-rejection proof [PLAN].** For every refusal reason in §6, a unit test asserts:
- the refusal message names the reason;
- exit code 2;
- the mocked `send_goal_async` was called **0 times**.

## 6. Preflight design and rejection behaviour

**Offline preflight** (no ROS; all must pass; any failure refuses with exit 2 and sends nothing)
**[PLAN]:**
1. The trajectory has exactly the 12 controller joint names in controller order. Refuse
   duplicates, unknown names, missing names, and the legacy-Classic order.
2. Every point has 12 positions and 12 velocities. Every value is finite (no NaN or inf).
3. `time_from_start` is strictly increasing, the first value is > 0, and spacing respects §5.
4. Every position is within the soft limits (URDF ± 0.05 rad). No clamping.
5. Every segment respects the 0.5 rad/s speed rule and the minimum durations.
6. The final velocity is exactly 0; for M6.0-D every waypoint velocity is 0.
7. The per-joint delta from neutral and the total duration are within the owner-decided envelope.
8. Point count is within the envelope. It is **one** trajectory, with no repetition flag.
9. Provenance is complete: source, generator commit, configuration hashes and a deterministic
   trajectory ID (§9). A **stale** trajectory, whose recorded source commit or configuration
   hashes differ from the current tree, is refused.

**Runtime preflight** (read-only ROS; any failure exits 2 before a goal exists) **[PLAN]**,
mirroring M4 (`m4_pose_validation.py:297-336`):
1. `/clock` is received.
2. `/controller_manager/list_controllers` reports `joint_state_broadcaster` and
   `leg_trajectory_controller` **exactly `active`**.
3. The action server `/leg_trajectory_controller/follow_joint_trajectory` is available (10 s).
4. `count_publishers('/joint_states') == 1`.
5. A fresh `/joint_states` message contains all 12 joints.
6. The start pose is within 0.05 rad of neutral.
7. The first trajectory point is consistent with the observed start (§5).

The same controller and publisher checks are repeated **after** the run.

> **Superseded by [§14](#14-owner-decision-addendum-2026-10-02).** No invalid goal is sent to the live controller. M6.0-C is now a
> mock-only test step; the paragraph below is kept for audit history only.

**Server-side rejection (M6.0-C) [PLAN, SUPERSEDED].** One deliberately invalid goal is sent and must be
rejected **without motion**:
- **Primary probe:** non-monotonic `time_from_start`, which the controller refuses as "not strictly
  increasing".
- **Alternatives:** an unknown joint name (−2 `INVALID_JOINTS`), or an empty trajectory.

All of its positions equal the observed start pose. So even if it were unexpectedly accepted, the
commanded motion would be zero. The client then cancels it and reports FAIL.

## 7. Conversion compatibility contract (M4.5/M5 → M6)

**[FACT] constraints:**
- **Joint order.** M4.5/M5 joint series come from `gait_kinematics.solve_cycle` per leg
  (`gait_kinematics.py:56-83`), in `leg_kinematics.ALL_LEGS` × [hip, thigh, foot] order. That is
  identical to the controller order (`leg_kinematics.py:613-615`).
- **Sample grid.** Samples are `u_k = k/n` for k = 0 … n−1, so the cycle excludes u = 1
  (`gait_phase.py:61`), and `t = u·T` (`gait_trajectory.py:80`).
- **IK seeding.** Sample 0 is IK-seeded from CAD neutral and later samples from the previous
  solution (`gait_kinematics.py:64, 81`).
- **No stored series.** M5 records keep **no** joint series (`eval_runner.py:425-426`).

**[RESULT] audit probe** (in memory, no files, at the M5 head build). For the six shipped gaits at
n = 200:
- the peak \|q\| over a cycle is 0.18–0.20 rad;
- \|q(u = 0)\| is up to 0.17 rad, so **a gait cycle does not start at neutral** and needs a
  lead-in;
- the largest per-sample step is ≤ 0.026 rad.

**Contract [PLAN]:**
1. **Regenerate, don't parse.** A trajectory is regenerated from an M5 effective configuration with
   the unchanged M4.5/M5 code at the recorded commit. The regenerated `config_id` and `eval_id`
   must equal the M5 record's, and its metrics must match the M5 record (pinned tolerances). The
   ignored `samples.csv` is never used as a source.
2. **No silent change.** Kinematics, joint conventions, gait data, phase offsets and sample counts
   are copied, not modified. Any mismatch refuses the conversion.
3. **Time mapping.** Point i of the cycle is at `lead_in + u_i·T`.
   - Single, non-cyclic playback appends the u = 1 point, which equals u = 0, only if the owner
     approves a closed cycle.
   - Body speed and T come from the effective configuration and are never rescaled silently.
     Rescaling time is allowed only as an explicit, recorded parameter, because M5 showed speed is
     a pure time scale.
4. **Neutral convention.** Neutral is CAD q = 0 (`spiderx_poses.yaml cad_neutral`). A lead-in and a
   lead-out to neutral follow the §5 duration rule.
5. **Limits.** Soft limits (URDF ± 0.05 rad) apply at every point. No clamping. An IK-infeasible M5
   record is not convertible.
6. **Provenance carried.** The trajectory record names:
   - the source (M5 `eval_id`, `config_id`, source gait, pattern, n, M5 study id, and the M5
     manifest SHA-256 if available);
   - the generator commit and dirty flag;
   - the SHA-256 of `m4_5_gaits.yaml`, `m5_study.yaml` and `spiderx_legs.yaml`, and of the expanded
     URDF;
   - a deterministic `trajectory_id`, which is a hash of the canonical trajectory.
7. **Explicit rejection.** Anything that cannot meet 1–6 is refused with exit 2 and a named reason.

## 8. Verification ladder and claims

Each step requires the previous one, and each needs owner approval before it runs.

| Step | Input | Expected evidence | Failure handling | Allowed claim | Prohibited claim |
|---|---|---|---|---|---|
| **M6.0-A** Offline conversion + preflight (no ROS) | A neutral-to-neutral waypoint spec and one M5 record (conversion only) | Deterministic trajectory files and ID; every §6 offline check passes for valid input; every mutation is refused with exit 2 and **0 send calls** | Refuse; nothing is sent | "Offline trajectory conversion and preflight implemented and tested" | Any motion, controller or Gazebo claim |
| **M6.0-B** Runtime preflight (`--dry-run`), no goal | Running `fortress_control.launch.py` | Both controllers `active`; action server available; one `/joint_states` publisher; start within 0.05 rad; **0 goals sent** (log); clean SIGINT shutdown and no leftovers | Exit 2, NOT READY; no goal | "The controller action interface and preflight were observed in Gazebo without sending a goal" | Motion, tracking, replay |
| ~~**M6.0-C** Deliberately invalid goal~~ **Superseded: see [§14](#14-owner-decision-addendum-2026-10-02) (mock-only)** | One invalid goal (non-monotonic time; all positions = start) | Goal **rejected**; result/rejection logged; joint positions unchanged within 0.05 rad over the settle; no leftovers | Unexpected acceptance: cancel, FAIL, report | "An invalid trajectory goal is rejected by the controller without motion" | Validity of any valid trajectory |
| **M6.0-D** One bounded neutral-to-neutral playback | One multi-point, non-cyclic, all-12-joint trajectory within the §5 envelope (content: D1) | Accepted; feedback logged; result code and string; per-joint commanded vs observed error at each waypoint and after the settle; returned to neutral within tolerance; controllers still `active`; one `/joint_states` publisher at the end; no leftovers (normal, rejection, timeout and Ctrl+C runs) | Watchdog or abort: cancel per D5, FAIL, logs kept | "One bounded multi-point joint trajectory was executed in Gazebo through the existing controller stack; observed joint positions tracked commanded ones within X rad (simulation, placeholder actuators)" | Walking, gait, stability, contact, body support, hardware, real-time |
| **M6.1** Protected replay (after M6.0-D) | One M5-derived gait cycle with lead-in and lead-out, on the base constraint chosen in D2 | Joint tracking statistics per joint; trajectory ID and M5 provenance; no contact or body claims | As in M6.0-D | "An offline-validated gait joint trajectory was replayed in Gazebo on a [fixed/raised] base; joint-space tracking within X" | Locomotion, translation, stability, contact, energy, hardware |
| **Future** Free-base, contact, dynamic experiments | Separate plan | – | – | – | Everything above, until planned |

## 9. Logging, provenance, determinism and cleanup

**[PLAN]**
- **Output location.** All output goes under the git-ignored `log/m6_playback/<trajectory_id>/<run>/`.
  A run never overwrites an existing directory, as in M5.
- **Offline files (M6.0-A).** `trajectory.json`, the canonical trajectory with joint names, points
  and provenance, and `preflight.json`. These must be **byte-identical** across repeated
  conversions at one commit. They contain no timestamps; the environment is recorded separately.
- **Runtime files (M6.0-B…D).**
  - `run.json`: preflight results, controller states before and after, publisher counts, goal
    accepted or rejected, result code and string, watchdog and cancel events, and the exit code.
  - `feedback.csv`: sim time, and desired, actual and error per joint.
  - `joint_states.csv`: sim time and the 12 positions.
  - `tracking.json`: per-waypoint and final commanded vs observed errors.
  - Runtime logs are not expected to be byte-deterministic. Their **schema** and their pass/fail
    criteria are.
- **Version recording.** Package versions of the stack in §2.2 are recorded on each machine.
- **Cleanup.** The Gazebo launch runs in its own process group (`setsid`). Shutdown is SIGINT to the
  group. Afterwards `pgrep -f 'ign gazebo|gz sim|parameter_bridge|robot_state_publisher|
  controller_manager/spawner|ros2 launch spiderx_bringup'` must be empty. This is checked after
  normal completion, rejection, watchdog timeout and Ctrl+C (`validate_m4_all_leg_ik.sh:186-195`).

## 10. Test strategy

**[PLAN]**

| Layer | What | Where |
|---|---|---|
| Unit (no ROS) | Conversion determinism; joint order; legacy-order refusal; finiteness; monotonic time; spacing; limits (no clamping); speed and duration; final-velocity rule; envelope; provenance and staleness; `trajectory_id` stability | `test_m6_trajectory.py` |
| Mutation and error-path | Each §6 offline rule removed or perturbed must turn a test red. The send function is mocked and must be called **0 times** on any refusal | same |
| Action client with a mock server (no Gazebo) | An in-process `rclpy` action server that accepts, rejects, aborts, never answers (watchdog) or receives a cancel. The client handles each correctly, never sends twice, never retries, and logs result and feedback | `test_m6_client.py` |
| Gazebo, only with owner approval per step | M6.0-B, C and D as in §8, run by a validator script modelled on `validate_m4_all_leg_ik.sh` (setsid, SIGINT to the group, `pgrep`) | `scripts/validate_m6_playback.sh` (future) |
| Regression | Full `colcon test`; M1–M4 static validators; M5 regression pins unchanged | existing |

## 11. Planned implementation batches

Each batch is independently testable, with a clean build and the full suite, and committed
separately; docs are committed separately. **[PLAN]**

| Batch | Content | Touches |
|---|---|---|
| **A** | Trajectory model, offline preflight, deterministic conversion (neutral waypoints; M5 record regeneration), `--validate-only` | New modules in `spiderx_controller` (D7) + unit tests. No ROS |
| **B** | Action client: multi-point goals, mock-server tests, `--dry-run` runtime preflight, logging schema, watchdog, cancel policy (D5) | New client module (does not modify `trajectory_client.py`, so M1–M4 tools are unchanged) + mock tests |
| **C** | Validator script for M6.0-B (M6.0-C is mock-only per [§14](#14-owner-decision-addendum-2026-10-02)), with cleanup checks | New script; Gazebo runs need owner approval |
| **D** | M6.0-D playback run and results docs | Owner approval required before running |
| **M6.1** | Separate plan after M6.0-D (base constraint D2) | – |

## 12. Risk register and owner decisions

### 12.1 Risks [FACT / PLAN]

| Risk | Mitigation |
|---|---|
| `SUCCESSFUL` without tracking: tolerances are unchecked by default | Independent `/joint_states` comparison (always); goal tolerances (D3) |
| The controller accepts a non-zero final velocity | Client-side refusal (§6.6) |
| No controller-side NaN or limit checks | Client-side finiteness and soft-limit checks; no clamping |
| A second goal preempts the first | One goal per process; no retries; the client refuses to send while a goal is active |
| Orphaned Gazebo server or GUI on a launch-only shutdown | Process-group SIGINT plus `pgrep` (M4.1 practice) |
| Free base on the ground: playing a gait would move or tip the model | M6.0-D is neutral-to-neutral only; gait replay waits for M6.1 and D2 |
| Placeholder actuators (100 N·m, 100 rad/s) make tracking look near-perfect | Every tracking claim says "placeholder actuators, simulation" |
| Version drift between cloud RoboStack and local apt packages | Record versions; repeat the interface checks locally (D6) |
| Spline behaviour for position + velocity points is not re-read from source | Observe feedback `desired` in M6.0-D; [HYPOTHESIS] until then |

### 12.2 Owner decisions [OWNER DECISION]

> **Decided 2026-10-02.** The options below are kept for audit history. The decisions taken are
> recorded in [§14](#14-owner-decision-addendum-2026-10-02).

**D1. Content of the first valid playback (M6.0-D).**

| Option | Content | Pro | Con |
|---|---|---|---|
| (a) | All waypoints = neutral (zero motion) | Safest | Proves the path and logging only, not tracking |
| (b) | Neutral → `crouch_10mm` → neutral, as **one** multi-point goal | Reuses an M4-validated, Gazebo-held IK pose; real but small motion | Joint deltas fixed by M4 |
| (c) | A bounded all-joint excursion ≤ 0.05 rad | Small and symmetric | New waypoints, not previously validated |
| (d) | One M5-derived gait cycle | – | Contradicts "neutral-to-neutral first"; belongs in M6.1 |

- **Recommendation:** (b).
- **Safe default:** (a).

**D2. Base constraint for M6.1** (gait replay). Today's model is a free base on the ground.

| Option | Pro | Con |
|---|---|---|
| (a) **Raised fixed base**: a test-only variant with base_link fixed in the air, so the feet touch nothing | Pure joint-space replay; no contact | Needs a separate model/launch variant later, which is a URDF/launch change subject to approval |
| (b) Fixed base at the nominal height | – | Feet may touch the ground, so contact enters |
| (c) Free base on the ground | – | That is a locomotion experiment, out of scope |

- **Recommendation:** (a).
- **Safe default:** no M6.1 until decided.

**D3. Success criteria.** Is the action result alone enough?

| Option | Criteria |
|---|---|
| (a) | Result only |
| (b) | Result **and** independent `/joint_states` comparison against the existing M1 tolerance of 0.05 rad (`m2_simulation_postures.yaml:56`) |
| (c) | (b) plus per-goal `goal_tolerance` and `goal_time_tolerance` in the goal message (no YAML change; aborts if violated) |

- **Recommendation:** (c), with values from the M1 tolerance and an owner-chosen goal-time
  allowance.
- **Safe default:** (b). (a) is not acceptable, because tolerances are unchecked by default.

**D4. Envelope numbers not derivable from the repository.** These are the maximum per-joint delta
from neutral, the maximum total duration, and the maximum point count.

| Option | Values |
|---|---|
| (a) | Delta ≤ the D1 pose's deltas (b) or ≤ 0.05 rad (c); duration ≤ 30 s of sim time; ≤ 5 points for M6.0-D |
| (b) | Delta ≤ 0.2 rad (M1-verified magnitude); duration ≤ 60 s; ≤ 16 points |
| (c) | Owner-specified values |

- **Recommendation:** (a).
- **Safe default:** (a).

**D5. Ctrl+C and abort policy.**

| Option | Behaviour | Note |
|---|---|---|
| (a) | Cancel only; the controller holds; report; the user then runs `test_neutral_pose.py` explicitly | – |
| (b) | Cancel, then automatically send a return-to-neutral trajectory, as today's M1/M4 tools do | That is an extra, un-preflighted command |
| (c) | (b) but preflighted (limits/speed) and logged as a separate goal | – |

- **Recommendation:** (a) for M6.0. It is the only option that never commands new motion on an
  interrupt.
- **Safe default:** (a).

**D6. Where the stack versions are recorded and re-verified.**

| Option | Where |
|---|---|
| (a) | Cloud only |
| (b) | Cloud and owner PC, with the interface checks (action file, parameter defaults, rejection strings) repeated |

- **Recommendation:** (b).
- **Safe default:** (b).

**D7. Location of the M6 client.**

| Option | Location | Note |
|---|---|---|
| (a) | New modules in `spiderx_controller` | Every dependency is already declared at `package.xml:18-29`, so no package change is needed |
| (b) | A new package `spiderx_playback` | New `package.xml` and CMake |

- **Recommendation:** (a), as new modules, leaving `trajectory_client.py` and the M1–M4 tools
  unchanged.
- **Safe default:** (a).

## 13. Out of scope

- Walking, gait execution claims, body support, translation, contact, friction or slip behaviour,
  dynamic stability, balance.
- M5.5 command-velocity bridge, `/cmd_vel`, odometry, SLAM, navigation.
- Body sway, dynamic gait generation, contact logic.
- Hardware, servos, firmware, power, energy, efficiency, actuator capability.
- Changes to the URDF/xacro, meshes, worlds, controller YAML, launch files or package metadata
  (any later fixed-base variant for M6.1 needs its own approved plan).
- Paper figures; PRs or merges during Phase 0.

## 14. Owner decision addendum (2026-10-02)

```text
Docs-only decision record. No M6 code exists. During this update, nothing was started:
no Gazebo, launch, controller manager, action client, goal (valid or invalid), playback,
/cmd_vel, M5.5, contact, fixed-base or free-base work, and no hardware.
The owner approved the Phase 0 audit commit cc6974d. M6 implementation still needs owner approval.
```

### 14.1 Decisions [OWNER DECISION]

| ID | Decision | Binding consequences |
|---|---|---|
| **D1** | **(b)** One bounded, non-repeating neutral → `crouch_10mm` → neutral playback | Reuses the **exact** M4-validated `crouch_10mm` pose (`config/m4_pose_targets.yaml:46-53`) and the M4 joint conventions. It is a **trajectory-execution and observability check only**. It does not validate gait playback, contact, body support, balance, locomotion or walking. The §14.2 conflict is **resolved** by option (i), [§14.7](#147-owner-decision-crouch-envelope-option-i-2026-10-02) |
| **D2** | **Defer M6.1 entirely** | Nothing for M6.1 in this branch: no fixed-base model, launch variant, raised base, free base, contact analysis or M6.1 runtime work. M6.1 needs a separate future plan and owner approval, and only after M6.0-D passes locally |
| **D3** | **(c)** Three evidence channels, **all required** | (1) The goal is accepted **and** the result is successful. (2) An independent `/joint_states` tracking check at **0.05 rad**. (3) Explicit `path_tolerance` / `goal_tolerance` in the FollowJointTrajectory **goal**, with **no controller-YAML change**. Each channel is recorded separately as `passed`, `failed`, `timed_out` or `unavailable`. **Action success alone is not tracking evidence** |
| **D4** | **(a)** Conservative envelope (§14.3) | If the pose exceeds the envelope: **stop and report; never widen the envelope automatically** |
| **D5** | **(a)** Cancel only; the controller holds | **No automatic return-to-neutral** after cancellation, rejection, timeout, tracking failure or error. Any later neutral-return mechanism needs its own preflight and owner approval |
| **D6** | **(b)** Cloud **and** owner Ubuntu PC | Installed versions are recorded in every run report. The §3 interface-compatibility checks are repeated on the owner's stack **before any valid local playback**. A version mismatch is a visible preflight outcome, never silently ignored |
| **D7** | **(a)** New M6-specific modules and scripts in `spiderx_controller` | `trajectory_client.py` and the M1–M4 tools stay **unchanged** |

### 14.2 Finding: D1 pose exceeded the original D4 per-joint limit [FINDING, RESOLVED]

> **Resolved 2026-10-02 by owner option (i)**, [§14.7](#147-owner-decision-crouch-envelope-option-i-2026-10-02). M6.0-D is no longer blocked. The
> text below is kept as the audit record of the finding.

**[RESULT]** This was computed read-only and in memory with the shipped loader:
`m4_pose_targets.load_and_evaluate()` followed by `pose_command(plan, …)`. Nothing was written and
no ROS node was created. It agrees with `docs/M4_PLAN.md:189`.

| Pose | Joint commands in canonical order [hip, thigh, foot] × lf, rf, lr, rr (rad) |
|---|---|
| `neutral_stance` | all 0.0 |
| `crouch_10mm` | lf [0, +0.0555, +0.1223], rf [0, −0.0555, −0.1223], lr [0, +0.0555, −0.1223], rr [0, −0.0555, −0.1223] |
| **Δ (crouch − neutral)** | **max \|Δ\| = 0.1223 rad (foot joints)**; thigh joints 0.0555 rad; hips 0 |

**Conflict.**
- D4 requires every per-joint delta to be **≤ the exact `crouch_10mm` delta and ≤ 0.05 rad**.
- The exact `crouch_10mm` delta is 0.0555 rad (thigh) and 0.1223 rad (foot). Both exceed 0.05 rad.
- So **8 of the 12 joints** violate the 0.05 rad cap, and D1 and D4 cannot both be met.

**Handling, per D4.** Stop and report; do not widen.
- The envelope stays at 0.05 rad.
- The pose is **not** scaled, clipped or replaced.
- **M6.0-D was BLOCKED** until the owner resolved the conflict (resolved: option (i), [§14.7](#147-owner-decision-crouch-envelope-option-i-2026-10-02)).
- M6.0-A, M6.0-B and M6.0-C do not depend on the pose content and are not blocked by this
  finding. Each still needs its own approval.

When the implementation exists, the M6.0-A offline preflight must reproduce this finding: with the
shipped config, the D1 trajectory must be **refused** with exit 2, a named reason
(`per_joint_delta_exceeds_envelope`), and 0 send calls.

**Owner options for a later decision** (nothing is chosen here):
- (i) Raise the per-joint cap explicitly to the exact `crouch_10mm` delta (0.1223 rad), keeping
  every other D4 rule.
- (ii) Keep 0.05 rad and approve different waypoints. These would not be the M4-validated pose, and
  so would contradict D1 as written.
- (iii) Change D1 to the zero-motion option (a).

### 14.3 Approved D4 envelope for M6.0-D [OWNER DECISION]

Rules that §14 does not restate keep their §5 values: the 0.5 rad/s speed rule, the
max(3.0 s, 2·|Δq|/0.5) segment duration, the 1.0 s settle, the 0.02 s minimum spacing, zero velocity
at every waypoint, and refusal without clamping.

| Rule | Value |
|---|---|
| Per-joint commanded displacement from neutral | ≤ **0.1223 rad**, M6.0-D only, with a documented numerical epsilon of 1e-9 rad (owner option (i), [§14.7](#147-owner-decision-crouch-envelope-option-i-2026-10-02)). Superseded value: ≤ 0.05 rad (§14.2) |
| Maximum total duration | **30 s** |
| Maximum points | **5** |
| Start delay | **Required**: the first moving point is never at `time_from_start` = 0 |
| Time | Strictly increasing `time_from_start` |
| Goals | **One goal only**: no cyclic or repeating mode, no retry, no concatenation |
| Point validity | Every value finite; all **12 joints in canonical controller order**; client-side soft-limit check at URDF limits − **0.05 rad** margin |
| Preflight order | Every check runs **before goal construction**. A goal object is never built for a refused trajectory |
| Envelope breach | Stop and report. **Never widen automatically** |
| Goal tolerances (D3 channel 3) | Explicit per-joint `path_tolerance` and `goal_tolerance` of **0.05 rad** position (the tracking tolerance, [§14.7](#147-owner-decision-crouch-envelope-option-i-2026-10-02)), set in the goal message only, plus `goal_time_tolerance` (value fixed in the implementation and listed in the M6 test results for owner review). No controller-YAML change |
| Abort | Cancel only (D5) |

### 14.4 Revised verification ladder

Each step requires the previous step and its own owner approval. **M6.0-B is the earliest live
interaction.**

| Step | Kind | Content | Evidence | Allowed claim | Non-claims |
|---|---|---|---|---|---|
| **M6.0-A** | Offline, no ROS | Conversion and offline preflight of the D1 waypoint spec; `--validate-only` | Deterministic trajectory and ID; all §6 and §14.3 checks; every mutation refused with exit 2 and **0 send calls**; the D1 crouch trajectory **passes** under the 0.1223 rad cap and a point beyond the cap is refused | "Offline conversion and preflight are implemented and tested" | No motion, controller, Gazebo, tracking or playback claim |
| **M6.0-B** | **Live, read-only; no goal** | Preflight against the running controller stack: action-server availability, controller states, `/joint_states` source and publisher count, exact 12-joint name contract, installed-version collection (D6) | Run report with each check's outcome; **0 goals sent** (logged and asserted); clean process-group shutdown with no leftovers | "The controller action interface was observed read-only, and the preflight ran without sending any goal" | No motion, tracking, rejection-behaviour or playback claim |
| **M6.0-C** | **Mock-only** (deterministic test double; no live controller) | A mock FollowJointTrajectory action server or test double | (1) Malformed, non-monotonic, out-of-limit, NaN and incomplete trajectories are **rejected before any valid-goal dispatch**. (2) A rejected status or result is surfaced as a structured `failed` outcome. (3) **No retry** occurs. (4) **No automatic neutral-return goal** is generated. (5) **No later valid goal** follows a rejection | "The M6 client's rejection and failure paths are proven against a deterministic mock" | Nothing about the live controller's rejection behaviour. **No invalid goal is sent to the live controller.** A live invalid-goal test is out of scope unless separately approved |
| **M6.0-D** | Live, one valid goal (unblocked by [§14.7](#147-owner-decision-crouch-envelope-option-i-2026-10-02); still needs separate owner approval and local verification) | One neutral → `crouch_10mm` → neutral goal within §14.3, after the D6 checks on the target machine | The three D3 channels, recorded separately; controllers still active; one `/joint_states` publisher; no leftovers | "One bounded multi-point joint trajectory was executed in Gazebo through the existing controller stack, and joint-space tracking was observed within 0.05 rad (simulation, placeholder actuators)" | No gait playback, walking, locomotion, contact, body support, balance, stability, real-time, actuator or hardware claim |
| M6.1 | Deferred (D2) | — | — | — | Needs a separate plan and approval after M6.0-D passes locally |

### 14.5 Implementation boundary

| Area | Allowed in M6 implementation (after approval) | Never in M6.0 |
|---|---|---|
| Code location | **New** M6-specific modules, tests and scripts in `spiderx_controller` (D7) | Changes to `trajectory_client.py` or any M1–M5 module, tool or script |
| Config | New M6-specific config files, if the implementation plan names them | Changes to controller YAML, `spiderx_legs.yaml`, M4 or M5 configs |
| Robot model and launch | Use the existing launch unchanged, for M6.0-B/D only | URDF/xacro, mesh, world or launch changes; fixed-base or raised-base variants (D2) |
| Goals to the live controller | M6.0-B: **none**. M6.0-D: exactly **one** valid goal, after approval | Invalid goals, retries, repeats, concatenation, cyclic playback, automatic neutral return |
| Testing of rejection paths | Mock action server or test double (M6.0-C) | Live invalid-goal tests unless separately approved |
| Interfaces | FollowJointTrajectory on `/leg_trajectory_controller/follow_joint_trajectory`; read-only `/joint_states` and controller queries | `/cmd_vel`, M5.5, odometry, navigation |
| Physical scope | Joint-space observation in simulation | Contact, free-base, body support, hardware, servos, firmware |
| Process | Docs and code committed separately on this branch | PRs, merges or pushes to `main` |

### 14.6 Activity statement for this update [FACT]

- Only `docs/M6_GAIT_PLAYBACK_SAFETY_PLAN.md` changed.
- No code, test, config, URDF, launch or package file was touched.
- Nothing was run except one read-only, in-memory offline computation (§14.2).

### 14.7 Owner decision: crouch envelope, option (i) (2026-10-02)

**[OWNER DECISION]** This resolves the §14.2 conflict with option (i).

**The cap.** For **M6.0-D only**, the maximum commanded per-joint displacement from verified
neutral is **0.1223 rad**.
- It is an exact, evidence-based cap, derived from the existing M4-validated `crouch_10mm` pose.
  The exact maximum \|Δ\| is 0.12229413600889982 rad (the `lr`/`rr` foot joints; §14.2).
- It is **not** a general M6 gait-playback envelope.
- It must **never widen automatically**.

**Three separate quantities.** They must never be confused in the plan, the code, the tests or
the reports.

| # | Quantity | Value | Used for | Must not be used for |
|---|---|---|---|---|
| 1 | **Maximum commanded displacement from neutral** (M6.0-D only) | **0.1223 rad**, inclusive only within a documented numerical epsilon of **1e-9 rad** (accept \|q − q_neutral\| ≤ 0.1223 + 1e-9) | The single approved neutral → `crouch_10mm` → neutral trajectory. Any point beyond the cap is rejected **before any action goal is constructed or sent** | Tracking, joint limits, any other trajectory or any later milestone |
| 2 | **Tracking tolerance** | **0.05 rad** | Comparing observed `/joint_states` with the commanded trajectory during and after a valid playback (D3 channel 2), and the goal-message `path_tolerance` / `goal_tolerance` (D3 channel 3) | Limiting commanded displacement |
| 3 | **Joint-limit soft margin** | **0.05 rad**, the existing `soft_limit_margin_rad` in `config/spiderx_legs.yaml` (`joint_safety.DEFAULT_MARGIN_RAD`) | Client-side mechanical-limit preflight: every commanded point must lie within URDF limits pulled inward by the margin; no clamping | Tracking or displacement |

**Effect on the ladder.**
- The §14.2 blocked status is removed.
- **M6.0-D remains un-run.** It still needs the D6 checks on the owner's stack, a passing local
  live read-only preflight (M6.0-B) and separate owner approval before any valid goal is sent.
- The M6.0 implementation (offline, mock and live read-only layers) proceeds on this branch.
- Cloud validation is limited to offline tests, mocked tests and static checks.
- No Gazebo, launch, controller manager, action client, goal, playback, `/cmd_vel`, M5.5, contact,
  fixed-base or free-base work, and no hardware.
- No PR was created.
