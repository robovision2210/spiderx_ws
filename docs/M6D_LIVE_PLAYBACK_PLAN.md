# M6.0-D Plan – Single Valid Live Playback Safety Plan

> **Status (2026-10-02): M6.0-D implementation cloud + local verified in offline/mock/isolated-domain testing; live dispatch remains hard-disabled and local live playback remains pending separate approval.**
> Local verification on the owner's Ubuntu PC at `412eb45` passed (results §12). Batches A–D (`28b72a4`, `e87c81a`,
> `d7fb6a2`, `7fd0152`) implement this plan with `LIVE_DISPATCH_ENABLED = False`; `--live` refuses
> with exit 3. No trajectory goal was sent to a live controller. Evidence:
> [M6D_LIVE_PLAYBACK_RESULTS.md](M6D_LIVE_PLAYBACK_RESULTS.md); usage:
> [SPIDERX_M6D_LIVE_PLAYBACK_GUIDE.md](SPIDERX_M6D_LIVE_PLAYBACK_GUIDE.md). D16 was implemented by
> composition (`m6_goal_fingerprint.build_live_goal`), so `m6_action_client.py` and its tests are
> unchanged. The gated live wiring, the two-line enabling commit and the manual checklist are in
> the [live-enabling design](M6D_LIVE_ENABLING_DESIGN.md); the gate is still `False`.

> **Adopted plan (2026-10-02).** The Phase 0 audit was written on
> `claude/spiderx-m6d-live-playback-plan` (`c55421b`, from `f5a7252`). It is adopted here on the
> implementation branch `claude/spiderx-m6d-live-playback`, created from `origin/main` @ `9885057`
> (M6.0-B merged), with the owner-approved decisions D1–D17 in [§16](#16-approved-owner-decisions-d1d17). Where this text and §16
> differ, **§16 governs**. During the audit and this adoption, no Gazebo, launch file, ROS node,
> graph query, controller query, action client, goal or command was started or sent. No valid
> `FollowJointTrajectory` goal has ever been sent to a live SpiderX controller.
>
> **Implementation permission and limits [OWNER DECISION].**
> - M6.0-D implementation is permitted after this plan commit.
> - **No live goal may be sent** during implementation, cloud validation, unit tests, mock tests,
>   isolated-domain tests or PR creation.
> - **A later, separate owner approval is mandatory** before any live M6.0-D command is executed.
>
> **The one future allowed trajectory:**
> - neutral → `crouch_10mm` → neutral;
> - 3 points at 3 s, 6 s and 9 s;
> - maximum commanded displacement 0.1223 rad + 1e-9 epsilon;
> - goal/path tracking tolerance 0.05 rad;
> - explicit goal velocity tolerance 0.05 rad/s;
> - goal-time tolerance 1.0 s;
> - one goal only; no cyclic mode; no retry; cancel only, then hold.

```text
M6.0-D would send exactly ONE preflighted goal, neutral -> crouch_10mm -> neutral, to the
simulated leg_trajectory_controller, observe it independently, and never send a second goal.
Even a full pass would show only that one bounded joint trajectory was executed in Gazebo and that
joint-space tracking stayed within 0.05 rad (simulation, placeholder actuators).
It would NOT show gait playback, walking, locomotion, contact quality, body support, balance,
stability, navigation, real-time behaviour, actuator capability or hardware readiness.
```

**Labels:**
- **[FACT]**: read in this repository or in the installed stack, with a path:line reference.
- **[RESULT]**: an earlier verified result.
- **[HYPOTHESIS]**: believed but unproven; checked at the step named.
- **[PLAN]**: a proposed design or procedure.
- **[OWNER DECISION]**: needed before implementation or before the live run (§14).

## Contents

1. [Scope and strict non-claims](#1-scope-and-strict-non-claims)
2. [Provenance](#2-provenance)
3. [Live action-adapter contract and dispatch binding](#3-live-action-adapter-contract-and-dispatch-binding)
4. [Single-goal state machine](#4-single-goal-state-machine)
5. [Trajectory envelope and every numeric limit](#5-trajectory-envelope-and-every-numeric-limit)
6. [Pre-dispatch live readiness contract](#6-pre-dispatch-live-readiness-contract)
7. [Goal, feedback, result, timeout and error handling](#7-goal-feedback-result-timeout-and-error-handling)
8. [Independent `/joint_states` tracking protocol](#8-independent-joint_states-tracking-protocol)
9. [Abort, Ctrl+C, cancel-only and cleanup](#9-abort-ctrlc-cancel-only-and-cleanup)
10. [Evidence model](#10-evidence-model)
11. [Failure categories and no-retry policy](#11-failure-categories-and-no-retry-policy)
12. [Test strategy](#12-test-strategy)
13. [Future live-run manual procedure (outline)](#13-future-live-run-manual-procedure-outline)
14. [Risks and owner decisions](#14-risks-and-owner-decisions)
15. [Out of scope](#15-out-of-scope)
16. [Approved owner decisions (D1–D17)](#16-approved-owner-decisions-d1d17)

---

## 1. Scope and strict non-claims

**[PLAN] M6.0-D, after separate owner approval, would:**
- start the existing GUI Fortress control launch (as in M6.0-B);
- repeat the live read-only readiness checks **in the same process, immediately** before
  dispatch (§6);
- send **one** `FollowJointTrajectory` goal, built only from the preflighted canonical
  neutral → `crouch_10mm` → neutral trajectory;
- record the action feedback and result and an independent `/joint_states` stream;
- evaluate the three D3 channels (action, tracking, goal tolerances);
- stop.

**[PLAN] It never:**
- sends a second goal, a retry or an automatic return-to-neutral goal;
- sends `/cmd_vel`, a controller switch or lifecycle request, or a parameter change on another
  node;
- uses hardware.

**Allowed claim after a full pass:**

> "One bounded multi-point joint trajectory (neutral → crouch_10mm → neutral, ≤ 0.1223 rad per
> joint, 9 s) was executed once in Gazebo through the existing controller stack; the action result,
> the goal tolerances and independent joint-space tracking within 0.05 rad were observed
> (simulation, placeholder actuators)."

**Prohibited claims, even after a full pass:**
- gait playback, walking, locomotion or a gait cycle;
- contact quality, foot loading, body support, balance or stability;
- body height or attitude;
- navigation, odometry or state estimation;
- real-time behaviour, actuator or servo capability, or hardware readiness;
- that any trajectory other than this one is safe;
- that M6.1 is approved.

## 2. Provenance

### 2.1 Repository state [FACT]

- **At the Phase 0 audit (`c55421b`)**, `origin/main` was `f5a7252` (the M6.0 offline/mock merge),
  and the M6.0-B evidence (`bffd00c`) was on an unmerged branch.
- **At adoption**, `origin/main` = **`9885057`**, the merge of PR #14 (M6.0-B). Since `f5a7252` it
  changed only these five files; no M6 source or test changed:
  - `STATUS.md`;
  - `docs/M6_GRAPH_PREFLIGHT_PLAN.md`;
  - `docs/M6_GRAPH_PREFLIGHT_RESULTS.md`;
  - `docs/SPIDERX_DEVELOPMENT_ROADMAP.md`;
  - `docs/SPIDERX_TESTING_GUIDE.md`.

  This branch starts from `9885057` (D14).
- **Live docs on `main` now record M6.0-B.**
  - `STATUS.md:5` reads "M6.0-B graph-mode read-only preflight passed locally; M6.0-D valid
    playback remains pending."
  - The roadmap ticks M6.0-B and leaves M6.0-D open (`docs/SPIDERX_DEVELOPMENT_ROADMAP.md:141-142`).
- **Remaining inaccuracy** (not edited here): `STATUS.md:3` still names the old branch.

### 2.2 Verified evidence this plan builds on [RESULT]

| Step | Evidence | Where |
|---|---|---|
| M6.0-A offline preflight | Cloud and local: 932 tests, 0 failures; `trajectory_id 44f0a7ad52e5c330` on both; byte-identical outputs | `docs/M6_TEST_RESULTS.md` (main) |
| M6.0-C mock client | Single goal, no retry, no auto-return, cancel-only; mutation tests | `docs/M6_TEST_RESULTS.md:147-218` |
| M6.0-B graph preflight (merged in `9885057`) | Owner's PC, GUI, `b64217d`: READY; one action server `/leg_trajectory_controller/follow_joint_trajectory`; both controllers active; one `/joint_states` publisher, 39 messages in 2 s; 12 joints; max neutral deviation 4.99e-9 rad; five evidence tiers passed; classification `warning` (8 newer local packages: JTC 2.54.0, `controller_manager` 2.54.2, `control_msgs` 4.9.0, `gz_ros2_control` 0.7.21, `rclpy` 3.3.19, …); clean Ctrl+C shutdown in 9.6 s | [M6.0-B results](M6_GRAPH_PREFLIGHT_RESULTS.md) |
| Owner decisions | D1–D7, option (i) cap 0.1223 rad, §14.9 (`goal_time_tolerance` 1.0 s, classification rule, start pose 0.05 rad, naming) | `docs/M6_GAIT_PLAYBACK_SAFETY_PLAN.md` §14 |

**What M6.0-B proves, and what it does not [RESULT].**
- **Shown:**
  - local GUI graph-mode preflight passed;
  - both controllers active;
  - one correct action server;
  - one fresh `/joint_states` publisher with the 12 canonical joints;
  - start pose within tolerance;
  - classification `warning`, due only to newer local package versions, with the contract
    passing;
  - clean Ctrl+C shutdown with no leftover process.
- **Not shown:** M6.0-B proves **readiness observation only**. It does not prove action-goal
  dispatch or tracking.

### 2.3 Audited modules [FACT]

| File | Lines | What matters for M6.0-D |
|---|---|---|
| `spiderx_controller/m6_envelope.py` | 88 | Constants: the 0.1223 rad cap and 1e-9 epsilon (19-22), the 0.05 tracking tolerance (25), 5 points and 30 s (31-32), 3.0 s segments, factor 2 and 1.0 s settle (35-37), 0.05 start pose (44), `GOAL_TIME_TOLERANCE_S = SETTLE_S` (49), the endpoint (52-56) |
| `spiderx_controller/m6_trajectory.py` | 586 | `load_sources` (232), `build_trajectory` (314), `preflight` (543) with 32 codes (51), `compute_trajectory_id` (188), the portable URDF hash (159-182) |
| `spiderx_controller/m6_action_client.py` | 412 | `ActionAdapter` interface (61-86; **no live implementation exists**), `build_goal` (105-132; re-runs the preflight), `_dispatch_allowed` (135-140), `reference_positions` / `evaluate_tracking` (143-214), `PlaybackSession` (254-405): `_dispatch` single-goal guard (293-299), cancel-and-hold (301-306), `run` (341-405), watchdog `10 × duration + 30 s` (280-284), `KeyboardInterrupt` → cancel-and-hold (381-382) |
| `spiderx_controller/m6_live_preflight.py` | 544 | Read-only `GraphProbe` (355-426), pure `evaluate` (173-288), no classification label (M6 plan §14.9) |
| `spiderx_controller/m6_offline_preflight.py` | 107 | Offline CLI, exit 0/2 |
| `scripts/m6_offline_preflight`, `scripts/m6_live_preflight` | 12 + 12 | Thin wrappers |
| `test/m6_mock_action.py` | 73 | The deterministic test double |
| `test/test_m6_*.py` | 109 + 14 + 63 + 42 | Unit, mock and mutation tests |

### 2.4 M1–M4 tools that must NOT be reused for M6.0-D dispatch [FACT]

- **`trajectory_client.JointTestNode.send_positions`** (`trajectory_client.py:65-92`) sends a
  single-point goal with no tolerances.
- **`m4_pose_validation.py:476-480`** and **`scripts/test_one_joint.py:79-81`** catch
  `KeyboardInterrupt`, cancel, and then **send a return-to-neutral goal**. D5 forbids this for M6.
- **`trajectory_client.py:24`** sets `use_sim_time=True` on its node, and
  `trajectory_client.py:94-102` cancels with a 5 s wait. Both are reusable *patterns*, not code.

## 3. Live action-adapter contract and dispatch binding

### 3.1 Finding [FACT]

**No live adapter exists.** `ActionAdapter` (`m6_action_client.py:61-86`) is an interface; its
only implementation is the test double (`test/m6_mock_action.py:15-62`). M6.0-D needs a new live
adapter (D7: new modules in `spiderx_controller`; `trajectory_client.py` and the M1–M4 tools stay
unchanged).

### 3.2 Proposed modules [PLAN, OD-D1]

| Module | Role |
|---|---|
| `spiderx_controller/m6_live_adapter.py` | `RclpyActionAdapter(ActionAdapter)`: one `ActionClient` for `env.ACTION_NAME`; one `/joint_states` recorder subscription; an optional feedback recorder; a sim-time clock; and the ROS context lifecycle, including signal handling (§9) |
| `spiderx_controller/m6_live_playback.py` | CLI: offline preflight → in-process live readiness (§6) → operator confirmation → `PlaybackSession.run` → evidence files |
| `scripts/m6_live_playback` | Thin wrapper, like the existing ones |

### 3.3 Dispatch binding: three independent gates [PLAN]

**1. Session gate (exists).**
- `PlaybackSession.run` calls `_dispatch_allowed(report, trajectory)`. It requires a passing
  preflight whose `trajectory_id` equals the trajectory's recomputed content hash
  (`m6_action_client.py:135-140, 347-352`).
- `build_goal` re-runs the preflight and raises `PreflightRefused` (`:105-112`).

**2. Adapter binding (new).**
- `RclpyActionAdapter` is constructed with the **expected `trajectory_id`** and a **goal
  fingerprint**: the SHA-256 of the canonical JSON of joint names, positions, velocities, times in
  ns, the tolerance entries and `goal_time_tolerance`, computed from the preflighted trajectory.
- `send_goal(goal)` recomputes the fingerprint from the `Goal` message and refuses (raises,
  sending nothing) unless it matches.
- So a goal not built from that exact trajectory, such as a neutral-return goal, cannot be sent
  through the adapter.

**3. Adapter single-use (new).**
- The adapter counts `send_goal` calls and raises on a second call **before** touching ROS.
- It never exposes `ActionClient` to the caller.

**API boundary.** `m6_live_playback` holds only:
- the trajectory dict;
- `Sources`;
- the session;
- the adapter.

The adapter's public surface is exactly the five `ActionAdapter` methods
(`server_ready`, `send_goal`, `wait_result`, `cancel`, `joint_state_samples`), plus read-only
`feedback_samples()` and `evidence()`.

## 4. Single-goal state machine

**[FACT]** The existing `PlaybackSession` states (`m6_action_client.py:254-262`):

```text
idle -> preflight -> refused
                  -> dispatched -> rejected | no_response
                                -> executing -> succeeded | failed | timed_out
                                             -> canceled_holding | error_holding
terminal states are final; run() again raises SecondGoalForbidden
```

**[PLAN] Additions for live use**, made inside the adapter and CLI; `PlaybackSession` semantics
are unchanged:

| From | Event | To | Action |
|---|---|---|---|
| `idle` | live readiness fails (§6) | `refused` | Nothing constructed or sent |
| `idle` | operator does not confirm | `refused` | Nothing sent |
| `executing` | SIGINT (first) | `canceled_holding` | **One** cancel request; wait ≤ 5 s for the cancel response; record it; never a goal |
| `executing` | SIGINT (second, during the cancel wait) | `canceled_holding` | Stop waiting; record `cancel_unconfirmed`; never a goal |
| `executing` | `/joint_states` stale (§8.4) | `canceled_holding` | Cancel-only (**OD-D6**) |
| `executing` | action server vanishes from the graph, or the result future fails | `error_holding` | Cancel attempt (may fail); record it |
| `executing` | wall watchdog (120 s) | `timed_out` | Cancel-only |
| any terminal | – | – | No further ROS call except subscription teardown and context shutdown |

## 5. Trajectory envelope and every numeric limit

| Quantity | Value | Source |
|---|---|---|
| Content | Exactly neutral → `crouch_10mm` → neutral, `mode: single` | D1; `m6_trajectory.py:314` |
| Points | 3 (≤ 5) | `m6_envelope.py:31` |
| Times | 3.0, 6.0, 9.0 s; duration 9.0 s (≤ 30 s) | `m6_envelope.py:32, 35`; `lead_in_s` / `segment_min_s` (`m6_trajectory.py:303-312`) |
| Start delay | 3.0 s = max(3.0, 2 × 0.05 / 0.5) | `m6_trajectory.py:303-307` |
| Segment minimum | max(3.0 s, 2 × max\|Δq\| / 0.5 rad/s) | `m6_trajectory.py:309-312` |
| Joint speed placeholder | 0.5 rad/s (SIMULATION_PLACEHOLDER) | `config/spiderx_legs.yaml:24` |
| **Commanded displacement cap** | **0.1223 rad**, epsilon 1e-9; actual maximum 0.1222941360 rad (`rr_foot_joint`) | Option (i); `m6_envelope.py:19-22` |
| Joint-limit soft margin | 0.05 rad, inside the URDF limits; no clamping | `spiderx_legs.yaml:17` |
| Velocities | Exactly 0 at every waypoint | `m6_trajectory.py:451-478` |
| **Tracking tolerance** | **0.05 rad**, independent `/joint_states` | `m6_envelope.py:25` |
| Goal `path_tolerance` / `goal_tolerance` position | 0.05 rad per joint | `m6_action_client.py:94-102`; D8 |
| **Goal `goal_tolerance` velocity** | **0.05 rad/s per joint, explicit, M6.0-D goal only** (D16). Today `tolerance_entries` sets it to 0 (= "unspecified"); an implementation batch must change it | D16 |
| `goal_time_tolerance` | 1.0 s (M6.0-D only) | M6 plan §14.9; `m6_envelope.py:49` |
| Controller default goal velocity tolerance | **0.01 rad/s**, `stopped_velocity_tolerance`. It applies today, because the goal's tolerance velocity is 0 (unspecified). **D16 replaces reliance on it** with the explicit 0.05 rad/s | **[FACT]** `tolerances.hpp:114`; parameters header line 97 |
| **Start-pose tolerance** | **0.05 rad**, observed | M6 plan §14.9 |
| Server wait | 10 s | `m6_action_client.py:46` |
| Goal-response wait | 10 s **[PLAN, OD-D9]** | – |
| Result watchdog (wall) | 10 × 9 + 30 = **120 s** | `m6_action_client.py:280-284` |
| Cancel-response wait | 5 s **[PLAN, OD-D9]** | pattern `trajectory_client.py:94-102` |
| Post-result settle before the final tracking sample | 1.0 s sim | `SETTLE_S` |
| `/joint_states` stale threshold | 0.5 s wall without a message during execution **[PLAN, OD-D6]** | – |
| Maximum sim-time gap between consecutive samples | 0.25 s **[PLAN, OD-D6]** | – |
| Readiness age at dispatch | ≤ 10 s wall between the passing readiness check and `send_goal` **[PLAN, OD-D3]** | – |

## 6. Pre-dispatch live readiness contract

**[PLAN, OD-D3]** All of the following must hold **in the dispatching process**, with nothing
sent, in this order:

**1. Offline.**
- `load_sources` has no errors.
- `build_trajectory` gives exactly the documented trajectory.
- `preflight(...).ok`.
- `trajectory_id == 44f0a7ad52e5c330` when the sources are unchanged. Otherwise the ID is printed
  and the owner must approve the new one.

**2. Live, read-only.** This is the same logic as `m6_live_preflight` (`evaluate`, lines 173-288)
over a `GraphProbe` built on the **adapter's node**. It needs READY:
- one action server of the right type;
- both controllers `active`;
- one fresh `/joint_states` publisher with all 12 joints;
- the interface contract passing;
- **start pose ≤ 0.05 rad** (OD-9).

**3. Classification ≠ `incompatible`.** This requires the label code in `m6_live_preflight`,
which M6 plan §14.9 makes an M6.0-D prerequisite (**OD-D4**).

**4. Adapter checks.**
- `ActionClient.server_is_ready()` within 10 s.
- **[PLAN]** Exactly one action server, and **zero other action clients** on the graph, apart
  from the adapter's own.

**5. Freshness.**
- At most 10 s wall between the end of step 2 and dispatch.
- The latest `/joint_states` is at most 0.5 s old (wall), and its start-pose deviation is
  re-checked on that message.

**6. Operator confirmation.**
- The CLI prints the trajectory, ID, limits and the three channels, and waits for the operator to
  type a fixed confirmation word (**OD-D10**).
- Steps 4 and 5 are re-checked after the confirmation.

**Any failure leaves the state at `refused`, sends nothing and exits 2 (refused) or 1 (not
ready).**

## 7. Goal, feedback, result, timeout and error handling

### 7.1 Installed controller behaviour [FACT, `joint_trajectory_controller` 2.48.0 cloud headers/strings]

| Behaviour | Evidence |
|---|---|
| `allow_partial_joints_goal` false (YAML), so a goal must name all 12 joints | `config/spiderx_ros2_controllers.yaml:48` |
| Defaults: `interpolation_method` "splines", `allow_nonzero_velocity_at_trajectory_end` true, `cmd_timeout` 0, `open_loop_control` false, `constraints.goal_time` 0, `stopped_velocity_tolerance` 0.01 | `joint_trajectory_controller_parameters.hpp:75-98` |
| `action_monitor_rate` 20 Hz, `state_publish_rate` 50 Hz, `update_rate` 100 Hz, `use_sim_time` true | `spiderx_ros2_controllers.yaml:15-16, 44-45` |
| Goal tolerances taken from the goal when > 0; any unknown joint name or illegal value makes the controller **silently** use its (unchecked) defaults | `tolerances.hpp:130-283` |
| Rejection messages: `Joints on incoming trajectory don't match the controller joints.`, `Empty joint names on incoming trajectory.`, `Can't accept new action goals. Controller is not running.`, `Velocity of last trajectory point of joint %s is not zero` | `libjoint_trajectory_controller.so` strings |
| Acceptance and completion messages: `Received new action goal`, `Accepted new action goal`, `Goal reached, success!` | same |
| Abort messages: `Aborted due to goal_time_tolerance exceeding by`, `Exceeded goal_time_tolerance: holding position...` | same |
| Cancel and preempt messages: `Got request to cancel goal`, `Canceling active action goal because cancel callback received.`, `Current goal cancelled due to new incoming action.`, `Current goal cancelled during deactivate transition.` | same |
| Result error codes 0 `SUCCESSFUL`, −1 `INVALID_GOAL`, −2 `INVALID_JOINTS`, −3 `OLD_HEADER_TIMESTAMP`, −4 `PATH_TOLERANCE_VIOLATED`, −5 `GOAL_TOLERANCE_VIOLATED` | `m6_action_client.py:33-41` (pinned by tests) |
| `GoalStatus` 4 SUCCEEDED, 5 CANCELED, 6 ABORTED | `m6_action_client.py:28-30` (pinned) |
| Feedback fields `desired`, `actual`, `error` (`JointTrajectoryPoint`) plus a header | `control_msgs` 4.8.0 `FollowJointTrajectory.action` |
| Action status topic `TRANSIENT_LOCAL`, depth 1 | `rcl_action/default_qos.h:26-31` |

**[HYPOTHESIS, must be re-confirmed on the owner's stack].** The local controller is 2.54.0,
`controller_manager` 2.54.2 and `control_msgs` 4.9.0. Before the live run, these must be
re-checked read-only on the owner's installed headers and `.so` strings (**OD-D15**):
- the same tolerance semantics;
- the hold on cancel and abort;
- the same log strings.

### 7.2 Expected outcome of a valid run [PLAN]

- **Action:** goal accepted; result status 4 (SUCCEEDED), error 0, `Goal reached, success!`.
- **Goal tolerances:** passed (error not −4 or −5). This requires the joints to settle within
  0.05 rad and below **0.05 rad/s** (D16) within 1.0 s of t = 9 s.
- **Tracking:** passed (§8).
- **Overall:** `succeeded` only if all three channels pass.

### 7.3 Handling [PLAN, existing session logic `m6_action_client.py:341-405`]

| Situation | Outcome | Further action |
|---|---|---|
| Rejected | `rejected`, action failed | None; no retry |
| No goal response within 10 s | `timed_out` (`no_response`) | Nothing to cancel; no retry |
| Abort −4 or −5 | `failed`, tolerance channel failed | Controller holds; no goal |
| Abort −1, −2 or −3 | `failed`, tolerance channel `unavailable` | None |
| Result watchdog | `timed_out` | Cancel-only |
| Adapter exception | `error`, holding | Cancel attempt |

### 7.4 Feedback [FACT / PLAN]

- **[FACT]** Feedback is the controller's own view (desired, actual, error) at about 20 Hz. It is
  **not independent** evidence.
- **[PLAN]** It is recorded to a CSV and used only to estimate the controller's trajectory start
  time (§8.2) and as a cross-check. A missing feedback stream is reported, not failed.

## 8. Independent `/joint_states` tracking protocol

### 8.1 Recording [PLAN]

- The adapter subscribes to `/joint_states` **before** dispatch.
- It records every message: the header stamp (sim time), the wall receive time, the names and
  positions.
- It keeps the samples from acceptance until 1.0 s of sim time after the result.

### 8.2 Time base [PLAN, OD-D5]

- **Sample time.** The adapter node uses `use_sim_time=True` (pattern `trajectory_client.py:24`).
  `T_accept` is the adapter's sim-clock time when the goal response arrives, and each sample's
  t = `header.stamp − T_accept`.
- **Header stamp.** The goal header stamp stays 0 (`m6_action_client.py:119`), so the controller
  starts "now" on receipt. The true start differs from `T_accept` by the response latency.
- **Cross-check.** From feedback, `T_start_est = feedback.header.stamp − desired.time_from_start`.
  The offset `T_start_est − T_accept` is reported.
- **Why the misalignment is small [FACT, arithmetic].** The cubic reference peaks at
  1.5 × Δq / T = 1.5 × 0.1222941 / 3.0 = 0.0611 rad/s. A 0.1 s misalignment therefore changes the
  reference by at most 0.0061 rad, well inside 0.05 rad.
- **[PLAN]** If |offset| > 0.1 s, the tracking channel is computed with `T_start_est` and the
  substitution is reported.

### 8.3 Comparison [FACT, existing]

- `evaluate_tracking` (`m6_action_client.py:165-214`) compares each complete sample after the
  first waypoint with the cubic-Hermite reference (`reference_positions`, `:143-163`) at 0.05 rad.
- It requires coverage of every segment and of the post-end interval, and treats incomplete or NaN
  samples as failure.

### 8.4 Additions [PLAN, OD-D6, OD-D7, OD-D8]

| Condition | Rule |
|---|---|
| Stale stream: no message for > 0.5 s wall during execution | Cancel-only; tracking `failed` with `stale_joint_states` |
| Sim time not advancing (Gazebo paused or stalled): no change in `/clock` or stamps for > 5 s wall | Cancel-only; `sim_time_stalled`; the watchdog remains the backstop |
| Gap > 0.25 s sim between consecutive samples | Tracking `failed`, `sample_gap` |
| A second `/joint_states` publisher appears | Cancel-only; `unexpected_publisher` |
| In-flight error > 0.05 rad | **Both** (D8): the controller-side `path_tolerance` aborts (−4), **and** the client independently sends its one cancel when the observed tracking error exceeds 0.05 rad. No new trajectory is ever sent |
| Controller disappears: the action server leaves the graph (checked once per second) | Cancel attempt; `error_holding` |

## 9. Abort, Ctrl+C, cancel-only and cleanup

### 9.1 Hazard [HYPOTHESIS, OD-D2]

- **[FACT]** `rclpy.init()` installs rclpy's own SIGINT and SIGTERM handlers by default
  (`rclpy/__init__.py:83-93`; `SignalHandlerOptions.ALL`).
- **[HYPOTHESIS]** The native handler (`_rclpy_pybind11` `signal_handler.cpp`, which contains
  "Shutdown context") **shuts the default context down** on SIGINT. A cancel request after Ctrl+C
  would then fail, and the goal would run to completion.
- **The existing M1–M4 tools** use the default and cancel afterwards
  (`m4_pose_validation.py:472-478`). This is not evidence that the cancel succeeds.

### 9.2 Design [PLAN]

**1.** `rclpy.init(signal_handler_options=SignalHandlerOptions.NO)`, so the context stays alive.
Python's default SIGINT handler raises `KeyboardInterrupt` in the main thread.

**2.** `KeyboardInterrupt` during `wait_result` is handled as follows:
- `PlaybackSession._handle_interrupt` calls `adapter.cancel(handle)` (`m6_action_client.py:321-325`);
- the adapter sends **one** cancel request and spins at most 5 s for the cancel response;
- it records the response (`ERROR_NONE` / `ERROR_REJECTED` / `ERROR_UNKNOWN_GOAL_ID` /
  `ERROR_GOAL_TERMINATED`) and then waits up to a further 5 s for the terminal status (CANCELED).

**3.** A second SIGINT during that wait stops waiting and records `cancel_unconfirmed`.

**4.** SIGTERM is treated like SIGINT (a handler installed by the CLI).

**5.** After a terminal state:
- destroy the subscriptions and the action client, then the node;
- `try_shutdown()`;
- exit 130 (interrupted) or the outcome's exit code.

**6. Hold.** The controller then holds its last command.
- **[FACT]** The string `Canceling active action goal because cancel callback received.` exists.
- **[HYPOTHESIS]** The controller holds position after a cancel, as it does after
  `Exceeded goal_time_tolerance: holding position...`. This is confirmed by observing
  `/joint_states` for 2 s after cancel in the live run (when a cancel occurs).

**7. Never a second goal.**
- The adapter's single-use counter and the session's `_dispatch` guard both refuse.
- No code path builds a neutral goal (mutation-tested, §12).

### 9.3 Launch cleanup [FACT, M6.0-B and M4.1]

- Stop the launch with **one Ctrl+C in Terminal A**, which sends SIGINT to the process group, and
  allow a 20 s grace (`docs/M4_1_TEST_RESULTS.md:376-385`).
- Run `ros2 daemon stop` and the leftover `pgrep`
  (`scripts/validate_m4_all_leg_ik.sh:190-195`).
- M6.0-B shut down cleanly in 9.6 s with no forced cleanup (`bffd00c` results §8).

## 10. Evidence model

Every artifact goes under the git-ignored `log/m6d_playback/<UTC>/` (**OD-D13**). Nothing is
overwritten. **[PLAN]**

| Tier | Artifact | Proves | Limits |
|---|---|---|---|
| Source | Static scan of the new modules: a single `send_goal(` call site (inside the adapter), no `create_publisher`, `switch_controller`, `set_parameters`, `cmd_vel`, `subprocess`; `git rev-parse HEAD` and a clean tree | Code paths | Not runtime |
| Tool | `outcome.json`: `goals_sent`, adapter `send_count`, state trace with wall and sim timestamps, the three channels, readiness report, `trajectory_id`, goal fingerprint | Intent and the client-side count | Self-reported |
| Action | Goal ID, accept/reject, result status and error, cancel request and response (if any); feedback CSV | Controller's response | Controller's view |
| Joint state | `joint_states.csv`: sim stamp, wall time, 12 positions; `tracking.json` (per-sample max error, worst joint and time, coverage, offset estimate) | **Independent** tracking | Simulation, placeholder actuators |
| Logs | `launch.log` counts: `Received new action goal` = 1, `Accepted new action goal` = 1, `Goal reached, success!` = 1 (on success), `Current goal cancelled due to new incoming action.` = 0, cancel strings (0, or 1 if interrupted), controller-manager lifecycle count unchanged between readiness and end | **At most one goal** reached the controller; no preemption; no switch | Depends on the log level; an unseen string is reported `unavailable` |
| Graph | Snapshots at T0 (readiness), T1 (during execution) and T2 (after): action servers = 1; action clients = 0 at T0, 1 (the adapter) at T1, 0 at T2; publishers on `/leg_trajectory_controller/joint_trajectory` = 0; `/cmd_vel` absent; the transient-local action `status` after T2 shows exactly **one** goal ID | One client and one goal ID | Snapshots, supporting only |
| Process | `ps` snapshots at T0 and T2; leftover `pgrep` after shutdown | No stray tool or process | – |

**Proof of "at most one goal"** needs all of:
- `send_count = 1`;
- the session's `goals_sent = 1`;
- the log count `Received new action goal = 1`;
- 0 preemption lines;
- one goal ID in the status topic.

Any `unavailable` component is reported as such.

## 11. Failure categories and no-retry policy

| ID | Category | When | Goal sent? |
|---|---|---|---|
| X1 | Offline preflight refused | Before readiness | No |
| X2 | Live readiness NOT READY (any M6.0-B code, including `start_pose_not_neutral`) or `incompatible` | Before dispatch | No |
| X3 | Readiness too old, or the operator did not confirm | Before dispatch | No |
| X4 | Gazebo start-up hang or discovery miss | Before readiness | No |
| X5 | Goal rejected or no goal response | Dispatch | Yes (one) |
| X6 | Abort −4 or −5 (tolerance) or −1, −2 or −3 | Execution | Yes (one) |
| X7 | Result watchdog | Execution | Yes (one), cancelled |
| X8 | Stale `/joint_states`, sim stall, extra publisher, controller disappearance | Execution | Yes (one), cancelled |
| X9 | Operator interrupt | Execution | Yes (one), cancelled |
| X10 | Tracking channel failed or `unavailable` with action success | After the result | Yes (one) |
| X11 | Evidence contradiction (for example a log goal count ≠ 1) | After | Investigate; FAIL |
| X12 | Cleanup failure (leftovers or daemon) | After | – |

**No-retry policy [PLAN, OD-D11].**
- **After a goal has been sent (X5–X11): no rerun** without a new, explicit owner approval after
  reviewing the evidence.
- **Before any goal (X1–X4): at most one manual rerun**, after full cleanup, and only for X4,
  as in M6.0-B OD-10.
- There is never an automatic retry.

## 12. Test strategy

**[PLAN]** These are batches, each needing owner approval. None is part of this plan's commit.

### 12.1 Unit, mock and mutation (cloud and local; no Gazebo, no ROS graph)

| Area | Tests | Mutations that must turn red |
|---|---|---|
| Fingerprint binding | Same trajectory gives the same fingerprint; any position, time, joint or tolerance change gives a different one; `send_goal` with a foreign goal sends nothing | Removing the fingerprint check; comparing the ID only |
| Single use | A second `send_goal` raises before any ROS call | Removing the counter |
| Signal handling | The adapter calls `rclpy.init` with `SignalHandlerOptions.NO` (fake `rclpy`); `KeyboardInterrupt` gives exactly one cancel and 0 goals; a second interrupt gives `cancel_unconfirmed` | Default signal options; cancel not sent; a goal sent after the interrupt |
| Stale, stall, gap | Fake clocks and streams for each §8.4 rule | Ignoring staleness; a missing gap check |
| Controller disappearance | Fake graph removing the server | Treating it as success |
| Readiness binding | READY needed; `incompatible` refuses; readiness older than 10 s refuses; no confirmation refuses | Bypassing readiness; accepting `incompatible` |
| Classification labels | The §14.9 rules (prerequisite OD-D4) | Mapping `joint_names_mismatch` to `warning` |
| Evidence writer | Deterministic schema; no overwrite | – |
| Static scan | Exactly one `send_goal(` call site; no forbidden words | Inserting a second call site or `create_publisher` |

### 12.2 In-process ROS test double (optional, **OD-D12**)

- A pure-`rclpy` `ActionServer` test double for `FollowJointTrajectory`, in the same test
  process, under an isolated `ROS_DOMAIN_ID`, with no Gazebo and no controller.
- It would prove that the real `RclpyActionAdapter` sends exactly one goal, that a cancel reaches
  the server after a simulated SIGINT, and that the context survives SIGINT.
- It creates real ROS nodes, so it needs owner approval for cloud and local runs.

### 12.3 Future live test

The live test is the §13 run on the owner's PC only, after every batch above passes cloud and
local verification.

## 13. Future live-run manual procedure (outline)

**[PLAN]** For the owner's Ubuntu PC only, with GUI and two terminals, as in M6.0-B. To be run
only after implementation, verification and a separate approval.

1. **Clean state.**
   - Check the approved commit and a clean tree.
   - Leftover `pgrep` is empty; `ros2 daemon stop`.
   - Static source scan (§10).
   - Read-only re-check of the controller's header and log strings on the local install
     (**OD-D15**).
2. **Offline.** `ros2 run spiderx_controller m6_offline_preflight --check-only`: PASS and the
   expected `trajectory_id`.
3. **Terminal A.**
   `ros2 launch spiderx_bringup fortress_control.launch.py 2>&1 | tee log/m6d_playback/<UTC>/launch.log`
4. **Terminal B.**
   - Wait until both controllers are `active` (≤ 240 s), then 3 s.
   - Record the T0 log counts and graph snapshot.
5. **Terminal B.** `ros2 run spiderx_controller m6_live_playback --out log/m6d_playback/<UTC>`:
   - offline preflight;
   - in-process readiness;
   - print the plan;
   - **operator confirmation**;
   - one goal;
   - record;
   - outcome.

   Wall cap: 120 s watchdog + 30 s margin.
6. **Terminal B, T1 and T2 evidence.**
   - Graph snapshots (action clients and servers, command topic, `/cmd_vel`).
   - Transient-local status read.
   - Log counts (§10).
7. **Observe 2 s more.** Read-only `/joint_states` for the hold, in the outcome tool.
8. **Terminal A.** One Ctrl+C; wait ≤ 20 s.
9. **Terminal B.** `ros2 daemon stop`; leftover `pgrep`; record.
10. **Results.** Write `docs/M6D_LIVE_PLAYBACK_RESULTS.md` (docs only).

**No step sends a second goal.** After a successful run the robot is held at neutral. After a
cancel it holds wherever it was, and the launch is simply stopped.

## 14. Risks and owner decisions

### 14.1 Risks

| Risk | Mitigation |
|---|---|
| Ctrl+C shuts the rclpy context down before the cancel (§9.1) | `SignalHandlerOptions.NO`; the §12 tests |
| Local controller 2.54.0 differs from the audited 2.48.0 | Read-only header and string re-check (OD-D15); the classification check |
| Goal tolerance uses the 0.01 rad/s stopped-velocity default (§5) | Reported. A `GOAL_TOLERANCE_VIOLATED` result is a FAIL to analyse, never retried |
| Placeholder actuators (100 N·m) make tracking look perfect | A non-claim; simulation only |
| The robot body may shift when the feet move (contact) | Not measured; no contact or body claims |
| Readiness evidence is snapshots | Several tiers; `unavailable` is reported |
| `main` docs stale; M6.0-B evidence on an unmerged branch | OD-D14 |

### 14.2 Owner decisions [OWNER DECISION]

> **Superseded.** The options below are the Phase 0 proposal, kept for the record. The approved
> decisions are D1–D17 in [§16](#16-approved-owner-decisions-d1d17), which governs. OD-D8 and OD-D16 were decided **differently**
> from these recommendations: both abort paths are used, and the goal velocity tolerance is an
> explicit 0.05 rad/s.

| ID | Decision | Options | Recommendation | Safe default |
|---|---|---|---|---|
| **OD-D1** | Adapter location | (a) new `m6_live_adapter.py` + `m6_live_playback.py` + wrapper in `spiderx_controller`; (b) extend `trajectory_client.py` | (a), per D7 | (a) |
| **OD-D2** | Signal handling | (a) `SignalHandlerOptions.NO` with our own SIGINT/SIGTERM path; (b) the rclpy default | (a) | (a) |
| **OD-D3** | Readiness | (a) in-process re-check right before dispatch, max age 10 s; (b) rely on a separate M6.0-B run | (a) | (a) |
| **OD-D4** | Classification labels | (a) implement the §14.9 labels in `m6_live_preflight` first, with `incompatible` blocking; (b) manual classification only | (a), already named an M6.0-D prerequisite | (a) |
| **OD-D5** | Tracking time base | (a) adapter sim-time `T_accept` with a feedback offset cross-check, goal stamp 0 unchanged; (b) an explicit future `header.stamp` (changes the goal and the tests) | (a) | (a) |
| **OD-D6** | Live stream limits | Stale 0.5 s wall → cancel-only; sim stall 5 s wall → cancel-only; gap 0.25 s sim → tracking failed; extra publisher → cancel-only | As listed | Cancel-only on any doubt |
| **OD-D7** | Controller disappearance | (a) graph check once per second → cancel attempt, `error_holding`; (b) watchdog only | (a) | (a) |
| **OD-D8** | In-flight tracking abort | (a) rely on the controller's 0.05 rad `path_tolerance` plus the post-hoc channel; (b) also a client-side cancel when the observed error > 0.05 rad | (a): one abort authority | (b), if the owner prefers maximum caution |
| **OD-D9** | Waits | Goal response 10 s; cancel response 5 s, then terminal status 5 s; result watchdog 120 s | As listed | As listed |
| **OD-D10** | Operator confirmation | (a) a typed confirmation word before dispatch; (b) a `--yes` flag; (c) none | (a) | (a) |
| **OD-D11** | Retry | No rerun after dispatch without new approval; one manual pre-dispatch rerun for X4 only | As listed | As listed |
| **OD-D12** | In-process ROS test double (§12.2) | (a) allowed in tests (isolated domain, no Gazebo); (b) not allowed | (a) | (b) until approved |
| **OD-D13** | Evidence location and results doc | `log/m6d_playback/<UTC>/`; `docs/M6D_LIVE_PLAYBACK_RESULTS.md` after the run | As listed | As listed |
| **OD-D14** | Merge order | (a) merge the M6.0-B branch (`bffd00c`) to `main` before the M6.0-D implementation branches from `main`; (b) branch the implementation from the graph-preflight branch | (a) | (a) |
| **OD-D15** | Local stack re-check | Read-only re-check of the controller headers and `.so` strings on the owner's 2.54.x install before the live run | Required | Required |
| **OD-D16** | Goal velocity tolerance | (a) keep it unspecified (controller default 0.01 rad/s), reported; (b) set an explicit goal velocity tolerance (a goal and test change) | (a) | (a) |
| **OD-D17** | Run environment | Owner's Ubuntu PC only, GUI, manual two terminals (as M6.0-B) | As listed | As listed |

## 15. Out of scope

- **This commit:** any code, test, script, config, launch, URDF, controller or YAML change, and
  any runtime operation. Only this plan file is added.
- **Even after approval:**
  - any second goal, retry or automatic neutral return;
  - any trajectory other than the D1 content;
  - any widening of the 0.1223 rad cap or the other limits.
- **Not done:**
  - `/cmd_vel` and M5.5;
  - controller switching or lifecycle requests;
  - parameter changes on other nodes;
  - controller-YAML tolerances.
- **Not measured:**
  - contact, foot-force, body-pose, balance or stability measurement;
  - walking, gait cycles and M6.1;
  - M7 odometry and state estimation, M8 SLAM, M9 Nav2, M10 hardware.
- **Not used:**
  - headless mode;
  - a live run in the cloud;
  - hardware, servos, firmware.
- **Not touched:** editing the stale `main` docs (§2.1) or the historical documents.
- Opening a PR or merging.

## 16. Approved owner decisions (D1–D17)

**[OWNER DECISION]** Approved after the Phase 0 review. They correspond to OD-D1 to OD-D17 above.

| ID | Decision |
|---|---|
| **D1** Adapter location | New M6-specific modules only (§3.2). `trajectory_client.py` and the existing M1–M5 command tools are not modified |
| **D2** Signal handling | The live adapter disables rclpy's installed default signal handlers (`SignalHandlerOptions.NO`). It owns SIGINT/SIGTERM handling, so it can attempt **at most one** cancel before the ROS context shuts down (§9.2). No automatic return-to-neutral command is permitted |
| **D3** Same-process readiness | The adapter re-checks readiness in the same process immediately before dispatch (§6). The required evidence may be no older than **10 s**. A failed or stale readiness result blocks goal construction and dispatch |
| **D4** Classification | `compatible`, `warning` and `incompatible` are implemented in code before any live goal. `incompatible` blocks dispatch. A version difference alone is `warning` when the endpoint, interface and joint-contract checks pass |
| **D5** Time base | The goal header stamp stays zero. Tracking is measured from goal acceptance time and cross-checked against action feedback (§8.2) |
| **D6** Live stream thresholds | `/joint_states` stale > 0.5 s → cancel only. Sim time stalled > 5 s → cancel only. Tracking sample gap > 0.25 s → tracking failure. Extra `/joint_states` publisher → cancel only |
| **D7** Controller disappearance | Controller status and availability are checked once per second while the goal is in flight. Loss triggers **one** cancel attempt, then the error-holding state |
| **D8** In-flight error | Controller-side path and goal position tolerances are 0.05 rad. The client independently cancels if its observed tracking error exceeds 0.05 rad. No new trajectory is ever sent automatically |
| **D9** Waits | Goal response 10 s; cancel response 5 s; final status after cancel 5 s; full goal/result watchdog 120 s |
| **D10** Operator confirmation | A typed confirmation word is required before dispatch. There is no `--yes` bypass for the first live goal |
| **D11** Retry | After a goal is sent, no retry or second goal without a new owner approval. Before dispatch, one manual retry only, for a documented Gazebo start-up hang or discovery-timing miss, after full cleanup |
| **D12** In-process test double | An isolated-domain, in-process ROS action test double is allowed **for tests only**. It never uses the default ROS domain and never runs with Gazebo |
| **D13** Evidence | Runtime evidence lives under the ignored `log/m6d_playback/<UTC>/`. Later tracked results: `docs/M6D_LIVE_PLAYBACK_RESULTS.md` |
| **D14** Dependency | The M6.0-B graph-preflight evidence, merged in `main`, is a required prerequisite. This implementation branch starts from `9885057` |
| **D15** Local stack | Before any live goal, the owner's installed 2.54.x controller and action headers and the relevant controller log strings are re-checked. Differences from the cloud are classified, not ignored |
| **D16** Goal velocity tolerance | Set the goal velocity tolerance explicitly to **0.05 rad/s**, for the M6.0-D goal only. No controller-YAML change (see the note below) |
| **D17** Runtime environment | Any live run happens only on the owner's Ubuntu PC, in GUI mode, with the manual two-terminal procedure. No cloud live playback |

**Note on D16 [FACT].** Two velocity fields are involved, and only one changes:
- **Trajectory point velocities (unchanged).** These are explicitly `0.0` at every waypoint
  (`m6_trajectory.py:326`), and the preflight refuses anything else (`:475-476`).
- **Goal tolerance velocity (the field D16 changes).** `tolerance_entries`
  (`m6_action_client.py:94-102`) currently sets `velocity: 0.0`. In `JointTolerance`, 0 means
  "unspecified", so the installed controller falls back to its default `stopped_velocity_tolerance`
  of 0.01 rad/s (`tolerances.hpp:114`).
- **The change.** D16 replaces that reliance with an explicit 0.05 rad/s in `goal_tolerance`, for
  the M6.0-D goal only.
- **Follow-on code updates.** The goal builder, the planned goal fingerprint (§3.3) and the
  existing goal tests (`test_m6_action_client.py`) must be updated in an implementation batch.
  `path_tolerance` velocity stays unspecified.

**Unchanged by adoption.**
- Every limit in §5, apart from the D16 row.
- The single-goal state machine (§4).
- The evidence model (§10).
- The no-retry policy (§11).
