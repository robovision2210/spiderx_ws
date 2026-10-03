# M6.0-D Live-Enabling Design (dispatch still disabled)

> **Status:** design and wiring implemented and verified offline and on an isolated test domain
> only, including the corrective batches from the PR #16 read-only review (§8). **Live dispatch
> remains hard-disabled** (`LIVE_DISPATCH_ENABLED = False`), so `m6_live_playback --live` still
> refuses with exit 3.
>
> **Nothing has been sent.** No live goal has ever been sent. Nothing in this change started
> Gazebo, a launch, a controller or a live ROS graph query.
>
> **What enabling requires.** A separate owner approval, then the two-line enabling commit of §4,
> then the manual checklist of §6.

- **Builds on:**
  - the [M6.0-D plan](M6D_LIVE_PLAYBACK_PLAN.md), with D1–D17 in §16;
  - the [M6.0-D results](M6D_LIVE_PLAYBACK_RESULTS.md);
  - the [M6.0-D guide](SPIDERX_M6D_LIVE_PLAYBACK_GUIDE.md).
- **Base:** `main` @ `e65c213`, with PR #15 merged.
- **Branch history.** This work is on `claude/stoic-shannon-ur2mes`, a reused branch name. Its
  earlier pull request, PR #4 (the Gazebo Fortress path), was **merged** into `main` on
  2026-09-25 (merge commit `d6bcec1`, head `d85968f`). The branch was restarted from `main` @
  `e65c213` for this work, so no PR #4 history is re-introduced. An earlier session report wrongly
  described PR #4 as "closed and unmerged".

## 1. Boundary (unchanged)

**At most one live goal ever, per owner approval:**
- neutral (3 s) → `crouch_10mm` (6 s) → neutral (9 s);
- 12 joints, with point velocities of 0;
- maximum displacement 0.12229413600889982 rad, at `rr_foot_joint`, against a cap of
  0.1223 rad + 1e-9.

**Limits:**
- path and goal position tolerance 0.05 rad;
- goal velocity tolerance 0.05 rad/s; path velocity tolerance unspecified;
- goal time tolerance 1.0 s; header stamp 0;
- trajectory ID `44f0a7ad52e5c330`;
- goal fingerprint `0d6ef4171f2d338a01e76b934be94d1a4386e7c0fc68fe74262fb3c65005ff83`.

**Behaviour:**
- **No retry, no second goal, no automatic return, no preemption.**
- At most one cancel request. A second Ctrl+C gives `CANCEL_UNCONFIRMED` (stop waiting).
- **Controller hold is intended, not tested.** The plan expects the controller to hold after a
  cancel or abort (re-checked on the owner's stack by D15). The tests prove only the client side:
  one cancel request at most and no further command. Closing the client does not stop an accepted
  goal.
- **One goal per session and process.** The guarantee is enforced inside one session, transport
  and process. It is not a durable cross-process prohibition: a separate invocation could send
  another goal, which only the checklist ("never re-run") and a new owner approval exclude. No
  durable dispatch marker exists (a deliberate decision, not part of this change).

**Unchanged:** the trajectory, every limit and tolerance, and every wait.
`InterruptLatch`, `StreamMonitor`, `graph_violation` and the fingerprint module are byte-identical
to `main` @ `e65c213`. `LiveSession`, `_run_live`, the readiness module and the mock module were
changed deliberately by the corrective batches (§8) and re-pinned. SHA-256 pins in
`test_m6d_live_enabling.py` enforce both groups.

## 2. The feature gate

| Item | Value |
|---|---|
| Name | `LIVE_DISPATCH_ENABLED` |
| Location | `src/spiderx_controller/spiderx_controller/m6_live_contract.py`, line 17 |
| Current value | **`False`** |
| Test-side expectation | `src/spiderx_controller/test/m6d_gate.py`: `EXPECTED_LIVE_DISPATCH_ENABLED = False` |
| Read by | `m6_live_playback.main()` (first statement of the `--live` branch, before configuration, ROS import or input), `m6_live_playback._live_main()` (defensive re-check before any evidence or ROS work) and `m6_live_playback._run_live()` (`PermissionError`) |
| Overrides | **None.** There is no CLI option, environment variable or configuration file for it. Static tests require exactly one assignment of the literal in the package. They also forbid `os.environ` and `getenv`, as well as `--yes`, `--force` and `--enable*` options |

**Why it exists.** The whole M6.0-D path can be implemented, reviewed and tested while a live goal
stays impossible. Turning it on is then a single, visible, owner-approved code change. It is not a
runtime choice an operator could make by accident.

## 3. What this change implements behind the gate

When `LIVE_DISPATCH_ENABLED` is `True`, `--live` runs `_live_main` and `_run_live` once, in this
order. Every step can only refuse; none can retry:

1. **No `--no-write`.** A live run always records evidence; `--no-write` refuses (exit 2).
2. **Domain id.** `--domain-id N` is required, in 0..232, and typed explicitly. It is never read
   from the environment. Without it: REFUSED, exit 2, before any ROS import.
3. **Evidence reservation.** `log/m6d_playback/live/<UTC>/live_outcome.json` is created with
   `O_CREAT | O_EXCL` (an existing file refuses) and a `NOT_DISPATCHED` record (`phase: reserved`)
   is saved. If either fails, nothing starts (exit 2). All later records are written through the
   descriptor this run created - never a rename or reopen - so no other run's evidence can be
   overwritten.
4. **Interrupt latch.** The latch takes over SIGINT/SIGTERM for the run and is always restored.
5. **Trajectory.** It is built and preflighted, and its approved fingerprint is computed.
6. **Transport.** One `RclpyLiveTransport` is created: its own context, rclpy signal handlers off,
   one action client, one `/joint_states` subscription, no publisher. If any step of `open()`
   fails, everything it created is released; the run is refused with nothing sent.
7. **Readiness #1.** It runs in the same process: `RclpyLiveTransport.graph_collector()`, the
   M6.0-B read-only `GraphProbe` on the transport's own executor (the `list_controllers` call
   completes on that executor). The result is aged from the **start** of the observation and
   must be ≤ 10 s old, ready, and `compatible` or `warning`; `incompatible` refuses.
8. **Confirmation.** Only now is the prompt printed. The operator must type
   `SEND-ONE-CROUCH-GOAL` exactly; anything else, EOF or Ctrl+C refuses. The prompt checks the
   interrupt latch every 0.1 s, so Ctrl+C ends it without Enter, and an interrupt wins over typed
   text.
9. **Readiness #2.** It is re-collected in the same process, with the same rules.
10. **Rebuild.** The goal is rebuilt, and its fingerprint must equal the preflighted one. The
    action server must be ready within 10 s.
11. **Pre-send record.** A `pre_send` record is saved; if it cannot be, the run refuses without
    sending. If this is the last record a reader finds, the goal status is unknown.
12. **At the send.** The latch is checked again and readiness #2 must still be fresh at this
    instant (after the server wait and the record write). Stale evidence refuses once - no new
    observation, no retry.
13. **One `send_goal`.** The transport verifies the fingerprint again before the ROS call. It is
    single-use.
14. **Supervision.** As in `LiveSession` (D6–D9), plus: any transport or processing failure after
    the send is recorded and requests at most the one cancel through the same guard; the outcome
    records whether the final goal status is known.
15. **Final record.** The complete outcome (`phase: final`) is saved and the transport is closed
    (a close failure is recorded, not hidden). If the final write fails, the outcome goes to
    stderr, the exit code is 1, no report is claimed and nothing is retried.

## 4. The enabling commit (later; needs separate owner approval)

It changes **exactly two lines** and nothing else:

```diff
--- a/src/spiderx_controller/spiderx_controller/m6_live_contract.py
+++ b/src/spiderx_controller/spiderx_controller/m6_live_contract.py
-LIVE_DISPATCH_ENABLED = False
+LIVE_DISPATCH_ENABLED = True
--- a/src/spiderx_controller/test/m6d_gate.py
+++ b/src/spiderx_controller/test/m6d_gate.py
-EXPECTED_LIVE_DISPATCH_ENABLED = False
+EXPECTED_LIVE_DISPATCH_ENABLED = True
```

**Commit details:**
- Suggested title: `feat(m6d): enable the one owner-approved live M6.0-D goal`.
- It goes on its own branch from the `main` that contains this design, after a final audit.

**Verified in advance [MEASURED, cloud].**
- This two-line flip was applied locally, the M6.0-D suites were run, and it was then reverted.
  It was never committed.
- 210 M6.0-D tests pass with the gate `False` and also with it `True`.
- With the gate `True`, every enabled-path test still uses the in-memory mock or the isolated
  test domain. A `--live` call without `--domain-id` refuses before any ROS import.

**Expected side effects of the flip:**
- The `--help` and banner text change from `HARD-DISABLED in this build` to
  `ENABLED for exactly one goal`.
- `--live` stops returning exit 3.
- The dry-run and mock reports record `live_dispatch_enabled: true`, so their SHA-256 values
  change from those in the M6.0-D results §6.

## 5. Tests in this change

| Area | Proof |
|---|---|
| Gate `False` | `main --live` returns 3 before `load_sources`, the adapter import, `rclpy` import or reading input. `_run_live` raises `PermissionError`. Nothing is written |
| Gate `True` (in-test only) and the other gates | All of these refuse with 0 goals: a missing or out-of-range `--domain-id` (no `rclpy` import); a wrong confirmation; each not-ready stack (no controller manager, no action server, no samples, two publishers); stale readiness; an existing report |
| Healthy mock | Exactly 1 goal and 0 cancels; `retries 0`; `automatic_return_goals 0`; the domain id is recorded; the transport is closed; the signal handlers are restored. The mock failure scenarios give one goal, at most one cancel and no retry |
| Prompt | It is printed only after readiness #1 passes |
| Protected baseline | SHA-256 pins: `InterruptLatch`, `StreamMonitor`, `graph_violation` and the fingerprint module unchanged since `e65c213`; `LiveSession`, `_run_live`, the readiness and mock modules deliberately re-pinned (§8); every limit |
| Evidence and exceptions (`test_m6d_evidence_safety.py`) | Failure injected at reservation, initial record, pre-send record, the send/acceptance boundary, polling, cancel, graph check, simulation time, final write and serialization, setup and close; each asserts sends, cancels and the retained uncertainty |
| Freshness and confirmation (`test_m6d_freshness_confirmation.py`) | Slow collections, slow server wait (10.0 s permitted, 10.05 s refused), long pause, interrupt during the server wait, a mutant without the send-time check, latched and real SIGINT at a blocked prompt, EOF / wrong word / first / second interrupt |
| Identity | Trajectory ID and goal fingerprint pinned under both test-local gate states; the gate changes report hashes only through `live_dispatch_enabled` fields |
| Isolated domain | Domain 150–199 with `ROS_LOCALHOST_ONLY=1`, in-process test doubles only, isolation proven before and after. The gated wiring refuses at readiness against an action server alone; `open()` failures release everything; and `main()` with the gate set True **in the test only** succeeds end to end against an in-process fake controller stack: real transport, real `list_controllers` completion on the transport's executor, both readiness checks, send-time freshness, exactly one goal received, result, tracking, evidence and clean teardown |

## 6. Future manual checklist (documented only; NOT executed)

This checklist is for the owner's Ubuntu PC only, in GUI mode, with two terminals (D17); never in
the cloud. Run it **only** after the enabling commit of §4 has been approved, merged and built.

**Clean state.**
1. Check the source:
   - `git status --short` shows a clean tree;
   - `git rev-parse HEAD` is the approved commit;
   - `grep -n '^LIVE_DISPATCH_ENABLED' src/spiderx_controller/spiderx_controller/m6_live_contract.py`
     shows `True`.
2. Build and check for leftovers:
   - `colcon build --symlink-install && source install/setup.bash`;
   - the leftover `pgrep` (M6.0-B plan §9) is empty;
   - `ros2 daemon stop`.

**D15: re-check the installed 2.54.x controller (read-only).**

3. Check the installed versions:
   - `ros2 pkg xml joint_trajectory_controller | grep '<version>'`;
   - `ros2 pkg xml controller_manager | grep '<version>'`;
   - `ros2 pkg xml control_msgs | grep '<version>'`.
4. Check the tolerance semantics in the installed `tolerances.hpp`. A goal tolerance > 0 must be
   taken from the goal, and the default `stopped_velocity_tolerance` must be 0.01:
   `grep -n 'stopped_velocity_tolerance\|goal_time' $(ros2 pkg prefix joint_trajectory_controller)/include/joint_trajectory_controller/tolerances.hpp`.
5. Check the log strings of plan §7.1 in the installed library:
   `strings $(ros2 pkg prefix joint_trajectory_controller)/lib/libjoint_trajectory_controller.so | grep -E 'Received new action goal|Accepted new action goal|Goal reached, success!|Got request to cancel goal|holding position'`.
6. Record the differences from the cloud 2.48.0 reference and classify each one. **Stop** if a
   tolerance or hold behaviour differs.

**Offline checks.**

7. Run each of these and check its result:
   - `ros2 run spiderx_controller m6_offline_preflight --check-only`: PASS, `44f0a7ad52e5c330`;
   - `ros2 run spiderx_controller m6_live_playback --dry-run --no-write`: the expected
     fingerprint.

**Terminal A: the approved launch, unmodified.**

8. `mkdir -p log/m6d_playback/<UTC> && ros2 launch spiderx_bringup fortress_control.launch.py 2>&1 | tee log/m6d_playback/<UTC>/launch.log`

**Terminal B: readiness.**

9. Wait until `ros2 control list_controllers` shows `joint_state_broadcaster` and
   `leg_trajectory_controller` both `active` (≤ 240 s), then wait 3 s more.
10. Run the read-only preflight: `timeout 60 ros2 run spiderx_controller m6_live_preflight --timeout 10 --window 2`.
    It must say `READY`. Record the version differences; a `warning` is acceptable,
    `incompatible` is not.
11. Record the T0 zero-command evidence (M6.0-B plan §9, step 5):
    - 0 publishers on the command topic;
    - 1 action server and 0 action clients;
    - an empty action status;
    - the T0 count of log strings.

**Terminal B: the one goal.**

12. `ros2 run spiderx_controller m6_live_playback --live --domain-id <the domain of terminal A; 0 if ROS_DOMAIN_ID is unset> --out log/m6d_playback`
    - The tool runs same-process readiness #1. It refuses unless the result is fresh, ready, and
      `compatible` or `warning`.
    - Then it prints the prompt. Type exactly `SEND-ONE-CROUCH-GOAL` once. Anything else refuses
      and nothing is sent.
    - Readiness #2, the pre-send record, the send-time freshness check and **one** goal follow.
    - Watch Gazebo and the terminal. The wall cap is 120 s (watchdog) + 30 s.
    - In an emergency, press Ctrl+C **once**. That sends one cancel request; the controller is
      expected to hold (intended, D15-checked, not proven by any test). A second Ctrl+C only ends
      the wait (`CANCEL_UNCONFIRMED`).
    - If the tool prints that the final goal status is UNKNOWN, the controller may still be
      executing the goal: observe and record; do not re-run the tool.
13. Record the result:
    - the final state, exit code and report path, under `log/m6d_playback/live/<UTC>/`, and the
      report's `evidence.phase` (`final`; `pre_send` as the last record means status unknown);
    - the T2 evidence: action status (exactly one goal ID), log-string counts, publishers on the
      command topic (0).

**Shutdown.**

14. Terminal A: press Ctrl+C **once**, and wait up to 20 s.
15. Terminal B: `ros2 daemon stop`. Then the leftover `pgrep` must be empty.
16. Write the results into `docs/M6D_LIVE_PLAYBACK_RESULTS.md`, as a docs-only change.

**Never, at any step:**
- re-run the tool after it has sent a goal;
- send a second goal, a return-to-neutral or any other command;
- retry after a rejection, abort, timeout or tracking failure;
- run it in the cloud, headless or against hardware.

A failure is recorded as it happened. A new attempt needs a new owner approval.

## 7. Non-claims

This design and its tests do not show:
- live dispatch or controller goal acceptance;
- joint tracking or Gazebo movement;
- contact, body support or balance;
- gait replay, locomotion or walking;
- navigation or hardware capability.

M6.1, M7–M10, navigation, gait playback, contact/base testing and hardware work are not started.

## 8. Corrective changes from the PR #16 read-only review

All offline, mock and isolated-domain only; the gate stayed `False` throughout.

| Finding | Resolution | Commit |
|---|---|---|
| 1. The live report could be lost or skipped (`--no-write`; write only after the run) | `--no-write` refused with `--live`; exclusive reservation and a `NOT_DISPATCHED` record before ROS; `pre_send` record before the send (refuse if unsaved); fd-only rewrites; stderr fallback and exit 1 if the final write fails | `5e24fed` |
| 2. Exceptions after dispatch lost the outcome | Failures in the graph check, polling, events, simulation time and the cancel call are recorded and request at most the one cancel through the existing guard; escaping exceptions become a truthful outcome; the `dispatch` block records acceptance, goal ID, cancel and final-status knowledge; closing the client is recorded as not stopping the goal | `5e24fed` |
| 3. Freshness checked at the gate, not at the send; collection time hidden | Readiness aged from the start of its observation (collection time recorded); freshness re-checked immediately before the send, after every wait; stale evidence refuses once | `ca93898` |
| 4. Ctrl+C at the prompt waited for Enter | Latch-aware prompt (0.1 s slices); an interrupt wins over typed text | `ca93898` |
| 5. One goal per process only | Documented as a boundary (per session and process; no durable marker) | docs |
| 6. A parametrized test never asserted its state | It now asserts the expected terminal state | `8937bdd` |
| 7. Guide examples out of date | Focused-test list and explicit `--domain-id` examples updated | docs |
| 8. A partially opened transport was not released | `open()` releases everything it created if a step fails; `close()` attempts every step | `5e24fed` |
| Found while fixing 4: two interrupts latched within one 50 ms poll skipped the one cancel (pre-existing in the merged `LiveSession`) | The first interrupt is always handled first: one cancel, then stop waiting | `ca93898` |
| Coverage gap: no isolated successful path | `test_m6d_isolated_success.py` drives `main()` end to end on an isolated domain against an in-process fake controller stack | `8937bdd` |
| Coverage gap: identity not pinned across gate states | Trajectory ID and fingerprint pinned under both test-local gate states; hash-vs-identity test | `8937bdd` |

**Validation of the corrective batches [MEASURED, cloud, gate False].**
- **Build and full suite.** Clean build of 8 packages, then `colcon test`: **1213 tests, 0 errors,
  0 failures, 0 skipped**.
- **M6.0-D tests: 273.**
  - `contract` 83, `session` 58, `cli_readiness` 29, `live_enabling` 33;
  - `evidence_safety` 23, `freshness_confirmation` 30;
  - `adapter_isolated` 15, `isolated_success` 2.
  - The isolated success-path test ran (not skipped) and passed in 4 separate runs, with no
    leftover process.
- **Determinism and identity.** The dry run and the six mock scenarios ran twice and were
  byte-identical. Every report carries trajectory ID `44f0a7ad52e5c330` and goal fingerprint
  `0d6ef4171f2d338a…`.
  - The dry-run report is unchanged from PR #15 (`b3174cd5…`).
  - The mock reports gained the `dispatch`, `errors`, `freshness_at_send` and readiness
    `collection_s` fields, so their hashes changed:

    | Mock scenario | New SHA-256 |
    |---|---|
    | `success` | `9ec6b6da2e62bdde7fb6c01f53e126c0b162e19fe2a20098ed35c09f6c1bdd92` |
    | `interrupt` | `977b6e6bd391d99139a60faf8d11d94ff47a9ad9a7131748c9db1214e6d30103` |
    | `tracking_error` | `f122802dc42c2c9773db6111ec0f32e9cc0b4fd09fb1ff19a85686fcc2fd533a` |
    | `stale_joint_states` | `0f6274b37e221c5241eca33e031aef75cb4691836b463dce074305f2823a305f` |
    | `controller_lost` | `9abad20116006d7ba802c074b8b5fb828ed8bd4829239ec67552927f75837a30` |
    | `rejected` | `f50aeb6434db95b5de2789d6daccdb066958ae5f802378f6dafa684f1c57db9c` |

- **Disabled `--live`.** `--live --domain-id 0` exits 3 in a fresh process. It imports no `rclpy`
  or adapter module, leaves the piped confirmation word unread, and creates no output directory.
- **Static validators (no arguments).** Fortress 58, M1 14, M2 17, M3 17 and M4 18 PASS, 0 FAIL.
  No Gazebo, launch, bridge, spawner, RViz or ROS daemon process was running afterwards.

**Remaining limitations.**
- No real controller, Gazebo or hardware has been exercised; the fake stack and mocks prove only
  the client side.
- An evidence write that fails part-way can leave an incomplete file; the stderr fallback then
  holds the complete outcome.
- The single-goal guarantee is per session and process (see §1).
- The isolated tests use domains 150-199, whose DDS ports may overlap the Linux ephemeral port
  range. The adapter tests assert isolation; the success-path test checks it first and skips with
  the reason if it cannot be established.
