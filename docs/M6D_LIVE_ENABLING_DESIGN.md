# M6.0-D Live-Enabling Design (dispatch still disabled)

> **Status:** design and wiring implemented and verified offline and on an isolated test domain
> only. **Live dispatch remains hard-disabled** (`LIVE_DISPATCH_ENABLED = False`), so
> `m6_live_playback --live` still refuses with exit 3.
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
- One cancel, then hold. A second Ctrl+C gives `CANCEL_UNCONFIRMED`.

**Unchanged by this design:**
- the trajectory, every limit and tolerance, and every wait;
- `LiveSession`, `StreamMonitor`, `InterruptLatch` and `graph_violation`;
- `_run_live`, and the fingerprint, readiness and mock modules.

These are byte-identical to `main` @ `e65c213`, and SHA-256 pins in `test_m6d_live_enabling.py`
enforce it.

## 2. The feature gate

| Item | Value |
|---|---|
| Name | `LIVE_DISPATCH_ENABLED` |
| Location | `src/spiderx_controller/spiderx_controller/m6_live_contract.py`, line 17 |
| Current value | **`False`** |
| Test-side expectation | `src/spiderx_controller/test/m6d_gate.py`: `EXPECTED_LIVE_DISPATCH_ENABLED = False` |
| Read by | `m6_live_playback.main()` (first statement of the `--live` branch, before configuration, ROS import or input) and `m6_live_playback._run_live()` (second gate: `PermissionError`) |
| Overrides | **None.** There is no CLI option, environment variable or configuration file for it. Static tests require exactly one assignment of the literal in the package. They also forbid `os.environ` and `getenv`, as well as `--yes`, `--force` and `--enable*` options |

**Why it exists.** The whole M6.0-D path can be implemented, reviewed and tested while a live goal
stays impossible. Turning it on is then a single, visible, owner-approved code change. It is not a
runtime choice an operator could make by accident.

## 3. What this change implements behind the gate

When `LIVE_DISPATCH_ENABLED` is `True`, `--live` runs `_live_main` and `_run_live` once, in this
order. Every step can only refuse; none can retry:

1. **Domain id.** `--domain-id N` is required, in 0..232, and typed explicitly. It is never read
   from the environment. Without it: REFUSED, exit 2, before any ROS import.
2. **Report target.** The target is `log/m6d_playback/live/<UTC>/live_outcome.json`. If the report
   already exists, the run refuses before anything starts.
3. **Interrupt latch.** The latch takes over SIGINT/SIGTERM for the run and is always restored.
4. **Trajectory.** It is built and preflighted, and its approved fingerprint is computed.
5. **Transport.** One `RclpyLiveTransport` is created: its own context, rclpy signal handlers off,
   one action client, one `/joint_states` subscription, no publisher.
6. **Readiness #1.** It runs in the same process: `RclpyLiveTransport.graph_collector()`, the
   M6.0-B read-only `GraphProbe` on the transport's own executor. The result must be fresh
   (≤ 10 s), ready, and `compatible` or `warning`; `incompatible` refuses.
7. **Confirmation.** Only now is the prompt printed. The operator must type
   `SEND-ONE-CROUCH-GOAL` exactly; anything else refuses.
8. **Readiness #2.** It is re-collected in the same process, with the same rules.
9. **Rebuild.** The goal is rebuilt, and its fingerprint must equal the preflighted one. The action
   server must be ready within 10 s.
10. **One `send_goal`.** The transport verifies the fingerprint again before the ROS call. It is
    single-use.
11. **Supervision.** As in the merged `LiveSession` (D6–D9). The run ends in a terminal state,
    writes one report, closes the transport and exits 0, 1 or 2.

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
| Unchanged logic | SHA-256 pins on the session, monitors, latch and `_run_live`, plus on the fingerprint, readiness and mock modules, plus every limit |
| Isolated domain | Domain 150–199 with `ROS_LOCALHOST_ONLY=1`; in-process test double only. `graph_collector()` sees only the test double (no controllers, no `/joint_states` publisher). The full gated `--live` wiring with the real rclpy transport refuses at readiness: the reader is never called, and the server receives 0 goals and 0 cancels |

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
    - Readiness #2, the fingerprint check and **one** goal follow.
    - Watch Gazebo and the terminal. The wall cap is 120 s (watchdog) + 30 s.
    - In an emergency, press Ctrl+C **once**. That sends one cancel, then the robot holds. A
      second Ctrl+C only ends the wait (`CANCEL_UNCONFIRMED`).
13. Record the result:
    - the final state, exit code and report path, under `log/m6d_playback/live/<UTC>/`;
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
