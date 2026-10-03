# M6.0-D Gate-Enabling Plan and Future Live-Run Checklist (design only)

> **Status: design note only. Nothing here is implemented, approved or run.**
> - `LIVE_DISPATCH_ENABLED` is `False` (`m6_live_contract.py:18`).
> - PR #16 is at `f0b1c1b`, a draft, unmerged.
> - Cloud verification is complete; local verification on the owner's PC is pending.
> - No live goal, Gazebo, launch or hardware activity is authorized. Writing this note started
>   none.
>
> **Supersedes** the checklist in [M6D_LIVE_ENABLING_DESIGN.md §6](M6D_LIVE_ENABLING_DESIGN.md)
> for the future run. Two corrections are noted in §2 (the `tolerances.hpp` path and hold
> evidence).

Related documents:
- [M6D_LIVE_PLAYBACK_PLAN.md](M6D_LIVE_PLAYBACK_PLAN.md) (D1–D17 in §16);
- [M6D_LIVE_ENABLING_DESIGN.md](M6D_LIVE_ENABLING_DESIGN.md) (wiring and corrections);
- [SPIDERX_M6D_LIVE_PLAYBACK_GUIDE.md](SPIDERX_M6D_LIVE_PLAYBACK_GUIDE.md);
- [M6_GRAPH_PREFLIGHT_PLAN.md §9](M6_GRAPH_PREFLIGHT_PLAN.md#9-local-execution-checklist)
  (zero-command evidence).

## 1. The enabling change

### 1.1 What changes

The intended change is two lines, `False` → `True`, and nothing else in the package:

```diff
--- a/src/spiderx_controller/spiderx_controller/m6_live_contract.py
+++ b/src/spiderx_controller/spiderx_controller/m6_live_contract.py
@@ line 18
-LIVE_DISPATCH_ENABLED = False
+LIVE_DISPATCH_ENABLED = True
--- a/src/spiderx_controller/test/m6d_gate.py
+++ b/src/spiderx_controller/test/m6d_gate.py
@@ line 8
-EXPECTED_LIVE_DISPATCH_ENABLED = False
+EXPECTED_LIVE_DISPATCH_ENABLED = True
```

**Finding from preparing this note [FACT, `f0b1c1b`].** The "exactly two lines" property no
longer holds as committed:
- `test/test_m6d_isolated_success.py:104` asserts `lc.LIVE_DISPATCH_ENABLED is False` directly,
  and would fail after the two-line flip.
- The other gate-related assertions are gate-independent, so they are unaffected:
  - `test_m6d_cli_readiness.py:140`, `test_m6d_contract.py:43` and
    `test_m6d_live_enabling.py:116`;
  - the identity, mock and dry-run tests, which compare against `m6d_gate`.

Two options, and the owner decides:

| Option | What it means |
|---|---|
| **A (recommended)** | Before PR #16 is merged, a reviewed one-line, test-only fix replaces that assertion with `lc.LIVE_DISPATCH_ENABLED is m6d_gate.EXPECTED_LIVE_DISPATCH_ENABLED` (and imports `m6d_gate`). The enabling commit then stays exactly the two lines above |
| B | The enabling commit also changes `test_m6d_isolated_success.py:104` (three lines) |

This note does not make either change.

### 1.2 Why no other code change is needed

- **The full live path is already in place behind the gate.**
  - `main()` → `_live_main()` → `_run_live()` → `LiveSession`, on `RclpyLiveTransport`;
  - the evidence file, both readiness checks, confirmation, pre-send record, send-time
    freshness, the single send, supervision, the one cancel and the final record.
- **Isolated testing.** That path has been exercised end to end on an isolated domain against an
  in-process fake controller stack, with the gate set `True` in the test only
  (`test_m6d_isolated_success.py`).
- **The gate is read in three places:**
  - `main()`, first in the `--live` branch;
  - `_live_main()` and `_run_live()`, as defensive re-checks.

  All three read the same constant. No other flag, option or environment variable exists, and
  static tests enforce that.
- **Unchanged by the flip:**
  - the trajectory, the limits and tolerances, the timeouts, the confirmation word;
  - the goal fingerprint and trajectory ID: identity is pinned under both gate states
    (`test_identity_is_pinned_under_both_gate_states`).

### 1.3 Expected side effects (all known, none behavioural for dry-run/mock)

- `--help` and the banner change from `HARD-DISABLED in this build` to
  `ENABLED for exactly one goal`.
- `--live` stops returning exit 3. It then requires `--domain-id`, an output path and the typed
  word.
- The dry-run and mock reports record `live_dispatch_enabled: true`, so their SHA-256 values
  change from those in the design note §8. Only the `live_dispatch_enabled` fields differ, and the
  identity is unchanged (`test_gate_fields_change_report_hashes_but_not_identity`).

### 1.4 Evidence that the enabling change is safe to merge

1. PR #16 is merged into `main`, and the enabling branch is created from that merge commit.
2. Local verification on the owner's PC passed at the PR #16 merge commit, with the gate `False`.
   It covers:
   - a clean build and the full suite (cloud reference: 1213 tests, 0 failures, 0 skipped);
   - the eight M6.0-D suites (273), including both isolated suites;
   - dry-run/mock determinism against the design note §8 hashes;
   - the disabled `--live` refusal;
   - the static validators;
   - clean `pgrep` and `git status`.
3. On the enabling branch, with the flip applied, the same build and full suite pass (expected:
   the same totals, 0 failures). The new dry-run/mock hashes are recorded, with identity
   unchanged. This run happens only after the owner's approval of §5.
4. The D15 controller audit (§2, steps 3–7) is recorded and passed.
5. The owner has reviewed this checklist (§2) and gives explicit written approval for **one** run.

### 1.5 Commit message

```text
feat(m6d): enable the one owner-approved live M6.0-D goal

Flip LIVE_DISPATCH_ENABLED to True (m6_live_contract.py:18) and the matching
test expectation (test/m6d_gate.py:8). No other change: the trajectory,
limits, tolerances, confirmation word, evidence handling and every other gate
are unchanged.

Owner approval: <reference and date>
Base: main @ <PR #16 merge SHA>
Local verification: <date, SHA, test totals>
D15 controller audit: <record location>
Scope: exactly one goal on the owner's Ubuntu PC in Gazebo GUI mode, per
docs/M6D_GATE_ENABLING_PLAN.md section 2. Re-disabled after the run (section 1.6).
```

### 1.6 Why this is a separate commit and PR, and why it is temporary

- **Reviewability.** A two-line diff is reviewed for exactly one thing: the decision to enable.
  PR #16 can be merged safe by default, with the gate `False`.
- **Traceability.** The approval, the audit records and the run are tied to one SHA.
- **Reversibility.** One revert restores the disabled state.
- **Temporary by design.** The approval covers one goal. The single-goal guarantee is per
  session and process, not durable across processes (design note §1). So the gate must not stay
  `True` after the run. Recommended sequence:
  1. Create the enabling branch from `main` and make the enabling commit.
  2. Owner review and approval.
  3. Do the run on the owner's PC from that **exact SHA**.
  4. Make an immediate re-disabling commit (`True` → `False` in the same two lines).
  5. Write the docs-only results commit.

  Whether the enabling commit is ever merged into `main`, or kept only on its branch, is the
  owner's decision. Either way `main` must end with the gate `False`.

## 2. Future live-run checklist (owner's Ubuntu PC only; NOT to be run now)

**Conditions for the run:**
- **Where:** the owner's Ubuntu 22.04 PC; ROS 2 Humble; Gazebo Fortress in **GUI** mode; two
  terminals, A and B.
- **When:** only after every criterion of §5 is true.
- **Never:** in the cloud, headless, or with hardware.

**Notation:**
- `<RUN>` is a UTC stamp such as `20261004T093000Z`, chosen once and used throughout.
- `<SHA>` is the approved enabling commit.
- `<N>` is the ROS domain of terminal A: `echo ${ROS_DOMAIN_ID:-0}` there.

### Pre-run (terminal B)

1. Check the workspace is clean and on the approved commit with the gate enabled:
   ```bash
   cd ~/spiderx_ws && git status --short && git rev-parse HEAD     # empty; equals <SHA>
   grep -n '^LIVE_DISPATCH_ENABLED' src/spiderx_controller/spiderx_controller/m6_live_contract.py
   #   expect: 18:LIVE_DISPATCH_ENABLED = True  (stop if anything else)
   ```
2. Build and source the workspace, and keep the build log:
   ```bash
   mkdir -p log/m6d_playback/<RUN>
   colcon build --symlink-install 2>&1 | tee log/m6d_playback/<RUN>/build.log
   source install/setup.bash
   ```
3. Check that nothing is left over from earlier runs:
   ```bash
   pgrep -af 'ign gazebo|gz sim|parameter_bridge|robot_state_publisher|controller_manager/spawner|ros2 launch spiderx_bringup' || echo "no leftovers"
   ros2 daemon stop
   ```
   Both terminals must use the same `ROS_DOMAIN_ID` and `ROS_LOCALHOST_ONLY` settings.

### D15 controller audit (read-only; terminal B; save all output)

Save all output to `log/m6d_playback/<RUN>/d15_audit.txt`.

4. Record the installed versions:
   ```bash
   for p in joint_trajectory_controller controller_manager control_msgs; do
     echo "$p $(ros2 pkg xml $p | grep -o '<version>[^<]*')"; done
   ```
   Expected: the `2.54.x` controller stack and `control_msgs` `4.9.0`, as recorded by M6.0-B.
   Classify every difference from the cloud reference (2.48.0).
5. Check the tolerance semantics. The headers are nested one directory deeper than the earlier
   checklist assumed, so locate them rather than assuming the path:
   ```bash
   I="$(ros2 pkg prefix joint_trajectory_controller)/include"
   T=$(find "$I" -name tolerances.hpp | head -1)
   H=$(find "$I" -name joint_trajectory_controller_parameters.hpp | head -1)
   echo "$T"; grep -n 'resolve_tolerance_source\|Specified illegal\|goal_time_tolerance =' "$T"
   echo "$H"; grep -n 'stopped_velocity_tolerance = ' "$H"
   ```
   In the cloud 2.48.0 headers, `resolve_tolerance_source` is at `tolerances.hpp:130`. It is
   applied to the goal's state tolerances (lines 209/212/215) and goal-state tolerances (lines
   258/261), and to `goal_time_tolerance` (line 176). The default
   `stopped_velocity_tolerance = 0.01` is at `joint_trajectory_controller_parameters.hpp:97`.

   The installed 2.54.x headers must show the same semantics:
   - a goal tolerance > 0 is taken from the goal;
   - an illegal value falls back to defaults, with a warning;
   - the default `stopped_velocity_tolerance` is 0.01.

   Read `resolve_tolerance_source` itself, not only these lines. **Stop** if any of these
   differs.
6. Check the controller log strings and the hold evidence:
   ```bash
   strings "$(ros2 pkg prefix joint_trajectory_controller)/lib/libjoint_trajectory_controller.so" \
     | grep -E 'Received new action goal|Accepted new action goal|Goal reached, success!|Got request to cancel goal|Canceling active action goal|holding position|Aborted due to goal_time_tolerance'
   ```
   The cloud 2.48.0 build contains all seven. **Hold is evidenced by a string only for the
   goal-time abort** (`Exceeded goal_time_tolerance: holding position...`). No string evidences
   hold after a cancel; that remains *intended* behaviour (D15). A nominal successful run will
   not exercise it. Record this limitation, and stop if any of the seven strings is missing.
7. Record a verdict: `PASS`, or `STOP` with the reason. On `STOP`, do not continue.

### Offline checks (terminal B)

8. Confirm the trajectory and goal fingerprint:
   ```bash
   ros2 run spiderx_controller m6_offline_preflight --check-only     # PASS, trajectory 44f0a7ad52e5c330
   ros2 run spiderx_controller m6_live_playback --dry-run --no-write
   #   expect: goal fingerprint 0d6ef4171f2d338a01e76b934be94d1a4386e7c0fc68fe74262fb3c65005ff83
   ```
   Stop on any difference.

### Terminal A: the approved launch, unmodified

9. Start the launch:
   ```bash
   cd ~/spiderx_ws && source install/setup.bash
   ros2 launch spiderx_bringup fortress_control.launch.py 2>&1 | tee log/m6d_playback/<RUN>/launch.log
   ```

### Terminal B: readiness and zero-command evidence

10. Wait until both controllers are `active` (at most 240 s), then wait 3 s more:
    ```bash
    ros2 control list_controllers      # joint_state_broadcaster and leg_trajectory_controller: active
    sleep 3
    ```
11. Run the read-only preflight:
    ```bash
    timeout 60 ros2 run spiderx_controller m6_live_preflight --timeout 10 --window 2 \
      | tee log/m6d_playback/<RUN>/preflight.txt
    ```
    - It must report `READY`. A version `warning` is acceptable; `incompatible` or `NOT READY`
      means **stop**.
    - Keep the report path it prints.
12. Record the T0 zero-command evidence (M6.0-B plan §9 step 5) into
    `log/m6d_playback/<RUN>/t0.txt`:
    - `ros2 topic info -v /leg_trajectory_controller/joint_trajectory`: 0 publishers;
    - `ros2 action info /leg_trajectory_controller/follow_joint_trajectory`: 1 server, 0 clients;
    - the transient-local action status: empty;
    - `grep -c 'Received new action goal' log/m6d_playback/<RUN>/launch.log`: 0;
    - `ros2 node list`.

### Terminal B: the one goal

13. Run the live tool once:
    ```bash
    ros2 run spiderx_controller m6_live_playback --live --domain-id <N> --out log/m6d_playback
    ```
    - Readiness #1 runs in the same process. The tool refuses unless the result is fresh, ready,
      and `compatible` or `warning`.
    - Only then does the prompt appear. Type `SEND-ONE-CROUCH-GOAL` **once**, exactly, then
      Enter. Anything else, EOF or Ctrl+C refuses, and nothing is sent.
    - Readiness #2, the pre-send record, the send-time freshness check and **one** goal follow.
    - Watch Gazebo and the terminal. Nominally the motion takes 9 s (neutral → `crouch_10mm` →
      neutral) and the tool exits 0. The wall cap is 120 s (watchdog) + 30 s.
14. **Emergency.** Press **one** Ctrl+C in terminal B.
    - That sends one cancel request. The controller is then *intended* to hold; this run is the
      first chance to observe it, and the observation must be recorded.
    - A second Ctrl+C only stops waiting (`CANCEL_UNCONFIRMED`).
    - Never send any other command.
15. **If the tool prints that the final goal status is UNKNOWN,** the controller may still be
    executing the goal. Observe until motion stops and record it. Do **not** re-run the tool.

### Record (terminal B)

16. Record the outcome:
    - the exit code;
    - the report `log/m6d_playback/live/<UTC>/live_outcome.json`, with:
      - `evidence.phase` (`final` expected);
      - `state` and `reason`;
      - `dispatch`: `goal_id`, `acceptance`, `final_result_known`,
        `goal_may_still_be_executing`;
      - `goals_sent` (must be 1, or 0 if refused);
      - `cancels_sent`;
      - `readiness[*].classification`;
      - `freshness_at_send.age_s`;
      - `tracking.max_inflight_error_rad`.
17. Record the T2 evidence into `log/m6d_playback/<RUN>/t2.txt`:
    - the action status shows **exactly one goal ID**, equal to `dispatch.goal_id` (32 hex
      characters; the status shows it as 16 bytes);
    - `grep -c` on the launch log: `Received new action goal` = 1 and `Accepted new action goal`
      = 1; `Got request to cancel goal` = 0 unless step 14 happened;
    - 0 publishers on `/leg_trajectory_controller/joint_trajectory`.

### Shutdown

18. Terminal A: **one** Ctrl+C. Wait up to 20 s.
19. Terminal B: confirm the processes have exited and the tree is unchanged:
    ```bash
    ros2 daemon stop
    pgrep -af 'ign gazebo|gz sim|parameter_bridge|robot_state_publisher|controller_manager/spawner|ros2 launch spiderx_bringup' || echo "no leftovers"
    pgrep -af _ros2_daemon || echo "no ros2 daemon"
    git status --short       # nothing tracked changed; evidence stays under the ignored log/
    ```
    Save this output to `log/m6d_playback/<RUN>/after.txt`.
20. Re-disable the gate (§1.6) and write the results to `docs/M6D_LIVE_PLAYBACK_RESULTS.md` as a
    docs-only commit.

**Never, at any step:**
- re-run the tool after a goal may have been sent;
- retry after a rejection, abort, timeout, tracking failure or refusal at the send;
- send a second goal, a return-to-neutral or any other command;
- run in the cloud, headless, or with hardware.

A failure is recorded as it happened. Any further attempt needs a new owner approval.

## 3. Evidence requirements

| # | Evidence | Where | Proves |
|---|---|---|---|
| E1 | `git rev-parse HEAD` (= `<SHA>`), empty `git status`, gate line 18 = `True`, `build.log` | step 1–2 output, `log/m6d_playback/<RUN>/build.log` | The approved code was run, built clean |
| E2 | D15 audit: versions, `tolerances.hpp` lines, library strings, verdict | `d15_audit.txt` | The controller semantics match the tolerances the goal relies on |
| E3 | Offline preflight PASS, dry-run fingerprint | step 8 output | Identity unchanged |
| E4 | `m6_live_preflight` report (READY, classification) | `preflight.txt` and the JSON path it prints | The stack was ready before the run |
| E5 | Launch-log snippet: both controllers configured and activated; `list_controllers` output | `launch.log`, step 10 output | The approved launch was in the expected state |
| E6 | T0 zero-command evidence | `t0.txt` | Nothing else was commanding the controller |
| E7 | `live_outcome.json` (phase `final`), with goal ID, state, reason, send/cancel counts, readiness, freshness at send and tracking; plus the exit code | `log/m6d_playback/live/<UTC>/` | What the one run did, and that it sent at most one goal |
| E8 | T2 evidence: one goal ID equal to the report's, log-string counts, 0 command publishers | `t2.txt` | Exactly one goal reached the controller |
| E9 | Hold observation if a cancel happened (Gazebo/`/joint_states` notes) | operator notes | Whether the intended hold was seen; otherwise "not exercised" |
| E10 | After-run `pgrep`, daemon and `git status` | `after.txt` | A clean shutdown, with no tracked change |
| E11 | Re-disabling commit, and the docs-only results commit summarizing E1–E10 | git | The approval was used once and closed |

## 4. Risk register (top 5)

| # | Risk | Effect | Mitigation already in place | Remaining action |
|---|---|---|---|---|
| R1 | **Controller tolerance mismatch.** Installed 2.54.x semantics differ from the cloud 2.48.0, or an unknown name or illegal value makes the controller silently use defaults | Goal accepted with weaker or stronger limits than intended | All 12 joint names and positive tolerances are in the fingerprinted goal; goal velocity tolerance is explicit (D16); the client independently cancels above 0.05 rad tracking error (D8) | D15 audit steps 4–7; stop on any difference |
| R2 | **Hold differs from expectation** after a cancel or abort | The robot keeps moving or drifts after the cancel | One cancel only; the tool records whether the final status is known and warns when the goal may still be executing | Hold after a cancel is not evidenced by any string or test. Observe and record (E9); one Ctrl+C in terminal A ends the simulation if needed |
| R3 | **Network/domain conflict.** Wrong `--domain-id`, mismatched `ROS_LOCALHOST_ONLY`, or a stray node or publisher | The tool talks to the wrong graph, or sees two `/joint_states` publishers | Explicit `--domain-id`; readiness refuses without exactly one action server, both controllers active, one `/joint_states` publisher and a neutral start pose; a second publisher during the run → one cancel | Step 3 leftover check, matched env in both terminals, T0 node list |
| R4 | **Report write failure** | The outcome is lost | Exclusive reservation and a `NOT_DISPATCHED` record before ROS; a `pre_send` record before the send (refuse if unsaved); on a final-write failure, stderr fallback and exit 1, never claimed as saved | Run on a disk with free space; keep the terminal B scrollback; the last saved phase tells the truth (`pre_send` = status unknown) |
| R5 | **Operator error** (wrong word, re-run, extra command, wrong commit) | A second goal, or a run of unapproved code | Exact word, case-sensitive; the prompt appears only after readiness #1; an interrupt wins; a session/transport sends one goal; no retry path; gate check in step 1 | The single-goal limit is per process: **never re-run** (checklist rule); re-disable immediately after the run (§1.6) |

## 5. Decision gate (all must be true before the enabling commit is made)

1. **PR #16 is merged** into `main`, with the gate `False`, after owner review.
2. **Option A or B of §1.1** is decided. With A, its one-line test fix is merged.
3. **Local verification passed** on the owner's PC at the PR #16 merge commit, with the gate
   `False` (§1.4 item 2), and is recorded.
4. **The D15 controller audit passed** (§2 steps 4–7) and is recorded. This can run before any
   enabling; it is read-only and starts no node.
5. **The checklist (§2), evidence list (§3) and risk register (§4) have been reviewed and
   understood by the operator.** That includes:
   - hold after a cancel is intended, not proven;
   - the tool must never be re-run;
   - the gate is re-disabled right after the run.
6. **The owner explicitly approves enabling, in writing, for exactly one run.** The approval names
   the base SHA and the run window.

Until all six are true: no enabling commit, no gate change, no live goal, no Gazebo run for
M6.0-D.
