# S4d-E: the level-2 emergency stop — frozen contract

**FROZEN 2026-11-11, before any S4d-E test was run.** Branch `claude/spiderx-s4d-e-preparation`, from `claude/spiderx-s4d-preparation` at `0257980` (the finalized S4d-M record).

**Authorized (owner, 2026-11-11):**
- the offline S4d-E tooling (driver, scorer and tests) and the narrow `local/m55-s4d-e-*` restore support;
- one fresh disabled build and full suite;
- **ONE S4d-E SHADOW rehearsal**, only if every prerequisite passes.

**Not authorized:** a live S4d-E attempt, an enabling patch or branch, any real-controller goal or cancel, an S4d-M rerun, hardware work or a merge. Section 2 describes a live experiment that is **NOT AUTHORIZED TO RUN**.

This contract changes no runtime code, gait, plant, gain, phase library, historical driver or scorer, and no S4d-M criterion. It realizes the S4d-E outline of `S4D_ACCEPTANCE_PLAN.md` §7.1 (OD10).

## 1. Facts from the code (MEASURED by reading; the line numbers are at `0257980`)
- **`x`** (`m55_teleop.py` 33) calls the node's `estop` service.
  - `_estop` (`m55_locomotion.py` 872) calls `_fault(ESTOP)` (666), in the request handler.
  - In that one call it logs the `fault` event, makes the `transition` to FAULTED (why `estop`) and clears the lease.
  - Then, only for an unresolved goal whose cancel was not yet sent, it sends **one** cancel (`cancel_sent`, never retried).
  - The handler then logs the `request` event (`estop`, accepted).
- **Request time.** The service callback takes `time.monotonic()` once, after a graph snapshot (`m55_rclpy.py` 232–234), and passes it to the handler. All of these events carry that one node time.
- **In FAULTED** (811–870):
  - `disarm` and `arm` are refused with `faulted_reset_required`;
  - `stop` is accepted with "not moving (FAULTED); held until a zero command";
  - no goal is dispatched;
  - only `reset`, then `arm`, leave FAULTED, and this contract never uses them.
- **The cancelled goal's result** (1128): CANCELED resolves it. SUCCEEDED means it "completed despite the cancel": the node then moves its rest boundary. A cancel request alone does not prove that execution stopped.
- **Shadow.** `ShadowTransport.cancel` (528) models a cancellation: `cancel_response` 0, then result CANCELED, "shadow cancel". Nothing reaches the controller.
- **`q`** (`m55_teleop.py` 213–247) sends `stop`, then `disarm`; the shutdown sequence then sends `stop` and `disarm` again. In FAULTED this gives two accepted stops and **exactly two** disarm refusals, `faulted_reset_required`. These are the documented close refusals; no other refusal is expected.
- **Feedback.** The node attributes it to its goal through the goal handle (`m55_rclpy.py` 92, 128). The node's status exposes only the maximum error, which starts at 0.0, so that field cannot prove that feedback exists. The driver therefore subscribes to the action's feedback topic itself (§3).
- **The goal.** Level 2, 0→1: the S1 four-foot **shift**, 1.9 s of sim. Its fingerprint is in the frozen library (`62f8a100…`; `library.json` `917fc4d7…`).

## 2. LIVE experiment design — NOT AUTHORIZED TO RUN
One fresh launch. Budget: **exactly one planned goal and one planned cancel.** An emergency response always overrides the budget and ends the attempt. There is no retry, recovery, second goal or second launch.
1. **Settle** before any key, as in S3: at least 10 s of sim, z ≥ 0.045 m, z range ≤ 5 mm, tilt ≤ 0.10 rad, fresh samples.
2. **Level.** `+ +` while DISARMED, with no goal and before arm. The teleop confirms `2/3`, then `3/3` (2.34 mm/s); each confirmation is attributed to its own write.
3. **Arm.** `a`: READY at boundary 0 (the node is at level 0 before goal 1).
4. **The goal.** Hold `w`. Goal 1 is level 2, 0→1, the shift. One `level_change` to 2 happens in READY before goal 1.
5. **`x`**, only when all of these hold:
   - at least one **attributed real feedback sample** of goal 1 has been received by the driver;
   - goal 1 is still active: the node reports goal 1, accepted, with no result;
   - at most **0.95 s of sim** (half the phase) has passed since the driver first saw goal 1 accepted.

   The repeat of `w` is released immediately after `x`. If any condition fails, the attempt ends through the driver's stop handling: the repeat stops and `x` is sent as the emergency response. No second goal is ever sent to rescue the attempt. The estop criteria are then UNMEASURED or FAIL, never PASS.
6. **Expected.** FAULTED (`estop`) in the request handler, with exactly one cancel, for goal 1, and no further goal. The result should be **CANCELED**. SUCCEEDED despite the cancel is recorded, and the cancellation criterion is then UNMEASURED.
7. **Hold**, measured over at least **10 s of sim** after the result:
   - the node stays FAULTED with no goal;
   - no feedback arrives for goal 1 after its result;
   - joint motion ≤ 0.01 rad, tilt ≤ 0.10 rad and z ≥ 0.045 m (the A6 limits);
   - the controller's reference (`controller_state` desired) changes by at most 0.01 rad.
8. **Close.** `q` after the hold. The node must record exactly stop ×2 (accepted) and disarm ×2 (refused, `faulted_reset_required`). Then:
   - the state stays FAULTED to the end;
   - no reset, re-arm, home or recovery happens: no `r`, no `a` after `x`, no `h`;
   - the teleop exits 0 within 15 s of wall.

**Safety limits.** These are the S3/S4a/S4c driver stops, unchanged:
- tilt > 0.10 rad in 2 consecutive fresh samples;
- z < 0.045 m in 2 consecutive fresh samples;
- **travel > 0.15 m from the arm pose**;
- a joint within 0.02 rad of its URDF limit;
- tracking > 0.05 rad (also the node's own abort);
- any command rejection;
- an unplanned FAULTED;
- any goal beyond the plan: a seq above 1, or goal 1 not level 2 0→1;
- each wait's cap (sim/0.1 + 60 s wall), and a **30-minute** script wall cap.

The one-goal E experiment does **not** get S4d-M's 0.40 m travel allowance (OD3).

## 3. Clocks and timing evidence
- **Node clock.** The node's `time.monotonic()`: the `wall` field of `node/events.jsonl`.
  - **R4: request handling to FAULTED** = wall(transition to FAULTED, why `estop`) − wall(the `estop` request). It must satisfy **0 ≤ Δ ≤ 0.050 s**, boundaries inclusive. With this code both carry the handler's one time, so Δ = 0.
  - The cancel must be logged in the same handling: wall(`cancel_sent`) − wall(request) ≤ 0.050 s.
  - Either event missing makes the check UNMEASURED.
- **Driver clock.** The driver's `time.monotonic()`, on the same host. Each scripted key is bracketed by `t_before_write` and `t_after_write`.
- **Key-write to request latency** = wall(`estop` request) − `t_after_write` of `x`. It is **recorded** separately and is not a pass criterion.
- **A status receipt time is never used as a transition time.**
- **Sim time** comes from the joint-state stamps, and the bag is aligned to the driver clock through the status pairs (as in S2–S4d). The x deadline and the hold window are in sim time.
- **Feedback attribution.** A feedback sample is attributed to goal 1 only if both hold:
  - every feedback sample of the run carries one and the same goal id;
  - it is received after the write of `w` and within goal 1's window: from the node's send to its result, mapped through the alignment, ± its uncertainty.

  Any other feedback is foreign: the driver stops, and the scorer marks the result UNMEASURED or FAIL.

## 4. Scoring (`scripts/s4/score_s4d_e.py`; frozen with this contract)
Every check is PASS, FAIL or UNMEASURED; missing evidence is UNMEASURED, never PASS. The overall result is FAIL if any check fails, otherwise INCONCLUSIVE if any is UNMEASURED, otherwise PASS.
- **E1 Plan and budget.**
  - Exactly one goal: seq 1, level 2, 0→1, the shift.
  - Exactly one cancel, for seq 1.
  - The keys are exactly `+ + a w x q`, plus the repeats of the held `w`.
  - The requests before `q` are exactly arm, then estop.
  - No fault but the one estop; no refusal before `q`.
  - Every status is in the mode required: live, DISPATCH with the gate `True`; shadow, SHADOW with the gate `False`.
  - The driver's result is "script complete".
- **E2 Level.** Two `+` while DISARMED before arm, confirmed and attributed `2/3`, then `3/3`. Exactly one `level_change` to 2, in READY, after arm and before goal 1. Goal 1 is at level 2.
- **E3 `x` timing and precondition.**
  - Exactly one estop-capable key, the scripted `x`, written successfully, with t_sent(goal 1) ≤ t_before ≤ t_after.
  - The node's `estop` request is after goal 1's acceptance and before its result.
  - The driver recorded the sim elapsed at `x` as ≤ 0.95 s.
  - **Live:** at least one attributed feedback sample was received by the driver before t_before, with goal 1's id. None means **UNMEASURED**.
  - **Shadow:** explicitly not applicable, because no real feedback exists.
- **E4 Fault response.**
  - Exactly one `estop` fault; the transition to FAULTED (why `estop`) within 0.050 s of the request (§3).
  - Exactly one cancel, for seq 1, in the same handling.
  - One teleop "[estop] ok" response, after the fault, with the node's message.
  - No estop-service fallback.
  - The latency (§3) is recorded.
- **E5 Cancellation outcome.**
  - The cancel response code is 0.
  - The result is CANCELED: SUCCEEDED makes this UNMEASURED; any other status, FAIL; no result, UNMEASURED.
  - **Live** also requires:
    - no goal-1 feedback after the result;
    - the controller log has exactly one cancel request after the acceptance, and no further goal;
    - the controller reference changes by at most 0.01 rad over the hold.
  - **Shadow:** the modeled cancellation ("shadow cancel"); the controller traffic is checked in E9.
- **E6 Hold.** Measured from the result over 10 s of sim, from the bag's joint states and poses:
  - every status in the window is FAULTED with no goal;
  - joint motion ≤ 0.01 rad, tilt ≤ 0.10 rad, z ≥ 0.045 m.

  A coverage gap makes it UNMEASURED.
- **E7 No further motion command.**
  - One goal in total, and none after the fault.
  - After `x`, no request but the close: no reset, arm or home.
  - The final state is FAULTED.
- **E8 Tracking.**
  - **Live:**
    - goal 1 needs at least one attributed sample before the cancel; none is UNMEASURED;
    - \|error\| ≤ 0.05 rad, otherwise FAIL;
    - above 0.02 rad is a finding;
    - unattributed feedback ids make it UNMEASURED.
  - **Shadow:** N/A, explicitly ("not applicable: dispatch disabled (shadow rehearsal)"). There is no retrospective N/A in live scoring.
- **E9 Ownership and controller traffic.**
  - 0 foreign action clients and 0 command publishers in every status's graph monitor.
  - The before/after captures show 0 publishers.
  - `joint_trajectory` is absent, with recorder and coverage support.
  - **Live:** the controller log has 1 goal received, 1 accepted and 1 cancel request; all feedback carries one goal id.
  - **Shadow:** the controller log has 0 received, 0 accepted and 0 cancel requests; 0 feedback in the bag and in the driver's subscription; the controller reference changes by 0.
- **E10 Close.**
  - After `q` the requests are exactly stop ×2 and disarm ×2.
  - Both stops are accepted with the FAULTED message.
  - Both disarms are refused with `faulted_reset_required`, and these are the run's only refusals.
  - No transition after `q`.
  - The teleop exits 0 within 15 s of wall.
- **E11 Margins and safety.**
  - The driver completed.
  - Minimum joint margin ≥ 0.02 rad.
  - Node-observed ages within the monitor limits.
  - From the bag after arm: travel ≤ 0.15 m from the arm pose, and tilt ≤ 0.10 rad.
- **E12 Restore, library and run installation.** As S4d-M's M15:
  - the restore report reads DISABLED AND VERIFIED;
  - `library.json` equals the disabled one;
  - the run installation imports all three gates `False`.

## 5. SHADOW contract (the one authorized launch)
- **Gates.** All three gates `False`, from fresh imports in the run installation and in the driver. The node is SHADOW with the gate `False` in every status.
- **One planned shadow goal.**
  - Recorded by the ShadowTransport and sent nowhere.
  - `x` is written once it is in flight, within the same 0.95 s sim deadline. The driver **does not wait for feedback, which cannot exist in shadow.**
  - Its modeled cancellation follows.
- **Zero actual controller goals, cancels and feedback** (E9).
- **Tracking** is N/A only in this explicitly identified shadow scoring (`--shadow`). The body does not move.
- **Result label.** The best shadow outcome is labelled "PASS (SHADOW ONLY: not S4d-E acceptance)". A shadow run is **never** live cancel-and-hold validation.
- **Harness.** A new run-local harness from the tested shadow tools:
  - a watchdog and its deadline, armed before the launch;
  - time and disk checks before the launch and before recording;
  - owned-process cleanup;
  - a `--verify-only` restore, because there is no enabling;
  - checks of both installations;
  - the recorder stopped before the launch stop, and the server's exit observed by strace (`evidence/m55/s4d_live_20261111T052817Z_shutdown_postmortem_20261111T101720Z/`).

## 6. Restore support
`scripts/m61_restore_and_verify_disabled.sh` may delete a verified, never-pushed `local/m55-s4d-e-*` branch, under the same rules as every stage. Look-alikes stay refused. The shadow does not use it, because nothing is enabled.

## 7. Claim boundaries
- **The shadow** shows only the script, the harness, the scorer and the node's modeled cancel path.
- **A future live PASS** would show only this: in simulation, at internal level 2, `x` during the S1 shift faulted the node, cancelled the goal once, and the hold was measured.

Neither shows S4e or physical-keyboard operation, hardware, navigation, unrestricted walking or full S4 acceptance.
