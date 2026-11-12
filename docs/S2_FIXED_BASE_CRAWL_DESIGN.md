# S2: M5.5 crawl phases on the welded fixed base (design only)

Date: 2026-10-13, branch `claude/spiderx-m55-keyboard-walking`. **No goal has been sent.** The
acceptance criteria (§4) and the preparation (B1–B3, the A2 rehearsal) are owner-approved and done
at `ec7f122`; the run itself needs the authorization in §5. Sources:
[M5.5 §10 phase 2](M55_KEYBOARD_WALKING.md), [path to simulated walking](PATH_TO_SIM_WALKING.md),
[M6.1 one-cycle plan](M61_CLOUD_ONE_CYCLE_PLAN.md).

**Purpose.** Play all 54 validated M5.5 phase goals (9 phases × 3 levels × 2 directions) on the
real `leg_trajectory_controller`, with the body welded in the air (no contact), and show that
each succeeds, is tracked and is continuous. Claim if it passes: *all M5.5 crawl phase goals
executed and tracked on a welded body*. Not claimed: walking, contact, balance, hardware.

## 1. Setup and reuse from M6.1-A

| Item | S2 use | Applies? |
|---|---|---|
| `fortress_m61a_fixed_base.launch.py` (weld at 0.125 m, identity spawn, ground-truth bridge) | the plant, unchanged | **yes** |
| M6.1-A frozen harness (`env.sh`: domain 0, localhost only, Xvfb `:77`; `stage_stop.sh`) | environment and shutdown | **yes** |
| `m61a_observe_fixed_base` read-only preflight (attachment, spawn identity, pose freshness, sim clock advancing) | before arming | **yes** |
| M6.1 E8 weld rule (body within 1 mm / 0.0033 rad of the weld, z 0.125 m ± 1 mm, physics state) | during motion; the node composes the same model · `dummy_link` pose | **yes** |
| E10 zero foreign commanders; `joint_trajectory` topic empty | plus the node's own graph check | **yes** |
| `scripts/m61_restore_and_verify_disabled.sh` | extended at `ec7f122`: all three imported gates `False`, both M6 `--live` exit 3, M5.5 node/teleop leftovers, `local/m55-s2-*` deletion | **yes** (B2 done) |
| `m61_readiness_rehearsal`, tracking-path probe, criterion 6 (scan), E3 trot identity, E9 link check | M6.1 session or goal specific | **no**. Replaced by the shadow rehearsal (§2 A2) and the library fingerprint (P7). E9 is optional |

The M5.5 node is started separately: `ros2 run spiderx_controller m55_locomotion_node
--evidence-dir <run>/node`. The §3 script is run by the status-driven driver `s2_driver.py`,
frozen with the run script in the A2 rehearsal's `tools/`. It drives the teleop through a
pseudo-terminal and follows `/spiderx/locomotion/status`. Every goal is built and sent by the
node; Claude sends only key presses to the teleop (→ `/cmd_vel` at 10 Hz and the node's
services).

## 2. Procedure

- **A. Disabled base.** Clean tree at the approved base; clean build; all three gates `False`
  (source and imported); `--live` exit 3 for both M6 tools; no leftover processes.
- **A2. Shadow rehearsal on the fixed base** (disabled build, nothing reaches the controller;
  done, see §8):
  - launch, then preflight READY;
  - node in SHADOW; the full §3 script, all levels;
  - expect 0 rate rejections, 0 command-topic messages, the scripted goals planned (58 in
    shadow: after the lease the measured posture is still neutral, so there is no home goal),
    joints unmoved (< 0.005 rad), no action client in the node.
  - This also serves as the S0b re-run. Then stop, and verify the disabled state.
- **B. Enabling branch** `local/m55-s2-<UTC>`, never pushed. It applies
  `docs/patches/m55_enable_s2.UNAPPLIED.patch` (exactly 2 files, 2 lines; SHA-256 in §5):
  `M55_LOCOMOTION_DISPATCH_ENABLED = True` and the pin in `test/m55_gate.py`. The M6.1 and M6
  gates stay `False`.
- **C. Enabled build, no simulation.** Every M5.5 test passes: since `ec7f122` the gate tests are
  mode-explicit (as for M6.1). The library fingerprint equals the disabled build's.
- **D–E. Launch, preflight, node, then the §3 script, once.** Use the A2 harness, frozen in the
  rehearsal's `tools/`:
  - Gazebo GUI at nice 19, node at nice −5;
  - no `/clock` or `_action/status` in the bag (§6);
  - abort if the launch or the preflight fails. A relaunch is a new attempt and needs no goal.
  - Record the bag.
- **F. Stop, then restore and verify.** This runs whatever happened:
  `scripts/m61_restore_and_verify_disabled.sh --base ec7f122 --evidence <run>/restore
  --enabling-ref local/m55-s2-<UTC> --discard-uncommitted-src --delete-enabling-branch`.

## 3. What Claude sends, in this order (59 goals, 1 cancel)

Each run starts READY at cycle boundary 0. The driver holds the key and presses `space` while
the cycle's 9th phase is in flight (seen on the status), then releases. The node stops at the
cycle boundary: there is no hold-window race. One goal is one phase (lead-in 0.2 s).

| # | Keys / requests | Level | Goals sent by the node | Sim s |
|---|---|---|---|---|
| 1 | `a`, hold `w` (0.00106 m/s) | 0 | 9 forward: S1 W(LR) S2 W(LF) S3 W(RR) S4 W(RF) S5 | 18.8 |
| 2 | hold `s` | 0 | 9 reverse (the same phases in reverse time, last first) | 18.8 |
| 3–4 | `+`, hold `w`; then hold `s` | 1 | 9 + 9 | 21.1 + 21.1 |
| 5–6 | `+`, hold `w`; then hold `s` | 2 | 9 + 9 | 25.6 + 25.6 |
| 7 | `-` `-`, hold `w`, `space` during goal 1 | 0 | 1 (b0→b1), STOPPING → READY; then `h`: 1 back to b0 | 3.4 |
| 8 | hold `w`; kill the teleop (SIGKILL, no clean quit) during goal 1 | 0 | 1; STOPPING → DISARMED (lease); new teleop, `a` (re-arm at its rest, boundary 1), `h`: 1 back | 3.4 |
| 9 | hold `w`, `x` during goal 1 | 0 | 1 goal, **1 cancel**: FAULTED (last, because the posture is mid-phase and cannot re-arm) | ≤ 1.7 |

Totals: 54 cycle goals + 5 exercise goals = **59 goals and 1 cancel**, ≈ 140 s sim.

## 4. Acceptance (M5.5 §10 phase 2) and stop conditions

| # | Criterion | Evidence |
|---|---|---|
| P1 | Runs 1–6: every goal `SUCCEEDED` (status 4), no fault; each run ends READY at boundary 0 | `events.jsonl` |
| P2 | Tracking: max feedback \|error\| ≤ 0.02 rad on every goal (node abort 0.05 rad) | bag `_action/feedback` → `tracking.csv` |
| P3 | Continuity: every `goal_sent.continuity_error_rad` ≤ 0.005 rad (measured joints) | `events.jsonl` |
| P4 | Ownership: goals sent = goals scripted = 59; cancels = 1; no foreign client or publisher; 0 `joint_trajectory` messages | node status, bag |
| P5 | Stop: READY by check 1.7 (sim bound). Lease: DISARMED by check 1.9 (sim bound). Estop: FAULTED with exactly 1 cancel | `events.jsonl`, status |
| P6 | Welded throughout: E8 rule from the physics state | status `ground_truth`, observer |
| P7 | Phase-library fingerprint equal to the disabled build's (54 goal fingerprints) | `library.json` SHA-256 |
| P8 | Restore ends DISABLED AND VERIFIED: all three gates `False`, imported, from this workspace | restore report |

**Stop the session on any of these.** No retry and no fix during the run; then go to F.
- An unplanned fault, a result other than `SUCCEEDED`, or a refused `arm` or dispatch.
- A goal whose tracking error exceeds 0.02 rad: finish that goal, `space`, then stop.
- A foreign commander, or a goal count off the script.
- Sim stall > 5 s, or RTF < 0.1 (the node faults itself), or a run exceeding its sim
  duration / 0.1 + 60 s in wall time.
- A rate rejection, or a teleop or driver error.
- The owner says stop.

A safe cancel path (the node faults and cancels once) is a FAIL of acceptance, not a hazard.

## 5. Gates and the exact owner authorization text

Blockers:
- **B1.** S0b pushed: the windowed rate rule; checks 1.7 and 1.9 in sim time. **Done**
  (`e4bd880`; 1.9 in the docs commit after `ec7f122`).
- **B2.** The restore script extended to the M55 gate, with hermetic tests. **Done** (`ec7f122`;
  14 tests, 4 of 4 mutants caught).
- **B3.** The enabling patch written, not applied. **Done** (`ec7f122`; SHA-256 below).
- **A2.** The shadow rehearsal. **Done** (§8).
- **B4.** This authorization, verbatim:

> I authorize ONE S2 fixed-base crawl session in Cloud on the welded body
> (`fortress_m61a_fixed_base.launch.py` with the frozen M6.1-A harness, the Gazebo GUI at nice 19),
> from base commit `ec7f122` (its `src/`), with the enabling patch
> `docs/patches/m55_enable_s2.UNAPPLIED.patch` (SHA-256
> `d1dacac5d9b2bc0b63220c3583526d45f5edacb657394b508710978972b20236`) applied on a local,
> never-pushed branch `local/m55-s2-<UTC>`. That branch sets only
> `M55_LOCOMOTION_DISPATCH_ENABLED` and its pin to True. The M6.1 and M6 gates stay False.
>
> The M5.5 node may send at most 59 phase goals and 1 cancel, in the order of
> S2_FIXED_BASE_CRAWL_DESIGN.md §3, driven by the frozen `s2_driver.py`:
> - levels 0, 1 and 2, each one cycle forward and one reverse;
> - then stop + home, lease + re-arm + home, and the emergency stop.
>
> Stop at once on any of these, with no retry and no fix during the run:
> - an unplanned fault or refusal, or a result other than SUCCEEDED;
> - a tracking error above 0.02 rad;
> - a foreign commander, or a goal count off the script;
> - a rate rejection;
> - a sim stall over 5 s, RTF below 0.1, or a wall cap exceeded;
> - a teleop or driver error;
> - my instruction.
>
> No free-base launch. Restore and verify the disabled state afterwards, whatever happens.

## 6. Evidence to keep (minimal)

`docs/evidence/m55/s2_fixed_base_<UTC>/` holds:
- `README.md` (P1–P8, one line each) and `summary.json`;
- the node's `events.jsonl`, `library.json` and `final_status.json`;
- `tracking.csv` (per goal: max error, duration, result) and the teleop driver log;
- the preflight JSON, enabling commit id + patch + SHA-256, the restore report, `SHA256SUMS`.

The bag (`/spiderx/locomotion/status`, `/joint_states`, `/spiderx/sim/world_poses`, `/cmd_vel`,
`/clock`, the action **feedback** topic, `controller_state`, `joint_trajectory`) stays in
`~/spiderx_evidence/`. Only its `metadata.yaml` is committed. **Never record the action's
`_action/status` topic:** the node counts its subscribers as action clients and refuses `arm`
(`command_owner_conflict`). Rehearsal attempt 1 did that and was refused, as designed.

## 7. Wall time (Cloud)

On the welded fixed base, the A2 rehearsal ran at **RTF 0.59**. That is faster than the free base
(0.20), because there is no contact. It took 4.4 min for the whole §3 script (58 goals, shadow).

| Step | At RTF 0.59 (measured A2) | At RTF 0.20 (worst case) |
|---|---|---|
| A + C: clean build, full suite, enabled-build M5.5 tests | ≈ 18 min | ≈ 18 min |
| D: launch, preflight, node start-up (86 s) | ≈ 2.5 min | ≈ 4 min |
| E: the §3 script (≈ 140 s sim plus pauses) | ≈ 4.5 min | ≈ 13 min |
| F: stop, restore, clean rebuild, verify | ≈ 10 min | ≈ 10 min |
| **Total** | **≈ 35 min** | **≈ 45 min** |

## 8. A2 shadow rehearsal and the wall-time bounds (S2 preparation, `ec7f122`)

Evidence: [`evidence/m55/s2_prep_ec7f122/`](evidence/m55/s2_prep_ec7f122/README.md).

**Rehearsal.** Five attempts; the fifth passed. The first four found harness issues, not node
issues:
1. The bag subscribed to `_action/status` → `arm` refused, `command_owner_conflict` (§6).
2. The driver expected re-arm at boundary 1. In shadow nothing moved, so it is boundary 0.
3. A host CPU stall of 0.8 s in the node tripped the 0.5 s `/joint_states` monitor. The stream
   itself never paused (max 0.081 s at the recorder).
4. The known Gazebo startup race. The harness now stops when the controllers or the preflight
   fail.

Attempt 5 ran the full §3 script and the driver exited 0:
- 58 goals planned, 1 cancel, 0 rate rejections in 2537 commands;
- 0 `joint_trajectory` messages, 0 action feedback, no action client in the node;
- joints unmoved (1e-19 rad), welded z 0.125 m, tilt 0;
- 1.7: 1.76 s sim (≤ 2.7); 1.9: 1.73 s sim (≤ 2.7); kill → STOPPING 0.48 s wall;
- restore `--verify-only`: DISABLED AND VERIFIED.

**Wall-time bounds reviewed for RTF 0.20.**
- RTF alone cannot trip any of them at RTF ≥ 0.1. Two documentation checks could: 1.7 and 1.9.
  Both are now in sim time; the phase-4 rows follow.
- Host CPU starvation can trip the `/joint_states` 0.5 s and body-pose 1.0 s monitors.
  - These are safety thresholds, so they are kept.
  - The load is reduced instead: Gazebo GUI at nice 19 (≈ 150 % CPU of 4 cores), node at nice −5,
    no `/clock` in the bag.
  - Residual risk: a host stall over 0.5 s faults the run safely (one cancel, acceptance FAIL).
- The full table is in the evidence README.
