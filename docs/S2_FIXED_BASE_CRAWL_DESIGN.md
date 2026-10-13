# S2: M5.5 crawl phases on the welded fixed base (design only)

Date: 2026-10-13, branch `claude/spiderx-m55-keyboard-walking`. **Nothing here has run.** This
design enables nothing and needs an owner authorization (§5) before any goal is sent. Sources:
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
| `scripts/m61_restore_and_verify_disabled.sh` | it checks the M6.1 and M6 gates only, and deletes only `local/m61-one-cycle-*` | **no, as is**: needs an M55 gate check and branch pattern (blocker B2) |
| `m61_readiness_rehearsal`, tracking-path probe, criterion 6 (scan), E3 trot identity, E9 link check | M6.1 session or goal specific | **no**. Replaced by the shadow rehearsal (§2 A2) and the library fingerprint (P7). E9 is optional |

The M5.5 node is started separately: `ros2 run spiderx_controller m55_locomotion_node
--evidence-dir <run>/node`. It is driven by the S0 pseudo-terminal teleop driver
(`docs/evidence/m55/shadow_cloud_20261013T050401Z/tools/teleop_driver.py`, adapted to the §3
script). Every goal is built and sent by the node; Claude sends only key presses to the teleop
(→ `/cmd_vel` at 10 Hz) and node service requests.

## 2. Procedure

- **A. Disabled base.** Clean tree at the approved base; clean build; all three gates `False`
  (source and imported); `--live` exit 3 for both M6 tools; no leftover processes.
- **A2. Shadow rehearsal on the fixed base** (disabled build, nothing reaches the controller):
  - launch, then preflight READY;
  - node in SHADOW; the full §3 script at level 0 only;
  - expect 0 rate rejections, 0 command-topic messages, `goals_recorded` as scripted, joints
    unmoved (< 0.005 rad).
  - This also serves as the S0b re-run. Then stop, and verify the disabled state.
- **B. Enabling branch** `local/m55-s2-<UTC>`, never pushed. It applies
  `docs/patches/m55_enable_s2.UNAPPLIED.patch` (to be written; exactly 2 files, 2 lines):
  `M55_LOCOMOTION_DISPATCH_ENABLED = True` and the pin in `test/m55_gate.py`. The M6.1 and M6
  gates stay `False`.
- **C. Enabled build, no simulation.** M5.5 tests pass in both gate modes, and the library
  fingerprint equals the disabled build's.
- **D–E. Launch, preflight, node, then the §3 script, once.** Record the bag (§6).
- **F. Stop, then restore and verify.** This runs whatever happened.

## 3. What Claude sends, in this order (59 goals, 1 cancel)

Each run starts READY at cycle boundary 0. The driver holds the key and releases it during the
9th phase, so the node stops at the next cycle boundary. One goal is one phase (lead-in 0.2 s).

| # | Keys / requests | Level | Goals sent by the node | Sim s |
|---|---|---|---|---|
| 1 | `a`, hold `w` (0.00106 m/s) | 0 | 9 forward: S1 W(LR) S2 W(LF) S3 W(RR) S4 W(RF) S5 | 18.8 |
| 2 | hold `s` | 0 | 9 reverse (the same phases in reverse time, last first) | 18.8 |
| 3–4 | `+`, hold `w`; then hold `s` | 1 | 9 + 9 | 21.1 + 21.1 |
| 5–6 | `+`, hold `w`; then hold `s` | 2 | 9 + 9 | 25.6 + 25.6 |
| 7 | `-` `-`, hold `w`, `space` during goal 1 | 0 | 1 (b0→b1), STOPPING → READY; then `h`: 1 back to b0 | 3.4 |
| 8 | hold `w`; close the teleop during goal 1 | 0 | 1; STOPPING → DISARMED (lease); new teleop, `a` (re-arm at its rest), `h`: 1 back | 3.4 |
| 9 | hold `w`, `x` during goal 1 | 0 | 1 goal, **1 cancel**: FAULTED (last, because the posture is mid-phase and cannot re-arm) | ≤ 1.7 |

Totals: 54 cycle goals + 5 exercise goals = **59 goals and 1 cancel**, ≈ 140 s sim.

## 4. Acceptance (M5.5 §10 phase 2) and stop conditions

| # | Criterion | Evidence |
|---|---|---|
| P1 | Runs 1–6: every goal `SUCCEEDED` (status 4), no fault; each run ends READY at boundary 0 | `events.jsonl` |
| P2 | Tracking: max feedback \|error\| ≤ 0.02 rad on every goal (node abort 0.05 rad) | bag `_action/feedback` → `tracking.csv` |
| P3 | Continuity: every `goal_sent.continuity_error_rad` ≤ 0.005 rad (measured joints) | `events.jsonl` |
| P4 | Ownership: goals sent = goals scripted = 59; cancels = 1; no foreign client or publisher; 0 `joint_trajectory` messages | node status, bag |
| P5 | Stop: READY by the rewritten check 1.7 (sim bound). Lease: DISARMED after the phase. Estop: FAULTED with exactly 1 cancel | `events.jsonl`, status |
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

Blockers, in order:
- **B1.** S0b pushed: the windowed rate rule and check 1.7 in sim time.
- **B2.** The restore script extended to the M55 gate, with tests, reviewed (no motion).
- **B3.** The enabling patch written and its SHA-256 recorded.
- **B4.** This authorization, verbatim, with the placeholders filled:

> I authorize ONE S2 fixed-base crawl session in Cloud on the welded body
> (`fortress_m61a_fixed_base.launch.py`), from base commit `<SHA>`, with the enabling patch
> `docs/patches/m55_enable_s2.UNAPPLIED.patch` (SHA-256 `<hash>`) applied on a local, never-pushed
> branch `local/m55-s2-<UTC>`. That branch sets only `M55_LOCOMOTION_DISPATCH_ENABLED` and its pin
> to True. The M6.1 and M6 gates stay False. The M5.5 node may send at most 59 phase goals and 1
> cancel, in the order of S2_FIXED_BASE_CRAWL_DESIGN.md §3, at levels 0, 1 and 2, forward and
> reverse. No retry, no fix during the run, no free-base launch. Stop on any §4 stop condition.
> Restore and verify the disabled state afterwards, whatever happens.

## 6. Evidence to keep (minimal)

`docs/evidence/m55/s2_fixed_base_<UTC>/` holds:
- `README.md` (P1–P8, one line each) and `summary.json`;
- the node's `events.jsonl`, `library.json` and `final_status.json`;
- `tracking.csv` (per goal: max error, duration, result) and the teleop driver log;
- the preflight JSON, enabling commit id + patch + SHA-256, the restore report, `SHA256SUMS`.

The bag (`/spiderx/locomotion/status`, `/joint_states`, `/spiderx/sim/world_poses`, the action
feedback and status topics, `joint_trajectory`) stays in `~/spiderx_evidence/`. Only its
`metadata.yaml` is committed.

## 7. Wall time at RTF 0.20 (Cloud)

| Step | Estimate |
|---|---|
| A + C: clean builds and checks (≈ 9 min per build + full suite as measured; focused tests only in C) | ≈ 15 min |
| A2: launch + preflight 3 min, node startup 70 s, level-0 shadow script ≈ 40 s sim | ≈ 8 min |
| D: launch + preflight + node | ≈ 5 min |
| E: ≈ 140 s sim ÷ 0.2 = 700 s, plus key timing and boundaries | ≈ 13 min |
| F: stop, restore, clean rebuild, verify | ≈ 10 min |
| **Total** | **≈ 50 min**, of which ≈ 13 min of motion |

Exposure: the longest goal is 4.7 s sim ≈ 24 s wall. The watchdog is duration + 2 s sim, with
a wall backstop at RTF 0.1. S0 held all wall monitors at RTF 0.20.
