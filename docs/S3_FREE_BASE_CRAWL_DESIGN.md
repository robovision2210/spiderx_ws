# S3: bounded free-base crawl at level 0 (design only)

Date: 2026-10-26, branch `claude/spiderx-m55-keyboard-walking`, `src/` = `ec7f122`.
- **Status.** Design only. Nothing here has run or is approved to run. S3 comes after an S2 PASS
  and needs its own owner authorization (§5).
- **Sources.** [S2 design](S2_FIXED_BASE_CRAWL_DESIGN.md),
  [path to simulated walking](PATH_TO_SIM_WALKING.md),
  [M5.5 §10 phase 3](M55_KEYBOARD_WALKING.md), [M2 results](M2_TEST_RESULTS.md).
- **Values.** Values marked *[repo]* are already pinned in the repository. Every new threshold
  is **PROPOSED** and is the owner's choice (§7).

## 1. What S3 tests that S2 cannot

S2 plays the phase goals on a welded body. Gravity loads the legs there, but there is no contact
and nothing can tip, slip or drift. S3 is the first run with the body free on the ground:

- **Static stability.** The M5.5 crawl keeps the COM 19.8 mm inside the support triangle at
  level 0 *[repo, PATH §a]*; 15 mm is required.
  - This is a **model value**: quasi-static, with CAD steel-density masses, point feet and
    assumed contact.
  - **It has not been measured in Gazebo.**
- **Ground contact** of the four feet through DART, with the body supported only by its legs.
- **Foot slip.** mu1 = mu2 = **0.2** on every link *[repo, `spiderx_fortress.gazebo.xacro`]*.
  These are template defaults, not measured values (audit item P4).
- **Body drift and tilt** over 7 cycles, against the planned 20 mm per cycle.

## 2. Plant

**Launch.** `fortress_m55_walking.launch.py`, the S0 free-base bench:
- the unchanged M2 posture-hold bench (`fortress_posture_hold.launch.py`);
- the unchanged model (no weld), spawned at `fortress.launch.py`'s default **z 0.075 m**
  *[repo]*;
- the robot drops a few millimetres and settles at base height **0.0545 m** *[repo, M2: min
  0.05448 m, range 0.00007 m over 10 s]*;
- the M5.5 node is started by the launch.

The S2 harness parts that are not fixed-base specific apply unchanged:
- `env.sh`, Xvfb, Gazebo GUI at nice 19, node at nice −5;
- a bag without `_action/status` or `/clock`;
- abort on a failed launch.

| | Welded (S2) | Free base (S3) |
|---|---|---|
| Model | wrapper xacro with a world weld at 0.125 m | the unchanged URDF; no weld |
| Spawn | identity (the weld carries the mount) | z 0.075 m, then settles |
| Contact, slip, tipping | none | all active (mu 0.2) |
| RTF in Cloud | 0.59 (A2) | ≈ 0.20 (S0) |

**What does not apply, and what replaces it.**

| M6.1-A / S2 item | S3 |
|---|---|
| E8 weld rule (1 mm / 0.0033 rad from the weld; z 0.125 m) | **Replaced** by free-base pose monitors from the same model · `dummy_link` pose the node already reads (status `ground_truth`): height, tilt, drift and travel (§3) |
| Weld-integrity and attachment gate, `frame_spawn_not_identity`, `m61a_observe_fixed_base --preflight` | **Replaced** by a settle check before `arm`, using the M2 criteria *[repo]*: body ≥ 0.045 m, height range ≤ 0.005 m, \|roll\|, \|pitch\| ≤ 0.10 rad over ≥ 10 s of sim; sim time advancing; node graph ownership 1/0/0 |
| E9 leg-link consistency | Reused only offline: stance-foot positions from the leg-link entries give per-foot slip (§4, A7) |
| Restore script | **Applies** unchanged, except that its deletion pattern must accept `local/m55-s3-*` (P2) |
| Enabling patch | **Reused unchanged** (§5) |

## 3. What Claude would send, in order, and the hard stops

Level 0 only (stride 20 mm, 1.06 mm/s; goals of 1.2–2.8 s sim, cycle 18.8 s with lead-ins).
Each run starts READY at boundary 0 after the settle check. A run ends with `space` while its
last phase is in flight, exactly as in S2.

| # | Keys | Goals | Planned body travel |
|---|---|---|---|
| A | `a`, hold `w` | 9: one cycle forward | +20 mm |
| B | hold `s` | 9: one cycle reverse | back to 0 |
| C | hold `w` across 5 cycles | 45: five consecutive cycles forward | +100 mm |
| E | hold `w`, `x` during goal 1 | 1 goal, **1 cancel**; then a 10 s hold observation | ≈ 0 |

- **Totals:** **64 goals, 1 cancel**, ≈ 133 s sim. Planned travel peaks at about **+0.10 m**
  forward, plus a 8–17 mm lateral sway per shift *[repo, PATH §a]*.
- **No lease exercise on contact.** S2 exercises stop, lease and estop on the real controller;
  E repeats the estop only, because its effect on contact is what S3 adds.

**Hard stops.** The driver presses `x` and the run ends: no retry, no fix during the run.

| Condition | Threshold | Source |
|---|---|---|
| Tilt | > 0.10 rad | PROPOSED: the M2 and phase-3 acceptance value *[repo]*. The node's own fault stays at 0.26 rad *[repo]* |
| Height drop (fall) | body z < 0.045 m | PROPOSED: the M2 minimum *[repo]*, 9.5 mm below standing |
| Travel from the arm pose | \|Δxy\| > 0.15 m | PROPOSED: 50 mm beyond the planned maximum |
| Cumulative drift | lateral > 30 mm or \|yaw\| > 0.15 rad | PROPOSED: 3 × the per-cycle acceptance |
| Slip (live proxy) | \|measured − planned\| body displacement > 20 mm | PROPOSED: status `delta_since_arm_xy` vs `planned_displacement_xy_m`; one cycle's stride |
| Joint limit | any measured joint within 0.02 rad of its URDF limit | PROPOSED: goals keep 0.05 rad *[repo]*, so reaching 0.02 means the legs are being forced |
| Tracking | > 0.05 rad (node abort *[repo]*) | Whether S2's 0.02 rad also stops S3 is an open decision (§7) |
| Foreign commander, goal count off the script, rate rejection, sim stall > 5 s, RTF < 0.1, a wall cap, an unplanned fault or refusal, a result other than SUCCEEDED, a teleop or driver error, the owner's instruction | as in S2 *[repo]* | |

**What an emergency stop leaves.**
- The node cancels the goal in flight once and holds; the controller holds the current joint
  positions.
- On a free base, the robot stands in a mid-phase posture: on 3 feet during a swing, or 4
  during a shift. Every sampled crawl posture keeps ≥ 15 mm static margin *in the model*, so the
  hold should be statically stable.
- Holding on contact is **unverified**. S2 shows the cancel on a welded body only.
- `arm` is then refused (mid-phase posture), and recovery means restarting the simulation. That
  is why E comes last.

## 4. Acceptance

| # | Criterion | Evidence |
|---|---|---|
| A1 | Runs A–C: every goal SUCCEEDED, no unplanned fault, each run READY at boundary 0 | node events |
| A2 | Every cycle (7): displacement along body +y 10–30 mm forward or back (planned 20); lateral ≤ 10 mm; \|yaw\| ≤ 0.05 rad; height within ±10 mm of standing (0.0545 m); max tilt ≤ 0.10 rad *[repo, M5.5 phase 3]* | status `ground_truth` at cycle boundaries |
| A3 | Tracking: per-goal max feedback error recorded; ≤ 0.05 rad (≤ 0.02 if chosen) | `tracking.csv` |
| A4 | Continuity: no `continuity` refusal (node tolerance 0.02 rad *[repo]*); values recorded | node events |
| A5 | Ownership: 64 goals sent, 1 cancel, no foreign client or publisher, 0 `joint_trajectory` messages | node status, bag |
| A6 | Estop on contact: FAULTED with exactly 1 cancel. Over the next 10 s: tilt ≤ 0.10 rad, z ≥ 0.045 m, no joint moving > 0.01 rad (PROPOSED) | status, `/joint_states` |
| A7 | Foot slip, offline from the leg-link poses: per stance phase ≤ 5 mm (PROPOSED) | `slip.csv` |
| A8 | Restore: DISABLED AND VERIFIED, all three imported gates `False`, `--live` exit 3 for both M6 tools, no leftover node or teleop | restore report |
| A9 | Library fingerprint equal to the disabled build's | `library.json` |

Missing evidence is UNMEASURED, never PASS. A completed script is not a pass.

**What a PASS shows.** A bounded, scripted level-0 crawl in Gazebo moved the body by the
measured amount, with tilt, height and drift within the stated limits.
- The conditions: CAD masses, assumed contact at mu 0.2, a 1 ms DART step.
- The emergency stop held the robot on contact.

**What a PASS does not show.**
- Keyboard walking (S4), or any level above 0.
- Walking on terrain, robustness or balance control: there is no feedback balance.
- Real friction or masses, and anything about hardware.

## 5. Patch, restore, evidence, authorization and estimate

**Prerequisites (P), each a blocker. No motion is needed for any of them.**
- **P1.** S2 PASS. PATH §c: S2 first; S2 and S3 never share a launch or an approval.
- **P2.** The restore script accepts `local/m55-s3-*`: one pattern line plus a hermetic test.
- **P3.** An S3 mode in `scripts/s2/s2_driver.py`: the §3 script and its live free-base stops,
  tested on the in-memory bench with injected tilt, height, drift and joint values.
- **P4.** A free-base shadow rehearsal of the S3 script (as S2's A2). It also measures the
  free-base RTF and the `/joint_states` stall margin.
- **P5.** The owner's choices in §7.

**Enabling patch.** `docs/patches/m55_enable_s2.UNAPPLIED.patch` is reused unchanged: the gate
and its pin, 2 lines, SHA-256 `d1dacac5d9b2bc0b63220c3583526d45f5edacb657394b508710978972b20236`.
It is applied on `local/m55-s3-<UTC>`, which is never pushed.

**Restore.** `scripts/m61_restore_and_verify_disabled.sh` with these arguments:
```
--base claude/spiderx-m55-keyboard-walking --enabling-ref local/m55-s3-<UTC>
--discard-uncommitted-src --delete-enabling-branch
```
It runs after success, failure or interruption.

**Evidence** (`docs/evidence/m55/s3_free_base_<UTC>/`):
- a summary (A1–A9);
- `tracking.csv`, `cycles.csv` (per-cycle displacement, drift, height, tilt) and `slip.csv`;
- the node evidence, the restore report and `SHA256SUMS`.

No bag in the repository.

**Authorization text (draft; placeholders in `<>`):**
> I authorize ONE S3 bounded free-base crawl session in Cloud on `fortress_m55_walking.launch.py`
> (the S2 harness, Gazebo GUI at nice 19), level 0 only, from base commit `<SHA>` (its `src/`),
> with `docs/patches/m55_enable_s2.UNAPPLIED.patch` (SHA-256 `d1dacac5…20236`) applied on a
> local, never-pushed branch `local/m55-s3-<UTC>` that sets only `M55_LOCOMOTION_DISPATCH_ENABLED`
> and its pin to True; the M6.1 and M6 gates stay False.
>
> The node may send at most 64 phase goals and 1 cancel, in the order of
> S3_FREE_BASE_CRAWL_DESIGN.md §3, driven by `scripts/s2/s2_driver.py` at `<SHA>`. Body travel
> is at most 0.15 m from the arm pose.
>
> Stop at once on any §3 hard stop, at the thresholds `<chosen>`, with no retry and no fix
> during the run. No other level, no S4, no hardware.
>
> Restore and verify the disabled state afterwards, whatever happens.

**Estimate.** About 0.5–1 session unit for P2–P4, then 1 unit for the run.

| Step | At RTF 0.20 | At RTF 0.60 |
|---|---|---|
| Build, full suite, enabled-build tests | ≈ 18 min | ≈ 18 min |
| Launch, settle check, node start-up | ≈ 4 min | ≈ 3 min |
| Motion: ≈ 133 s sim plus pauses and the 10 s estop hold | ≈ 13 min | ≈ 5 min |
| Restore and verify | ≈ 10 min | ≈ 10 min |
| **Total** | **≈ 45 min** | **≈ 36 min** |

## 6. Risks

| Risk | Treatment |
|---|---|
| **Free-base fall or tip.** The 19.8 mm margin is a model value, with CAD masses, point feet and 1 ms DART contact | **Accepted**: a fall in simulation costs only the session (FAIL). **Mitigated** by the tilt, height, drift and travel stops, and by level 0 only, the largest margin |
| **Slipping** at mu 0.2 (template default, not measured) | **Accepted** as the model as it stands; it is measured (A2, A7). **Mitigated** by the live slip proxy and the travel bound |
| **Gazebo / controller startup race** (1 in 5, M6.1-A) | **Mitigated**: the harness aborts on a failed launch; one relaunch is allowed, before any goal |
| **CPU stalls tripping the 0.5 s `/joint_states` monitor** (seen in S2 attempt 3) | **Mitigated**: GUI at nice 19, node at nice −5, no `/clock` in the bag. **Accepted** residual: a stall faults the run safely (one cancel, robot held mid-phase; FAIL, not a hazard). Free-base RTF ≈ 0.2 means more contact computation per wall second, so P4 measures the margin first |
| Estop hold on contact unverified | **Accepted**, and measured as A6. E is last, so a failed hold ends nothing else |
| Tracking larger under load than on the welded body | **Open**: 0.02 rad (S2) may stop a correct run (§7) |

## 7. Open decisions for the owner

1. The PROPOSED thresholds in §3 and §4: tilt 0.10 rad; z < 0.045 m; travel 0.15 m; drift
   30 mm / 0.15 rad; slip proxy 20 mm; joint limit 0.02 rad; estop hold (10 s, 0.01 rad); foot
   slip 5 mm per stance phase.
2. Run C: five forward cycles (64 goals, ending about 0.10 m away), or five forward plus five
   back (109 goals, returning near the start; travel bound unchanged).
3. Tracking: stop at S2's 0.02 rad, or only at the node's 0.05 rad abort, with 0.02 recorded.
4. Foot slip (A7): an acceptance criterion, or recorded only.
5. Whether S3 and S4 may share one approval. Recommended: **no**. PATH §c allows it only behind a
   hard stop after S3: an automatic check of the S3 criteria plus the owner's go/no-go.
