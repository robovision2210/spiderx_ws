# S3: bounded free-base crawl at level 0 (design only)

Date: 2026-10-26, branch `claude/spiderx-m55-keyboard-walking`, `src/` = `ec7f122`.
- **Status: DESIGN APPROVED PENDING S2 PASS.** S3 itself is **NOT authorized**. It needs an S2
  PASS and its own owner authorization (§5). Nothing here has run.
- **Sources.** [S2 design](S2_FIXED_BASE_CRAWL_DESIGN.md),
  [path to simulated walking](PATH_TO_SIM_WALKING.md),
  [M5.5 §10 phase 3](M55_KEYBOARD_WALKING.md), [M2 results](M2_TEST_RESULTS.md).
- **Values.** *[repo]* marks values pinned in the repository. *[owner]* marks the owner's
  decisions of 2026-10-26.

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
- **Body drift and tilt** over 6 forward cycles, against the planned 20 mm per cycle.

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
Each run starts READY at boundary 0 after the settle check, and ends with `space` while its last
phase is in flight, exactly as in S2. **No reverse in S3** *[owner]*.

| # | Keys | Goals | Planned body travel |
|---|---|---|---|
| A | `a`, hold `w` | 9: one cycle forward | +20 mm |
| C | hold `w` across 5 cycles | 45: five consecutive cycles forward *[owner]* | +100 mm (+120 mm in total) |
| E | hold `w`, `x` during goal 1 | 1 goal, **1 cancel**; then a 10 s hold observation | ≈ 0 |

- **Totals: 55 goals, 1 cancel**, ≈ 125 s sim. That is 6 cycles, 113 s, plus goal E and the
  10 s hold.
  - The owner's decision named 64 goals. That count included the reverse cycle, which "no
    reverse" removes.
  - Keeping 64 would need a second forward cycle instead. That gives 7 forward cycles and
    makes the travel conflict in §3.2 worse.
- **Planned travel:** +120 mm forward, plus a 8–17 mm lateral sway per shift *[repo, PATH §a]*.
- **No lease exercise on contact.** S2 exercises stop, lease and estop; E repeats the estop
  only, because its effect on contact is what S3 adds.

### 3.1 Hard stops

The driver presses `x` and the run ends: no retry, no fix during the run.
- **Samples.** The driver reads the node's status at 2 Hz. A "2-sample debounce" means two
  consecutive status samples, so the reaction comes ≤ 1.0 s wall after the first exceeding one.
- **Cycle checks** use the first status sample after the node's cycle count changes. At up to
  0.5 s wall late, that sample is off by under 1 mm at RTF ≤ 0.6.

| Condition | Stop when | Source | Acceptance it must not undercut (§4) |
|---|---|---|---|
| Tilt | > 0.10 rad in 2 consecutive samples | *[owner]*; M2 / phase-3 value *[repo]*; node fault 0.26 rad *[repo]* | tilt ≤ 0.10 rad |
| Fall | body z < 0.045 m in 2 consecutive samples | *[owner]*; M2 minimum *[repo]* | height within ±10 mm of standing |
| Travel from the arm pose | \|Δxy\| > 0.15 m | *[owner]* | 10–30 mm per cycle, lateral ≤ 10 mm |
| Cycle progress (replaces the live slip proxy) | a completed cycle moved < 0 mm or > 40 mm along body +y | reconciled (§3.2) | 10–30 mm per cycle |
| Cycle drift (replaces cumulative drift) | a completed cycle drifted > 20 mm lateral or > 0.10 rad yaw | reconciled (§3.2) | lateral ≤ 10 mm, \|yaw\| ≤ 0.05 rad per cycle |
| Joint limit | any measured joint within 0.02 rad of its URDF limit | *[owner]*; goals keep 0.05 rad *[repo]* | none directly (§3.2) |
| Tracking | feedback \|error\| > 0.05 rad: the node's abort, one cancel | *[owner]*, *[repo]* | values > 0.02 rad are **findings, not stops** *[owner]* |
| Foreign commander, goal count off the script (55), rate rejection, sim stall > 5 s, RTF < 0.1, a wall cap, an unplanned fault or refusal, a result other than SUCCEEDED, a teleop or driver error, the owner's instruction | as in S2 *[repo]* | | |

### 3.2 Reconciliation with the acceptance bands (6 cycles)

Per cycle, the acceptance bands are:
- travel *t* ∈ [10, 30] mm (planned 20);
- lateral |*l*| ≤ 10 mm;
- |yaw| ≤ 0.05 rad;
- height within ±10 mm of standing (*z*s ≈ 0.0545 m, so [0.0445, 0.0645] m);
- tilt ≤ 0.10 rad.

**Consistent: a run that meets every band cannot trip these stops.**
- **Tilt and fall:**
  - The tilt stop needs two samples > 0.10 rad. Any such sample already fails "tilt ≤ 0.10", so
    the stop is less strict than acceptance (a single-sample spike fails acceptance without
    stopping).
  - The fall stop is consistent except for a 0.5 mm strip, listed below.
- **Cycle progress.** It fires only on a completed cycle with *t* < 0 or *t* > 40 mm. Both lie
  outside [10, 30], so that cycle has already failed A2.
  - The 10 mm gap on each side covers the live sample's sub-millimetre offset.
  - The old proxy was a cumulative |measured − planned| > 20 mm. It **could** cancel a correct
    run: two cycles at 10 mm give −20 mm, and two at 30 mm give +20 mm.
- **Cycle drift.** It fires on a completed cycle with |*l*| > 20 mm or |yaw| > 0.10 rad. Both
  exceed the per-cycle bands, so that cycle has already failed A2.
  - The old cumulative limits (30 mm, 0.15 rad) **could** cancel a correct run: 6 cycles × 10 mm
    = 60 mm > 30 mm, and 6 × 0.05 = 0.30 rad > 0.15 rad.
  - No cumulative drift stop remains. A cumulative limit consistent with the bands would be
    60 mm / 0.30 rad, which only the travel and tilt stops can usefully bound.
- **Tracking** stops at 0.05 rad, the node abort. A3 accepts up to 0.05, so the stop is not
  stricter.

**Still able to stop a run that meets every band:**
1. **Travel 0.15 m.** The bands allow up to 6 × 30 = 180 mm along +y and 6 × 10 = 60 mm lateral,
   so |Δxy| ≤ √(180² + 60²) ≈ 190 mm at the boundaries.
   - Add the 17 mm sway mid-cycle and it reaches √(180² + 77²) ≈ 196 mm.
   - A run averaging more than 25 mm per cycle trips 0.15 m while every cycle passes. **0.20 m
     would be consistent.**
   - 0.15 m stays as approved. It is stricter by design: it bounds the travel to 30 mm beyond
     the plan. The owner decides (§7).
2. **Fall 0.045 m vs the band's lower edge 0.0445 m** (with *z*s = 0.0545 m). A run that settles
   0.5–10 mm low stays within the band, but trips the stop between 0.0445 and 0.045 m.
   - The stop is stricter by 0.5 mm, because it keeps the M2 minimum *[repo]*.
   - Making it relative (*z* < *z*s − 10 mm, debounced) would be consistent. The owner decides.
3. **Joint limit vs tracking.** The goals keep ≥ 0.05 rad from every URDF limit *[repo]*, but a
   tracking error under 0.05 rad (not a stop) can bring a joint within 0.02 rad.
   - That happens where the planned margin is < 0.07 rad and the error toward the limit exceeds
     the margin − 0.02.
   - Prerequisite P6: compute the level-0 minimum planned margin per joint offline. If it is
     ≥ 0.07 rad, this stop can only follow a tracking error > 0.05 rad, which stops the run
     anyway.
4. **The 0.5 s `/joint_states` monitor on a CPU stall.** It is independent of motion, an
   accepted risk (§6), and a safe fault.

**What an emergency stop leaves.**
- The node cancels the goal in flight once and holds; the controller holds the current joint
  positions.
- On a free base the robot stands in a mid-phase posture: on 3 feet during a swing, or 4 during
  a shift. Every sampled crawl posture keeps ≥ 15 mm static margin *in the model*, so the hold
  should be statically stable.
- Holding on contact is **unverified**; S2 shows the cancel on a welded body only.
- `arm` is then refused (mid-phase posture), and recovery means restarting the simulation. That
  is why E comes last.

## 4. Acceptance

| # | Criterion | Evidence |
|---|---|---|
| A1 | Runs A and C: every goal SUCCEEDED, no unplanned fault, each run READY at boundary 0 | node events |
| A2 | Every cycle (6), at the cycle boundaries: travel along body +y 10–30 mm (planned 20); lateral ≤ 10 mm; \|yaw\| ≤ 0.05 rad; height within ±10 mm of standing; max tilt ≤ 0.10 rad *[repo, M5.5 phase 3]* | bag pose stream at the `phase_done` sim times; `cycles.csv` |
| A3 | Tracking: no goal above 0.05 rad (node abort). Per-goal maximum recorded; any value > 0.02 rad is reported as a **finding**, not a failure *[owner]* | `tracking.csv` |
| A4 | Continuity: no `continuity` refusal (node tolerance 0.02 rad *[repo]*); values recorded | node events |
| A5 | Ownership: **55** goals sent, 1 cancel, no foreign client or publisher, 0 `joint_trajectory` messages | node status, bag |
| A6 | Estop on contact: FAULTED with exactly 1 cancel. Over the next 10 s: tilt ≤ 0.10 rad, z ≥ 0.045 m, no joint moving > 0.01 rad *[owner]* | status, `/joint_states` |
| A7 | Restore: DISABLED AND VERIFIED, all three imported gates `False`, `--live` exit 3 for both M6 tools, no leftover node or teleop | restore report |
| A8 | Library fingerprint equal to the disabled build's | `library.json` |

**Recorded only, not acceptance** *[owner]*: per-foot slip per stance phase, from the leg-link
poses (`slip.csv`). Values over 0.02 rad tracking error are findings.

Missing evidence is UNMEASURED, never PASS. A completed script is not a pass.

**What a PASS shows.** A bounded, scripted level-0 forward crawl in Gazebo (six cycles, about
0.12 m) moved the body by the measured amount, with tilt, height and drift within the stated limits.
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
- **P3.** An S3 mode in `scripts/s2/s2_driver.py`: the §3 script and the §3.1 stops (debounce,
  cycle-boundary checks), tested on the in-memory bench with injected tilt, height, drift and
  joint values.
- **P4.** A free-base shadow rehearsal of the S3 script (as S2's A2). It also measures the
  free-base RTF and the `/joint_states` stall margin.
- **P5.** The owner's remaining choices in §7.
- **P6.** The level-0 minimum planned joint margin, computed offline (§3.2, case 3).

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
- a summary (A1–A8);
- `tracking.csv`, `cycles.csv` (per-cycle displacement, drift, height, tilt) and `slip.csv`;
- the node evidence, the restore report and `SHA256SUMS`.

No bag in the repository.

**Authorization text (draft; placeholders in `<>`):**
> I authorize ONE S3 bounded free-base crawl session in Cloud on `fortress_m55_walking.launch.py`
> (the S2 harness, Gazebo GUI at nice 19), level 0, forward only, from base commit `<SHA>` (its
> `src/`), with `docs/patches/m55_enable_s2.UNAPPLIED.patch` (SHA-256 `d1dacac5…20236`) applied on
> a local, never-pushed branch `local/m55-s3-<UTC>` that sets only
> `M55_LOCOMOTION_DISPATCH_ENABLED` and its pin to True; the M6.1 and M6 gates stay False.
>
> The node may send at most **55** phase goals and **1** cancel, in the order of
> S3_FREE_BASE_CRAWL_DESIGN.md §3, driven by `scripts/s2/s2_driver.py` at `<SHA>`.
>
> Stop at once, with no retry and no fix during the run, on any of these:
> - tilt > 0.10 rad, or body z < 0.045 m, each in 2 consecutive status samples;
> - travel > `<0.15 or 0.20>` m from the arm pose;
> - a completed cycle moving < 0 or > 40 mm, or drifting > 20 mm lateral or > 0.10 rad yaw;
> - a joint within 0.02 rad of its URDF limit;
> - tracking > 0.05 rad;
> - a foreign commander, a goal count off the script, or a rate rejection;
> - a sim stall > 5 s, RTF < 0.1, or a wall cap;
> - an unplanned fault or refusal, or a result other than SUCCEEDED;
> - a teleop or driver error;
> - my instruction.
>
> No reverse, no other level, no S4, no hardware.
>
> Restore and verify the disabled state afterwards, whatever happens.

**Estimate.** About 0.5–1 session unit for P2–P4, then 1 unit for the run.

| Step | At RTF 0.20 | At RTF 0.60 |
|---|---|---|
| Build, full suite, enabled-build tests | ≈ 18 min | ≈ 18 min |
| Launch, settle check, node start-up | ≈ 4 min | ≈ 3 min |
| Motion: ≈ 125 s sim (6 cycles, goal E, the 10 s estop hold) plus pauses | ≈ 11.5 min | ≈ 4.5 min |
| Restore and verify | ≈ 10 min | ≈ 10 min |
| **Total** | **≈ 44 min** | **≈ 36 min** |

## 6. Risks

| Risk | Treatment |
|---|---|
| **Free-base fall or tip.** The 19.8 mm margin is a model value, with CAD masses, point feet and 1 ms DART contact | **Accepted**: a fall in simulation costs only the session (FAIL). **Mitigated** by the tilt, height, drift and travel stops, and by level 0 only, the largest margin |
| **Slipping** at mu 0.2 (template default, not measured) | **Accepted** as the model as it stands; it is measured (A2, A7). **Mitigated** by the live slip proxy and the travel bound |
| **Gazebo / controller startup race** (1 in 5, M6.1-A) | **Mitigated**: the harness aborts on a failed launch; one relaunch is allowed, before any goal |
| **CPU stalls tripping the 0.5 s `/joint_states` monitor** (seen in S2 attempt 3) | **Mitigated**: GUI at nice 19, node at nice −5, no `/clock` in the bag. **Accepted** residual: a stall faults the run safely (one cancel, robot held mid-phase; FAIL, not a hazard). Free-base RTF ≈ 0.2 means more contact computation per wall second, so P4 measures the margin first |
| Estop hold on contact unverified | **Accepted**, and measured as A6. E is last, so a failed hold ends nothing else |
| Tracking larger under load than on the welded body | **Decided** *[owner]*: the stop is at 0.05 rad (node abort); values > 0.02 rad are findings |

## 7. Decisions

**Taken by the owner, 2026-10-26:**
- tilt 0.10 rad and fall z < 0.045 m, each with a 2-sample debounce;
- travel 0.15 m, the joint-limit stop at 0.02 rad, and the 10 s estop hold;
- tracking: stop at 0.05 rad, report values > 0.02 rad as findings;
- per-foot slip recorded only;
- Run C five cycles forward, no reverse;
- S3 and S4 do not share an approval.

**Open:**
1. **Travel stop** (§3.2, case 1): 0.15 m as approved, which is stricter than the bands by design,
   or 0.20 m, which is consistent.
2. **Fall stop** (§3.2, case 2): absolute 0.045 m as approved (0.5 mm stricter than the band), or
   relative *z*s − 10 mm.
3. **Goal count:** 55, the reading taken here (no reverse), or 64 with a second forward cycle
   (7 cycles; this needs travel ≥ 0.23 m to stay consistent).
4. The replacement slip and drift stops (§3.1), which are new: please confirm.
