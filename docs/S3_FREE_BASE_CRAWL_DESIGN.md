# S3: bounded free-base crawl at level 0 (design only)

Date: 2026-10-26; preparation status updated 2026-10-31 on `claude/spiderx-s3-preparation` (§8).
- **Status: RUN ONCE (2026-11-01).** S3 bounded level-0 free-base crawl accepted retrospectively under criteria revision S3-r1 (§9). The frozen scorer's verdict, INCONCLUSIVE with A3 UNMEASURED, is preserved and linked in §9. S4, hardware and autonomous navigation remain unvalidated.
- **S2 (P1):** S2 fixed-base validation accepted retrospectively under criteria revision r1. Not walking, contact or balance validation. The frozen verdicts stay FAIL (2026-10-29 tracking; 2026-10-30 Phase C).
- **Plant:** the gain-1.0 actuator model (`docs/S2_SIM_ACTUATOR_MODEL_DESIGN.md`) through the
  overlay `~/spiderx_gz_overlay_ws_r2` (§2). Results measured at gain 0.1 (M2, S0) do **not**
  validate it.
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

**Actuator model.** Position control with `position_proportional_gain` 1.0 from
`gz_ros2_control_plant.yaml`, applied only by the pinned overlay. Use the r2 overlay
(`~/spiderx_gz_overlay_ws_r2`: 0.7.15 + patch 0001 + the shutdown backport 0002; libraries
`3a8f2044…` and `06777e14…`, `third_party/gz_ros2_control_overlay/README.md`). Its runtime
loading must be shown on the free-base launch, as Phase B and Phase C showed it on the welded one.

**Launch.** `fortress_m55_walking.launch.py`, the S0 free-base bench:
- the unchanged M2 posture-hold bench (`fortress_posture_hold.launch.py`);
- the unchanged model (no weld), spawned at `fortress.launch.py`'s default **z 0.075 m**
  *[repo]*;
- the robot drops a few millimetres and settled at base height **0.0545 m** at gain 0.1 *[repo,
  M2: min 0.05448 m, range 0.00007 m over 10 s]*. That value is **not** assumed for gain 1.0: the
  driver's settle check and the scorer measure the standing height of each run;
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
| Restore script | **Applies**; it accepts `local/m55-s3-*` since 2026-10-31 (P2, done) |
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
   - Prerequisite P6, **done** (2026-10-31, `scripts/s3/level0_joint_margins.py`): the minimum
     planned margin over the 9 level-0 forward goals is **0.3541 rad** (`rf_foot_joint`, lower
     limit; per joint 0.354–0.500 rad; waypoints and the controller's cubic interpolation agree to
     0.1 mrad). A joint can come within 0.02 rad of a limit only after deviating ≥ 0.334 rad
     from the plan, far beyond the node's 0.05 rad tracking abort. During a goal, the tracking
     abort fires first; the joint-limit stop remains the backstop for deviations outside goal
     tracking (between goals and during the estop hold, where no goal is active).
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

**Prerequisites (P), each a blocker** (status 2026-10-31; details in §8):
- **P1. Done.** S2 fixed-base validation accepted retrospectively under criteria revision r1. Not walking, contact or balance validation.
- **P2. Done.** The restore script accepts `local/m55-s3-*`, with 4 new hermetic tests.
- **P3. Done.** A separate S3 driver, `scripts/s3/s3_driver.py`, on the unchanged S2 driver's
  plumbing, with the §3 script and §3.1 stops, tested on the in-memory bench (42 cases, 17 of them for the shadow rehearsal mode); and the
  S3 scorer `scripts/s3/score_s3.py` (A1–A8, 27 cases).
- **P4. Done** (rehearsal 2 on 2026-10-31: PASS, shadow only; attempt 1 INCONCLUSIVE, §8). A free-base shadow rehearsal of the S3 script on the **gain-1.0** plant (as
  S2's A2), with settling verification. It also measures the free-base RTF and the
  `/joint_states` stall margin. It needs an S3 run harness and its own authorization (§8).
- **P5. Done.** The owner retained the approved limits and the 55-goal forward-only scope on
  2026-10-31 (§7).
- **P6. Done.** The level-0 minimum planned joint margin: 0.3541 rad (§3.2, case 3).
- **P7. Done** (2026-10-31, shadow rehearsal 1: PASS, §8). A launch-only check that the r2 overlay's shutdown backport gives a clean
  Gazebo exit (§8).

**Enabling patch.** `docs/patches/m55_enable_s2.UNAPPLIED.patch` is reused unchanged: the gate
and its pin, 2 lines, SHA-256 `d1dacac5d9b2bc0b63220c3583526d45f5edacb657394b508710978972b20236`.
It is applied on `local/m55-s3-<UTC>`, which is never pushed. The baseline is a commit of
`claude/spiderx-s3-preparation` (or its successor) whose `src/` is recorded in the run evidence.

**Restore.** `scripts/m61_restore_and_verify_disabled.sh` with these arguments:
```
--base <the baseline branch> --enabling-ref local/m55-s3-<UTC> --delete-enabling-branch
--build-cmd 'rm -rf build install && PYTHONPATH=/opt/mm/st59site colcon build --symlink-install'
--launch-pid-file <run>/launch.pid --xvfb-pid-file <run>/xvfb.pid
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
> The plant is the gain-1.0 actuator model through the r2 overlay `~/spiderx_gz_overlay_ws_r2`
> (libraries `3a8f2044…` and `06777e14…`), verified loaded at runtime before arming.
>
> The node may send at most **55** phase goals and **1** cancel, in the order of
> S3_FREE_BASE_CRAWL_DESIGN.md §3, driven by `scripts/s3/s3_driver.py` at `<SHA>`.
>
> Stop at once, with no retry and no fix during the run, on any of these:
> - tilt > 0.10 rad, or body z < 0.045 m, each in 2 consecutive status samples;
> - travel > 0.15 m from the arm pose;
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

**Taken by the owner, 2026-10-31** (the previously open items):
1. **Travel stop** stays at 0.15 m, stricter than the bands by design (§3.2, case 1).
2. **Fall stop** stays absolute, z < 0.045 m in 2 consecutive fresh samples (§3.2, case 2).
3. **Goal count:** 55 (forward only: one cycle, five cycles, the estop goal).
4. The per-cycle progress (< 0 or > 40 mm) and drift (> 20 mm lateral or > 0.10 rad yaw) stops
   are confirmed.

These conservative stops can end a run whose individual cycle measurements all lie within the
acceptance bands (travel, §3.2 case 1; fall, case 2). That is accepted, not a defect; the limits
are not widened to match the bands.

## 8. Preparation status (2026-10-31, offline; nothing has run in a simulator)

Branch `claude/spiderx-s3-preparation`. Evidence of the offline work:
`docs/evidence/m55/s3_preparation_20261031/`.

**Driver, `scripts/s3/s3_driver.py`.** It reuses the unchanged S2 driver's teleop pseudo-terminal,
key repeat, status subscription, estop fallback (the node's estop service when no teleop is
alive) and wall caps; it does not use the S2 script or its 0.02 rad stop.
- **Script:** settle check, then `a`; run A (1 cycle), run C (5 cycles), each stopped at its
  boundary by `space` during its last phase; run E (`w`, then `x` while its goal is in flight);
  a 10 s sim-time hold observation; `q`. At most 55 goals and 1 cancel; no `s`, `+` or recovery.
- **Settle check (before arming):** at least 10 s of sim with body z ≥ 0.045 m, a z range
  ≤ 0.005 m, tilt ≤ 0.10 rad (tilt bounds roll and pitch from above) and graph ownership 1/0/0.
  It records the standing height. Settling samples never trigger the tilt or fall stop.
- **Fresh samples:** a status counts only if its body pose is ≤ 1.0 s old (`pose_stale_s`) and
  its sim time is later than the last counted sample's. A duplicate publication or a stalled sim
  clock therefore cannot complete the 2-sample debounce.
- **Tilt** is the node's ground-truth tilt: the angle between the `dummy_link` (= `base_link`) z
  axis and the world z axis, independent of yaw.
- **Body frame:** +y forward, +x right (`spiderx_legs.yaml`; `dummy_joint` has no origin). Cycle
  displacement is measured in the frame of the previous boundary's yaw; yaw differences are
  wrapped to (−π, π].
- **Stops:** as approved (§3.1). Tracking > 0.02 rad is recorded as a finding; > 0.05 rad (the
  node's abort) stops. A stop releases the held key and presses `x`; SIGINT and SIGTERM do the
  same (exit 4), as does any driver error (exit 3).
- **Bench:** 25 cases (42 with the shadow rehearsal mode) on the real `LocomotionSession`, with a free-base ground truth driven by
  the node's planned displacement: the full 55-goal script at RTF 0.2 and 0.6; debounce (one
  spike, a duplicate publication, a stalled sim clock, two fresh samples); fall; travel 0.15 m
  ending a run whose five completed cycles (28 mm each) lie within the bands; progress, lateral
  and yaw stops; yaw across ±π; joint limit; tracking finding vs abort; estop-service fallback;
  driver error; interruption; timeout; a settle failure that never arms. Every case checks that
  nothing is dispatched after the run ends.

**Scorer, `scripts/s3/score_s3.py`** (A1–A8, specified before any S3 run; criteria revision r1's
evidence handling built in):
- A2 per cycle at the `phase_done` boundaries: progress along body +y in [10, 30] mm, |lateral|
  ≤ 10 mm, |yaw| ≤ 0.05 rad, height within ±10 mm of the run's measured standing height, max tilt
  ≤ 0.10 rad over the cycle. A missing boundary pose or standing height is UNMEASURED.
- A3: > 0.05 rad fails; > 0.02 rad is a finding; missing feedback on any goal but the last is
  UNMEASURED. The last goal (the estop) may be "tracking N/A — planned cancellation before
  feedback" only with proof: identity, scripted `x`, exactly one early cancel processed by the
  server (code 0), CANCELED, no prior refusal or unplanned fault, no feedback.
- A5: 55 goals, 1 cancel, no foreign client or publisher; an absent command topic counts as zero
  only with recorder, continuity, owner-monitor and capture support; a present topic with any
  message fails.
- A6 is never inferred from CANCELED: it needs pose and joint observations over 10 s of sim after
  the cancel (tilt ≤ 0.10 rad, z ≥ 0.045 m, joint motion ≤ 0.01 rad), with no coverage gap beyond
  the node's staleness limits; otherwise UNMEASURED.
- A1, A4, A7 and A8 as in §4; per-foot slip is written to `slip.csv` and not scored.
- 27 synthetic cases. A loader smoke test on the S2 Phase C bag (read-only; not S3 evidence)
  aligned 506 of 506 status pairs and parsed the body and the four foot links.

**Restore.** `scripts/m61_restore_and_verify_disabled.sh` accepts `local/m55-s3-*`; every other
protection is unchanged. Restore tests: 18, of which 4 are new (full S3 restore, kept after a
failed check, never a pushed branch, never a look-alike name).

**Level-0 planned joint margins** (P6; the 9 forward goals; cubic interpolation; lead-in excluded):

| Joint | To lower (rad) | To upper (rad) | Minimum |
|---|---|---|---|
| `lf_foot_joint` | 0.6055 | 0.3595 | **0.3595** (upper) |
| `lf_hip` | 0.4910 | 0.5757 | **0.4910** (lower) |
| `lf_thigh_joint` | 0.4220 | 0.6852 | **0.4220** (lower) |
| `lr_foot_joint` | 0.3751 | 0.6055 | **0.3751** (lower) |
| `lr_hip` | 0.5641 | 0.4910 | **0.4910** (upper) |
| `lr_thigh_joint` | 0.4220 | 0.6628 | **0.4220** (lower) |
| `rf_foot_joint` | 0.3541 | 0.5901 | **0.3541** (lower) |
| `rf_hip` | 0.5828 | 0.4832 | **0.4832** (upper) |
| `rf_thigh_joint` | 0.6222 | 0.5004 | **0.5004** (upper) |
| `rr_foot_joint` | 0.3807 | 0.5901 | **0.3807** (lower) |
| `rr_hip` | 0.4832 | 0.5718 | **0.4832** (lower) |
| `rr_thigh_joint` | 0.5923 | 0.5004 | **0.5004** (upper) |

Overall minimum **0.3541 rad** (`rf_foot_joint`). The interaction with the stops is in §3.2,
case 3. No trajectory, limit or threshold was changed.

**Shutdown.** The r2 overlay backports upstream gz_ros2_control #686 (no second node removal at
destruction), with guards for a partial `Configure()`. Offline gtest: 7 cases pass. Its runtime
effect is **not verified**.

**Checks.** All three dispatch gates False on a fresh import; the phase-library digest is
unchanged (`62f8a100…`, 54 goals); S2 driver bench 8 of 8.

**Shadow rehearsal 1** (2026-10-31, `docs/evidence/m55/s3_shadow_20261031T190613Z/`): one free-base
launch with the r2 overlay and the run-local harness, dispatch disabled.
- **P7 PASS.** The r2 libraries are mapped with the provenance hashes; gain 1.0, 100 Hz and
  0.001 s. The server exited with status 0 after SIGINT, with no abort and no leftovers.
- **Settling PASS.** Standing height 0.054500 m, range 9.0 µm over 10 s of sim, tilt 1.5e-7 rad.
- **P4 INCONCLUSIVE.** The driver stopped after the first shadow cycle: the lower progress bound
  (< 0 m) fired on the standing body's creep, −3.28e-10 m.

**Shadow rehearsal mode** (`s3_driver.py --shadow-rehearsal`, added after rehearsal 1).
- **Refused before start** (exit 5) unless all three gates are False, both in a fresh interpreter
  and in the driver process.
- **Mode checks.** The node must report SHADOW with the gate False before the teleop starts, and in
  every status and before every scripted key after that. A missing, conflicting or changed mode
  stops the run.
- **Progress.** Only in that verified mode is the lower forward-progress bound not applied: no
  trajectory runs, so the measured displacement is the standing body's creep. It is still measured
  and recorded as "Minimum forward-progress condition not applicable: dispatch disabled." Every
  other stop is unchanged, including the 40 mm upper bound and the lateral and yaw bounds.
- **Default behaviour** is unchanged: progress < 0 or > 40 mm stops.
- **Not acceptance.** A shadow rehearsal is not crawl acceptance.

**Completed.** P4 passed in shadow (rehearsal 2, 2026-10-31, `docs/evidence/m55/s3_shadow_20261031T193303Z/`). The authorized live attempt ran on 2026-11-01 (§9).

## 9. Execution and retrospective acceptance (2026-11-01)
- **Run.** One authorized free-base attempt, `s3_free_base_20261101T115002Z`.
  - Disabled baseline `d918799`, with a local enabling commit that was never pushed and was deleted after the restore verified.
  - The r2 overlay at gain 1.0.
  - The committed driver in live (default) mode.
- **What ran.** The script completed with 55 goals and 1 cancel: one cycle, five cycles, the estop, and the 10 s hold.
  - The six cycles moved 20.015–20.029 mm each, about 0.1201 m in all.
  - Per cycle: \|lateral\| ≤ 19 µm, \|yaw\| ≤ 8.7e-5 rad, height within 35 µm, tilt ≤ 0.0026 rad.
- **Frozen verdict, preserved: INCONCLUSIVE.** A1, A2 and A4–A8 PASS; A3 UNMEASURED. Evidence: `docs/evidence/m55/s3_free_base_20261101T115002Z/` (commit `a73ea95`).
  - Goal 55, the scripted estop, has no feedback.
  - The pre-specified exception's ordering check compared the node's reaction with a key time the driver logs only after its write returns.
- **Review.** `docs/evidence/m55/s3_free_base_20261101T115002Z_a3_review/` (`2859c8c`), with `ADDENDUM_1.md` on rounding. Tool: `scripts/s3/review_a3.py` (`ea0203f`).
- **Owner decision (2026-11-01).** "S3 bounded level-0 free-base crawl accepted retrospectively under criteria revision S3-r1." Record: `docs/evidence/m55/s3_free_base_20261101T115002Z_adoption_s3r1/README.md`.
- **Limitations.**
  - Goal 55's tracking is N/A, not a measured value.
  - The estop source is inferred from the recorded chain.
  - There is no direct contact-point-slip measurement.
  - Simulation only, with an idealized actuator model.
- **Not shown.** Interactive keyboard walking (S4), reverse or faster-level motion on contact, terrain, balance control, hardware or autonomous navigation. The next step is planned in `docs/S4_KEYBOARD_TELEOP_PLAN.md`; nothing there is authorized.
