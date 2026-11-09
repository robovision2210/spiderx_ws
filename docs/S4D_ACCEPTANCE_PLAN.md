# S4d: full S4 simulation acceptance: design and owner-decision proposal

**DRAFT — OWNER DECISIONS PENDING — NO RUNTIME AUTHORIZATION.**

Date: 2026-11-03. Branch `claude/spiderx-s4d-plan`, from `claude/spiderx-s4c-preparation` at `40f5f2c`.

This document is a design only. It authorizes no implementation, launch, enabling branch, goal, cancel or change to a safety limit. Every value marked **PROPOSED** is an owner decision (§10). Nothing here changes historical evidence, README or STATUS verdicts, runtime code, existing scorers or the dispatch gates.

Three kinds of statement are kept apart:
- **REQUIRED**: an existing requirement, with its source.
- **MEASURED**: a result from committed evidence, with its level and plant.
- **PROPOSED**: new in this document.

## 1. Sources
- [S4 plan](S4_KEYBOARD_TELEOP_PLAN.md): §2 criteria, §3 stages (S4d row), §3.1–§3.3 decisions and results, §4 stops, evidence and cleanup, §6 open decisions.
- [M5.5](M55_KEYBOARD_WALKING.md):
  - §4 frames;
  - §5 phase durations (60 mm row);
  - §6 state machine, home and faults;
  - §7 monitors;
  - §8 teleop keys;
  - §10 checks 1.7, 1.9, 1.11 and 1.13, and the phase 3 and phase 4 tables;
  - §16.1 stopping timeline.
- [Path to simulated walking](PATH_TO_SIM_WALKING.md): the S4 row; §c ordering; §d claim boundaries.
- Code (line numbers at `40f5f2c`):
  - `src/spiderx_controller/spiderx_controller/m55_locomotion.py`:
    - `PhaseLibrary.goal` 442;
    - `resolve_arm_posture` 551;
    - `_fault` 666;
    - `_arm` 811;
    - `_disarm` 851, `_stop` 863, `_estop` 872, `_home` 889–903;
    - cycle counting 965;
    - `_after_rest` 971–1009;
    - `step` 1027–1051;
    - `_ready_tick` 1053–1065.
  - `src/spiderx_controller/spiderx_controller/m55_teleop.py`:
    - `TeleopState.key` 68–101 (`+`/`-` at 87);
    - `twist` hold window 103–111;
    - `handle` QUIT 222–225;
    - `_shutdown_sequence` 236–247.
  - `src/spiderx_controller/config/m55_locomotion.yaml`: lease 0.5 s, tick 20 Hz, status 2 Hz, heartbeat 10 Hz, hold 0.8 s, tracking 0.05 rad, minimum RTF 0.1.
  - Drivers and scorers:
    - `scripts/s3/s3_driver.py` 63–82: S3 constants and stops; 300–345: travel and cycle checks;
    - `scripts/s4/s4a_driver.py`, `s4b_driver.py`: release, switch, home and lease helpers, 98–242;
    - `scripts/s4/s4c_driver.py`: plan, level and close checks, 87–272;
    - `scripts/s4/score_s4b.py`, `score_s4c.py`: C1–C11 at 40–80, `assess_displacement` at 315;
    - bench tests `scripts/s4/test_s4*_driver_live.py`, `test_score_s4*.py`.
  - `scripts/m61_restore_and_verify_disabled.sh`: enabling-branch patterns, 42–43 and 274–290.
- Evidence (`docs/evidence/m55/`):
  - `s2_phase_c_20261030T062140Z`;
  - `s3_free_base_20261101T115002Z` and its `_adoption_s3r1` record;
  - `s4a_live_20261101T153411Z`;
  - `s4b_live_20261102T092327Z`;
  - `s4c_l1_live_20261102T182303Z` and its `_restore_followup_20261102T185957Z`;
  - `s4c_l2_shadow_20261103T094244Z`, `s4c_l2_live_20261103T111733Z`.

## 2. Existing requirements (REQUIRED, unchanged)
From M5.5 §10 phase 4, the PATH S4 row and S4 plan §2:

| # | Requirement | Bound as written |
|---|---|---|
| R1 | Forward 0.3 m at the fastest level (3/3), then back | no unplanned fault; tilt ≤ 0.10 rad; lateral deviation ≤ 30 mm; heading drift ≤ 0.1 rad. **No distance tolerance is given** |
| R2 | Key release while walking | READY within the goals still to finish + 1.0 s of sim, as 1.7: the in-flight goal, and the next one if a boundary fell within the 0.95 s wall detection. That is ≤ 2 × 4.7 + 1.0 s at 60 mm; wall backstop 120 s |
| R3 | Speed change, reversal, `home` | take effect at the next cycle or phase boundary; `home` ends at boundary 0 |
| R4 | `x` while walking | FAULTED within one 50 ms tick; one cancel; no further goal |
| R5 | Close the teleop | DISARMED by check 1.9 (sim bound, 120 s wall backstop); no motion until `a`. Check 1.9 is a close *while walking*: a clean quit, or a kill on the crash path |
| R6 | Evidence | a bag and the evidence directory; a screen recording only if the owner makes one |
| R7 | Ordering | the owner's go/no-go before any faster level (done through S4c); each stage has its own authorization |
| R8 | Human keyboard | checks 1.11 (key repeat) and 1.13 (crash path) need a physical keyboard: S4e, on the owner's PC |

Inherited limits that S4d keeps **unchanged** (S3 driver 63–82; S4 plan §4; S4c §3.3):
- tilt > 0.10 rad, or z < 0.045 m, each in 2 fresh samples;
- per-cycle drift > 20 mm lateral or > 0.10 rad yaw (stop);
- a joint within 0.02 rad of its URDF limit;
- tracking > 0.05 rad (node abort; > 0.02 rad is a finding);
- the ownership, freshness, stall, RTF (node watchdog backstop at RTF 0.1), timeout and driver-error protections;
- per-cycle stop bands: level 1 [0, 60] mm forward; level 2 [0, 80] mm forward and [−80, 0] mm reverse;
- per-cycle acceptance bands:
  - level 1 [30, 50] mm; level 2 [50, 70] and [−70, −50] mm;
  - |lateral| ≤ 10 mm and |yaw| ≤ 0.05 rad per cycle;
  - height within ±10 mm of standing.

**The one inherited limit S4d cannot keep** is the travel cap: 0.15 m from the arm pose (S3 driver 70). It cannot accommodate 0.3 m of travel (D3).

## 3. What existing evidence shows (MEASURED)

| Behaviour | On contact (free base) | Level | Fixed base only |
|---|---|---|---|
| Forward cycles | S3: 6 cycles, 0.1201 m, 20.015–20.029 mm each. S4c: +40.052 mm (L1), +60.015 mm (L2) | 0, 1, 2 (one cycle each at 1 and 2) | S2: all levels |
| Reverse cycles | S4a −19.992 mm; S4c −39.967 mm (L1), −59.959 mm (L2) | 0, 1, 2 (one cycle each) | S2 |
| Multi-cycle walking | S3: 5 consecutive cycles | **0 only** | — |
| Key release mid-phase | S4b: detected 0.889 s and 0.818 s after the recorded release; READY in 2.82 s and 2.50 s of sim | **0 only** | — |
| Reversal at a phase boundary | S4b: 5→6 then 6→5 | **0 only** | S2 |
| `home` | S4b: 4 phases forward, `homed` in 8.23 s of sim | **0 only** | S2: stop + home, lease + re-arm + home |
| Speed change | S4c: level chosen **before arm**, adopted from rest with goal 1 (0 → L) | from rest only | S2: level keys at rest |
| Speed change **while walking** | never | — | — |
| `x` while walking | S3 (1 cancel, source inferred); S4a (planned, goal 19, 1 cancel, measured hold) | **0 only** | S2: estop, goal 59 cancelled |
| Close while walking | S4b: SIGKILL crash path, DISARMED at 1 in 1.79 s of sim | **0 only**, crash path | — |
| Close at READY | S4c L1/L2: `q` at READY, READY → DISARMED | 1, 2 | — |
| Reconnect never re-arms | unit and ROS tests (`test_m55_locomotion.py` 730, `test_m55_ros.py` 234); post-close stillness | — | — |
| Tracking | ≤ 0.00225 rad in the S4a–S4c live runs | 0–2 | S2 |
| RTF | live runs 0.306–0.418 overall (10 s windows 0.135–0.662); shadow runs 0.167–0.193 | — | S2: 0.70 |

**Level-0 evidence is not used as proof of level-2 behaviour.** Every row marked "0 only" is either re-measured at level 2 by S4d (§5) or listed as a limitation (§7).

## 4. Facts from the code that constrain the script
1. **`+`/`-` are accepted at any time** (`m55_teleop.py` 87–92).
   - The teleop changes its own level and prints `speed level N/3: X mm/s`.
   - While a motion key is held, the next heartbeat carries the new speed.
   - The "level key refused while armed" rule was only the S4c driver's own restriction.
2. **The node adopts a new level only at boundary 0** (`m55_locomotion.py` 1002–1005 while walking, 1061–1064 from READY).
   - At a mid-cycle boundary it keeps the current level.
   - Walking through boundary 0 with a different intent level logs one `level_change` and dispatches the next cycle at the new level, without stopping.
3. **Reversal**: at the next phase boundary, `_after_rest` dispatches in the intent's direction (1006), which plays the same phase back. `PhaseLibrary.goal(level, 0, −1)` is 9→8 (442–447).
4. **`home`** (889–903):
   - It is accepted only in READY. At boundary 0 it is accepted with no motion.
   - From b ≥ 5 it walks forward, `P − b` phases; from b ≤ 4 it walks backward, `b` phases.
   - It ignores motion keys and obeys stop, estop, faults and the lease.
   - At most 4 phases, and at most 12.0 s of sim at 60 mm (M5.5 §6).
5. **Cycle count** (965): +1 on arriving at boundary 9 forward, −1 on arriving at boundary 0 backward. A backward home from b ≤ 4 therefore counts −1 although no full reverse cycle was walked.
6. **Release** (1050–1051): the node goes to STOPPING (`intent_stop`) once the teleop's hold window has lapsed, then READY (`stopped`, 992) at the end of the phase. Detection takes 0.8–0.95 s wall (M5.5 §16.1). The S4b scorer expects STOPPING at least H − 1/30 s − T = 0.717 s after the recorded release; above 0.983 s is a finding.
7. **Space** calls the `stop` service, handled on arrival: STOPPING (`stop`), then READY (`stopped`). The stop latch clears on the first zero command (1059–1060).
8. **`q` while moving** (teleop 222–225, 236–247):
   - The teleop issues stop then disarm, and stop then disarm again in its shutdown sequence. All are asynchronous calls to two services; the order the node records can differ (S4c L2 live recorded disarm, stop, disarm, stop).
   - The node enters STOPPING on the first request (`stop` or `disarm`), with the disarm-after-stop flag set.
   - At the end of the in-flight phase it disarms with reason `lease_expired` if the lease has lapsed by then (line 990), otherwise `disarm`. The lease lapses 0.5 s after the teleop's last command, which is its final zero heartbeat. A phase lasting longer than that after the close will therefore normally end in DISARMED (`lease_expired`).
9. **`x`** (872, 666): FAULTED in the request handler; one cancel of the goal in flight, never retried; nothing further is sent.
   - Leaving FAULTED needs `reset` and `arm`; this plan never does either after an emergency.
   - After an estop, `disarm` is refused (`reset required`).
10. **Arm at neutral** (574–576) takes the level from this process's rest record, if any; a fresh process starts at level 0. Arm at a mid-cycle boundary needs the same (level, boundary) record (579–585).
11. **Durations** (sim, including the 0.2 s lead-in; M5.5 §5):

    | k | Phase | Level 1 | Level 2 |
    |---|---|---|---|
    | 0 | S1 shift | 1.8 | 1.9 |
    | 1 | W(LR) swing | 3.2 | 4.6 |
    | 2 | S2 shift | 1.2 | 0.8 |
    | 3 | W(LF) swing | 3.3 | 4.7 |
    | 4 | S3 shift | 3.1 | 3.2 |
    | 5 | W(RR) swing | 3.0 | 4.2 |
    | 6 | S4 shift | 1.2 | 0.8 |
    | 7 | W(RF) swing | 3.1 | 4.2 |
    | 8 | S5 shift | 1.2 | 1.2 |
    | | **Cycle** | **21.1** | **25.6** |

    Statuses arrive every 0.5 s wall (`status_hz` 2.0).

12. **Sway**: each shift adds an 8–17 mm lateral sway that S5 removes. Mid-cycle boundaries are therefore swayed, and only boundary 0 is a clean reference for displacement.

## 5. Proposed experiment: S4d-M (one launch)

### 5.1 Outline
1. **Settle.**
2. **Select level 1 before arm**, then **arm**.
3. **Speed change while walking (SC).** One level-1 cycle; `+` is pressed mid-cycle, and the node adopts level 2 at the cycle boundary without stopping.
4. **0.3 m forward (F).** Five level-2 cycles; space stops at boundary 0.
5. **0.3 m return (R).** Five level-2 reverse cycles; space stops at boundary 0.
6. **Exercises (X).** S4b's validated pattern at level 2:
   - release while walking;
   - reversal at a phase boundary, ending with a second release;
   - `home` (four phases).
7. **Close while moving (Q).** `q` while a goal is in flight.
8. **Observe** 10 s of sim DISARMED.
9. **Optional reconnect probe (P, D11).**

No estop: estop and close-while-moving are both terminal, so they do not share one launch without a re-arm (§7, D10).

### 5.2 Measurement frames (PROPOSED)
- **Body pose.** `T_world_body` from `/spiderx/sim/world_poses` (M5.5 §4): xy, yaw about world z, z and tilt. Body axes are +y forward and +x right (base_link, not REP-103).
- **Displacement of pose B relative to reference A**, as `s3_driver.body_delta`:
  - forward = (xy_B − xy_A) · f(A);
  - lateral = (xy_B − xy_A) · r(A);
  - dyaw = wrap(yaw_B − yaw_A) into (−π, π].
  - f(A) and r(A) are A's body +y and +x in world xy.
- **Boundary pose B_k.** The bag body pose at the rest instant after goal k completes.
  - The instant is goal k's `phase_done`, mapped to sim time through the recorded status alignment.
  - The pose is the bag sample nearest that instant, within 0.05 s of sim, and at rest: |Δxy| ≤ 0.5 mm over ±0.1 s of sim. Otherwise the pose is UNMEASURED.
  - At a stopped boundary, the first fresh READY sample is used, as in S4c.
  - **Displacements are taken only at boundary 0**, which is sway-free.
- **Measurements:**

  | Measurement | From → to | Frame | Acceptance |
  |---|---|---|---|
  | SC cycle | B_0 (arm) → B_9 | B_0 | level-1 band [30, 50] mm |
  | Each F cycle j (j = 1–5) | B_{9+9(j−1)} → B_{9+9j} | that cycle's start boundary | [50, 70] mm |
  | Each R cycle j (j = 1–5) | B_{54+9(j−1)} → B_{54+9j} | that cycle's start boundary | [−70, −50] mm |
  | **F leg** | B_9 (where level 2 is adopted) → B_54 | B_9 | D1 |
  | **R leg**, from the forward endpoint | B_54 → B_99 | B_54 | D1 |
  | **Out-and-back drift** | B_9 → each B_k at boundary 0, k = 9, 18, …, 99 | B_9 | D2 |
  | Net return (recorded only) | B_9 → B_99, and B_0 → B_99 | B_9, B_0 | none |
  | X composite cycle | B_99 → B_110 (`homed`) | B_99 | [50, 70] mm |
  | Q | B_110 → the DISARMED pose | — | recorded only (mid-cycle, swayed) |

- Every displacement is **measured from the bag, never from the node's planned positions**. Five nominal cycles are not taken as proof of 0.3 m.
- **Live stops are separate from acceptance.** They use the existing fresh-sample cycle check: the first fresh status after the node's cycle count changes. In continuous walking that sample can fall within the next phase's lead-in, so it serves only as a stop, never as acceptance.

### 5.3 Step table (derived from §4; to be confirmed on the bench)
All goals are at internal level 2 unless marked L1. "Goals" means the goals the node dispatches after this action and before the next scripted action.

| # | Key/action | Precondition | Level | Phase / boundary | Expected transition | Goals | Cancels | Verification |
|---|---|---|---|---|---|---|---|---|
| 0 | settle | DISARMED, teleop up, no goal | node 0 | b 0 | — | 0 | 0 | S3 settle: ≥ 10 s sim, z range ≤ 5 mm, tilt ≤ 0.10 rad, fresh ages, graph 1/0/0 |
| 1 | `+` | DISARMED, no goal, before arm | teleop 0→1 | b 0 | none (no service) | 0 | 0 | `speed level 2/3: 1.90 mm/s`, attributed to this write (S4c C2 rules) |
| 2 | `a` | DISARMED, neutral posture | node 0 | b 0 | DISARMED→READY `arm` | 0 | 0 | READY at 0, level 0 |
| 3 | hold `w` | READY at 0, latch clear | L1 | 0→1, 1→2 | READY→STARTING→WALKING; `level_change` 0→1 with goal 1 | 1–2 (L1) | 0 | goals 1–2 at L1 |
| 4 | `+` while walking | goal 2 (L1 1→2, 3.2 s) in flight | teleop 1→2; node stays L1 | 2→3 … 8→9, then at b 0: one `level_change` 1→2 **in WALKING**, then F | none, then continuous | 3–9 (L1), 10–54 (L2) | 0 | 3/3 at 2.34 mm/s attributed; intent level 2 in the next statuses; goals 3–9 still L1 (no mid-cycle adoption); no READY or STOPPING between goals 9 and 10; goal 10 at L2; cycles 0→1 at goal 9, then +1 at goals 18, 27, 36, 45, 54 |
| 5 | space | goal 54 (8→9, 1.2 s) in flight | L2 | → b 0 | WALKING→STOPPING `stop` → READY `stopped` | 0 | 0 | READY at 0 within 1.2 + 1.0 s sim; cycles 6; F cycle checked at a fresh sample before `s` |
| 6 | hold `s` | READY at 0, latch clear | L2 | 9→8 … 1→0 × 5 | READY→STARTING→WALKING | 55–99 | 0 | every goal reverse at L2; cycles −1 at goals 63, 72, 81, 90, 99 |
| 7 | space | goal 99 (1→0, 1.9 s) in flight | L2 | → b 0 | STOPPING `stop` → READY `stopped` | 0 | 0 | READY at 0 within 2.9 s sim; cycles 1; checked before the next key |
| 8 | hold `w` | READY at 0 | L2 | 0→1 … 4→5 | READY→STARTING→WALKING | 100–104 | 0 | — |
| 9 | **release** `w` (no space) | goal 104 (4→5, 3.2 s) in flight | L2 | → b 5 | STOPPING `intent_stop` → READY `stopped` | 0 | 0 | STOPPING ≥ 0.717 s wall after the recorded release (finding above 0.983 s); READY at 5 within 4.2 s sim; no extra goal |
| 10 | hold `w` | READY at 5 | L2 | 5→6 | READY→STARTING→WALKING | 105 | 0 | — |
| 11 | switch to `s` (S4b `switch_held_key`) | goal 105 (5→6, 4.2 s) in flight | L2 | at b 6: 6→5 | stays WALKING | 106 | 0 | 6→5 sent at boundary 6, with no STOPPING or READY between goals 105 and 106 |
| 12 | **release** `s` | goal 106 (6→5, 4.2 s) in flight | L2 | → b 5 | STOPPING `intent_stop` → READY `stopped` | 0 | 0 | as row 9; READY at 5 within 5.2 s sim |
| 13 | `h` | READY at 5 | L2 | 5→6, 6→7, 7→8, 8→9 (homing forward) | READY→STARTING→WALKING → READY `homed` | 107–110 | 0 | exactly 4 homing goals; `homed` at 0 within 12.0 s sim (planned 10.4 s); cycles 1→2; X composite cycle checked |
| 14 | hold `w` | READY at 0 | L2 | 0→1 (S1, 1.9 s) | READY→STARTING→WALKING | 111 | 0 | — |
| 15 | `q` | goal 111 in flight | L2 | → b 1 | → STOPPING (`stop` or `disarm`, whichever the node handles first) → DISARMED at 1 (`lease_expired` expected, §4.8) | 0 | 0 | requests after `q`: stop × 2 and disarm × 2, as counts (order recorded); DISARMED within 1.9 + 1.0 s sim; teleop exit 0 |
| 16 | observe | DISARMED | — | b 1 | none | 0 | 0 | 10 s of sim DISARMED with no goal; still within the S3 hold limits |
| 17 | P (optional, D11) | DISARMED; a second teleop: `+ +`, hold `w` 5 s wall, release, `q` | — | b 1 | none | 0 | 0 | no goal, no arm, DISARMED throughout; the probe's close requests are accepted no-ops (counted) |

- **Keys:** exactly `+ a w + ' ' s ' ' w w s h w q`, plus two releases. A release stops the emulated repeat and writes nothing.
- **Requests before `q`:** exactly arm, stop, stop, home.
- **Budget: 111 goals, 0 planned cancels.** That is SC 9 + F 45 + R 45 + X 11 + Q 1. The 0.3 m segment alone is the planned 90 (F + R).
- **Cycle count:** 0, then 1 (goal 9), 6 (goal 54), 1 (goal 99), 2 (goal 110). It does not change at goal 111.
- **Nominal positions** along the arm pose's +y: +40 mm (B_9), +340 mm (B_54), +40 mm (B_99), +100 mm (B_110), about +115 mm at the end. These are planned values, never acceptance values.

### 5.4 Timing margins (no accidental extra goals)
Each in-flight action must reach the node before its phase ends, or the node dispatches the next phase. Status lag is ≤ 0.5 s wall.

| Action | Phase | Latency to the node (wall) | Safe while RTF < |
|---|---|---|---|
| space (rows 5, 7) | 1.2 s / 1.9 s sim | status lag + service ≈ 0.55 s | ≈ 2.2 / 3.4 |
| release (rows 9, 12) | 3.2 s / 4.2 s sim | status lag + ≤ 0.983 s detection ≈ 1.5 s | ≈ 2.1 / 2.8 |
| switch (row 11) | 4.2 s sim | status lag + 0.2 s quiesce + heartbeat + tick ≈ 0.85 s | ≈ 4.9 |
| `q` (row 15) | 1.9 s sim | ≈ 0.55 s | ≈ 3.4 |

Measured RTF has not exceeded 0.662 in any 10 s window. A late action produces an off-plan goal; the driver stops the run (`x`, or the estop service once the teleop is gone), and the attempt fails. It is never absorbed into the budget.

## 6. Criterion map
| Requirement | Existing evidence that legitimately covers it | New S4d measurement | Deferred (S4e, human keyboard) |
|---|---|---|---|
| R1 0.3 m forward at 3/3, then back | none at 0.3 m; single L2 cycles (S4c) | F and R legs (D1), per-cycle bands, out-and-back drift (D2) | — |
| R1 no unplanned fault, tilt | per-run only | whole S4d-M run | — |
| R2 release while walking | level 0 only (S4b) | two level-2 releases (rows 9 and 12) | real key-up and repeat timing (1.11, 1.13) |
| R3 speed change | adoption from rest only (S4c); fixed base (S2) | L1→L2 while walking, adopted at the cycle boundary (row 4) | — |
| R3 reversal | level 0 (S4b); fixed base (S2) | level 2 at boundary 6 (row 11) | — |
| R3 `home` | level 0 (S4b); fixed base (S2) | level 2, 4 phases forward, `homed` at 0 (row 13) | — |
| R4 `x` while walking | **level 0 only** on contact (S3, S4a); fixed base (S2) | **not in S4d-M**; optional S4d-E at level 2 (§7, D10) | — |
| R5 close the teleop (check 1.9) | crash path at level 0 (S4b); close at READY at levels 1–2 (S4c, a different test) | clean close **while moving** at level 2 (row 15) | crash path on a real terminal (1.13) |
| R5 no motion until `a` | post-close and post-lease stillness (S4b, S4c); unit and ROS tests | 10 s observation; optional reconnect probe P (D11) | — |
| R6 evidence | — | bag, evidence directory, scorer output, restore report | owner-made screen recording, if wanted |
| R7 ordering | S4c L1 → L2 go/no-go | S4d separately authorized | — |
| R8 physical keyboard | none | none (the Cloud pseudo-terminal is not a person) | S4e |

**Not covered by S4d-M:**
- the crash-path close (SIGKILL) at level 2;
- an estop at level 2, unless S4d-E runs;
- a release whose detection window spans a phase boundary: the second goal R2 allows. The script avoids it by design, so it is not exercised.

## 7. Conflicts and the smallest split
1. **Estop vs close.** Both are terminal.
   - After `x` the node is FAULTED, and this plan forbids reset and re-arm.
   - After a clean close it is DISARMED. A later `x` from a new teleop would not be "while walking".
   - One launch can therefore include both only with a scripted re-arm after the close. That adds a second teleop, a mid-cycle arm on a rest record, and (unless more goals are added) a cancel during a swing.
   - **Recommendation:** keep S4d-M estop-free, and propose a separately authorized **S4d-E** after S4d-M is reviewed:
     - settle; `+ +` (2/3, 3/3); `a`; hold `w`;
     - goal 1 = level-2 0→1, the S1 four-foot shift, mirroring S4a;
     - `x` once goal 1 has at least one feedback sample;
     - FAULTED (`estop`) in the request's tick, one cancel, no further goal;
     - 10 s of sim hold measured; `q`, after which disarm is refused (`reset required`), as expected.
   - **Budget: 1 goal, 1 cancel.**
   - **Without S4d-E**, the estop criterion rests on level-0 contact evidence (S3, S4a) and fixed-base evidence (S2). The cancel-and-hold of a 60 mm phase on contact stays undemonstrated, and the S4 claim must say so.
2. **Reversal at the turnaround** (alternative to rows 5–6, 10–12; D14).
   - Switching `w` to `s` while goal 54 (8→9) is in flight would reverse at boundary 0 without stopping, saving 2 goals (109).
   - The margin depends on the 1.2 s S5 phase (safe only below RTF ≈ 1.4).
   - The forward endpoint would no longer be a stopped, fresh-sample-checked READY.
   - Not recommended.
3. **Backward home** (alternative to rows 8–13; D15).
   - X can shrink to 4–8 goals with a backward home from b ≤ 4.
   - The node then counts −1 on arriving at 0 (§4.5) for a net-zero round trip. That needs a new net-zero tolerance and special cycle-check handling.
   - Not recommended.

The requirements fit **one bounded launch (S4d-M, 111 goals)**, with the level-2 estop as the one item that needs a separate run.

## 8. Proposed acceptance for a future `score_s4d.py` (D-checks)
Missing evidence is UNMEASURED. A contradiction FAILs. Overall PASS needs every check PASS; nothing changes after a run.
- **D1 Plan.**
  - Exactly the 111 goals of §5.3, in order, each SUCCEEDED (level, b_from, b_to, direction).
  - 0 cancels, faults or refusals before `q`.
  - The keys and pre-`q` requests exactly as in §5.3.
  - DISPATCH with the gate `True` in every status (live).
- **D2 Level and speed change.**
  - `+` #1 while DISARMED before arm; `+` #2 written while goal 2 is in flight.
  - Each confirmation is attributed by full-precision receipt to its own write: the receipt falls between that write's start and the next scripted key's write start. Repeat bytes are not scripted keys. Lines are rebuilt across output chunks.
  - Goals 1–9 at L1, including 3–9, which were dispatched after the key.
  - One `level_change` 0→1 before goal 1, and exactly one 1→2, between goal 9's `phase_done` and goal 10's `goal_sent`, in WALKING.
  - Goals 10–111 at L2.
- **D3 SC cycle.** Level-1 band and per-cycle drift (§5.2).
- **D4 Forward leg.**
  - F leg within D1.
  - Every F cycle within [50, 70] mm.
  - Space while goal 54 was in flight; READY at 0 within 2.2 s sim (120 s wall backstop); checked before `s`.
- **D5 Return leg.**
  - R leg within D1.
  - Every R cycle within [−70, −50] mm.
  - Space while goal 99 was in flight; READY within 2.9 s sim.
- **D6 Drift and attitude.**
  - Out-and-back lateral and heading within D2.
  - Every cycle: |lateral| ≤ 10 mm, |yaw| ≤ 0.05 rad; height within ±10 mm at every boundary 0.
  - Tilt ≤ 0.10 rad over all samples.
  - Net return recorded only.
- **D7 Releases.** Rows 9 and 12: released while the goal was in flight; STOPPING `intent_stop` per §4.6; READY at 5 within the goal's duration + 1.0 s sim; no extra goal.
- **D8 Reversal.** Row 11: the switch was written while goal 105 was in flight, and goal 106 = 6→5 followed with no stop between.
- **D9 Home.** Row 13: exactly 4 forward homing goals, with no motion key; READY `homed` at 0 within 12.0 s sim of the request; X composite cycle within [50, 70] mm.
- **D10 Close while moving.**
  - `q` was written while goal 111 was in flight.
  - stop × 2 and disarm × 2 after it, scored as counts with the order recorded.
  - One STOPPING (`stop` or `disarm`), then one DISARMED at 1 within 2.9 s sim (120 s wall backstop). Its reason is `lease_expired` if the phase ended more than 0.5 s wall after the teleop's last command, otherwise `disarm`; the scorer decides from recorded times, and any other reason FAILs.
  - Teleop exit 0.
  - No goal, key or other request after `q`, apart from P if approved.
  - 10 s of sim DISARMED and still.
- **D11 Tracking** (C6, unchanged): attributed feedback for every goal; max |error| ≤ 0.05 rad; above 0.02 rad is a finding.
- **D12 Continuity** (C7, unchanged).
- **D13 Ownership.** 111 goals and 0 cancels; the controller log shows 111 received, 111 accepted and 0 cancel requests; no foreign client or command publisher.
- **D14 Margins and freshness** (C9, unchanged), plus the driver result `script complete`.
- **D15 Restore** (C10) **and library** (C11), unchanged.
- **D16 Reconnect probe** (only if P is approved): no goal, no arm, DISARMED throughout.

**`--shadow`** drops the signed bands, D1 distances and D11 tracking (N/A). It requires zero goals, cancels and feedback at the controller. |progress| stays within the outer stop bounds, and the drift, budget and ownership checks remain. The overall reads SHADOW ONLY.

**S4d-E checks** (if chosen): exactly 1 goal and 1 cancel; FAULTED `estop` within 0.05 s wall of the request's handling; no further goal; the canceled-goal rule (D6 in §10); 10 s hold; refusals after `q` as expected; restore and library.

## 9. Live stops for S4d-M
- **Kept unchanged** (§2):
  - tilt and z;
  - per-cycle level- and direction-aware stop bands: SC [0, 60] mm; F and the X composite [0, 80] mm; R [−80, 0] mm;
  - per-cycle 20 mm and 0.10 rad drift;
  - joint margin, tracking, ownership, freshness, stall, RTF and driver errors;
  - any goal beyond or off the plan, or at the wrong level.
- **CHANGED, PROPOSED, flagged:** the travel cap (D3).
- **NEW, PROPOSED, flagged:**
  - a whole-script wall cap (D8);
  - optional whole-leg stops (D4; not recommended).
- **A stop is an emergency response.** The driver releases, sends `x` (or calls the estop service if the teleop has exited) and exits. The attempt ends; a stop is never suppressed to fit the budget, and nothing is retried.

## 10. Owner decisions (none approved here)
| # | Item | Recommended (PROPOSED) | Engineering basis | Alternatives | Consequence |
|---|---|---|---|---|---|
| D1 | **Distance acceptance** | F leg within [0.285, 0.315] m along B_9's +y; R leg within [−0.315, −0.285] m along B_54's +y; boundary-0 poses from the bag | The sources give no tolerance. ±5 % is far above the measured repeatability (≤ 0.06 mm per L2 cycle) yet tight enough to call the result 0.3 m | ±10 mm; strict ≥ 0.300 m (fails on −0.1 mm); per-cycle bands only (implied [0.25, 0.35] m) | Defines what "0.3 m" means in any S4 claim |
| D2 | **Cumulative drift** | lateral deviation ≤ 30 mm and heading drift ≤ 0.10 rad at every boundary 0 from B_9 to B_99, in B_9's frame; the maximum over all samples recorded only | One frame for the whole out-and-back reads R1 strictly; boundary 0 excludes the gait's designed 8–17 mm sway | per-leg frames (allows compounding to 60 mm); all samples (mixes sway with drift) | The pass/fail meaning of R1's drift |
| D3 | **Travel cap (CHANGES an existing limit)** | 0.40 m Euclidean from the arm pose, for S4d only (0.15 m elsewhere unchanged) | Nominal maximum 0.34 m plus one 60 mm stride; clearance ≥ ≈ 1.4 m to any obstacle (S4 plan §3) | 0.45 m; 0.36 m (may stop a run with long but in-band cycles); leg-relative caps | Without a change, S4d cannot run |
| D4 | **Per-cycle vs whole-leg limits** | per-cycle stops unchanged; no new whole-leg live stop (acceptance only, D2) | The Euclidean travel cap already bounds lateral excursion; fewer new thresholds | add whole-leg stops at 50 mm lateral or 0.15 rad heading from B_9 (new limits) | Earlier stop of a drifting run vs a new false-stop source |
| D5 | **Budget** | S4d-M: **exactly 111 goals, 0 planned cancels**. S4d-E (if chosen): **exactly 1 goal, 1 planned cancel** | §5.3; the 90-goal segment plus SC 9, X 11, Q 1 | 109 (D14 alternative); 102–106 (with the D15 alternative) | Fixes what the driver allows; any excess stops the run |
| D6 | **Tracking and canceled goals** | C6 unchanged for every S4d-M goal. S4d-E: `x` only after at least one attributed feedback sample of goal 1; those samples ≤ 0.05 rad (above 0.02 a finding); the hold measured by joint stillness; no feedback before the cancel = UNMEASURED | Keeps "feedback for every goal" honest without an N/A exception | S4a-style pre-specified N/A for the cancelled goal | Whether a cancelled goal can be scored at all |
| D7 | **Timing (sim vs wall)** | §4.6–§4.8 and §8: release STOPPING 0.717 s wall minimum (finding above 0.983 s); READY or DISARMED within the in-flight goal's duration + 1.0 s **sim**; 120 s **wall** backstop; `homed` within 12.0 s sim; teleop exit within 15 s wall; S4d-E FAULTED within 0.05 s wall | M5.5 1.7, 1.9 and §16.1; S4b and S4c rules. No wall-time bound on motion without an RTF assumption | R2's two-goal allowance (≤ 10.4 s sim) | Stricter than R2, consistent with an exact budget |
| D8 | **RTF waits, duration, watchdog** | each wait = sim cap / 0.1 + 60 s wall; whole script ≤ 90 min wall; watchdog 7200 s from arming (before enabling); the node's RTF 0.1 backstop unchanged | Script ≈ 340 s sim (§11): 57 min at RTF 0.1. With 10–13 min of enabling, launch and checks before the script, the established 3600 s watchdog is too short below RTF ≈ 0.12 | 5400 s watchdog with a lower script cap; abort the run if the measured RTF over 60 s falls below 0.12 (new) | Too short: a healthy slow run is cut off; too long: a slower response after an interruption |
| D9 | **S4e** | separate, optional, owner-run on a physical keyboard (1.11, 1.13); not required for simulation acceptance unless the owner decides otherwise (S4 plan §6 item 4) | The pseudo-terminal cannot show human key repeat or reaction | require S4e for "full S4" | Scope of any "full S4" claim |
| D10 | **Estop at level 2** | separate S4d-E after S4d-M review (§7.1) | The only way to show the level-2 cancel-and-hold on contact without a re-arm | accept the level-0 limitation; single launch with a scripted re-arm after the close (more risk and complexity) | Whether R4 is claimed at the fastest level |
| D11 | **Reconnect probe P** | include (0 goals, about 1 min) | Direct contact evidence for "no motion until `a`" | rely on the 10 s observation and the unit and ROS tests | One more teleop session in the driver |
| D12 | **Build procedure** (§13) | B1: a **clean enabled build in new colcon bases**, sourced by the harness; nothing deleted | A clean build without a deletion; the existing installation untouched | B2: the established incremental build, explicitly not clean (as S4c) | Whether S4d can be called a clean enabled-build run |
| D13 | **Speed-change design** | L1→L2 while walking, before F (9 goals, 21.1 s sim) | Adjacent levels; F starts at the adoption boundary; S4c-validated L1 plant behaviour | L0→L2 (two `+` while walking; 18.8 s sim); a down-change after R (extends travel) | Which levels the speed-change evidence covers |
| D14 | **Reversal placement** | mid-cycle in X (5→6, then 6→5) | S4b pattern; 4.2 s phases; stopped forward endpoint | turnaround at the end of F (−2 goals; thinner margin; rolling endpoint) | Robustness vs 2 goals |
| D15 | **Home composite** | forward 4-phase home (X = 11 goals, cycle count +1) | S4b pattern; reuses the approved [50, 70] mm band | backward home (X = 4–8; new net-zero tolerance; −1 count handling) | Fewer goals vs new rules |
| D16 | **Disk and bag** | at least 5 GB free before each launch; the bag stays outside the repository | Expected bag 0.3–1.8 GB (25–30 MB per wall minute measured); 21 GB free now | none | A full disk ends recording mid-run |

## 11. Duration, RTF and resources (PROPOSED estimates)
- **Sim time:**

  | Part | Sim time |
  |---|---|
  | SC | 21.1 s |
  | F | 128.0 s |
  | R | 128.0 s |
  | X | 34.0 s (15.2 + 4.2 + 4.2 + 10.4) |
  | Q | 1.9 s |
  | **Motion** | **313.0 s** |
  | Total, with settle, the 10 s observation and READY checks | ≈ 335–345 s |

- **Script, wall:**

  | RTF | Wall time |
  |---|---|
  | 0.6 | ≈ 9.5 min |
  | 0.3 (typical live) | ≈ 19 min |
  | 0.17 (typical shadow) | ≈ 33 min |
  | 0.1 (floor) | ≈ 57 min |

  Add about 1 min for result latency over 111 goals.
- **Session, wall:**
  - enabling, build and M5.5 tests ≈ 5–8 min;
  - launch ≈ 2 min; pre-arm checks ≈ 2 min; stop and restore ≈ 3 min;
  - live total ≈ 32–35 min at RTF 0.3, ≈ 47–50 min at 0.17, ≈ 70–75 min at 0.1;
  - the shadow rehearsal is similar at RTF 0.17–0.19.
- **Risk.**
  - Measured 10 s RTF windows reached 0.135 with the GUI running (reniced +19).
  - A long run has more exposure to a dip below the node's 0.1 backstop. That would be a fault, and the attempt ends safely.
  - Reducing host load (for example the GUI) is a launch change, outside this plan.
- **S4d-E:** ≈ 2 s of motion; session ≈ 20–25 min.

## 12. Offline implementation contract (future; separately authorized; no runtime)
1. **`scripts/s4/s4d_driver.py`** (new).
   - Subclasses the S4a/S4b/S4c drivers. It reuses:
     - the S3 fresh samples, debounce, settle and `body_delta`;
     - the S4a full-precision key-write brackets;
     - the S4b `release`, `switch_held_key` and home helpers;
     - the S4c chunk-receipt confirmation attribution and close handling.
   - A plan table `PLANNED[1..111]` of (level, b_from, b_to, direction). Every status goal must match its entry; the level may change only at the step-4 boundary.
   - Live checks per §9, with level- and direction-aware cycle stops and the D3 cap.
   - Sim-based wait caps (D8).
   - **On any failure:** release, then `x` or the estop service, then a clean exit with S3 exit codes. Cleanup always runs.
   - `--shadow-rehearsal` fails closed on the gates, as S3/S4c.
   - **S4d-E** is a separate mode or a separate small driver with its own exact plan.
2. **`scripts/s4/score_s4d.py`** (new). D1–D16 per §8 with PASS, FAIL or UNMEASURED:
   - boundary poses from the bag (§5.2); never planned positions;
   - the bag opened read-only, with its hash checked before and after.
3. **Bench tests** (in-memory `LocomotionSession` bench, as `test_s4c_driver_live.py`):
   - the complete script at RTF 0.1–1.0: 111 goals, 0 cancels;
   - adoption only at the boundary (a patched early adoption must stop);
   - late space, release, switch or `q` producing an extra goal must stop;
   - both close-request orders;
   - `lease_expired` vs `disarm` reasons;
   - split output chunks; duplicate statuses;
   - drift, travel, tilt and joint stops;
   - missing confirmations.
4. **Scorer tests:** bench runs pass; every check FAILs under its mutation; missing evidence is UNMEASURED; band edges are inclusive; shadow and live runs are never scored as each other.
5. **Restore support.**
   - Add only `local/m55-s4d-m-*` and `local/m55-s4d-e-*` to `scripts/m61_restore_and_verify_disabled.sh` (42–43, 274–290).
   - Tests in `src/spiderx_controller/test/test_m61_restore_script.py`.
6. **Run-local harness** (not in the repository): from the S4c L2 live harness, with:
   - S4d input checks; the D16 disk check; the D8 watchdog;
   - the D12 build procedure;
   - the same capture set: driver log and summary, key brackets, teleop receipts, node events, launch, controller and recorder logs, before/after graph, harness hashes, disabled library, restore report;
   - one bag outside the repository, without `_action/status` or `/clock`.
7. **Lessons carried forward** (already verified):
   - full-precision timestamps before and after every key write and repeat stop;
   - teleop output rebuilt across chunks with receipt times;
   - each `+` attributed to exactly one confirmation;
   - asynchronous service calls: order recorded, scored as counts;
   - the S3 fresh-sample definition;
   - motion measured independently from the bag;
   - cleanup and restore on every exit path.

No runtime source, threshold, gain, trajectory or existing scorer changes.

## 13. Build and restore procedure (to be fixed before any run)
1. **Disabled validation.**
   - The new driver, scorer, tests and restore script change executable inputs. So, before the S4d-M shadow rehearsal, run **one fresh disabled build and the full suite in new, uniquely named colcon bases** (the restoration follow-up's procedure), with nothing deleted, plus the direct `scripts/s4` tests.
   - Reuse that result for later runs while `src/`, `scripts/` and `third_party/` stay unchanged.
2. **Enabled build (D12).**
   - **B1**: build the local enabling branch in new bases from a toolchain-only shell, with st59site on `PYTHONPATH` for the build only. The harness sources that installation (after the r2 overlay); input checks verify its paths, gates (M55 `True`; M61 and M6D `False`) and library identity.
   - **B2**: the established incremental `colcon build --symlink-install` in the workspace, recorded as not clean.
   - Neither deletes `build/`, `install/` or `log/`.
3. **Restore, fixed before enabling and never changed after the launch.**
   - `scripts/m61_restore_and_verify_disabled.sh --base <disabled baseline> --enabling-ref local/m55-s4d-m-<UTC> --delete-enabling-branch`, with the established toolchain-only incremental rebuild of the workspace installation and the run's pid files.
   - The branch is deleted only after DISABLED AND VERIFIED.
   - With B1, also record that the fresh enabled installation now imports the gates `False`: its modules are symlinks into the restored `src/`. That installation is kept, not deleted.
4. **Historical caveats** (S4c L1 and L2 incremental builds, the restoration follow-up) stay unchanged.

## 14. Authorization boundaries (each step needs its own approval; none is granted here)
1. The owner decides D1–D16.
2. Offline implementation on a new branch (§12), reviewed. No launch.
3. Fresh disabled build and full suite in new bases (§13.1). No launch.
4. One S4d-M **shadow** rehearsal (one launch, SHADOW, no goal at the controller).
5. One S4d-M **live** attempt, only after the shadow result is reviewed.
6. Optional S4d-E shadow, then live, only after the S4d-M review (D10).
7. Optional S4e on the owner's PC (D9).

No step continues automatically into the next. There is no retry, recovery or second launch inside an attempt. Enabling branches are never pushed. The gates stay `False` on every pushed commit.

## 15. Claim boundaries
A passing S4d-M would support this claim only: keyboard-path crawl walking in simulation.
- Automated input through the teleop.
- About 0.3 m forward and back at internal level 2, measured.
- Release, speed change, reversal, `home` and a clean close while moving, at level 2, within the stated limits.
- Idealized actuator model, CAD masses, assumed contact.

It would not support:
- human keyboard operation (S4e);
- hardware, rough terrain, odometry or navigation;
- dynamic stability or unrestricted walking;
- the level-2 estop, without S4d-E;
- the level-2 crash-path close.
