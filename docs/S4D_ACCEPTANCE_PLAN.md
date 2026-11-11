# S4d: simulation acceptance at internal level 2 (S4d-M): design, owner decisions and offline contract

**STATUS (2026-11-11): S4d-M PASSED one simulation acceptance attempt at internal level 2 (M1–M16; §16). S4d-E PASSED one live simulation attempt at internal level 2 under its frozen contract (E1–E12; §17). Its cancel came before measurable motion.**

History: on 2026-11-09 this header read **OWNER DECISIONS OD1–OD16 RECORDED (2026-11-09) — OFFLINE S4d-M PREPARATION ONLY — NO SHADOW OR LIVE RUN AUTHORIZED.** Earlier on 2026-11-11, before the S4d-E attempt, its second sentence read **S4d-E is not validated, and a live S4d-E attempt is not authorized.**

Draft: 2026-11-03, commit `34396cf` on `claude/spiderx-s4d-plan`. Amended 2026-11-09 on `claude/spiderx-s4d-preparation` with the owner's decisions (§10).

The decisions authorize only offline work: the S4d-M driver, scorer and bench tests, and the narrow `local/m55-s4d-m-*` restore support (§12). They authorize no build, enabling patch or branch, simulator launch, ROS graph, goal, cancel, S4d-E work, hardware run or merge (§14).

As written on 2026-11-09: nothing here changes historical evidence, README or STATUS verdicts, runtime code, the phase library, existing scorers, shared constants or the dispatch gates, and **S4d had not been run and had not passed**. The outcome since then is recorded in §16; the criteria of this document are unchanged.

Identifiers are kept distinct:
- **R1–R8**: existing requirements (§2).
- **OD1–OD16**: owner decisions (§10).
- **M1–M16**: S4d-M scorer checks (§8).
- **LS1–LS10**: S4d-M live stops (§9).

Three kinds of statement are kept apart:
- **REQUIRED**: an existing requirement, with its source.
- **MEASURED**: a result from committed evidence, with its level and plant.
- **DECIDED** (OD) or **PROPOSED**: new in this document. DECIDED items are owner decisions; PROPOSED items are implementation details.

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
- Code (line numbers at `40f5f2c`; unchanged at `34396cf`):
  - `src/spiderx_controller/spiderx_controller/m55_locomotion.py`:
    - `PhaseLibrary.goal` 442;
    - `resolve_arm_posture` 551;
    - `_fault` 666, `_lease` 688–696, `on_command` 698;
    - `_arm` 811, `_disarm` 851, `_stop` 863, `_estop` 872, `_home` 889–903;
    - `_to_disarmed` 905 (it clears the lease), `_begin_stop` 913;
    - cycle counting 965;
    - `_after_rest` 971–1009 (the disarm reason at 990);
    - `step` 1027–1051 (`intent_stop` at 1051);
    - `_ready_tick` 1053–1065;
    - `status` 1146–1194: `intent` 1176, `lease_age_s` 1179, `commands_accepted` 1180, goal `elapsed_s` 1184.
  - `src/spiderx_controller/spiderx_controller/m55_command.py`: `CommandLease.on_command` 144–159 (a zero command is valid and renews the lease), `current` 161.
  - `src/spiderx_controller/spiderx_controller/m55_rclpy.py`: a status is published on every state change, in the same callback (259–262), on a single-threaded executor (332); `events.jsonl` carries each event's node wall time.
  - `src/spiderx_controller/spiderx_controller/m55_teleop.py`:
    - `TeleopState.key` 68–101 (`+`/`-` at 87);
    - `twist` hold window 103–111;
    - `handle` QUIT 222–225;
    - `_shutdown_sequence` 236–247 (a zero heartbeat at 240);
    - the main loop's heartbeat 300.
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
  - `s4c_l2_shadow_20261103T094244Z`, `s4c_l2_live_20261103T111733Z`;
  - bag sizes (each README) and recording durations (`run/bag_metadata.yaml`) of the eight S4a–S4c shadow and live bags (OD16).

## 2. Existing requirements (REQUIRED, unchanged)
From M5.5 §10 phase 4, the PATH S4 row and S4 plan §2:

| # | Requirement | Bound as written |
|---|---|---|
| R1 | Forward 0.3 m at the fastest level (3/3), then back | no unplanned fault; tilt ≤ 0.10 rad; lateral deviation ≤ 30 mm; heading drift ≤ 0.1 rad. **No distance tolerance is given**; OD1 selects one |
| R2 | Key release while walking | READY within the goals still to finish + 1.0 s of sim, as 1.7: the in-flight goal, and the next one if a boundary fell within the 0.95 s wall detection. That is ≤ 2 × 4.7 + 1.0 s at 60 mm; wall backstop 120 s. OD7 applies the one-goal bound; the two-goal allowance is not exercised |
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

**The one inherited limit S4d cannot keep** is the travel cap: 0.15 m from the arm pose (S3 driver 70). It cannot accommodate 0.3 m of travel. OD3 replaces it **for S4d only**, in S4d tooling; the shared 0.15 m constant and every earlier stage are unchanged.

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
| Bag size per wall minute | 27.0–30.8 MB in the eight S4a–S4c shadow and live bags (README size ÷ metadata duration; 132.8–441.8 s recordings) | at overall RTF 0.17–0.42 only | — |

**Level-0 evidence is not used as proof of level-2 behaviour.** Every row marked "0 only" is either re-measured at level 2 by S4d (§5) or listed as a limitation (§7).

## 4. Facts from the code that constrain the script
1. **`+`/`-` are accepted at any time** (`m55_teleop.py` 87–92).
   - The teleop changes its own level and prints `speed level N/3: X mm/s`.
   - While a motion key is held, the next heartbeat carries the new speed.
   - The "level key refused while armed" rule was only the S4c driver's own restriction. S4d does not apply it to the speed change (row 4).
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
8. **`q` while moving** (teleop 222–225, 236–247, 300; node 851–870, 990, 1051):
   - The teleop sets its motion to zero, then calls stop and disarm. Its main loop may publish one zero heartbeat on the way out (300). Its shutdown sequence always publishes one zero heartbeat (240), then calls stop and disarm again, waits at most 1 s for the replies and exits.
   - All four calls are asynchronous; the order the node records can differ (S4c L2 live recorded disarm, stop, disarm, stop).
   - The node enters STOPPING on whichever input it handles first:
     - the stop request (`stop`);
     - the disarm request (`disarm`), which also sets the disarm-after-stop flag;
     - or, if a zero heartbeat is handled and a 20 Hz tick runs before either request, the intent stop (`intent_stop`, 1051).
   - A request handled in STOPPING changes only the flags: stop sets the stop latch, disarm sets disarm-after-stop.
   - The zero heartbeat is a valid command. It renews the lease (`m55_command.py` 144–159). The teleop's last command is therefore its shutdown zero heartbeat.
   - At the end of the in-flight phase (`_after_rest`, once a fresh joint sample has arrived), the node disarms (990). The reason is `lease_expired` if more than 0.5 s of **the node's wall clock** has passed since it received its last valid command; otherwise it is `disarm`.
   - That decision uses only the node's receipt time of the last valid command and the node time of `_after_rest`. Neither the key-write time, the teleop's exit, nor sim time determines it. A close early in a long phase normally ends in `lease_expired`; a close within 0.5 s wall of the phase end ends in `disarm`.
   - If the disarm request were handled only after the phase ended, the node would go READY (`stopped`) first. The script's timing excludes that (§5.4); the scorer FAILs it (M10).
   - `_to_disarmed` clears the lease (905–906). After DISARMED, the status's `lease_age_s` no longer shows the close's last command.
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
13. **Commands while DISARMED** (698, 1027–1051; `m55_command.py` 144–159):
    - Every valid `/cmd_vel` message is counted (`commands_accepted`), renews the lease and sets the intent, whatever the state.
    - In DISARMED, `step` neither dispatches nor reacts to the intent or to lease expiry; only `arm` leaves DISARMED.
    - `stop` in DISARMED is accepted: "not moving (DISARMED); held until a zero command". `disarm` in DISARMED is accepted: "already disarmed". Neither is a refusal.
    - A held motion key at the teleop's selected level is a valid command, so **no rejection is expected**. The node rejects a command only for non-finite values, lateral or yaw components, or the receive rate (more than 20 Hz over 1 s, or more than 4 in 0.1 s); the teleop sends none of these at its 10 Hz heartbeat.

## 5. The experiment: S4d-M (one launch)

### 5.1 Outline
1. **Settle.**
2. **Select level 1 before arm**, then **arm**.
3. **Speed change while walking (SC; OD13).** One level-1 cycle; `+` is pressed mid-cycle, and the node adopts level 2 at the cycle boundary without stopping.
4. **0.3 m forward (F).** Five level-2 cycles; space stops at boundary 0.
5. **0.3 m return (R).** Five level-2 reverse cycles; space stops at boundary 0.
6. **Exercises (X).** S4b's validated pattern at level 2:
   - release while walking;
   - reversal at a phase boundary, ending with a second release (OD14);
   - `home` (four phases forward; OD15).
7. **Close while moving (Q).** `q` while a goal is in flight.
8. **Observe** 10 s of sim DISARMED.
9. **Reconnect probe (P; OD11).** Zero goals.

No estop: estop and close-while-moving are both terminal, so they do not share one launch without a re-arm (§7). The level-2 estop is S4d-E, separately reviewed and not part of this work (OD10).

### 5.2 Measurement frames
- **Body pose.** `T_world_body` from `/spiderx/sim/world_poses` (M5.5 §4): xy, yaw about world z, z and tilt. Body axes are +y forward and +x right (base_link, not REP-103).
- **Displacement of pose B relative to reference A**, as `s3_driver.body_delta`:
  - forward = (xy_B − xy_A) · f(A);
  - lateral = (xy_B − xy_A) · r(A);
  - dyaw = wrap(yaw_B − yaw_A) into (−π, π].
  - f(A) and r(A) are A's body +y and +x in world xy.
- **Boundary pose B_k.** The bag body pose at the rest instant after goal k completes.
  - The instant is goal k's `phase_done` (node clock), mapped to bag time through the recorded status alignment and to sim time through the joint-state stamps.
  - The pose is the bag sample nearest that instant, within 0.05 s of sim, and at rest: |Δxy| ≤ 0.5 mm over ±0.1 s of sim. Otherwise the pose is UNMEASURED.
  - B_0 is the pose at the arm transition.
  - **Alignment.** The offset is the median of the paired status receipts (as S2–S4c). Its uncertainty is the larger of the |5th| and |95th| percentile deviations. A missing alignment, or an uncertainty above 0.05 s, makes every bag-based measurement UNMEASURED (PROPOSED).
  - **Displacements are taken only at boundary 0**, which is sway-free.
- **Measurements:**

  | Measurement | From → to | Frame | Acceptance |
  |---|---|---|---|
  | SC cycle | B_0 (arm) → B_9 | B_0 | level-1 band [30, 50] mm |
  | Each F cycle j (j = 1–5) | B_{9+9(j−1)} → B_{9+9j} | that cycle's start boundary | [50, 70] mm |
  | Each R cycle j (j = 1–5) | B_{54+9(j−1)} → B_{54+9j} | that cycle's start boundary | [−70, −50] mm |
  | **F leg** | B_9 (where level 2 is adopted) → B_54 | B_9 | OD1: [0.285, 0.315] m |
  | **R leg**, from the forward endpoint | B_54 → B_99 | B_54 | OD1: [−0.315, −0.285] m |
  | **Cumulative drift** | B_9 → each B_k at boundary 0, k = 9, 18, …, 99 | B_9 | OD2: \|lateral\| ≤ 30 mm, \|heading\| ≤ 0.10 rad |
  | All-sample maxima (recorded only) | B_9 → every bag sample from B_9 to B_99 | B_9 | none; includes the designed sway |
  | Net return (recorded only) | B_9 → B_99, and B_0 → B_99 | B_9, B_0 | none |
  | X composite cycle | B_99 → B_110 (`homed`) | B_99 | [50, 70] mm |
  | Q | B_110 → the DISARMED pose | — | recorded only (mid-cycle, swayed) |

- Every displacement is **measured from the bag, never from the node's planned positions**. Five nominal cycles are not taken as proof of 0.3 m.
- Acceptance bands are inclusive at both ends.
- **Live stops are separate from acceptance** (§9). They use the existing fresh-sample cycle check: the first fresh status after the node's cycle count changes. In continuous walking that sample can fall within the next phase's lead-in, or after it, so it serves only as a stop, never as acceptance.

### 5.3 Step table (derived from §4; exercised by the bench tests of §12.3)
All goals are at internal level 2 unless marked L1. "Goals" means the goals the node dispatches after this action and before the next scripted action. Plan entries are (level, b_from, b_to, direction).

| # | Key/action | Precondition | Level | Phase / boundary | Expected transition | Goals | Cancels | Verification |
|---|---|---|---|---|---|---|---|---|
| 0 | settle | DISARMED, teleop up, no goal | node 0 | b 0 | — | 0 | 0 | S3 settle: ≥ 10 s sim, z range ≤ 5 mm, tilt ≤ 0.10 rad, fresh ages, graph 1/0/0 |
| 1 | `+` | DISARMED, no goal, before arm | teleop 0→1 | b 0 | none (no service) | 0 | 0 | `speed level 2/3: 1.90 mm/s`, attributed to this write (S4c C2 rules) |
| 2 | `a` | DISARMED, neutral posture | node 0 | b 0 | DISARMED→READY `arm` | 0 | 0 | READY at 0, level 0 |
| 3 | hold `w` | READY at 0, latch clear | L1 | 0→1, 1→2 | READY→STARTING→WALKING; `level_change` 0→1 with goal 1 | 1–2 (L1) | 0 | goals 1–2 at L1 |
| 4 | `+` while walking | goal 2 (L1 1→2, 3.2 s) in flight | teleop 1→2; node stays L1 | 2→3 … 8→9, then at b 0: one `level_change` 1→2 **in WALKING**, then F | none, then continuous | 3–9 (L1), 10–54 (L2) | 0 | 3/3 at 2.34 mm/s attributed; intent level 2 in the next statuses; goals 3–9 still L1 (no mid-cycle adoption); no READY or STOPPING from goal 1 to goal 54; goal 10 at L2; cycles 0→1 at goal 9, then +1 at goals 18, 27, 36, 45, 54 |
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
| 15 | repeat stopped; `q` 0.2 s later | goal 111 in flight; still inside the 0.8 s hold window | L2 | → b 1 | → STOPPING (`stop`, `disarm` or `intent_stop`, §4.8) → DISARMED at 1 (`lease_expired` or `disarm`, decided by the node clock, §4.8) | 0 | 0 | requests after `q`: stop × 2 and disarm × 2, as counts (order recorded); DISARMED within 1.9 + 1.0 s sim; the reason matches the node-clock evidence (M10); teleop exit 0 |
| 16 | observe | DISARMED | — | b 1 | none | 0 | 0 | 10 s of sim DISARMED with no goal; still within the S3 hold limits |
| 17 | P (OD11) | DISARMED; a second teleop: `+ +`, hold `w` 5 s wall, release, `q` | — | b 1 | none | 0 | 0 | §5.5: delivery shown, no arm, no goal, DISARMED throughout, no rejection; the close requests are accepted no-ops |

- **Keys:** exactly `+ a w + ' ' s ' ' w w s h w q`, then the probe's `+ + w q`. Besides keys: four releases (after each space, and rows 9 and 12), the switch (row 11) and the repeat stop before `q`. A release or repeat stop stops the emulated repeat and writes nothing.
- **Requests before `q`:** exactly arm, stop, stop, home.
- **Budget (OD5): 111 goals, 0 planned cancels.** That is SC 9 + F 45 + R 45 + X 11 + Q 1. The 0.3 m segment alone is the planned 90 (F + R). An emergency response always overrides the budget and ends the attempt.
- **Cycle count:** 0, then 1 (goal 9), 6 (goal 54), 1 (goal 99), 2 (goal 110). It does not change at goal 111.
- **Nominal positions** along the arm pose's +y: +40 mm (B_9), +340 mm (B_54), +40 mm (B_99), +100 mm (B_110), about +107 mm at the end. These are planned values, never acceptance values.
- **Travel envelope (OD3)**, Euclidean XY from the arm pose:
  - nominal maximum 0.340 m, at B_54;
  - largest accepted: SC 50 mm + F 315 mm = 0.365 m along +y, with up to 30 mm cumulative lateral: 0.366 m. The designed sway adds at most 17 mm mid-cycle (0.368 m);
  - R returns to B_99 at +0 … +80 mm; X adds 50–70 mm (B_110 at +50 … +150 mm); Q adds one S1 shift;
  - the S4d-only cap is 0.40 m, at least 32 mm above the largest accepted pose. Every obstacle is at least about 1.4 m from the spawn (S4 plan §3, clearance).

### 5.4 Timing margins (no accidental extra goals)
Each in-flight action must reach the node before its phase ends, or the node dispatches the next phase. Status lag is ≤ 0.5 s wall.

| Action | Phase | Latency to the node (wall) | Safe while RTF < |
|---|---|---|---|
| `+` (row 4) | 3.2 s sim (goal 2) | status lag + write ≈ 0.55 s; adoption waits for boundary 0 anyway | ≈ 5.8 (to be written inside goal 2) |
| space (rows 5, 7) | 1.2 s / 1.9 s sim | status lag + service ≈ 0.55 s | ≈ 2.2 / 3.4 |
| release (rows 9, 12) | 3.2 s / 4.2 s sim | status lag + ≤ 0.983 s detection ≈ 1.5 s | ≈ 2.1 / 2.8 |
| switch (row 11) | 4.2 s sim | status lag + 0.2 s quiesce + heartbeat + tick ≈ 0.85 s | ≈ 4.9 |
| `q` (row 15) | 1.9 s sim | status lag + 0.2 s quiesce + service ≈ 0.75 s | ≈ 2.5 |

Measured RTF has not exceeded 0.662 in any 10 s window. A late action produces an off-plan goal; the driver stops the run (`x`, or the estop service once the teleop has read `q`), and the attempt fails. It is never absorbed into the budget.

### 5.5 The reconnect probe (OD11)
After the 10 s observation, the driver starts a second teleop on the DISARMED node. It never presses `a`.

| Step | Bound (wall, driver clock) | Evidence |
|---|---|---|
| Startup | ≤ 30 s from the spawn | the teleop's own status line (`state DISARMED …`), received after the spawn, shows it has the node's levels; a status with `commands_accepted` above its pre-probe value shows the node is receiving its heartbeats |
| `+`, `+` | each confirmation ≤ 5 s after its write | `speed level 2/3`, then `3/3`, attributed by the S4c rules (cursor from the write, lines rebuilt across chunks, stale, duplicate or wrong lines stop) |
| Hold `w` | 5 s: the release 5.0–6.0 s after the write returned | delivery: a status received during the hold shows `intent` forward at level 2 with no lease reason, and `commands_accepted` rising |
| Release | intent back to stop within 5 s | a status with `intent.direction` 0 |
| `q` and exit | exit seen ≤ 15 s after the `q` write, code 0 | `teleop_closed` record |

Throughout the probe: every status is DISARMED with no goal; the node logs no `arm` request, goal, transition, refusal or `command_rejected` event, and `command_rejections` stays empty (§4.13). After the probe's `q`, the node logs exactly stop × 2 and disarm × 2, accepted as no-ops with the §4.13 messages. Any departure stops the driver (LS9) and FAILs M16. The probe's checks are separate from M10: the close's own window ends when the probe's teleop starts, and no probe event can satisfy or excuse a close check.

## 6. Criterion map
| Requirement | Existing evidence that legitimately covers it | New S4d measurement | Deferred (S4e, human keyboard) |
|---|---|---|---|
| R1 0.3 m forward at 3/3, then back | none at 0.3 m; single L2 cycles (S4c) | F and R legs (OD1), per-cycle bands, cumulative drift (OD2) | — |
| R1 no unplanned fault, tilt | per-run only | whole S4d-M run | — |
| R2 release while walking | level 0 only (S4b) | two level-2 releases (rows 9 and 12), one-goal bound (OD7) | real key-up and repeat timing (1.11, 1.13) |
| R3 speed change | adoption from rest only (S4c); fixed base (S2) | L1→L2 while walking, adopted at the cycle boundary (row 4; OD13) | — |
| R3 reversal | level 0 (S4b); fixed base (S2) | level 2 at boundary 6 (row 11; OD14) | — |
| R3 `home` | level 0 (S4b); fixed base (S2) | level 2, 4 phases forward, `homed` at 0 (row 13; OD15) | — |
| R4 `x` while walking | **level 0 only** on contact (S3, S4a); fixed base (S2) | **not in S4d-M**; S4d-E at level 2, separately reviewed (OD10) | — |
| R5 close the teleop (check 1.9) | crash path at level 0 (S4b); close at READY at levels 1–2 (S4c, a different test) | clean close **while moving** at level 2 (row 15) | crash path on a real terminal (1.13) |
| R5 no motion until `a` | post-close and post-lease stillness (S4b, S4c); unit and ROS tests | 10 s observation; reconnect probe P (OD11) | — |
| R6 evidence | — | bag, evidence directory, scorer output, restore report | owner-made screen recording, if wanted |
| R7 ordering | S4c L1 → L2 go/no-go | S4d separately authorized | — |
| R8 physical keyboard | none | none (the Cloud pseudo-terminal is not a person) | S4e (OD9) |

**Not covered by S4d-M:**
- the crash-path close (SIGKILL) at level 2;
- an estop at level 2 (S4d-E, OD10);
- a release whose detection window spans a phase boundary: the second goal R2 allows. The script avoids it by design, so the two-goal allowance is **not exercised** (OD7).

## 7. Conflicts and the smallest split
1. **Estop vs close.** Both are terminal.
   - After `x` the node is FAULTED, and this plan forbids reset and re-arm.
   - After a clean close it is DISARMED. A later `x` from a new teleop would not be "while walking".
   - One launch could include both only with a scripted re-arm after the close: a second teleop, a mid-cycle arm on a rest record, and (unless more goals are added) a cancel during a swing.
   - **Decision (OD10):** S4d-M is estop-free. The level-2 estop is a separately reviewed **S4d-E**, outlined here for that review only:
     - settle; `+ +` (2/3, 3/3); `a`; hold `w`;
     - goal 1 = level-2 0→1, the S1 four-foot shift, mirroring S4a;
     - `x` once goal 1 has at least one attributed feedback sample;
     - FAULTED (`estop`) in the request's tick, one cancel, no further goal;
     - 10 s of sim hold measured; `q`, after which disarm is refused (`reset required`), as expected;
     - budget: 1 goal, 1 cancel.
   - **No S4d-E driver, mode, scorer or restore pattern is implemented in this work.** S4d-M alone does not demonstrate the level-2 estop: the estop criterion rests on level-0 contact evidence (S3, S4a) and fixed-base evidence (S2), and the S4 claim must say so.
2. **Reversal at the turnaround** (the rejected alternative to rows 5–6, 10–12; OD14).
   - Switching `w` to `s` while goal 54 (8→9) is in flight would reverse at boundary 0 without stopping, saving 2 goals (109).
   - The margin depends on the 1.2 s S5 phase (safe only below RTF ≈ 1.4).
   - The forward endpoint would no longer be a stopped, fresh-sample-checked READY.
3. **Backward home** (the rejected alternative to rows 8–13; OD15).
   - X could shrink to 4–8 goals with a backward home from b ≤ 4.
   - The node then counts −1 on arriving at 0 (§4.5) for a net-zero round trip. That needs a new net-zero tolerance and special cycle-check handling.

The requirements fit **one bounded launch (S4d-M, 111 goals)**, with the level-2 estop as the one item that needs a separate run.

## 8. S4d-M scorer checks (M1–M16; `scripts/s4/score_s4d.py`)
Each check is True, False or None (missing evidence). Any False FAILs the M-check; otherwise any None makes it UNMEASURED; otherwise it is PASS. Overall PASS needs every M-check PASS; any FAIL makes the overall FAIL; otherwise INCONCLUSIVE. Bands are inclusive. No threshold changes after a run.
- **M1 Plan.**
  - Exactly the 111 goals of §5.3, in order, each SUCCEEDED (level, b_from, b_to, direction).
  - 0 cancels, faults or refusals anywhere in the run.
  - The main keys exactly as in §5.3, each written completely; requests before `q` exactly arm, stop, stop, home; the arm before goal 1.
  - One goal per seq; a consistent event order.
  - DISPATCH with the gate `True` in every status (live), or SHADOW with the gate `False` (`--shadow`).
- **M2 Level and speed change (OD13).**
  - The node's strides are 40 mm at level 1 and 60 mm at level 2.
  - `+` #1 while DISARMED with no goal, before the `a` key and the arm request; `+` #2 written while goal 2 was in flight.
  - Each confirmation is attributed by full-precision receipt to its own write: the receipt falls between that write's start and the next key's write start. Repeat bytes are not scripted keys. Lines are rebuilt across output chunks.
  - Exactly two `level_change` events: 0→1 between the arm and goal 1, and 1→2 in WALKING between goal 9's `phase_done` and goal 10's `goal_sent`.
  - Goals 1–9 at L1, including 3–9, which were dispatched after the key; goals 10–111 at L2.
- **M3 SC cycle.** Level-1 band and per-cycle drift and height (§5.2); the cycle count 1 after goal 9.
- **M4 Forward leg (OD1).**
  - F leg within [0.285, 0.315] m along B_9's +y.
  - Every F cycle within [50, 70] mm, with per-cycle drift and height.
  - Walking continuously from goal 1 to goal 54: no STOPPING or READY.
  - Space while goal 54 was in flight; one STOPPING (`stop`) in it; READY (`stopped`) at 0 within 2.2 s sim (120 s wall backstop); no goal between; cycle count 6; the cycle checked before `s`.
- **M5 Return leg (OD1).** As M4 for goals 55–99: R leg within [−0.315, −0.285] m along B_54's +y; every R cycle within [−70, −50] mm; READY within 2.9 s sim; cycle count 1; checked before the next `w`.
- **M6 Drift and attitude (OD2).**
  - At every boundary 0 from B_9 to B_99, in B_9's frame: |lateral| ≤ 30 mm and |heading| ≤ 0.10 rad.
  - Tilt ≤ 0.10 rad over every sample from the arm to the close.
  - Recorded only: the all-sample maxima of |lateral| and |heading| from B_9 to B_99 in B_9's frame, including the designed sway; the net return.
- **M7 Releases (OD7).** Rows 9 and 12, with the S4b rules: released while the goal was in flight; one STOPPING `intent_stop`, at least 0.717 s after the release (finding above 0.983 s); READY (`stopped`) at 5 within the in-flight goal's duration + 1.0 s sim; no extra goal. The two-goal R2 allowance is not exercised and not accepted.
- **M8 Reversal (OD14).** Row 11: the switch was written while goal 105 was in flight, with no release; goal 106 = 6→5 followed at boundary 6 with no stop between; then released (M7).
- **M9 Home (OD15).** Row 13: exactly 4 forward homing goals, with no motion key; READY `homed` at 0 within 12.0 s sim of the request; X composite cycle within [50, 70] mm with per-cycle drift and height; cycle count 2.
- **M10 Close while moving (OD7).**
  - The repeat stopped, then `q` written within the hold window, while goal 111 was in flight.
  - After `q`, before the probe: stop × 2 and disarm × 2, scored as counts with the order recorded; no refusal; no goal; no key.
  - Exactly two transitions: STOPPING (`stop`, `disarm` or `intent_stop`) after the `q` write, then DISARMED (`lease_expired` or `disarm`) at boundary 1. A READY in between FAILs.
  - DISARMED within goal 111's duration + 1.0 s sim (from the last status before `q`), 120 s wall backstop.
  - **The reason matches the node-clock evidence.** The scorer recomputes the node's own decision (§4.8):
    - the last status received before the transition with goal 111 in flight (WALKING or STOPPING) gives the node time `t_s = goal_sent(111) + goal.elapsed_s` and the last valid command as of then, `lv_s = t_s − lease_age_s`, both on the node clock; `t_tr` is the DISARMED event's node time and `lease_s` comes from the run's configuration (0.5 s);
    - the count at the transition is that of the first DISARMED status received after it, and only while that status's `lease_age_s` is empty: the transition clears the lease (905–906), so an empty lease age proves no command was accepted since;
    - `t_tr − lv_s ≤ lease_s`: expected `disarm` (later commands only renew the lease);
    - otherwise, if the count at the transition equals the status's count: expected `lease_expired`;
    - otherwise, if the count rose and `t_tr − t_s ≤ lease_s`: expected `disarm` (a command arrived after the status);
    - otherwise, within 1 µs of `lease_s`, or without that evidence: UNMEASURED. A falling count is a contradiction (FAIL).
    - Key-write times (driver clock), the teleop's exit (driver clock) and sim time are recorded for the timeline only; none of them sets the expected reason.
  - The teleop's exit seen within 15 s wall of the DISARMED status, with code 0.
  - 10 s of sim DISARMED with no goal, and still (S3 hold limits, from the bag).
- **M11 Tracking (OD6).** Attributed feedback for every one of the 111 goals, with no N/A exception; max |error| ≤ 0.05 rad; above 0.02 rad is a finding. A goal without attributed feedback is UNMEASURED.
- **M12 Continuity** (C7, unchanged).
- **M13 Ownership.** 111 goals and 0 cancels; the controller log shows 111 received, 111 accepted and 0 cancel requests; no foreign client or command publisher; no `joint_trajectory` message.
- **M14 Margins and freshness** (C9, unchanged), plus the driver result `script complete`.
- **M15 Restore and library** (C10 and C11, unchanged), plus OD12's second verification: the run's own installation imports all three gates `False` after restoration. The harness records it as JSON (`--run-install-gates`): `{"install_base": …, "gates": {"spiderx_controller.m61_live_contract.M61_LIVE_DISPATCH_ENABLED": false, "spiderx_controller.m6_live_contract.LIVE_DISPATCH_ENABLED": false, "spiderx_controller.m55_contract.M55_LOCOMOTION_DISPATCH_ENABLED": false}}`; exactly those three keys, each `false`.
- **M16 Reconnect probe (OD11).** §5.5, from the node events, the driver's statuses and the teleop's output.

**Missing evidence is UNMEASURED; a contradiction FAILs.**

**`--shadow`** drops the signed bands and the F and R leg bands (recorded only), and makes M11 N/A. It requires zero goals, cancels and feedback at the controller. |progress| per cycle stays within 80 mm; the drift, budget and ownership checks remain. The overall reads `SHADOW ONLY: not S4d acceptance`. A shadow run scored as live, or a live run scored as shadow, FAILs M1.

## 9. Live stops for S4d-M (LS1–LS10)
Every stop is an emergency response: the driver stops the repeat, sends `x` (or calls the estop service once the teleop has read `q`) and exits with code 2. The attempt ends; a stop is never suppressed to fit the budget, and nothing is retried.
- **LS1 Plan.** A goal beyond 111 or above the step's budget; a goal off the plan (level, b_from, b_to, direction); a node level other than the plan's (0 before goal 1; the in-flight goal's planned level, or the last completed goal's at rest); a moving intent at a level other than allowed (1 before `+` #2; 1 or 2 once it is written; 2 once the node reports 2; 2 in the probe).
- **LS2 State.** FAULTED; STOPPING or READY while a motion key is held with no scripted stop; after `q`, anything but WALKING or STOPPING with goal 111, or DISARMED with no goal; after DISARMED, anything but DISARMED with no goal.
- **LS3 Attitude.** Tilt > 0.10 rad or z < 0.045 m in 2 consecutive fresh samples (unchanged).
- **LS4 Travel (OD3).** Euclidean XY from the arm pose > 0.40 m, from the node's `delta_since_arm_xy`. S4d only; the reference is never reset; the 0.15 m S3 constant is unchanged.
- **LS5 Per cycle (unchanged).** At the first fresh status after each cycle-count change: SC [0, 60] mm; F and the X composite [0, 80] mm; R [−80, 0] mm; |lateral| > 20 mm or |yaw| > 0.10 rad; a count change that is not one cycle in the step's direction.
- **LS6 Cumulative (OD4).** At the same fresh boundary-0 checks during F and R (counts 2–6, then 5–1): |lateral| > 50 mm or |heading change| > 0.15 rad from B_9,live, in B_9,live's frame.
  - **Reference.** B_9,live is the pose of the SC cycle's own check (the first fresh status after the count changes 0 → 1). It is set once and never reset.
  - **Duplicate or stale samples.** Only the S3 fresh-sample rule counts: ground truth present, pose age ≤ 1.0 s, and sim time strictly increasing. A duplicate publication or a stalled clock is not fresh and is skipped; the check waits for the next fresh sample. A cycle with no fresh sample makes the next count change skip a step, which stops the run (LS5).
  - **Lead-in.** In continuous walking the check sample can fall in the next goal's 0.2 s lead-in, or just after it, in the S1 shift's first motion. This adds well under a millimetre at the planned RTF; acceptance never uses this sample (§5.2).
  - These stops supplement the acceptance limits (OD2: 30 mm and 0.10 rad, from the bag), and never replace them.
- **LS7 Joints and tracking (unchanged).** A joint within 0.02 rad of its URDF limit; tracking > 0.05 rad (> 0.02 rad is a finding).
- **LS8 Ownership and mode (unchanged).** Command rejections; with `--shadow-rehearsal`, any status that is not SHADOW with the gate `False`. The node's own monitors fault, which is LS2.
- **LS9 Teleop.** A missing, stale, duplicate or wrong level confirmation; the teleop not exiting within 15 s or exiting non-zero; any probe bound or probe evidence of §5.5.
- **LS10 Time (OD8).** Each wait's backstop, sim cap / 0.1 + 60 s wall, from the start of that wait; the 90-minute script cap, from the driver's start.

## 10. Owner decisions (recorded 2026-11-09)
| # | Item | Decision | Implementation consequence |
|---|---|---|---|
| OD1 | **Distance acceptance** | Nominal 0.300 m ± 0.015 m for each measured leg: forward [0.285, 0.315] m along B_9's +y; reverse [−0.315, −0.285] m along B_54's +y (its defined start frame). Claims say "approximately 0.3 m", never "at least 0.3 m". This is an explicit owner-selected interpretation, not a tolerance present in the original requirement (R1) | M4, M5; claim wording §15 |
| OD2 | **Cumulative drift** | 30 mm lateral and 0.10 rad heading, at boundary 0, in B_9's frame, across F and R. All-sample maxima, including the designed sway, are recorded separately | M6 |
| OD3 | **Travel cap (an S4d-only change)** | 0.40 m Euclidean XY from the arm pose. The reference is not reset; the 0.15 m limit elsewhere is unchanged. Envelope in §5.3 | LS4, in S4d tooling only |
| OD4 | **Live drift stops** (changed from the draft) | Keep the per-cycle stops and add cumulative live stops: \|lateral\| > 50 mm or \|heading change\| > 0.15 rad relative to B_9, at fresh boundary-0 cycle checks during F and R. They supplement the acceptance limits. Whole-script travel, tilt and the other continuous protections stay | LS5, LS6 (reference, freshness and lead-in rules in §9) |
| OD5 | **Budget** | S4d-M exactly 111 goals, zero planned cancels. An emergency response always overrides the budget and ends the attempt | LS1, M1, M13 |
| OD6 | **Tracking** | Tracking for every M goal, with no N/A exception. The canceled-goal rule is kept for the future S4d-E design only; E is not implemented or run here | M11; §7.1 |
| OD7 | **Timing** | The release, home and exit timing of §4.6–§4.8, and the exact-plan one-in-flight-goal bounds: READY or DISARMED within the in-flight goal's duration + 1.0 s sim; 120 s wall backstop; `homed` within 12.0 s sim; teleop exit within 15 s wall. The two-goal release allowance is **not exercised**. The moving close keeps node lease timing, teleop exit timing, sim time and wall time apart; no last-command time is inferred from a key write; the recorded evidence of M10 decides, and uncertainty is UNMEASURED | M7, M9, M10; §4.8 |
| OD8 | **Waits, script cap, watchdog** | Each wait: sim cap / 0.1 + 60 s wall, from the start of that wait. Whole script ≤ 90 min wall, from the driver's start. Outer watchdog 7200 s, from the moment the run-local harness starts enabling. Before the launch, the harness checks that the remaining watchdog time covers the 90-minute script cap plus 15 min for stop, cleanup, restore and verification; otherwise it does not launch and restores. The watchdog is a last resort: the driver handles SIGINT and SIGTERM promptly (emergency stop, exit 4), and every stop condition acts at once | LS10; §12.6 |
| OD9 | **S4e** | Human keyboard work stays separate (S4e). No physical-keyboard validation and no unqualified "full S4" claim | §15 |
| OD10 | **Estop at level 2** | A separately reviewed S4d-E. S4d-M alone does not demonstrate the level-2 estop. No E driver, mode or restore pattern in this implementation | §7.1; §12 |
| OD11 | **Reconnect probe** | Included: zero goals; bounded startup, confirmations, a 5 s held `w`, release, close and exit; command delivery shown; no arm, no goal and DISARMED throughout; expected node behaviour from the code (§4.13); the probe cannot weaken the close checks | §5.5; LS9; M16 |
| OD12 | **Build procedure** | B1: a fresh enabled build in new colcon bases for a later authorized live run, with explicit `--symlink-install` where restoration relies on links into `src/`. The documented incremental restoration is kept. Both the normal and the run installation are verified after restoration. No build or enabling is authorized by this decision | §13; M15 |
| OD13 | **Speed change** | L1 → L2 while walking, adopted only at the cycle boundary | rows 3–4; M2 |
| OD14 | **Reversal** | Mid-cycle in X, preserving the stopped F endpoint | rows 10–12; M8 |
| OD15 | **Home** | Four-phase forward home; X remains 11 goals | row 13; M9 |
| OD16 | **Disk and bag** | No fixed "enough" figure. Before future builds, account for the new build and install footprint. Before recording, require at least 5 GiB free **and** room for twice the conservative recording estimate plus the restore needs. The estimate uses the recording-duration cap, not the typical RTF. Bags stay outside the repository. No storage rate is invented | §13.4 |

## 11. Duration, RTF and resources (estimates)
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

  Add about 1 min for result latency over 111 goals, and about 1 min for the probe. The script cap is 90 min (OD8).
- **Session, wall:**
  - enabling, the fresh B1 build and M5.5 tests: not yet measured for B1; the incremental build took about 5–8 min;
  - launch ≈ 2 min; pre-arm checks ≈ 2 min; stop and restore ≈ 3 min;
  - the shadow rehearsal is similar at RTF 0.17–0.19.
- **Risk.**
  - Measured 10 s RTF windows reached 0.135 with the GUI running (reniced +19).
  - A long run has more exposure to a dip below the node's 0.1 backstop. That would be a fault, and the attempt ends safely.
  - Reducing host load (for example the GUI) is a launch change, outside this plan.

## 12. Offline implementation contract (S4d-M only)
1. **`scripts/s4/s4d_driver.py`** (new).
   - Built on the S4c driver. It reuses:
     - the S3 fresh samples, debounce, settle, hold observation and `body_delta`;
     - the S4a full-precision key-write brackets;
     - the S4b timed release and `switch_held_key`;
     - the S4c chunk-receipt confirmation attribution, teleop exit status and post-close observation.
   - It does **not** reuse S4c's DISARMED-only level selection for the speed change, or any level-0 assumption.
   - A plan table `PLANNED[1..111]` of (level, b_from, b_to, direction); stage-aware state rules; LS1–LS10.
   - New S4d limits (0.40 m, 50 mm, 0.15 rad, 90 min, the probe bounds) are constants of the S4d driver, never of shared or historical modules.
   - **On any failure:** stop the repeat, then `x` or the estop service, then a clean exit with the S3 exit codes. Cleanup always runs.
   - `--shadow-rehearsal` fails closed on the gates, as S3/S4c.
2. **`scripts/s4/score_s4d.py`** (new). M1–M16 with PASS, FAIL or UNMEASURED:
   - boundary poses from the bag (§5.2); never planned positions;
   - the bag opened read-only, with its hash checked after the scan;
   - pure parts (`score_events`, the boundary and leg measurements, the close-reason recomputation) testable without ROS.
3. **Bench tests** (`scripts/s4/test_s4d_driver_live.py`; the in-memory `LocomotionSession` bench, as S4c):
   - the complete script at RTF 0.1–1.0: 111 goals, 0 cancels, the probe;
   - adoption only at the boundary (an early adoption must stop);
   - extra, wrong-level and wrong-direction goals stop;
   - late space, release, reversal and close stop;
   - split, stale and duplicate confirmations; full-precision attribution;
   - per-cycle and cumulative drift, travel, tilt, stale samples, inclusive and live-stop boundaries;
   - both asynchronous close-request orders and the intent-stop order; both disarm reasons;
   - probe delivery, no arm, no goal, exit and expected (absent) rejections.
4. **Scorer tests** (`scripts/s4/test_score_s4d.py`): bench runs pass M1, M2, M7–M10 and M16; every check FAILs under its mutation; missing evidence is UNMEASURED; boundary extraction and alignment gaps; inclusive bands; shadow and live are never scored as each other.
5. **Restore support.**
   - Add only `local/m55-s4d-m-*` to `scripts/m61_restore_and_verify_disabled.sh` (42–43, 274–290). No S4d-E pattern.
   - Hermetic tests in `src/spiderx_controller/test/test_m61_restore_script.py`; look-alikes (`local/m55-s4d-*` without `-m-`, `local/m55-s4d-e-*`, `local/m55-s4d-mx…`) stay refused.
6. **Run-local harness** (future, not in the repository): from the S4c L2 live harness, with:
   - S4d input checks; the OD16 disk checks; the OD8 watchdog and its pre-launch time check;
   - the OD12 build procedure;
   - the same capture set: driver log and summary, key brackets, teleop receipts, node events, launch, controller and recorder logs, before/after graph, harness hashes, disabled library, restore report, and the run installation's gate check (M15);
   - one bag outside the repository, without `_action/status` or `/clock`.
7. **Lessons carried forward** (already verified):
   - full-precision timestamps before and after every key write and repeat stop;
   - teleop output rebuilt across chunks with receipt times;
   - each `+` attributed to exactly one confirmation;
   - asynchronous service calls: order recorded, scored as counts;
   - the S3 fresh-sample definition;
   - motion measured independently from the bag;
   - cleanup and restore on every exit path.

No runtime source, threshold, gain, rate, trajectory, phase library, physics, historical scorer or historical evidence changes.

## 13. Build and restore procedure (to be fixed before any run)
1. **Disabled validation.**
   - The new driver, scorer, tests and restore script change executable inputs. So, before the S4d-M shadow rehearsal, run **one fresh disabled build and the full suite in new, uniquely named colcon bases** (the restoration follow-up's procedure), with nothing deleted, plus the direct `scripts/s4` tests.
   - Reuse that result for later runs while `src/`, `scripts/` and `third_party/` stay unchanged.
2. **Enabled build (OD12: B1).**
   - Build the local enabling branch in new, uniquely named bases (`--build-base`, `--install-base`) with `colcon build --symlink-install`, from a toolchain-only shell, with st59site on `PYTHONPATH` for the build only.
   - The harness sources that run installation (after the r2 overlay). Input checks verify its paths, its symlinks into `src/`, the gates (M55 `True`; M61 and M6D `False`) and the library identity.
   - Nothing in `build/`, `install/` or `log/` is deleted.
3. **Restore, fixed before enabling and never changed after the launch.**
   - `scripts/m61_restore_and_verify_disabled.sh --base <disabled baseline> --enabling-ref local/m55-s4d-m-<UTC> --delete-enabling-branch`, with the documented toolchain-only incremental rebuild of the normal workspace installation and the run's pid files.
   - The branch is deleted only after DISABLED AND VERIFIED.
   - **Both installations are verified.** The restore script verifies the normal installation. The harness then verifies the run installation in a fresh environment: it imports all three gates `False` through its symlinks into the restored `src/`. That installation is kept, not deleted. Both results are evidence (M15).
4. **Storage (OD16).**
   - **Before the builds:** free space must cover the new B1 build and install bases, measured from the current workspace's `build/` and `install/` sizes at that time, plus the recording requirement below.
   - **Before recording:** free space on the bag's filesystem must be at least 5 GiB **and** at least 2 × E + the restore needs, where:
     - E = r × T. T is the recording-duration cap: the recorder runs from after the launch until cleanup, and the 7200 s watchdog (OD8) bounds it, so T = 7200 s minus the watchdog time already used when recording starts;
     - r is a conservative rate taken from committed evidence: the highest observed rate (30.8 MB per wall minute, S4b live) was measured only at overall RTF 0.17–0.42. Bag growth at higher RTF is unmeasured, so the harness doubles it (61.6 MB per wall minute; an assumption, not a measurement) unless the owner sets another figure. For T = 7200 s that gives E ≈ 7.4 GB and 2 × E ≈ 14.8 GB;
     - the restore needs are the incremental rebuild of the normal installation and the restore evidence, measured from the last restore.
   - The bag stays outside the repository. A failed check means no recording and no launch.
5. **Historical caveats** (S4c L1 and L2 incremental builds, the restoration follow-up) stay unchanged.

## 14. Authorization boundaries (each step needs its own approval)
1. The owner decides OD1–OD16 — **done 2026-11-09 (§10).**
2. Offline S4d-M implementation on `claude/spiderx-s4d-preparation` (§12), reviewed as a draft PR. **Authorized 2026-11-09, offline only.** No build, launch, enabling, goal, S4d-E or merge.
3. Fresh disabled build and full suite in new bases (§13.1). No launch. Not authorized (2026-11-09). **Authorized and done 2026-11-10:** 1983 tests, 0 failures, 0 skipped ([evidence](evidence/m55/s4d_fresh_disabled_20261110T071506Z/README.md)).
4. One S4d-M **shadow** rehearsal (one launch, SHADOW, no goal at the controller). Not authorized (2026-11-09). **Authorized and done 2026-11-10:** PASS (SHADOW ONLY) ([evidence](evidence/m55/s4d_shadow_20261110T072920Z/README.md)).
5. One S4d-M **live** attempt, only after the shadow result is reviewed. Not authorized (2026-11-09). **Authorized and done 2026-11-11:** PASS, M1–M16 (§16).
6. S4d-E design review, then its own shadow and live runs (OD10). Not authorized (2026-11-09). 2026-11-11: the offline S4d-E preparation and one S4d-E **shadow** rehearsal were authorized, on a separate branch; a live S4d-E attempt is not authorized. Later on 2026-11-11, ONE live S4d-E attempt was separately authorized and run: PASS under the frozen contract (§17).
7. Optional S4e on the owner's PC (OD9). Not authorized.

No step continues automatically into the next. There is no retry, recovery or second launch inside an attempt. Enabling branches are never pushed. The gates stay `False` on every pushed commit.

## 15. Claim boundaries
A passing S4d-M would support this claim only: keyboard-path crawl walking in simulation.
- Automated input through the teleop.
- Approximately 0.3 m forward and back at internal level 2 (each leg 0.300 m ± 0.015 m along its start frame's +y, OD1), measured from the bag. Never "at least 0.3 m".
- Release, speed change, reversal, `home` and a clean close while moving, at level 2, within the stated limits.
- Idealized actuator model, CAD masses, assumed contact.

It would not support:
- human keyboard operation or physical-keyboard validation (S4e, OD9);
- an unqualified "full S4" claim;
- hardware, rough terrain, odometry or navigation;
- dynamic stability or unrestricted walking;
- the level-2 estop (S4d-E, OD10);
- the level-2 crash-path close.

## 16. Outcome (recorded 2026-11-11; the criteria above are unchanged)
**S4d-M: PASS, for the approved simulation scope only.** One authorized live attempt in Cloud Gazebo on 2026-11-11, from a local enabling branch that was never pushed. The committed `score_s4d.py`, run without `--shadow`, rated M1–M16 PASS ([evidence](evidence/m55/s4d_live_20261111T052817Z/README.md)).
- **Attempt.** One attempt: exactly the 111 planned goals, all succeeded, zero cancels. Automated keyboard-path input through the real teleop.
- **Measured.**
  - Approximately 0.3 m forward and back (OD1): F leg +0.30006 m, R leg −0.29985 m.
  - Cumulative drift at every boundary 0: at most 0.56 mm lateral and 0.0068 rad heading.
  - Tracking feedback for all 111 goals, maximum error 0.00225 rad.
  - The close while moving ended `lease_expired`, matching the node-clock evidence (M10). The reconnect probe stayed DISARMED with no goal (M16).
- **Shutdown: not graceful.** After the script and the recording had ended, the simulator server did not exit within the `ign gazebo` launcher's 5 s grace period after SIGINT, and the launcher killed it with SIGKILL. No stated cleanup criterion was violated; the cause is not established ([postmortem](evidence/m55/s4d_live_20261111T052817Z_shutdown_postmortem_20261111T101720Z/README.md)).
- **Restoration.** The full restore gave DISABLED AND VERIFIED. Both the workspace and the run installation import all three gates `False`.
- **Shadow rehearsal.** 2026-11-10, PASS (SHADOW ONLY) ([evidence](evidence/m55/s4d_shadow_20261110T072920Z/README.md)).
- **Claim (§15).** Keyboard-path crawl walking in simulation at internal level 2: approximately 0.3 m forward and back, with release, speed change, reversal, `home` and a close while moving, within the stated limits. The "clean close while moving" of §15 is the teleop close (M10), not the simulator's shutdown.
- **Not shown.** S4d-E (the level-2 estop), S4e and physical-keyboard operation, hardware, navigation, unrestricted walking and full S4 acceptance.

## 17. S4d-E outcome (recorded 2026-11-11; the frozen contract `docs/S4D_E_CONTRACT.md` is unchanged)
**S4d-E: PASS, under its frozen contract.** One authorized live attempt in Cloud Gazebo on 2026-11-11, from a local enabling branch that was never pushed. The committed `score_s4d_e.py`, run without `--shadow`, rated E1–E12 PASS ([evidence](evidence/m55/s4d_e_live_20261111T113831Z/README.md)).
- **Attempt.**
  - `+ +` while DISARMED (2/3, then 3/3), then `a`.
  - ONE level-2 0→1 S1 shift goal, received and accepted by the real controller.
  - ONE real feedback sample attributed to it, then `x`.
  - ONE cancel, a measured hold, then `q`. Automated keyboard-path input through the real teleop.
- **Measured.**
  - FAULTED (`estop`) on the request's node-clock instant (R4 ≤ 0.050 s). The key-write latency, 0.51 ms, is recorded only.
  - Cancel response 0, then result CANCELED. The controller logged the cancel.
  - The controller reference was unchanged over the 10 s (sim) hold. Joint motion was 5.9e-12 rad.
  - Exactly the two `faulted_reset_required` disarm refusals at `q`. There was no reset, re-arm, home or second goal.
- **Qualification.** The cancel came about 0.04 s of sim after the goal started, about 2% of its 1.9 s duration, before measurable motion (≤ 1.2e-10 rad).
  - The attempt shows the estop, cancel and hold path with a real active goal. It does not show a moving body being stopped.
  - Tracking (E8) rests on one feedback sample (0.0 rad).
  - This follows the frozen design; no criterion was changed after the result.
- **Shutdown and restoration.** The simulator server exited 0 on SIGINT. The full restore gave DISABLED AND VERIFIED. Both installations import all three gates `False`.
- **Shadow rehearsal.** 2026-11-11, PASS (SHADOW ONLY) ([evidence](evidence/m55/s4d_e_shadow_20261111T104429Z/README.md)).
- **Not shown.** S4e and physical-keyboard operation, hardware, navigation, unrestricted walking and full S4 acceptance.
