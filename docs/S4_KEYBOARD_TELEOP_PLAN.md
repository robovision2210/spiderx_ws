# S4: interactive keyboard teleop validation: plan (design only)

Date: 2026-11-01. Written on branch `claude/spiderx-s3-preparation`; the S4a decisions and corrections were added on `claude/spiderx-s4a-preparation`.

**Status: S4a and S4b PASSED; S4c–S4e PLAN.** S4a level-0 reverse-and-return validation passed in simulation using automated input through the keyboard-teleop path (2026-11-01, §3.1, [evidence](evidence/m55/s4a_live_20261101T153411Z/README.md)). S4b level-0 release, boundary reversal, home and lease-loss validation passed in simulation using automated keyboard-path input (2026-11-02, §3.2, [evidence](evidence/m55/s4b_live_20261102T092327Z/README.md)). **No further S4 stage is authorized to run.** Each stage needs its own owner authorization, its own local enabling branch (never pushed) and one launch. The dispatch gates stay `False` on every pushed commit.

**Automated keyboard path versus human operation.** Every Cloud stage (S4a–S4d) drives the real `m55_teleop_keyboard` from a script, through a pseudo-terminal with emulated key repeat. That tests the teleop's key decoding, hold window, lease and service calls, and the node's response. It is **not** human operation. A person at a physical keyboard is only S4e, on the owner's PC: real key repeat and reaction times, checks 1.11 and 1.13.

Sources:
- [M5.5 §8 teleop and §10 phases 3–4](M55_KEYBOARD_WALKING.md);
- [path to simulated walking](PATH_TO_SIM_WALKING.md) (the S4 row and §c);
- [S3 design §9](S3_FREE_BASE_CRAWL_DESIGN.md);
- the [S3 adoption record](evidence/m55/s3_free_base_20261101T115002Z_adoption_s3r1/README.md).

## 1. What S3 already demonstrated, and what it did not
**Demonstrated:** "S3 bounded level-0 free-base crawl accepted retrospectively under criteria revision S3-r1."
- **Run.** Gain-1.0 r2 plant, free base. Six level-0 **forward** cycles moved the body about 0.1201 m (20.015–20.029 mm per cycle).
  - Per cycle: \|lateral\| ≤ 19 µm, \|yaw\| ≤ 8.7e-5 rad, tilt ≤ 0.0026 rad, height within 35 µm.
  - Boundary stops with space. An estop with exactly one cancel, then a 10 s hold on contact.
  - 55 goals and 1 cancel.
- **How keys reached the robot.** A driver wrote them into the real `m55_teleop_keyboard` through a pseudo-terminal (`a`, held `w` with emulated auto-repeat, space, `x`, `q`).
  - So the teleop's key decoding, hold window, lease and service calls ran on contact.
  - **No human operator.**
- **Limitations.** As listed in the adoption record: goal 55's tracking is N/A, the estop source is inferred, contact-point slip was not measured, and the actuator model is idealized.

**Not demonstrated on contact:**

| Item | Status |
|---|---|
| Reverse motion (`s`) | Never on contact in S3 (forward only); S2 ran reverse on the welded base only. S4a later ran one level-0 reverse cycle on contact (§3.1) |
| Levels 1–2 (`+`/`-`, 40 and 60 mm strides) | Never on contact; S2 fixed base only. The PATH doc forbids combining S3's first contact with any level above 0 |
| Key release mid-cycle; `home`; reversal; speed change at a boundary | Never on contact (S0 and the shadow runs only) |
| Teleop close or kill (lease loss), and `r`/`a` after an estop | Never on contact |
| A physical keyboard (checks 1.11 and 1.13 on the crash path) | Never. Cloud has no keyboard; the pseudo-terminal emulates a 0.5 s repeat delay then 30 Hz |

## 2. The existing S4 criteria (unchanged by this plan)
From M5.5 §10 phase 4 and the PATH doc's S4 row:
- **Distance and drift.** Forward 0.3 m at the fastest level (3/3), then back: no unplanned fault; tilt ≤ 0.10 rad; lateral deviation ≤ 30 mm; heading drift ≤ 0.1 rad.
- **Key release while walking.** READY within the time of the goals still to finish plus 1.0 s of sim (≤ 2 × 4.7 + 1.0 s at 60 mm). Wall backstop: 120 s.
- **Speed change, reversal, `home`.** Each takes effect at the next cycle or phase boundary; `home` ends at boundary 0.
- **`x` while walking.** FAULTED within one 50 ms tick; one cancel; no further goal.
- **Closing the teleop.** DISARMED within check 1.9's sim bound (120 s wall backstop); no motion until `a`.
- **Evidence.** A bag and the evidence directory. A screen recording only if the owner makes one.
- **Ordering.** The owner gives a go/no-go before any faster level. S3's first contact is never combined with a level above 0.

Wherever a stage needs a value these criteria do not give (stop limits, per-level bands, budgets), it is an **owner decision** (§6), not assumed here.

## 3. Stages, smallest first

| Stage | Purpose | Script (level, keys) | Budget |
|---|---|---|---|
| **S4a (passed 2026-11-01, §3.1)** | First contact in **reverse**, at the validated level 0 | settle, arm; hold `s` for 1 cycle, space at the boundary; hold `w` for 1 cycle back, space; estop (`x` with a goal in flight); 10 s hold; `q` | ≤ 19 goals, 1 cancel; the return need not land on the arm pose |
| **S4b (passed 2026-11-02, §3.2)** | Operator input on contact, level 0 | key release mid-phase (no space); reversal at a phase boundary; `h` home; teleop SIGKILL (lease, check 1.9); no estop, reset or re-arm (§3.2) | 12 goals, 0 cancels (§3.2) |
| S4c | First contact at level 1, then at level 2, with the owner's go/no-go between them | 1 cycle forward and 1 back per level; space at each boundary; optionally one estop exercise | Cycles: 2 levels × 2 cycles × 9 = 36 goals. One estop exercise would add its in-flight goal and the cancel: **at most 37 goals and 1 cancel**. Unresolved |
| S4d | The S4 acceptance run (§2) | 0.3 m forward at level 3/3 (5 cycles of 60 mm), then back; release, speed change, reversal, `home`, `x`, close | ≈ 90 cycle goals plus checks, fixed in its authorization |
| S4e (optional, owner's PC) | A physical keyboard | checks 1.11 (key repeat) and 1.13 (crash path) on the owner's machine | Owner-run |

### 3.1 S4a decisions (owner, 2026-11-01)
- **Scope.** Level 0 only: settle and arm; one reverse cycle stopped at its boundary; one forward return cycle stopped at its boundary; a final scripted estop with one goal in flight; the existing 10 s sim-time contact hold; quit.
- **Budget.** At most 19 goals and 1 cancel. No faster level, extra cycle, recovery motion or retry.
- **Cycle rules.** Measured along body +y in the frame of the cycle's start boundary, with yaw wrapped.

  | Cycle | Live stop if outside | Acceptance |
  |---|---|---|
  | Reverse | [−40, 0] mm | [−30, −10] mm |
  | Forward return | [0, 40] mm | [10, 30] mm |

  The return is measured from the end of the reverse cycle; it need not land on the arm pose.
- **Unchanged.** Travel cap 0.15 m. Every other approved S3 safety and acceptance limit.
- **Tools** (offline, `claude/spiderx-s4a-preparation`): `scripts/s4/s4a_driver.py`, `scripts/s4/score_s4a.py`, and the `local/m55-s4a-*` restore pattern. The driver records a timestamp immediately before and after each key write, with the write result.
- **Results.**
  - Shadow rehearsal: PASS, shadow only ([evidence](evidence/m55/s4a_shadow_20261101T150658Z/README.md)).
  - One live attempt: PASS, A1–A8 ([evidence](evidence/m55/s4a_live_20261101T153411Z/README.md)).
    - Reverse −19.992 mm, return +20.024 mm, net +0.031 mm after the return.
    - Drift ≤ 14 µm lateral and ≤ 3.8e-5 rad yaw; tilt ≤ 0.0026 rad.
    - Maximum tracking error 0.00225 rad. Goal 19 is N/A under the estop exception (17/17 checks).
    - Measured 10 s hold.
    - 19 goals, 1 cancel.
  - The simulator server exited 0, with no forced termination, and the restore was DISABLED AND VERIFIED.
  - This was automated input through the keyboard-teleop path, not human operation.

### 3.2 S4b scope (owner, 2026-11-02): passed
- **Scope.**
  - Level 0 on the existing plant.
  - Key release while moving, never replaced by space; READY within the sim bound.
  - `home` from a stopped boundary to boundary 0.
  - Reversal at its documented phase boundary.
  - Teleop SIGKILL: DISARMED within the bound, with no re-arm and no further dispatch.
- **No estop exercise, and a planned cancel budget of zero.** There is no reset or re-arm after a mid-phase fault. An unexpected safety cancel is an emergency response: it ends the attempt and is never suppressed to fit the budget.
- **Limits.** At most 30 phase goals were offered; the script uses 12. Travel cap 0.15 m. The S4a level-0 direction-aware stops and bands, and every other validated limit, are unchanged.
- **Script.** It was derived from `m55_locomotion` and checked on the in-memory bench (`scripts/s4/s4b_driver.py`):

  | # | Keys | Goals (b_from→b_to) | Ends |
  |---|---|---|---|
  | 1 release | hold `w`; release while 4→5 is in flight | 1–5: 0→1 … 4→5 | STOPPING `intent_stop`, READY at 5 |
  | 2 reversal | hold `w`; while 5→6 is in flight, stop the repeat and hold `s` 0.2 s later; release while 6→5 is in flight | 6: 5→6, 7: 6→5 (the same phase played back) | READY at 5 |
  | 3 home | `h` | 8–11: 5→6 … 8→9 (the fewest phases, forward) | READY `homed` at 0 |
  | 4 lease loss | hold `w`; SIGKILL while 0→1 is in flight | 12: 0→1 | STOPPING and DISARMED `lease_expired` at 1; then 10 s of sim observed |

  Exercises 1–3 make one complete forward cycle. The node's cycle count changes once, by +1, so the S4a forward stop [0, 40] mm and the acceptance band [10, 30] mm apply.

  A home from boundary 4 or below would go backward and count −1 on arriving at boundary 0, as would a reversal across boundary 0. The script avoids both.
- **Determinism.**
  - Each release and the kill happen while a goal is in flight: 2.8, 2.5 and 1.7 s of sim.
  - Release detection: M5.5 §16.1 documents 0.8–0.95 s wall from the last key character. The driver records the release as its stop of the emulated repeat, and the last repeat can precede that by one 30 Hz period. The scorer therefore expects STOPPING at least H − 1/30 s − T = 0.717 s after the recorded release; above 0.95 s + 1/30 s = 0.983 s is a finding, not a failure. The lease: at least L − B − T = 0.35 s after the kill; above the documented 0.55 s is a finding. The phase ends later than that for any RTF up to about 1.6; the Cloud runs had 0.17–0.64.
  - An extra goal is never hidden. Any goal beyond the plan, or different from it, stops the script.
- **Acceptance** (`scripts/s4/score_s4b.py`, B1–B11, written before any run; missing evidence is UNMEASURED, a contradiction FAILs):
  - **Plan (B1).** Exactly these 12 goals, every one SUCCEEDED. 0 cancels and no fault. The keys are exactly `a w w s h w`. No space, `x`, `r`, `z` or `q`, and no stop, estop, reset or disarm request.
  - **Release (B2, B3).** Released while the goal was in flight. One STOPPING (`intent_stop`), at least H − 1/30 s − T (0.717 s) after the recorded release: the hold window decided it, which distinguishes it from an explicit stop. A detection latency above 0.983 s is a finding. READY at boundary 5 within the goal's duration + 1.0 s of sim (check 1.7), with a 120 s wall backstop.
  - **Reversal (B3).** `s` written while 5→6 was in flight. 6→5 sent at that boundary, with no stop between.
  - **Home (B4).** One forward homing of 4 phases; READY `homed` at 0. Within the documented 12.0 s sim homing bound, with no motion key during home.
  - **Lease loss (B5).** Measured from the recorded kill bracket. STOPPING `lease_expired` at least L − B − T after the kill. DISARMED at 1 within goal 12's duration + 1.0 s of sim (check 1.9), with a 120 s wall backstop. After the kill: no goal, request, key or teleop restart. DISARMED throughout, and still over 10 s of sim (the frozen S3 hold limits).
  - **Other checks.** The composite cycle (B6). Tracking for every goal, with no N/A (B7). Continuity (B8). Ownership: 12 goals and 0 cancels, and the controller log showing 12 accepted and 0 cancel requests (B9). Restore (B10) and library (B11).
- **Results.**
  - Shadow rehearsal: PASS, shadow only ([evidence](evidence/m55/s4b_shadow_20261102T085832Z/README.md)).
  - One live attempt: PASS, B1–B11 ([evidence](evidence/m55/s4b_live_20261102T092327Z/README.md)).
    - 12 successful goals and zero cancels; tracking feedback for every goal (maximum error 0.00224 rad).
    - Releases detected 0.889 s and 0.818 s after the recorded release; READY in 2.82 s and 2.50 s of sim. Home: `homed` at 0 in 8.23 s of sim.
    - Lease loss: STOPPING 0.466 s after the kill, DISARMED at 1 in 1.79 s of sim, then DISARMED stillness measured over 10 s of sim.
    - Composite cycle +20.026 mm. Goal 12's single phase (`homed` to DISARMED) moved the body 19.9 mm forward and 16.6 mm laterally; recorded only.
  - The simulator server exited 0, with no forced termination, and the restore was DISABLED AND VERIFIED. A 0.23 s startup wait of `gz_ros2_control` for `robot_state_publisher` is recorded as a finding.
  - This was automated keyboard-path input, not human operation. It does not validate faster levels, full S4, hardware or navigation.

**Why S4a first.** It adds exactly one physical unknown: reverse on contact. Everything else stays as validated in S3:
- the same speed level, plant, launch, harness, stops and evidence;
- the forward return cycle re-checks S3 behaviour after reversing;
- the estop is re-tested on contact.

Its acceptance bands already exist: M5.5 phase 3 says "10–30 mm forward / back", and the PATH S3 row says "1 reverse then stop … 10–30 mm per cycle". The run is small: about 19 goals and about 50 s of sim, including the 10 s hold.

**Clearance.** The robot spawns at the origin of `spiderx_fortress.sdf`. Every obstacle (`box_a`, the pillar, `box_b`) is at least about 1.4 m from the spawn, and the wall along +y is at about y = 3.0 m. That leaves ample room for S4a's ±20 mm and for S4d's ±0.3 m.

## 4. Stops, evidence and cleanup (every stage)
- **Stops.** The S3 stops are kept as approved:
  - tilt > 0.10 rad or z < 0.045 m, each in 2 fresh samples;
  - the travel limit (§6);
  - per-cycle forward progress < 0 or > 40 mm, and drift > 20 mm lateral or > 0.10 rad yaw;
  - a joint within 0.02 rad of its URDF limit;
  - tracking > 0.05 rad (> 0.02 rad is a finding);
  - the ownership, freshness, stall, RTF, timeout and driver-error protections;
  - the owner's instruction.

  Reverse cycles use the mirrored progress rule (§6). No threshold changes during a run, and measured displacement never stands in for planned.
- **Evidence.** As S3:
  - pre-arm checks: plugin libraries and hashes, gain, rate, timestep, ownership, settling;
  - one bag, kept outside the repository with its hash;
  - captures, the driver log and node evidence;
  - the scorer's output and the restore report;
  - at most two screenshots, at rest. A screenshot is never proof of motion.
- **Cleanup.** The tested watchdog and trap: SIGINT, the server exit observed, then the full restore (clean rebuild, `--delete-enabling-branch`). The result must be DISABLED AND VERIFIED with all three gates `False`. A failed script never waives cleanup.

## 5. Offline work needed before any S4 run (separately authorized; no motion)
1. **S4 driver** (`scripts/s4/`; **done for S4a** (`s4a_driver.py`) and **S4b** (`s4b_driver.py`, §3.2); S4c–S4d are not implemented). The S3 driver is forward-only by design and its tests enforce that. The new driver:
   - reuses the tested S3 helpers: fresh samples, debounce, the body frame, the settle check and `--shadow-rehearsal`;
   - adds reverse and mirrored checks, level keys at boundaries, `home`, release mid-cycle, a lease exercise and reset/re-arm;
   - is tested on the in-memory bench.

   It should record a timestamp **before** each key write as well as after. That removes the inferred-window issue S3-r1 had to handle retrospectively.
2. **S4 scorer, written before any run** (**done for S4a** (`score_s4a.py`) and **S4b** (`score_s4b.py`, §3.2)). It covers the §2 criteria and the stage's bands. It specifies in advance the bracketed estop exception, using the recorded before- and after-write times rather than a retrospective inference. Missing evidence is UNMEASURED.
3. **One shadow rehearsal of each stage's script** (as P4 for S3), with no goal reaching the controller.

## 6. Owner decisions needed
1. ~~Reverse stop rule~~ **Decided for S4a** (§3.1).
2. **Travel limits:** 0.15 m is decided for S4a and S4b. **Unresolved** for S4c–S4d; S4d needs its own. S3 used 0.15 m for 0.12 m planned.
3. **Per-cycle bands for levels 1–2** (40 and 60 mm strides); none is specified today.
4. **Whether S4 acceptance needs a human at a physical keyboard** (S4e), or whether the Cloud pseudo-terminal emulation is enough for simulation.
5. **The order S4a → S4b → S4c → S4d**, each with its own authorization and go/no-go. This plan recommends it; combining stages needs the owner's explicit choice.
6. **For S4c–S4d,** these all remain unresolved: scripts, budgets, stop limits, whether an estop exercise is included, and authorizations. S4b ran once in shadow and once live, as authorized (§3.2). This plan does not change their acceptance criteria (§2).

**Not authorized by this plan:** any launch, enabling branch, goal, cancel, S4c–S4e execution, S4c–S4e implementation, hardware or navigation work. The S4a and S4b tools each ran once in shadow and once live, as authorized (§3.1, §3.2).
