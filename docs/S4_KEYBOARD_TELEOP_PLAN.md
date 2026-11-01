# S4: interactive keyboard teleop validation: plan (design only)

Date: 2026-11-01, branch `claude/spiderx-s3-preparation`.

**Status: PLAN ONLY; NOTHING HERE IS AUTHORIZED.** Each stage below needs its own owner authorization, its own local enabling branch (never pushed) and one launch. The dispatch gates stay `False` on every pushed commit.

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
| Reverse motion (`s`) | Never on contact. The approved S3 scope was forward only; S2 ran reverse on the welded base only |
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
| **S4a (recommended first)** | First contact in **reverse**, at the validated level 0 | arm; hold `s` for 1 cycle, space at the boundary; hold `w` for 1 cycle back, space; estop (`x` with a goal in flight); 10 s hold; `q` | ≤ 19 goals, 1 cancel; net ≈ 0 m |
| S4b | Operator input on contact, level 0 | release mid-cycle (READY by the 1.7 sim bound); `h` home; reversal at a boundary; teleop kill (lease, check 1.9); `r`, `a` after an estop | To be fixed in its authorization (≈ 30 goals, 1 cancel) |
| S4c | First contact at level 1, then at level 2, with the owner's go/no-go between them | 1 cycle forward and 1 back per level; space at each boundary | ≤ 36 goals, plus 1 cancel for the estop |
| S4d | The S4 acceptance run (§2) | 0.3 m forward at level 3/3 (5 cycles of 60 mm), then back; release, speed change, reversal, `home`, `x`, close | ≈ 90 cycle goals plus checks, fixed in its authorization |
| S4e (optional, owner's PC) | A physical keyboard | checks 1.11 (key repeat) and 1.13 (crash path) on the owner's machine | Owner-run |

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
1. **S4 driver** (`scripts/s4/`). The S3 driver is forward-only by design and its tests enforce that. The new driver:
   - reuses the tested S3 helpers: fresh samples, debounce, the body frame, the settle check and `--shadow-rehearsal`;
   - adds reverse and mirrored checks, level keys at boundaries, `home`, release mid-cycle, a lease exercise and reset/re-arm;
   - is tested on the in-memory bench.

   It should record a timestamp **before** each key write as well as after. That removes the inferred-window issue S3-r1 had to handle retrospectively.
2. **S4 scorer, written before any run.** It covers the §2 criteria and the stage's bands, and specifies in advance the S3-r1 bracketed estop exception (not retrospectively). Missing evidence is UNMEASURED.
3. **One shadow rehearsal of each stage's script** (as P4 for S3), with no goal reaching the controller.

## 6. Owner decisions needed
1. **Reverse stop rule:** completed reverse-cycle progress > 0 or < −40 mm along body +y (the mirror of S3's rule). Acceptance band −10 to −30 mm (M5.5 phase 3). Confirm.
2. **Travel limits:** keep 0.15 m for S4a to c; set one for S4d. S3 used 0.15 m for 0.12 m planned.
3. **Per-cycle bands for levels 1–2** (40 and 60 mm strides); none is specified today.
4. **Whether S4 acceptance needs a human at a physical keyboard** (S4e), or whether the Cloud pseudo-terminal emulation is enough for simulation.
5. **The order S4a → S4b → S4c → S4d**, each with its own authorization and go/no-go. This plan recommends it; combining stages needs the owner's explicit choice.

**Not authorized by this plan:** any launch, enabling branch, goal, cancel, driver or scorer implementation, S4 run, hardware or navigation work.
