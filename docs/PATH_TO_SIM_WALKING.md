# SpiderX: the path to simulated keyboard walking (design only)

Date: 2026-10-13, branch `claude/spiderx-m55-keyboard-walking` (head after `a8aa56f`). This note uses only
existing repository results. It runs nothing, enables nothing and claims no walking.
- Sources: [M4.5 results](M4_5_TEST_RESULTS.md), [M5 results](M5_TEST_RESULTS.md),
  [M5.5](M55_KEYBOARD_WALKING.md), the [M6.1 design](M61_TROT_REPLAY_DESIGN.md) and the
  [Cloud report §15](M61A_CLOUD_VERIFICATION.md).
- Every stability number below is a **quasi-static model value**: CAD steel-density masses, point
  feet, flat ground, assumed contact, level body. None is a measurement.

## a) The gait for the first free-base test: the M5.5 body-shift crawl, level 0

| Gait (source) | Feet down | Minimum static margin | Verdict for a first free-base test |
|---|---|---|---|
| `trot` (M4.5, M6.1 design) | 2 (diagonal) | **none**: line support; the COM is 4.0 / 3.8 mm off the two diagonals | Rejected. It tips by design: about 4.5° per swing as an inverted pendulum, τ ≈ 0.085 s |
| `pace` (M4.5) | 2 | none | Rejected (as trot) |
| `amble` β 0.55 (M4.5) | 2–3 | +0.84 mm over the 20 % three-foot share only | Rejected: mostly two-foot |
| `ripple` β 0.65 (M4.5) | 2–3 | −2.10 mm | Rejected |
| `wave` β 0.85 (M4.5) | 3–4 | **−2.58 mm**; best stance shift +0.59 mm (M5 Stage 4) | Rejected: below the 5 mm screen |
| `tripod_crawl` β 0.75 (M4.5) | 3–4 | **−4.07 mm**; best stance shift −0.89 mm | Rejected |
| **M5.5 crawl with body shift and sway** (`m55_crawl`) | 3–4 | **19.8 / 19.6 / 19.4 mm** at the 20 / 40 / 60 mm strides (≥ 15 mm required) | **Recommended** |

- **Why the M4.5 crawls fail.** The COM (51.0, −57.6, 71.7) mm sits about 5.8 mm behind the foot
  centre. Lifting a rear leg without a body shift leaves −4.0 / −3.8 mm; a fixed stance shift only
  trades one support triangle for another (M5 Stage 4).
- **Why the M5.5 crawl works in the model.** Before each swing, the M5.5 crawl shifts the body so the
  COM lies inside the coming support triangle, inset by 20 mm. The sway is 8–17 mm per shift.
- **Recommendation: level 0.** Stride 20 mm, cycle 17.0 s, 1.06 mm/s.
  - Margin 19.8 mm, the largest of the three levels.
  - Peak joint speed |q̇| 0.225 rad/s (bound 0.25).
  - Controller spline error 7e-6 rad.
  - Swing order LR, LF, RR, RF; 12 mm apex.
  - The sampled-minimum caveat is in M5.5 §5: neighbour samples differ by at most 0.47 mm.
- The trot stays a **fixed-base** test case (M6.1, passed in Cloud).

## b) Minimum remaining stages

One effort unit = one focused Claude session (plan, run, evidence, docs), comparable to one M6.1
cycle round. Every stage that sends a goal needs **the owner's explicit approval** and a separate
**local, never-pushed enabling branch** that flips `M55_LOCOMOTION_DISPATCH_ENABLED` and its pin
only. Each such approval covers one run.

| Stage | Purpose | Acceptance (from M5.5 §10 unless noted) | Goal sent? Owner approval | With fakes | Main risks | Effort |
|---|---|---|---|---|---|---|
| **S0 Shadow** (done in Cloud, 2026-10-13) | Real graph, real monitors, keyboard path; nothing sent | §10 phase 1, checks 1.1–1.13: 10 PASS, 1.7 FAIL as written, 1.11 N/A, 1.13 partial ([evidence](evidence/m55/shadow_cloud_20261013T050401Z/README.md)) | No | Already covered: `m55_fake`, isolated-domain tests | — | done |
| **S0b Fixes** (code and tests done, 2026-10-13; the re-run is folded into S2's A2) | Two S0 findings: (1) the 50 Hz receive-time rate check rejects bunched heartbeats and stops a held walk; (2) check 1.7 bounds a sim-time phase by wall time | (1) a tolerant rate rule with tests (fakes, injected jitter) and S0 re-run, 0 spurious rejections; (2) 1.7 restated in sim time or scaled by RTF | No; the owner approves the criteria change | Yes, fully | Masking a real flood: keep a hard flood limit | 0.5 |
| **S2 Fixed-base crawl** ([design](S2_FIXED_BASE_CRAWL_DESIGN.md)) | All 54 phase goals on the real controller, no contact | Each level, one cycle forward and one reverse: every phase SUCCEEDED; tracking ≤ 0.02 rad (abort 0.05); continuity ≤ 0.005 rad; goals = phases; stop, estop and lease once each | **Yes**: approval, enabling branch, a cycle-style plan (A2 rehearsal, restore and verify, as M6.1) | Rehearsal with `m55_fake` and a read-only probe | RTF 0.2: one 17–24 s cycle takes about 85–120 s wall; the long run raises the stall and watchdog exposure | 1–1.5 |
| **S3 Bounded free-base crawl** | First contact; level 0 only | 1 cycle forward then stop, 1 reverse then stop, then 5 cycles: tilt ≤ 0.10 rad; 10–30 mm per cycle; drift lateral ≤ 10 mm, yaw ≤ 0.05 rad; height ±10 mm | **Yes**: a new approval and enabling branch | Not for contact; only the session logic | The contact model is assumed (slip, compliance); CAD masses; first-contact tipping | 1–2 |
| **S4 Keyboard walking** | The goal itself | 0.3 m forward at the fastest level, then back: tilt ≤ 0.10 rad, lateral ≤ 30 mm, heading ≤ 0.1 rad; release, speed change, reversal, `home`; estop gives one cancel; teleop close gives DISARMED ≤ 6 s | **Yes**: a new approval | No | 0.3 m at 2.34 mm/s ≈ 128 s sim ≈ 11 min wall at RTF 0.2; drift accumulates | 1 |

The M5.5 doc names a Phase 1 "on the owner's PC". S0 ran it in Cloud. A local re-run is only
needed for the real-keyboard checks (1.11, and 1.13's crash path) and is optional for simulation.

**Total to simulated keyboard walking: about 4–5 units**, plus owner approvals for S2, S3 and S4.

## c) The cheapest valid order

1. **S0b** now (no approval for motion; code and tests with fakes; a Cloud shadow re-run in the
   same session).
2. **S2** under one approval: all three levels in one launch.
3. **S3**, reviewed.
4. **S4.** S3 and S4 *may* share one enabling branch and one launch **only** with a hard stop
   after S3's 5 cycles: an automatic check of the S3 criteria plus the owner's go/no-go before
   any faster level.

**Allowed combinations:** S0b with any read-only rehearsal; the forward and reverse runs of all
levels within S2; S3 with S4 behind the hard stop above.

**Must NOT be combined:**
- the fixed base (S2) and the free base (S3/S4) in one launch or one approval: different plant,
  launch and claims;
- the M5.5 and M6.1 gates enabled together;
- S3's first contact with any level above 0;
- skipping S2: it is the only check of the 54 phase goals on the real controller without
  contact;
- any enabling branch pushed or merged; any retry inside a run; any fix during a run.

## d) Claim boundaries

| Evidence | Allowed claim | Not allowed |
|---|---|---|
| **Fixed base** (M6.1, Cloud, done) | One approved trot-cycle joint trajectory was executed and tracked (0.0085 rad) on a body welded at 0.125 m | Walking, stepping, contact, balance |
| **Fixed base, S2** | All M5.5 crawl phase goals executed and tracked on a welded body | The same |
| **Free base, S3** | A bounded level-0 crawl in simulation moved the body by the measured amount with tilt and drift within the stated limits (CAD masses, assumed contact) | Walking on command, robustness, balance control (there is no feedback balance) |
| **Walking, S4** | Keyboard-commanded crawl walking in simulation, forward and back, within the stated limits, at 1–2.3 mm/s | Real-world or hardware walking, rough terrain, odometry or navigation, dynamic stability |
| Any stage | — | Anything about hardware: M10 |
