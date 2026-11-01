# S3-r1 adoption record (owner decision, 2026-11-01)

**Decision.** The owner adopted criteria revision S3-r1 for the recorded run `s3_free_base_20261101T115002Z`.

**Qualification, exactly:**

> S3 bounded level-0 free-base crawl accepted retrospectively under criteria revision S3-r1.

## Linked records
| Record | Path | Commit |
|---|---|---|
| The run (raw evidence, frozen scorer output) | [`../s3_free_base_20261101T115002Z/`](../s3_free_base_20261101T115002Z/README.md) | `a73ea95` |
| Review tool and tests | `scripts/s3/review_a3.py`, `scripts/s3/test_review_a3.py` | `ea0203f` |
| The A3 and foot-travel review, which defines S3-r1 (§B) | [`../s3_free_base_20261101T115002Z_a3_review/REVIEW.md`](../s3_free_base_20261101T115002Z_a3_review/REVIEW.md) | `2859c8c` |
| Addendum 1: rounding wording (no change to the assessment) | [`../s3_free_base_20261101T115002Z_a3_review/ADDENDUM_1.md`](../s3_free_base_20261101T115002Z_a3_review/ADDENDUM_1.md) | this change |

## Preserved unchanged
- **Original frozen verdict: INCONCLUSIVE, with A3 UNMEASURED.** It is in `../s3_free_base_20261101T115002Z/score/summary.json`, from the frozen scorer `scripts/s3/score_s3.py` (SHA-256 `1c8ef984…`).
- **The review's `review.json`, including `"adopted": false`.** That was its historical state when it was generated, before this decision. It is not edited.
- **The raw evidence, the review's `REVIEW.md`, and every historical manifest.**

## Result under S3-r1
S3-r1 replaces only the final-estop exception's point-time check. The new check uses the inferred write window of the scripted `x` and adds recorded provenance checks. All 17 checks hold.

| # | Original (frozen) | Under S3-r1 |
|---|---|---|
| A1 | PASS | PASS |
| A2 | PASS: 6 cycles, forward 20.015–20.029 mm (≈ 0.1201 m in all), \|lateral\| ≤ 19 µm, \|yaw\| ≤ 8.7e-5 rad, height within 35 µm, tilt ≤ 0.0026 rad | PASS (unchanged) |
| A3 | **UNMEASURED** | **PASS**: goals 1–54 measured, max \|error\| 0.00225 rad (abort 0.05, no finding > 0.02); goal 55 N/A |
| A4 | PASS | PASS |
| A5 | PASS: 55 goals, 1 cancel, no foreign client or publisher | PASS |
| A6 | PASS: 10 s sim hold, joints 1.2e-11 rad, tilt 3.1e-6 rad, z ≥ 0.054495 m | PASS |
| A7 | PASS: DISABLED AND VERIFIED | PASS |
| A8 | PASS: `library.json` `917fc4d7…` | PASS |

## Known limitations of this acceptance
- **Goal 55's tracking remains N/A, not a measured value.** The scripted estop cancelled it before any feedback. The A6 hold measurements do not replace trajectory tracking.
- **Estop-source attribution is inferred** from the recorded chain:
  - one scripted estop key;
  - one matching teleop response;
  - the node's only estop request;
  - no fallback call;
  - the controller's log order.

  The node does not record which client called its estop service.
- **There is no direct contact-point-slip measurement.** The 27.3 mm in `slip.csv` is the travel of the foot link's origin, the knee/foot-joint axis. The CAD-tip material point moved at most 0.11 mm horizontally on stance feet; that is a proxy, and it does not establish zero slipping.
- **Simulation only, with an idealized actuator model.**
  - Gazebo Fortress with DART at a 1 ms step.
  - The gain-1.0 position-proportional `gz_ros2_control` overlay (r2).
  - The 100 N·m placeholder effort.
  - CAD masses, and assumed contact at mu 0.2.
- **Scope.**
  - The test was scripted teleop keys through a pseudo-terminal, level 0, forward only.
  - Reverse motion and the faster levels have not been run on contact.

## Not shown
Interactive keyboard walking (S4), reverse or faster-level motion on contact, terrain, balance control, odometry, autonomous navigation, and anything about hardware.
