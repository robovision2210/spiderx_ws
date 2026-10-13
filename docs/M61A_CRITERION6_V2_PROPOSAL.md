# M6.1-A phase-1 criterion 6: decision on version 1 and a frozen version-2 proposal

**Status: PROPOSAL, frozen before its single validation observation.** Nothing here relabels an
earlier result. Version 1 stays the criterion of record for runs 1–3, and it **failed** there.
Version 2 applies only to observations made after this file and its spec were committed and
pushed. Its first and only validation is one fresh no-motion observation (`run_04`). Whether
version 2 replaces version 1 for phase 1 is the owner's decision.

Machine-readable definition (frozen):
[`evidence/m61a_cloud/criterion6_v2/crit6_v2_spec.json`](evidence/m61a_cloud/criterion6_v2/crit6_v2_spec.json).
Evaluation tool: `crit6_v2.py`. Synthetic self-test: `selftest_crit6_v2.py`.

## 1. Version 1: the original wording and what it measures

From `docs/M61A_FIXED_BASE_IMPLEMENTATION.md` §10 (introduced in `9d24b1e`), verbatim:

> 6. Independent cross-check of the pose source: the `/scan` ranges to the world walls agree with
> the welded pose (0, 0, 0.125 m, yaw 0) to within the M0 ±8 mm. A disagreement means
> `pose/info` does not describe the simulated body, and nothing else in this list can be
> trusted.

- **Objective:** the pose. The criterion checks that `pose/info` describes the simulated body.
- **Measurement as written:** individual ranges. "The ranges … agree … within ±8 mm", with the
  tolerance taken from M0 (`MIGRATION_FORTRESS.md`). M0 compared individual wall and pillar
  distances at expected bearings.
- It is neither a surface statistic nor a fitted pose. Those were supplementary metrics added in
  the closeout (6a, 6b).

## 2. Version-1 results are retained

| Run | 40-scan means within ±8 mm | Single-scan ranges within ±8 mm | Version 1 |
|---|---|---|---|
| run_01 | 85.8 % (max 17.4 mm) | 52.4 % | **FAIL** |
| run_02 | 88.4 % (max 18.0 mm) | 52.7 % | **FAIL** |
| run_03 | 84.4 % (max 18.9 mm) | 52.5 % | **FAIL** |

- The supplementary 6a and 6b values stay as reported in the verification document §8.
- They are not a criterion-6 PASS for any of these runs.

## 3. Why a revision is justified (independently of any metric that passes)

The justification comes from the sensor model, characterized on the run_01–03 data. Those runs
are used for characterization only and are not judged under version 2.

1. **Noise alone defeats single ranges.** The `gpu_lidar` noise is σ = 10 mm per scan (SDF; the
   measured per-beam sd is 9.85–9.99 mm). A perfect pose and a perfect model would therefore put
   about 42 % of single-scan ranges outside ±8 mm. Observed: 52–53 % within.
2. **The 40-scan means carry a systematic rendering error that no pose can remove.**
   - The error is about 5 mm RMS. It grows with the angle of incidence: 2.4 mm RMS at 0–15°,
     5.4 at 15–30°, 8.7 at 30–40°.
   - It is identical across launches. The run-to-run difference has an RMS of 2.16 mm, against a
     noise-only expectation of 2.24 mm; the correlation between runs is 0.92.
   - It is uncorrelated between neighbouring beams (|r| ≤ 0.18 at lags 1–8).
   - A minimax search over the planar body pose finds the smallest maximum |residual| any pose can
     reach: **16.7–17.1 mm** in every run, confirmed by exact ray casting.

   **No pose of the body satisfies version 1.** Version 1 would fail a body that is exactly where
   `pose/info` says, so it cannot tell a correct pose source from a wrong one: it measures the
   sensor, not the pose.

## 4. Version 2 (proposal)

### 4.1 Metric and scope

**Metric.** The body (`base_link`) planar pose (x, y, yaw) is fitted to the 40-scan per-beam
means against the world collision geometry. The fit is a robust Gauss–Newton:
- Huber 0.02 m, step clip 0.05, at most 40 iterations;
- a **fixed** beam set with no re-selection.

The fitted pose is compared with the body pose that `pose/info` reports in the same capture
window (`T_world_model · T_model_dummy · T_dummy_body`). `|dxy|` is the Euclidean planar
distance.

**Frozen beam exclusions** (from the closeout, as explicit lists):
- cube-map seams: 45, 46, 135, 136, 225, 226, 315, 316;
- edge grazing at the weld pose: 117, 118, 210, 222, 309, 334.

That leaves 346 beams.

**Scope.** x, y and yaw only. A horizontal scan of vertical walls cannot observe z, roll or
pitch; see §5.

### 4.2 Thresholds and how they are derived

**Objective.** Detect a `pose/info` body pose that is wrong by the size the run itself must
catch: the G8 attachment tolerance, **δ_xy = 3 mm** and **δ_yaw = 0.01 rad**
(`config/m61a_fixed_base.yaml` `attachment`).

**Uncertainty.** For each component, the uncertainty is the larger of two estimates:
- σ_robust: 1.4826 · MAD of the per-beam residuals × √diag((JᵀJ)⁻¹). The per-beam errors are
  independent (§3.2), so this estimate is valid for them.
- σ_jackknife: a leave-one-surface-out jackknife over the 7 surfaces. It captures surface-level
  systematic error.

**Decision-error targets** (conventional): a false FAIL at zero discrepancy ≤ 1 %, and a miss at δ
≤ 5 %.
- **xy:** with no discrepancy, |dxy| follows a Rayleigh distribution, so τ ≥ σ·√(−2 ln 0.01) =
  3.035 σ. At δ, a miss requires τ ≤ δ − 1.645 σ. Both hold only if σ ≤ δ / 4.680.
- **yaw:** two-sided, τ ≥ 2.576 σ and τ ≤ δ − 1.645 σ. Both hold only if σ ≤ δ / 4.221.

| Quantity | xy | yaw |
|---|---|---|
| Required resolving power σ_max | **0.641 mm** | **2.369 mrad** |
| Threshold τ = δ − 1.645 σ_max | **1.946 mm** | **6.103 mrad** |

**Sensitivity of the targets.** A stricter pair (0.3 % / 2.3 %) would need σ_xy ≤ 0.55 mm. The
jackknife reached 0.585 mm in one characterization run (0.33–0.59 mm across the three; σ_robust
was 0.25–0.31 mm). The conventional pair is used, and this is recorded rather than tuned away.

### 4.3 Verdict rule

- **PASS:** every prerequisite holds, |dxy| ≤ τ_xy and |dyaw| ≤ τ_yaw.
- **INCONCLUSIVE, never PASS:** any prerequisite fails. The prerequisites are:
  - at least 40 scans;
  - at least 95 % of the frozen beams finite;
  - at least 50 beams on each of the four walls;
  - `pose/info` constant within the window (≤ 0.1 mm and ≤ 0.1 mrad spread);
  - σ_used ≤ σ_max.
- **FAIL:** everything else.

### 4.4 Counterfactual sensitivity

Recorded with the result. The capture's own measured per-beam error pattern is re-applied to a
body displaced by:
- (+3, 0) mm;
- (0, +3) mm;
- (−2.12, +2.12) mm;
- ±0.01 rad;
- (+1, 0) mm;
- +0.002 rad.

The frozen fit must recover each offset within 3 σ_used. Every offset at or above δ must FAIL.

**Synthetic self-test** (`selftest_crit6_v2.py`; no recorded run used):
- no discrepancy: PASS;
- body displaced 3 mm while `pose/info` reports the weld: FAIL;
- 30 scans: INCONCLUSIVE.

## 5. What version 2 does not claim

- **It does not confirm z, roll or pitch** (verification document §9, U-L2).
- **It does not certify criterion 2's 1 mm / 0.0033 rad margins.** The sensor resolves the
  planar pose to about 0.3–0.6 mm (1σ), which is enough for the 3 mm G8 tolerance but not for a
  1 mm acceptance band. Those margins remain checks of the pose source against itself.
- **Its independence is limited:**
  - It is independent of the SceneBroadcaster → bridge → ROS path, of the entry selection and
    frame composition in `m61a_fixed_base`, and of `frame_id`.
  - It is **not** independent of Gazebo's entity-component state. The `gpu_lidar` is rendered
    from the same model and link pose components that `pose/info` publishes.
  - The lidar mount (`lidar_joint`, URDF) and the body chain are checked jointly. Two errors that
    cancel exactly are not excluded.

## 6. Validation plan (one observation)

- **Order.** Commit and push this file, the spec, the tool and the self-test; only then run
  `run_04`.
- **Procedure.** `run_04` is one fresh `fortress_m61a_fixed_base.launch.py` launch with the frozen
  harness of the closeout:
  - `run_one.sh` and the `observation_7f4c30f/tools/` files, SHA-256s unchanged;
  - no motion and no goal;
  - its own log directory.
- **Evaluation.** `crit6_v2.py` is applied once, unmodified, to `run_04/captures/capture.json`.
  Versions 1 and 2 are reported side by side.
- **If a defect is found.** Any defect in the tool or the definition after `run_04` makes the
  result void. A corrected version would be version 3, frozen before a new observation.
