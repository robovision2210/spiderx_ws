# M5 Plan – Offline Research Evaluation Protocol for the M4.5 Gait Framework

> **Phase 0 (audit and plan only).** This plan was written and committed **before** any M5 code.
> Branch `claude/spiderx-m5-offline-evaluation-plan`, created from `origin/main` @ `3db1aec`
> (the merge of PR #11, M4.5). The tree of `3db1aec` is identical to the M4.5 head `c24c4c0` that
> was verified in the cloud and locally (`git diff c24c4c0 3db1aec` is empty).
>
> **Status: Cloud-validated and locally verified offline evaluation.** Offline model results only; no Gazebo, runtime, real-time or hardware validation.
> - **Phase 0.** The owner approved the audit and plan (`4c1bd1d`) and decided D1–D5; see
>   [§16](#16-owner-decision-addendum).
> - **Phase 1.** Implemented in Batches A–E (`1f8e323`, `5450ab0`, `39c0f6d`, `1aa33d5`,
>   `d2c32f5`), plus the owner-approved M4.5 phase-boundary fix (`ece1e23`) that the Stage 0/1
>   gate found.
> - **Cloud validation.** 700 tests with 0 failures, and two byte-identical complete study runs.
>   See [M5_TEST_RESULTS.md](M5_TEST_RESULTS.md) and
>   [SPIDERX_M5_EVALUATION_GUIDE.md](SPIDERX_M5_EVALUATION_GUIDE.md).
> - **Not complete.** Local verification passed. The study is not complete until the owner
>   merges the PR.

```text
OFFLINE model analysis only. M5 evaluates gait CONFIGURATION CLASSES on the SpiderX URDF
kinematic model with the unchanged M4.5 evaluator. No Gazebo, no gait playback, no dynamic
walking, no hardware. Nothing in SpiderX is verified as walking, navigating or running on hardware.
```

**Labels used in this document**

| Label | Meaning |
|---|---|
| **[FACT]** | Read in this repository at `3db1aec`, with a file:line reference, or proven by its tests |
| **[RESULT]** | An existing M4.5 output (committed docs or regression pins), or an audit probe run in memory at the M4.5 tree. It is true *of the model*, not of the robot |
| **[HYPOTHESIS]** | A proposed scientific interpretation, to be tested and reported whichever way it falls |
| **[PLAN]** | A proposed M5 Phase 1 implementation or protocol rule |
| **[OWNER DECISION]** | A choice only the owner can make (§15.2) |

---

## Contents

1. [Status and scope boundary](#1-status-and-scope-boundary)
2. [Audited M4.5 foundation and provenance](#2-audited-m45-foundation-and-provenance)
3. [Definitions and units](#3-definitions-and-units)
4. [Research question and hypotheses](#4-research-question-and-hypotheses)
5. [Baseline reference table](#5-baseline-reference-table)
6. [Experiment matrix and run-count estimate](#6-experiment-matrix-and-run-count-estimate)
7. [Variables and controls](#7-variables-and-controls)
8. [Metric validity and non-claims](#8-metric-validity-and-non-claims)
9. [Sampling, resolution and numerical sensitivity](#9-sampling-resolution-and-numerical-sensitivity)
10. [Inclusion, exclusion and negative controls](#10-inclusion-exclusion-and-negative-controls)
11. [Data schema, artifacts and reproducibility](#11-data-schema-artifacts-and-reproducibility)
12. [Planned M5 Phase 1 implementation batches](#12-planned-m5-phase-1-implementation-batches)
13. [Test strategy](#13-test-strategy)
14. [Paper-safe reporting language](#14-paper-safe-reporting-language)
15. [Risks, limitations and owner decisions](#15-risks-limitations-and-owner-decisions)
16. [Owner-decision addendum](#16-owner-decision-addendum)

---

## 1. Status and scope boundary

**Status.**
- **[FACT]** M4.5 is merged into `main` (`3db1aec`) and verified in the cloud and locally
  (`docs/M4_5_TEST_RESULTS.md:9`, `:200`).
- **[PLAN]** M5 Phase 0 delivers this document only. Phase 1 starts after the owner reviews it
  and answers §15.2.

**M5 may:**
- compare gait **configuration classes** (footfall pattern × duty factor × stroke × lift) as
  **offline model predictions**;
- report how the M4.5 metrics respond to controlled parameter changes;
- state where a metric is undefined, provisional or non-discriminating.

**M5 must never claim:**
- walking, gait performance or a "best gait" for the robot;
- dynamic stability, balance or terrain robustness;
- trajectory tracking;
- energy, power, efficiency or cost of transport;
- servo feasibility;
- anything about hardware.

**Phase 0 boundary.** Phase 0 makes no changes to:
- code, tests or any YAML (including `m4_5_gaits.yaml`);
- the URDF, meshes, launch files, Gazebo files or controller files;
- dependencies or the existing docs, STATUS or roadmap.

It generates no data and no plots, runs no runtime validators and opens no PR.

**Phase 1 boundary [PLAN].**
- **Additive only.** New modules and a new study file; the M4.5 modules and `m4_5_gaits.yaml`
  are reused **unchanged**.
- **Body sway is excluded.** It stays future work only (owner decision recorded at
  `docs/M4_5_TEST_RESULTS.md:300`).

## 2. Audited M4.5 foundation and provenance

### 2.1 Repository state [FACT]

| Item | Value |
|---|---|
| `origin/main` | `3db1aec` (merge of PR #11). Tree identical to `c24c4c0` |
| M4.5 commits | plan `2d8b07d`; Batches A `6dca08e`, B `fd03a90`, C `c39f501`, D `f0a3245`, E `ab9f08e`, F `3e59f33`; docs `4a6553c`, `c24c4c0` |
| Test evidence | 556 tests, 0 failures, cloud and local; M4.5 adds 165 pytest cases (`docs/M4_5_TEST_RESULTS.md:44-60`, `:200-235`) |
| Determinism evidence | 40 output files byte-identical across two runs, in the cloud and locally (separately). Cloud and local outputs were **not** byte-compared with each other; verdicts and metrics agreed at the reported precision |
| Artifact policy | `.gitignore:3` `log/` (M4.5 writes to `log/m4_5_gait_analysis/`, `m4_5_gait_analysis.py:22`) |
| Literature | No paper, citation or bibliography exists anywhere in `docs/` (searched). See §15.2 D5 |

### 2.2 Files audited (all at `3db1aec`)

**Docs**
- `docs/M4_5_PLAN.md` (§5.1 conventions :132, §5.2 gait set :166, §5.4 metric labels :211,
  §7 naming issues :244);
- `docs/M4_5_TEST_RESULTS.md` (results :91-140, findings :141-182, local verification :200,
  owner decisions :300);
- `docs/SPIDERX_GAIT_FRAMEWORK.md` (terminology :56, debugging and reach :231);
- `docs/SPIDERX_DEVELOPMENT_ROADMAP.md` (M4.5 :91, future gait playback :106, **M5 =
  `/cmd_vel` bridge :113**);
- `STATUS.md` (:5, :96, M5 bridge :98);
- `docs/SPIDERX_TESTING_GUIDE.md` (layer 23 :40);
- `docs/URDF_INERTIA_AUDIT.md:46-50` (steel-density masses).

**Config:** `config/m4_5_gaits.yaml` and `config/spiderx_legs.yaml` (:17 margin, :24
placeholder).

**Code:** `gait_config.py`, `gait_phase.py`, `gait_trajectory.py`, `gait_kinematics.py`,
`gait_results.py`, `gait_metrics.py`, `gait_report.py` and `m4_5_gait_analysis.py`, plus the reused
`leg_kinematics.py` (`inverse` :465-579, `load_all_geometries` :603, `IK_POSITION_TOL_M` :40,
`DEFAULT_MARGIN_RAD` :44).

**Tests:** `test_gait_config.py` (52), `test_gait_trajectory.py` (41), `test_gait_kinematics.py`
(12), `test_gait_metrics.py` (21), `test_gait_report.py` (19), `test_gait_regression.py` (20; pins
at :18-33).

### 2.3 Reused interfaces [FACT]

Data flow: `m4_5_gaits.yaml → gait_config.validate_config → GaitSpec → gait_trajectory.sample_cycle →
gait_kinematics.solve_cycle (leg_kinematics.inverse) → gait_metrics.evaluate_gait → GaitEvaluation`.

| Interface | Location | M5 use [PLAN] |
|---|---|---|
| `validate_config(cfg, legs_cfg)` | `gait_config.py:249-309` | Every generated configuration passes **the same** validation as the shipped YAML |
| `GaitSpec` (frozen) | `gait_config.py:60-84` | One per experiment point |
| `load_inputs()` | `gait_metrics.py:336-347` | Geometry and `MassModel` loaded once per study |
| `evaluate_gait(spec, cfg, geoms, mass, n)` | `gait_metrics.py:299-333` | The only evaluator; it is not modified |
| `GaitEvaluation.checks / metrics / series` | `gait_metrics.py:146-160` | Raw outputs; `series` gives the per-sample support and margin for derived tables |
| `fmt`, `jsonable`, `write_json`, `write_csv` | `gait_report.py:41-66`, `:125` | Deterministic number formatting (10 significant digits) |

## 3. Definitions and units

All quantities are in `base_link`: **+x right, +y front, +z up** (not REP-103). The body moves
along +y at constant speed, level, with no sway (`gait_trajectory.py:1-17`). SI units throughout.

| Symbol | Name | Definition | Unit | Source |
|---|---|---|---|---|
| u | normalised cycle time | t / T ∈ [0, 1) | – | `gait_phase.py:3-10` |
| β | duty factor | fraction of the cycle a foot is in stance | – | `gait_config.py:211-213` |
| φᵢ | phase offset | leg i swings while (u − φᵢ) mod 1 < 1 − β | – | `gait_phase.py:34-40` |
| L | stroke (`step_length_m`) | foot travel **relative to the body** during stance | m | `gait_config.py:14` |
| h | step height | swing lift amplitude (cycloid) | m | `gait_trajectory.py:13` |
| v | body speed | v = L / (β T) | m/s | `gait_config.py:230-235` |
| T | cycle period | T = L / (β v) | s | same |
| S | stride | S = v T = L / β, body travel per cycle | m | `gait_config.py:78-81` |
| n | samples per cycle | u_k = k / n, k = 0 … n−1 | – | `gait_phase.py:48-52` |
| K | **speed-normalised peak joint-speed demand** [PLAN, derived] | max joint speed / v | rad/m | §8, probe §6.1 |
| v_adm(ω) | admissible body speed under a joint-speed reference ω [PLAN, derived] | ω / K | m/s | §8 |
| "pattern" | footfall pattern class | the phase-offset vector, up to the swing-order check | – | `gait_config.py:158-196` |

**Body-frame swing path [FACT, derived from `gait_trajectory.py:53-56`].** Substituting S = L/β
and the swing duration τ = (1 − β)T gives

  y(s) = −L/2 + L·s − (L/β)·sin(2πs)/(2π),  z(s) = h(1 − cos 2πs)/2,  s ∈ [0, 1).

The path depends on **L, h and β only**, not on v. Its fore-aft excursion beyond ±L/2 is set by
β (§4, H3).

## 4. Research question and hypotheses

**Primary research question.**

> Within the SpiderX URDF kinematic model and the unchanged M4.5 offline evaluator, how do footfall
> pattern, duty factor β, stroke L and step height h change:
> - (i) sampled-IK feasibility and workspace margins;
> - (ii) the quasi-static support-margin approximation;
> - (iii) speed-normalised peak joint-speed demand?
>
> Heuristic energy proxies are reported descriptively only.

Speed is **not** a free factor: the audit proves it is a pure time scale (§6.1). Step height h
enters (i)–(iii) through the swing path.

**Hypotheses.** These are pre-registered before any Stage 2 or Stage 3 run. Each one has an
explicit falsification criterion. **[HYPOTHESIS]**

| ID | Statement | Supported if | Falsified if | Evidence today |
|---|---|---|---|---|
| **H1** (support) | For the lateral-sequence (LS) pattern with zero stance-centre offset, the minimum three-or-more-foot static margin approximation stays below the 5 mm screening value for every tested L at β ≥ 0.75. The binding support triangle is FL–FR–RR, which is the rear-left swing | every LS config with β ∈ {0.75, 0.85} has min margin < 5 mm, **and** the binding triangle is FL–FR–RR in each | any such config reaches ≥ 5 mm, or a different triangle binds | [RESULT] wave −2.58 mm, tripod_crawl −4.07 mm at L = 40 mm; all listed failures are FL–FR–RR (`docs/M4_5_TEST_RESULTS.md:141-160`); COM ≈ 5.8 mm behind the foot centre |
| **H2** (joint-speed demand) | K (rad/m) increases strictly with β at fixed (L, h), and increases with h at fixed (β, L) | strictly monotone in β for every (L, h) cell that is IK-feasible at all five β; non-decreasing in h for every (β, L) | any reversal larger than the Stage 1 discretisation band (§9) | [RESULT, derived] K at L = 40 mm, h = 15 mm: 25.4 (β 0.5), 29.3 (0.55), 40.6 (0.65), 61.0 (0.75), 108.1 (0.85) rad/m |
| **H3** (feasibility boundary) | For each (β, h) there is a largest IK-feasible stroke L_max(β, h). It decreases as h increases, and it decreases as β decreases, because the body-frame swing overshoot grows as β falls (§3) | L_max is non-increasing in h and non-decreasing in β across the tested grid | a tested cell violates either monotonicity by more than one L level | No data yet. Today every baseline configuration is feasible at L = 40 mm |

**Energy proxies.** No hypothesis is stated about them. They are secondary, descriptive outputs
and are never optimisation targets or ranking criteria (§8, §14).

## 5. Baseline reference table

### 5.1 Configurations [FACT]

Source: `config/m4_5_gaits.yaml:45-97`. All six share L = 0.04 m, h = 0.015 m, v = 0.005 m/s,
stance offsets 0 and a cycloid swing (`:53-59`).

| Gait | YAML lines | β | Offsets (LF, RF, LR, RR) | Pattern class | T (s) | S (mm) | `requires_static_stability` |
|---|---|---|---|---|---|---|---|
| `wave` | 62-67 | 0.85 | 0.25, 0.75, 0.00, 0.50 | LS singles | 9.41 | 47.1 | true |
| `tripod_crawl` | 68-73 | 0.75 | same | LS singles | 10.67 | 53.3 | true |
| `ripple` | 74-79 | 0.65 | same | LS singles | 12.31 | 61.5 | false |
| `amble` | 80-85 | 0.55 | same | LS singles | 14.55 | 72.7 | false |
| `pace` | 86-91 | 0.50 | 0.00, 0.50, 0.00, 0.50 | lateral pairs | 16.00 | 80.0 | false |
| `trot` | 92-97 | 0.50 | 0.00, 0.50, 0.50, 0.00 | diagonal pairs | 16.00 | 80.0 | false |

**Pattern structure [FACT].** The four "LS singles" gaits are **one** footfall pattern at four duty
factors. So the six shipped gaits span only three pattern classes, and β and pattern are
partly confounded in the shipped set. This is why M5 varies β within each pattern (§6).

### 5.2 Results at n = 200 [RESULT]

Sources: `test_gait_regression.py:18-33` (pins) and `docs/M4_5_TEST_RESULTS.md:91-182`.

| Gait | Verdict | Failed checks | Min static margin | Stable* | Max joint speed | K (rad/m) | v_adm(0.5) (mm/s) | Lift proxy (J/m) | Joint travel (rad/m) |
|---|---|---|---|---|---|---|---|---|---|
| `wave` | FAIL | `static_stability` (61/200), `joint_speed` (16 breaches, 4 per thigh joint; max on `lf_thigh_joint`) | −2.58 mm | 84.5 % | 0.5405 rad/s | 108.1 | 4.63 | 5.124 | 90.72 |
| `tripod_crawl` | FAIL | `static_stability` (108/200) | −4.07 mm | 77.0 % | 0.3048 | 61.0 | 8.20 | 4.642 | 81.38 |
| `ripple` | PASS | – | −2.10 mm (info) | 50.5 % | 0.2029 | 40.6 | 12.3 | 4.174 | 72.25 |
| `amble` | PASS | – | +0.84 mm (info) | 20.0 % | 0.1465 | 29.3 | 17.1 | 3.719 | 63.37 |
| `pace` | PASS | – | n/a (never ≥ 3 feet) | 0 % | 0.1269 | 25.4 | 19.7 | 3.498 | 59.01 |
| `trot` | PASS | – | n/a (never ≥ 3 feet) | 0 % | 0.1269 | 25.4 | 19.7 | 3.498 | 59.01 |

\* `fraction_statically_stable` counts samples with margin **> 0**, out of **all** samples
(`gait_metrics.py:320-321`). It does not use the 5 mm threshold. See §8.

**Common to all six [RESULT]:**
- sampled IK feasible at every sample;
- worst FK residual 9.0e-17 m;
- min singularity margin 1.2116 rad;
- min joint-limit margin 0.2050–0.2051 rad;
- no branch flips;
- max transition velocity jump 1.4e-9 m/s.

**Provisional values [FACT].**
- The 0.5 rad/s reference is a `SIMULATION_PLACEHOLDER` (`spiderx_legs.yaml:24`).
- The 5 mm static margin is a convention fixed in M4.5 (`m4_5_gaits.yaml:47`).
- The masses are CAD steel-density placeholders (`URDF_INERTIA_AUDIT.md:46-50`).
- The K and v_adm columns are derived arithmetic from the pinned values (§6.1).

## 6. Experiment matrix and run-count estimate

### 6.1 Structural facts that shape the design

These come from the code, confirmed by an in-memory audit probe at the M4.5 tree. The probe
wrote no files.

1. **Speed is a pure time scale [FACT + RESULT].** The body-frame path does not contain v (§3).
   Joint angles depend only on the path, and T = L/(βv). The probe (wave and trot, v vs 2v,
   n = 200) gave:
   - **identical** joint-angle series;
   - max joint speed ×2.000 exactly;
   - transition acceleration jump ×4.000;
   - static margin, stable fraction, singularity margin, limit margin, max joint step, foot path,
     joint travel and lift proxy all **unchanged** to every printed digit.

   **[PLAN]** Speed is therefore **not gridded**. Each configuration reports K and v_adm(ω) for
   any reference ω, plus the v₀ = 5 mm/s values for continuity with M4.5.
2. **Per-leg metrics do not depend on the pattern [FACT, structural].** Every leg follows the same
   offset trajectory, shifted in u by its φᵢ (`gait_trajectory.py:43-60`). When every φᵢ·n is an
   integer, each leg's sampled series is an exact cyclic shift. So IK feasibility, FK residual,
   singularity and limit margins, joint step, joint speed, joint travel and the lift proxy are
   identical across patterns. [RESULT] pace = trot on all of these (`test_gait_metrics.py:106-116`).
   Only **support metrics** (support counts, COM, static margin) depend on the pattern.
   **[PLAN]** The design splits into a *kinematic block* (pattern-independent) and a *support
   block* (pattern-dependent), and a test confirms the invariance.
3. **Grid alignment matters [FACT].** Support fractions are exact only if every phase transition,
   at u = φᵢ and φᵢ + 1 − β, lies on the 1/n grid. Otherwise they are sampled approximations.
   **[PLAN]** Every level combination must satisfy φᵢ·n ∈ ℤ and (1 − β)·n ∈ ℤ for every n used.
   The generator refuses misaligned points.

### 6.2 Staged design (recommended; alternative in §15.2 D3) [PLAN]

**Fixed for every stage:**
- v₀ = 0.005 m/s (time scale only);
- stance height offset 0 and stance centre (0, 0) (Stage 4 is the only exception);
- cycloid swing;
- the M4.5 `analysis` thresholds;
- n = 200, except in Stage 1.

| Stage | Purpose | Factors and levels | Evaluations | New unique (config, n) |
|---|---|---|---|---|
| **0 + 1** Baseline and resolution | Reproduce the M4.5 pins exactly at n = 200; quantify discretisation | 6 shipped configs × n ∈ {40, 100, 200, 400, 800} | **30** | 30 |
| **2** Kinematic block | H2, H3: feasibility boundary, margins, K; descriptive proxies | LS pattern × β ∈ {0.50, 0.55, 0.65, 0.75, 0.85} × L ∈ {0.02, 0.04, 0.06, 0.08, 0.10, 0.12} m × h ∈ {0.005, 0.010, 0.015, 0.020, 0.025, 0.030, 0.035} m | **210** | 206 (4 equal the baseline wave, tripod_crawl, ripple, amble) |
| **3** Support block | H1: support margin and its binding triangle vs pattern and β | {LS, trot pairs, pace pairs} × β ∈ {0.50, 0.55, 0.65, 0.75, 0.85} × L ∈ {0.02, 0.04, 0.06} m, h = 0.015 m | **45** | 28 (15 LS equal Stage 2 points; trot and pace at β 0.5, L 0.04 equal the baseline) |
| **4** (approved, D4) Stance-centre translation sensitivity | Mechanism check for H1: a fixed, constant translation, **not** body sway (§16 D4) | LS × β ∈ {0.75, 0.85} × stance-centre y ∈ {−10, −5, 0, +5} mm, L = 0.04 m, h = 0.015 m | 8 | 6 |

**Totals:**
- **Core staged design (Stages 0–3): 285 evaluations, 264 unique (configuration, n)
  evaluations.**
- **With the approved Stage 4: 293 evaluations, 270 unique.**

The runner deduplicates by configuration hash, so 270 evaluations actually run.

**Overlap accounting (exact):**

| Stage | Planned | Already evaluated in an earlier stage | New unique (configuration, n) |
|---|---|---|---|
| 0 + 1 | 6 configs × 5 n = 30. Stage 0 (the baseline at n = 200) *is* the n = 200 subset of Stage 1, so it is counted once | – | 30 |
| 2 | 5 β × 6 L × 7 h = 210 | 4: the LS points at L = 0.04 m, h = 0.015 m with β ∈ {0.55, 0.65, 0.75, 0.85} are amble, ripple, tripod_crawl and wave at n = 200 | 206 |
| 3 | 3 patterns × 5 β × 3 L = 45 | 17: all 15 LS points (h = 0.015 m, L ∈ {0.02, 0.04, 0.06} m) are Stage 2 points; trot and pace at β = 0.5, L = 0.04 m are the shipped trot and pace | 28 |
| 4 | 2 β × 4 y = 8 | 2: y = 0 at β 0.75 and 0.85 are the shipped tripod_crawl and wave | 6 |
| **Sum** | **285 (Stages 0–3) / 293** | **21 / 23** | **264 / 270** |

**Counting n.** "Unique" counts (configuration, n) pairs. Ignoring n, Stages 0–3 contain
6 + 206 + 28 = **240** distinct configurations and Stages 0–4 contain 240 + 6 = **246**, because
Stage 1 evaluates each of the 6 shipped configurations at 5 sample counts. *(Corrected in Phase 1:
the original sentence summed to 246 but printed 240. The generated matrix and
`m5_study.yaml` `expected_counts` give 240 / 246.)*

**Equality rule.** Two points are "equal" when their canonical configuration JSON is equal: β,
offsets, L, h, v, stance offsets, swing profile and n. `requires_static_stability` is excluded
from the comparison because it is derived (§10.3).

**Runtime estimate [RESULT-based estimate].** The audit probe measured ≈ 2.3 s per evaluation at
n = 200 in the cloud, and the cost grows linearly with n. So:
- Stage 1: 6 × 7.7 × 2.3 ≈ 106 s;
- Stage 2: 206 × 2.3 ≈ 474 s;
- Stage 3: 28 × 2.3 ≈ 64 s.

Total ≈ **11 minutes** single-threaded (± 50 %; infeasible configurations finish sooner).

**Why these levels [PLAN].**
- **Shipped values included.** Every level set contains the shipped value (β set, L = 0.04,
  h = 0.015), so the baseline is a point inside the grid.
- **Grid alignment.** All β values put (1 − β)·n on the grid for n ∈ {40, 100, 200, 400, 800}.
- **β below 0.5 excluded** (§10).
- **L and h straddle the reach box.** The levels deliberately cross the reach measured in the
  M4.5 audit (y from −122 to +69 mm and z from −43 to +33 mm about the neutral tip;
  `docs/SPIDERX_GAIT_FRAMEWORK.md` §5). That makes H3 falsifiable: the swing peak is at y ≈ 0,
  z = h, so h = 35 mm is expected to fail, and the forward swing overshoot ≈ 0.61·L at β = 0.5
  makes L = 120 mm expected to fail. These expectations are [HYPOTHESIS], because the reach
  region is not a box.

### 6.3 Invariance checks inside the matrix [PLAN]

**Stages 0 and 1 are the gate for the staged design (§16 D3).** Their checks verify the two
assumptions that justify separating the blocks:
1. time scaling (speed effects), and
2. phase-pattern-dependent support effects.

**Stages 2–4 are interpreted only if these checks pass.** If any check fails, the staged results
are not reported as findings; the failure is reported, and the design is escalated (§16 D3).

**Assumption-verification runs.** These 6 evaluations re-run the Stage 0 configurations at 2v₀.
They are counted **separately** from the 285 and 293 above, and they are recorded in
`derived/invariance.csv`. They check that:
- the joint series are identical;
- K is identical;
- max joint speed is ×2;
- the acceleration jump is ×4;
- every geometric metric is unchanged.

**Pattern-separation checks.** These use only Stage 1 evaluations, with no extra runs. At every
n, pace and trot (equal β, L and h, different patterns) must give identical per-leg metrics.
Support fractions must be equal across all aligned n. Stage 3 extends the per-leg pattern check
to β > 0.5.

**The invariance checks themselves:**

These are reported, not assumed.
- **Pattern invariance.** Stage 3 LS vs trot vs pace at equal (β, L, h) must give identical
  per-leg metrics to 10 significant digits.
- **Speed invariance.** Re-run the 6 baseline configurations at 2v₀. This is 6 extra
  evaluations, inside the test suite rather than the study.

## 7. Variables and controls

### 7.1 Existing YAML parameters [FACT]

Sources: `gait_config.py:36-40`, `:199-245`, `:282-309` and `m4_5_gaits.yaml`.

| Parameter | Unit | Shipped / default | Accepted by the validator | Role |
|---|---|---|---|---|
| `duty_factor` β | – | 0.5–0.85 per gait | (0, 1) exclusive | **independent** |
| `phase_offsets` | – | per gait | each in [0, 1); all 4 legs, aliases resolved | **independent** (pattern) |
| `swing_order` | – | per gait | must equal the order of the sorted offsets; legs with equal phase are grouped | **coupled** to the offsets (consistency check only, not a free variable) |
| `step_length_m` L | m | 0.04 | > 0, **no upper bound**; reach is checked only by IK | **independent** |
| `step_height_m` h | m | 0.015 | > 0, no upper bound | **independent** |
| `stance_height_offset_m` | m | 0.0 | any finite number (+z lowers the body) | independent; **held at 0** |
| `stance_center_offset_m` [x, y] | m | [0, 0] | 2 finite numbers | independent; held at 0, except Stage 4 |
| `body_speed_m_s` v | m/s | 0.005 | > 0; exactly one of v or T | independent **time scale only** (§6.1) |
| `cycle_period_s` T | s | derived | > 0; exactly one of v or T | **derived** from L, β, v |
| `swing_profile` | – | cycloid | only `cycloid` | fixed (single level) |
| `requires_static_stability` | bool | per gait | must be a bool | **verdict policy, not physics** |
| `analysis.samples_per_cycle` n | – | 200 | integer ≥ 8 | resolution (Stage 1) |
| `analysis.min_static_margin_m` | m | 0.005 | ≥ 0 | screening threshold (convention) |
| `analysis.min_singularity_margin_rad` | rad | 0.2 | > 0 | screening threshold (convention, M4 value) |
| `analysis.max_joint_step_rad` | rad | 0.05 | > 0 | algorithmic branch-flip detector (n-dependent) |
| `analysis.joint_speed_reference_rad_s` | rad/s | 0.5 | > 0, **must equal** `spiderx_legs.yaml:24` | placeholder reference |
| `analysis.velocity_jump_tol_m_s` | m/s | 1e-3 | > 0 | numerical tolerance |
| `validation_margin_rad`, `ik_position_tol_m` | rad, m | 0.05, 1e-6 | must equal `spiderx_legs.yaml:17` and `leg_kinematics.py:40` | fixed by M1/M3 (not variables) |

### 7.2 Couplings to respect [FACT]

- **Fixing v vs T.** Fixing T instead of v makes v = L/(βT), so every change in L or β also
  changes the speed. M5 fixes v (as M4.5 did), so joint-speed demand compares like with like.
  K removes the speed dependence entirely.
- **β and the phase offsets.** Together they set the support pattern. For example, LS at
  β ≥ 0.75 always has three or more stance feet; pairs always have a minimum of two. The
  `requires_static_stability` policy must therefore be **derived**, not chosen per point
  (§10.3).
- **Proxies and stride.** The energy proxies are per metre of travel, i.e. divided by
  S = L/β. Any β or L change moves them through S as well as through the path.
- **Reach coupling.** L, h and the stance offsets all draw on the same workspace. Only L and h
  vary in the primary design.

### 7.3 Controls [PLAN]

| Controlled | Held at | Why |
|---|---|---|
| Robot model | URDF at `3db1aec`, SHA-256 of the expanded XML recorded | Single model; the masses are placeholders |
| Evaluator | `gait_metrics.evaluate_gait`, unchanged | Comparability with M4.5 |
| Thresholds | M4.5 `analysis` block | Screening flags keep the M4.5 meaning |
| v | 5 mm/s | Time scale only; K reported |
| Stance height, centre | 0, (0, 0) | Body sway and pose are out of scope |
| n | 200 (Stage 1 varies it) | Resolution is reported, not hidden |

## 8. Metric validity and non-claims

"Threshold kind": **invariant** = a property of the model or construction; **convention** = an
engineering choice; **placeholder** = an acknowledged provisional value; **numerical** = a solver
or finite-difference tolerance.

| Metric (key) | Calculation (source) | Unit | Valid support | Cannot support | Caveats | Threshold (kind) |
|---|---|---|---|---|---|---|
| `ik_feasible` | count of samples × legs where `inverse()` is not `ok` (`gait_kinematics.py:113-123`; `leg_kinematics.py:465-579`) | samples | "every sampled target is reachable within soft limits **in the URDF model**" | reachability between samples; real reach; servo capability | Labelled `exact` but evaluated only at u_k. An excursion between samples can be missed. Use Stage 1 | 0 failures (**invariant**: feasible or not) |
| `fk_residual` | ‖FK(IK(p)) − p‖ (`gait_kinematics.py:126-137`) | m | the solver is self-consistent | positioning accuracy of a real leg | ~1e-16 is round-off; non-discriminating | 1e-6 m (**numerical**, `leg_kinematics.py:40`) |
| `singularity_margin` | min over samples of min(hip margin = min(α, π−α), knee margin = min(ψ, π−ψ)) (`leg_kinematics.py:516-554`; `gait_kinematics.py:78-79`) | rad | angular distance from the reach-boundary configurations of the analytic IK | manipulability, force capacity | 1.2116 rad for **all six** baseline gaits, so it is non-discriminating there; it may discriminate at large L or h | 0.2 rad (**convention**, M4 value) |
| `joint_limit_margin` | min over samples and joints of the distance to [URDF lower + 0.05, upper − 0.05] (`gait_kinematics.py:51-53`) | rad | workspace margin in joint space | safety of a real joint | **≥ 0 by construction** for solved samples, because IK rejects violations; its threshold 0 is a tautology (`:184-186`). Report it as a continuous margin | 0 (**invariant**); 0.05 rad soft margin (**convention**, M1) |
| `static_stability` / `min_static_margin_m` | signed distance from the COM ground projection to the convex hull of the stance-foot **targets**; only with ≥ 3 stance feet (`gait_metrics.py:126-142`, `:172-210`) | m | a **quasi-static support-margin approximation** for this mass model | dynamic stability, balance, ZMP or CoP, tipping, terrain, slip | Point feet; flat ground; level body; no inertia; CAD steel-density masses; uses targets, not FK (residual ~1e-16). **Undefined with < 3 feet.** The check counts < 3-foot samples as failures; the metric ignores them | 5 mm (**convention**, `m4_5_gaits.yaml:47`) |
| `fraction_statically_stable` | samples with margin **> 0** / **all** n samples (`gait_metrics.py:320-321`) | – | share of the cycle with the COM inside the polygon | anything at the 5 mm level | Mixes two things: it is 0 for pace and trot because the margin is undefined there, not because it is negative. **[PLAN]** Split into `fraction_support_ge3`, `fraction_margin_pos_given_ge3` and `fraction_margin_ge_thr_given_ge3` (§10.4) | none |
| support counts and fractions | from the phase only (`gait_phase.py:55-67`) | – | the footfall pattern | – | Exact only on an aligned grid (§6.1) | none |
| `joint_speed` / `max_joint_speed_rad_s` | max of the central difference \|q_{k+1} − q_{k−1}\| / 2Δt, cyclic (`gait_kinematics.py:97-105`; `gait_metrics.py:213-232`) | rad/s | kinematic joint-rate **demand** of the reference trajectory | servo feasibility, torque, tracking | O(Δt²) error; **∝ v exactly** (§6.1), so report K = max speed / v (rad/m) | 0.5 rad/s (**placeholder**, `spiderx_legs.yaml:24`) |
| `joint_continuity` / `max_joint_step_rad` | max \|q_k − q_{k−1}\|, cyclic (`gait_kinematics.py:155-172`) | rad | detects IK branch flips | smoothness of motion | **n-dependent**: trot gives 0.0504 rad at n = 40 vs 0.0101 at n = 200 (`docs/M4_5_TEST_RESULTS.md:73-89`). Compare only at equal n | 0.05 rad (**convention**, algorithmic) |
| `transition_velocity_jump` | ‖v(u − 1e-9) − v(u)‖ at every lift-off and touch-down, analytic (`gait_metrics.py:240-253`) | m/s | the cycloid joins stance continuously | smoothness of a real foot | **Non-discriminating by construction**: every cycloid configuration passes. The 1.4e-9 value is the finite offset, not a physical jump. It is a regression guard | 1e-3 m/s (**numerical**) |
| `max_transition_acceleration_jump_m_s2` | numeric acceleration step, dt = 1e-6 s (`gait_metrics.py:256-268`) | m/s² | the size of the cycloid's acceleration discontinuity | impact or contact forces | ∝ v² (§6.1); information only | none |
| `foot_path_per_m` | 4 × analytic world-frame swing arc / S (`gait_metrics.py:271-283`) | m/m | geometric foot travel per metre | energy | Independent of IK, pattern and v; depends only on (S, h) | none (**heuristic**) |
| `joint_travel_rad_per_m` | Σ \|Δq\| over the cycle / S (`gait_metrics.py:287-288`) | rad/m | total joint excursion per metre | energy, wear | A sampled total variation; it converges from below as n grows (Stage 1) | none (**heuristic**) |
| `lift_work_proxy_j_per_m` | Σ_legs m_leg·g·Σ max(0, Δz_legCOM) / S (`gait_metrics.py:289-295`) | J/m | gravitational lift of the leg masses per metre | **energy, power, efficiency, cost of transport** | Ignores the torso, negative work, the static **holding torque of stance legs** (likely the dominant load), actuator losses and friction. Uses CAD masses | none (**heuristic**) |
| `steps_per_m` | 4 / S | 1/m | cadence by definition | – | Exact | none |
| Determinism | 10 significant digits, sorted JSON keys, no timestamps (`gait_report.py:41-66`, `:272-283`) | – | byte-identical files **on one machine** | identity across machines (not yet compared byte-for-byte) | Cross-machine comparison is planned at 10 significant digits (§11.4) | – |

**Not a metric:** the per-gait **verdict**. It depends on `requires_static_stability` (a policy) and
on placeholder thresholds. **[PLAN]** M5 reports individual check outcomes and continuous values as
primary outputs. The M4.5 verdict is a secondary column under an explicitly named policy (§10.3).

## 9. Sampling, resolution and numerical sensitivity

**Why n = 200 [FACT].**
- It is the configured resolution (`m4_5_gaits.yaml:46`) and the one every pinned result uses
  (`test_gait_regression.py`).
- It puts all shipped transitions on the grid. For example, (1 − 0.85)·200 = 30 samples per wave
  swing.
- It keeps trot's `joint_continuity` below its 0.05 rad threshold, which n = 40 does not.

**Policy [PLAN]:**
1. **Fixed n.** Every Stage 2–4 result uses n = 200. No result at another n is mixed into a
   comparison.
2. **Stage 1 resolution study.** The 6 baseline configurations are evaluated at
   n ∈ {40, 100, 200, 400, 800}, all aligned. For each metric the study reports:
   - the value at each n;
   - the relative change |m(n) − m(800)| / |m(800)|;
   - the "discretisation band", the change from 200 to 400.

   Hypothesis tests (§4) treat a difference smaller than this band as **no difference**.
3. **Metrics expected to be n-sensitive [HYPOTHESIS]:**
   - `max_joint_step_rad` (∝ 1/n);
   - `joint_travel_rad_per_m` (converges from below);
   - minimum margins (a sampled minimum may miss the true minimum);
   - `max_joint_speed_rad_s` (finite-difference error).

   Support fractions are expected to be n-invariant on aligned grids.
4. **Never hidden.** Sensitivity tables are a required output, even when they show no change.
   If a Stage 2 or 3 conclusion flips between n = 200 and n = 400 on a re-check, it is reported
   as **resolution-limited**.
5. **No statistics.** The model is deterministic, so there is no sampling noise. M5 reports no
   p-values or confidence intervals. The uncertainty sources are discretisation (Stage 1) and the
   modelling assumptions (§15.1), and they are stated as such.

## 10. Inclusion, exclusion and negative controls

### 10.1 Included [PLAN]

- **All grid points, never discarded.** Every point of §6.2 is included, whatever its outcome.
- **Negative controls.** `wave` and `tripod_crawl` are kept as **known-failing negative
  controls**: the study must reproduce their failures exactly (Stage 0). If the failures cannot
  be reproduced, the study run is invalid.
- **Pattern classes:** LS singles (the pattern of four shipped gaits), diagonal pairs (trot) and
  lateral pairs (pace).

### 10.2 Excluded, with reasons [PLAN]

| Excluded | Reason |
|---|---|
| β < 0.5 | Flight or below-two-foot phases; the quasi-static model is meaningless there |
| Diagonal-sequence (DS) singles | Not a shipped configuration class. It can be added as one more Stage 3 pattern (+15 configs) on owner request; it is not assumed |
| Body sway or time-varying COM shift | Owner decision: future work only |
| Non-zero stance height | Changes body height, a pose study that is out of scope |
| Swing profiles other than the cycloid | Only `cycloid` exists (`gait_config.py:33`) |
| Turning or lateral motion | The model travels along +y only |
| Gazebo or any runtime | Offline only |

### 10.3 Representation of failures [PLAN]

| Case | How it appears in the results |
|---|---|
| Invalid configuration (schema) | Never silently dropped. The generator refuses the **whole study** with a message (exit 2); a grid point that the M4.5 validator rejects is a study-definition bug, not a result |
| IK-infeasible point | A row in `evaluations.csv` with `ik_feasible = false`, the failure count, and a reason histogram (`unreachable_hip`, `unreachable_knee`, `joint_limits`, …). Metrics that need all samples are empty (`None`), as M4.5 already does (`gait_metrics.py:285-286`), and **are not imputed**. The point counts toward H3 |
| Static-margin failure | A row; the margin value is kept. Flagged against 5 mm; the binding support triangle and the failing-sample count are recorded |
| Joint-speed reference exceeded | A row; K and v_adm are always reported; the 0.5 rad/s flag is secondary (§15.2 D2) |
| Verdict | Recomputed under the named policy **`m45_policy`**: `requires_static_stability` = (the pattern guarantees ≥ 3 stance feet at every sample), derived from `gait_phase.support_summary`. This rule reproduces all six shipped flags (wave and tripod_crawl true, the rest false), and a test enforces it |

**Rankings.** No "best configuration" ranking is produced. Where an ordering is shown (for
example K vs β), it is computed **only over IK-feasible points**, and the number of excluded
points is printed beside it.

### 10.4 Static-support applicability [PLAN]

The static margin is defined **only** for samples with three or more stance feet. Every
static-support output carries the applicability field `support_ge3_fraction`. The three fractions
are reported separately (all values in [0, 1]):
- `fraction_support_ge3`;
- `fraction_margin_pos_given_ge3`;
- `fraction_margin_ge_threshold_given_ge3`.

This replaces reading `fraction_statically_stable` (§8) on its own. No owner decision is needed: it
is a reporting clarification, and the M4.5 field is kept unchanged for continuity.

**N/A rule (owner requirement, §16).**
- **Margin-based outputs.** For configurations with no sample of three or more stance feet (pace
  and trot at β = 0.5), every margin-based output is reported as **N/A (not applicable: no
  three-contact support)**. This covers the minimum margin, the two `given_ge3` fractions and the
  static-support status. It is never reported as zero stability or as a stability failure.
- **Not-applicable checks.** M4.5's raw `static_stability` check stores `passed = false,
  applicable = false` for these configurations (`gait_metrics.py:190-210`). The raw table keeps
  that verbatim and adds an explicit `status` column ∈ {pass, fail, n/a}. Every derived table and
  figure shows `n/a`.
- **The legacy field.** The legacy M4.5 field `fraction_statically_stable = 0` appears only in the
  baseline-reproduction table. It is footnoted "0 = no applicable samples, not instability".

### 10.5 Null and negative results [PLAN]

- **Every hypothesis is reported.** H1–H3 are reported with the outcome *supported*, *falsified*
  or *resolution-limited*, including null and negative outcomes. A falsified hypothesis is a
  valid result, and the paper text must state it.
- **Levels are frozen.** Grid levels, thresholds and hypotheses are frozen in the committed
  `m5_study.yaml` before Stage 2 or 3 runs. Any later change needs a new `study_id` and a
  documented reason, and both studies stay on record.
- **Proxies are not "efficiency".** An energy-proxy pattern that contradicts expectations is
  reported as a proxy observation, not reinterpreted as an "efficiency" result.

## 11. Data schema, artifacts and reproducibility

### 11.1 Layout [PLAN]

Everything goes under the git-ignored `log/`, at a deterministic path:

```text
log/m5_offline_evaluation/<study_id>/
  manifest.json                 study definition, levels, stages, counts, provenance (no timestamps)
  configs/<config_id>.json      canonical generated configuration (YAML-equivalent) per point
  raw/evaluations.csv           one row per (config_id, n): parameters + every check + every metric
  raw/checks.csv                long format: config_id, n, check, passed, applicable, value,
                                threshold, unit, label, failure_count
  raw/ik_reasons.csv            config_id, n, leg, reason, count
  raw/support_events.csv        config_id, n, binding triangle, min margin, failing-sample count
  raw/samples/<config_id>_n<n>.csv   per-sample table (M4.5 samples_rows), Stages 0-1 only
  derived/resolution.csv        metric x n x config, relative change vs n=800, discretisation band
  derived/invariance.csv        pattern and speed invariance results (max abs/rel differences)
  derived/feasibility_boundary.csv   beta, h -> L_max (largest feasible tested L), first failing reason
  derived/speed_demand.csv      config_id -> K (rad/m), v_adm(0.5) (m/s), v0 values
  derived/support_by_pattern.csv     pattern x beta x L -> the applicability-split support fractions
  derived/hypotheses.json       H1-H3: criteria, inputs, outcome, resolution note
  derived/summary.json          counts (total, feasible, failing per check), scope text, labels
  environment.json              Python, matplotlib, OS; EXCLUDED from the byte-identity comparison
```

- **`study_id`** = `<study name>-<first 12 hex of the SHA-256 of the canonical study JSON>`.
- **`config_id`** = the first 12 hex of the SHA-256 of the canonical configuration JSON (sorted
  keys, 10 significant digits).

### 11.2 Provenance in `manifest.json` [PLAN]

The manifest records:
- the git commit (`git rev-parse HEAD`) and a **dirty-tree flag**. The runner refuses a dirty tree
  unless `--allow-dirty` is given, and then records that it was;
- the SHA-256 of `m4_5_gaits.yaml`, `m5_study.yaml`, `spiderx_legs.yaml` and the expanded URDF XML;
- the M4.5 `config_version`;
- the `analysis` thresholds;
- the n values;
- the stage definitions;
- the evaluation counts (planned, deduplicated, run);
- the command line;
- the honesty-label legend.

### 11.3 Required tables and future figures

The required tables are listed in §11.1.

**Figures [PLAN, not produced in Phase 0].** They follow the M4.5 style rules: one hue per
quantity, FAIL marked by label and not by colour alone, a CSV beside each figure, and no
dual-axis charts.
1. Feasibility map: L × h per β, with infeasible cells hatched and the reason annotated.
2. K vs β (lines per L), with the derived v_adm on a **separate** panel.
3. Support-fraction bars by pattern × β (three applicability-split fractions).
4. Minimum static margin vs β by pattern, with the 5 mm line and the binding triangle annotated.
5. Resolution sensitivity: metric vs n for the 6 baseline configurations.
6. A descriptive proxy panel, explicitly titled "heuristic proxies — not energy".

### 11.4 Reproducibility checklist [PLAN]

| # | Check | Cloud | Local |
|---|---|---|---|
| 1 | Clean tree at a recorded commit; branch and commit hash recorded in the manifest | ☐ | ☐ |
| 2 | `colcon build` and full `colcon test`: 0 failures | ☐ | ☐ |
| 3 | `--check-only` validates the study and prints the planned and deduplicated counts | ☐ | ☐ |
| 4 | Stage 0 reproduces the six M4.5 pins exactly | ☐ | ☐ |
| 5 | Two runs into different directories: SHA-256 manifests of `manifest.json`, `configs/`, `raw/` and `derived/` identical | ☐ | ☐ |
| 6 | Cloud vs local: CSV and JSON compared at 10 significant digits. If they differ, report the max abs and rel difference per metric; do not claim identity | ☐ (both) | |
| 7 | All outputs under `log/`; `git status` clean afterwards | ☐ | ☐ |
| 8 | No Gazebo process, no hardware, no runtime validator | ☐ | ☐ |

## 12. Planned M5 Phase 1 implementation batches

Every batch gets a full `colcon build` and `colcon test`, and is committed separately. Docs are
committed separately from code. The M4.5 modules and YAML stay **unchanged** throughout. **[PLAN]**

| Batch | Adds | Content | Tests |
|---|---|---|---|
| **A** | `config/m5_study.yaml`, `spiderx_controller/eval_study.py` | Strict study schema: stages, levels, n, pattern definitions, hypotheses with criteria. Grid-alignment rule; `offline_only: true` required | `test_eval_study.py` |
| **B** | `eval_generate.py` | Expands stages into canonical configuration dicts → `gait_config.validate_config` (the same validator). Adds `config_id`, deduplication and the `m45_policy` derivation | `test_eval_generate.py` (baseline points equal the shipped `GaitSpec`s; flags reproduce the shipped ones) |
| **C** | `eval_runner.py`, `m5_offline_evaluation.py`, `scripts/m5_offline_evaluation` | Runs `evaluate_gait` unchanged and writes the raw tables and manifest. CLI: `--study`, `--stage`, `--out`, `--check-only`, `--dry-run`, `--allow-dirty`. Exit 0 = completed, 2 = invalid study | `test_eval_runner.py` (tiny study, byte-identical twice, infeasible rows retained) |
| **D** | `eval_derived.py` | Resolution, invariance, feasibility-boundary, speed-demand (K, v_adm) and support-applicability tables; the hypothesis evaluator (pre-registered criteria) | `test_eval_derived.py` |
| **E** | `eval_plots.py` | The §11.3 figures, after separate owner approval of the figure list | `test_eval_plots.py` |
| **F** | docs | Study run (cloud), `docs/M5_TEST_RESULTS.md`, roadmap, STATUS and testing guide; local-verification handoff | – |

## 13. Test strategy

### 13.1 Determinism and provenance [PLAN]

- **Byte-identical outputs.** The same study run twice must give byte-identical `manifest.json`,
  `configs/`, `raw/` and `derived/`.
- **`config_id` stability.** `config_id` must not change under YAML key reordering or alias
  spelling.
- **Provenance fields.** The manifest must contain the commit, the SHA-256 hashes and the counts.
- **Dirty tree.** A dirty tree is refused without `--allow-dirty`.

### 13.2 Scientific guards [PLAN]

- **Baseline reproduction.** The generated baseline equals the `test_gait_regression.py` pins
  (verdicts and metrics).
- **Speed invariance.** At 2v the joint series are identical, K is identical, max joint speed is
  ×2 and the acceleration jump is ×4.
- **Pattern invariance.** At equal (β, L, h), LS, trot and pace give identical per-leg metrics.
- **Policy derivation.** `m45_policy` flags equal the shipped flags.
- **Stage 1 alignment.** Every Stage 1 point is grid-aligned, and support fractions are equal
  across n.

### 13.3 Error paths and mutation tests (the checks must bite) [PLAN]

- **Bad study definitions** are refused with a named key:
  - an off-grid level (for example β = 0.62 with n = 40);
  - β ≤ 0 or β ≥ 1;
  - L ≤ 0;
  - an unknown pattern;
  - duplicate levels;
  - a hypothesis without a falsification criterion.
- **Infeasible points stay in the results.** A deliberately infeasible point (L = 0.30 m) appears
  as a row with `ik_feasible = false` and reasons, never as a missing row.
- **Tampering is detected.** Deleting a raw row or perturbing a stored metric must make the
  derived recomputation check fail.
- **Mutation catches.** Each of these must turn tests red:
  - reversing the K normalisation;
  - mis-deriving `requires_static_stability`;
  - dropping failing rows.
- **No stray writes.** Nothing is written outside `--out`. Unless `--out` is given explicitly, the
  default root is `log/m5_offline_evaluation/`.

## 14. Paper-safe reporting language

| Use | Do not use |
|---|---|
| "offline kinematic evaluation of gait configurations on the SpiderX URDF model" | "gait performance", "the robot walks", "walking gait" |
| "sampled-IK feasible (n = 200)" | "feasible gait", "achievable on the robot" |
| "quasi-static support-margin approximation (point feet, CAD masses, level body)" | "stable gait", "dynamically stable", "balance" |
| "speed-normalised peak joint-speed demand K (rad/m)" | "servo requirement", "actuator limit" |
| "exceeds the 0.5 rad/s provisional screening flag (simulation placeholder)" | "too fast for the servos", "actuator limit", "hardware limit" |
| "K: model-derived joint-rate-per-distance indicator (rad/m)" | "motor capability", "measured joint speed" |
| "N/A: no three-contact support" | "0 % stable", "unstable" (for pace or trot) |
| "heuristic proxies (foot path, joint travel, lift of leg masses) per metre" | "energy", "energy consumption", "electrical power", "actuator work", "efficiency", "torque cost", "battery or runtime estimate", "cost of transport" |
| "configuration class (pattern × duty factor)" | "best gait", "optimal gait" |
| "model prediction / offline result" | "experimental result", "measured", "validated" |
| "not evaluated: dynamics, contact, terrain, tracking, hardware" | silence about these |

**Mandatory scope statement for every M5 report, table header and figure caption:**

> Offline kinematic analysis of the SpiderX URDF model. Not walking, not dynamic simulation,
> not hardware. Stability is a quasi-static approximation; energy values are heuristic proxies;
> the joint-speed reference is a placeholder.

## 15. Risks, limitations and owner decisions

### 15.1 Risks and limitations [FACT / PLAN]

| Risk or limitation | Effect | Mitigation |
|---|---|---|
| CAD steel-density masses (`URDF_INERTIA_AUDIT.md:46-50`) | The COM, static margin and lift proxy are only as good as the masses | Label everything model-based. A future mass-sensitivity study needs measured masses (out of M5 scope) |
| Single URDF model; reach measured as a box | H3 predictions may be wrong because the reach region is not a box | H3 is falsifiable; the boundary is measured, not assumed |
| Sampled minima | True minima between samples can be missed | Stage 1 resolution study |
| Non-discriminating metrics (FK residual, velocity jump, at baseline also the singularity and limit margins) | They add no comparative information | Reported as guards, not findings |
| Pattern and β confounded in the shipped set | Misattribution | β varied within each pattern (Stages 2 and 3) |
| Placeholder joint-speed reference | Pass/fail depends on an unmeasured value | Continuous K and v_adm (D2) |
| Cross-machine floating-point differences | A byte-identity claim could fail | Compare at 10 significant digits; report the differences (§11.4) |
| Scope creep (sway, playback, Gazebo) | Violates the boundary | Excluded in §10.2; tests forbid runtime imports |

### 15.2 Owner decisions [OWNER DECISION]

**Decided.** All five decisions were made by the owner, choosing D1 (a), D2 (A), D3 (A), D4
include and D5 (B). See §16. The alternatives below are kept as the decision record.

Only choices that change the science or the project structure are listed. Everything else above
is a [PLAN] default.

**D1. Milestone naming conflict.**
- **[FACT]** The roadmap and STATUS define **M5 as the `/cmd_vel` → gait bridge**
  (`docs/SPIDERX_DEVELOPMENT_ROADMAP.md:113`, `STATUS.md:98`). This plan calls the offline
  evaluation "M5".
- **Alternatives:**

  | Option | Change | Pro | Con |
  |---|---|---|---|
  | (a) | Insert **"M5 – Offline research evaluation"** and rename the bridge **"M5.5 – `/cmd_vel` → gait bridge"** | M6–M9 numbering unchanged; matches the owner's wording | One renumbered roadmap item |
  | (b) | Name this work **"M4.6"** and keep M5 as the bridge | No roadmap renumbering | Contradicts the owner's M5 naming in the request |
  | (c) | Renumber the whole chain: bridge → M6, odometry → M7, … | Clean sequence | Touches many docs and status rows |

- **Recommendation:** (a).
- **Default if unanswered:** the work keeps the name "M5 offline evaluation" in its own files,
  and the roadmap and STATUS stay untouched until the owner chooses.
- **Consequence:** the Phase 1 Batch F docs commit applies the chosen naming.

**D2. Treatment of the 0.5 rad/s joint-speed placeholder.**

| Option | Treatment | Pro | Con |
|---|---|---|---|
| (A) | Report K and v_adm(ω) continuously; keep 0.5 rad/s only as a **labelled secondary screening flag** | Honest about a placeholder; speed results stay valid for any future measured servo limit; continuity with M4.5 | Two representations to explain |
| (B) | Continuous only; drop the pass/fail flag from M5 | Simplest, fully threshold-free | Breaks continuity with the M4.5 verdict |
| (C) | Keep it as a hard verdict criterion (M4.5 policy) | Identical to M4.5 | A placeholder decides results |

- **Recommendation:** (A).
- **Default if unanswered:** (A).
- **Consequence:** this decides whether `joint_speed` appears in the M5 verdict column (C) or
  only as a flag (A).

**D3. Staged design vs full factorial.**

| Option | Runs | Evaluations | Unique | Cloud time |
|---|---|---|---|---|
| (A) Staged (§6.2) | Stages 0–3 | 285 | 264 | ≈ 11 min |
| (B) Full factorial | 3 patterns × 5 β × 6 L × 7 h, plus Stage 1 | 660 | 654 | ≈ 26 min |
| (C) Screening first | Stage 2 at L, h ∈ {low, base, high} (5 × 3 × 3 = 45), then refine around the boundary | ≈ 90–150, adaptive | | short |

- **(A) Staged.** Exploits the proven invariances (§6.1) and attributes each effect to one block.
- **(B) Full factorial.** Assumption-free, but 2/3 of the per-leg results duplicate, and the
  pattern invariance is assumed instead of tested.
- **(C) Screening.** Adaptive level choice weakens pre-registration.
- **Recommendation:** (A), with the invariance checks of §6.3. The levels are frozen before any
  run.
- **Default if unanswered:** (A).
- **Consequence:** this fixes `m5_study.yaml` and the run budget.

**D4. Optional Stage 4: constant support-polygon shift.** (Decided: included and renamed "stance-centre translation sensitivity"; see §16.)
- **Options:**
  - **(A) Include** 8 configurations: a stance-centre y shift that is constant over the cycle.
    It uses an existing YAML parameter and is labelled "constant shift, **not body sway**". It
    tests H1's mechanism (COM vs polygon).
  - **(B) Exclude.** The stance centre stays at 0 everywhere.
- **Pro of (A):** it explains *why* H1 holds. Earlier M4.5 exploratory numbers (−0.89 mm at
  −5 mm) are already in the docs.
- **Con of (A):** it sits close to the sway topic the owner deferred.
- **Recommendation:** (A), labelled as above.
- **Default if unanswered:** (B), exclude.
- **Consequence:** +8 evaluations (+6 unique).

**D5. Literature anchoring.**
- **[FACT]** The repository contains no citations.
- **What needs a reference:** the paper needs verified references for:
  - duty factor and phase-offset gait description;
  - the wave gait;
  - the static stability margin;
  - footfall sequences (lateral vs diagonal);
  - the cycloid swing trajectory.
- **Options:**
  - **(A)** The owner supplies the reference list.
  - **(B)** Claude drafts candidate references in a Phase 1 docs batch, each one checked against
    its source and marked "to verify", and the owner approves them.
  - **(C)** Defer citations to paper writing; until then the docs use only repository-defined
    terms.
- **Recommendation:** (B).
- **Default if unanswered:** (C).
- **Consequence:** no citation is written anywhere until (A) or (B) completes.

**Not owner decisions** (they are resolved as [PLAN] above, with reasons):
- **Including the failing gaits as negative controls (§10.1).** Excluding them would bias the
  study, and their reproduction validates each run.
- **The ≥ 3-contact applicability label (§10.4).** A reporting clarification that changes no
  value.
- **Energy proxies as secondary descriptive metrics (§4, §8).** They cannot support an energy
  claim, so they cannot be targets.

---

## 16. Owner-decision addendum

The owner reviewed Phase 0 and approved the audit and the plan commit `4c1bd1d` in principle. The
owner then decided D1–D5 as follows. Where this addendum refines an earlier section, it governs.
The alternatives in §15.2 stay in place as the record of what was considered. **[OWNER
DECISION]**

### D1 — Milestone naming: option (a)

**Names.**
- This work is **M5: Offline evaluation study**.
- The former roadmap item "M5 – `/cmd_vel` → gait bridge" is renamed **M5.5: Command-velocity
  bridge**.

**M5.5.** It remains **future work**. It is not implemented and not scheduled, and it must not
be described as available or complete.

**Docs.** The roadmap and `STATUS.md` were updated only to remove the naming conflict. They mark
M5 as "planned; Phase 0 evaluation protocol approved; implementation not started". Historical
records keep their original wording, for example `docs/M4_5_PLAN.md:268` and
`docs/M4_PLAN.md:362`.

### D2 — Joint-speed treatment: option (A)

**Primary quantity.** Joint-speed demand is reported **continuously**. The primary quantity is
the peak demand per metre of travel:

  K = (max joint speed) / v, in rad/m, with the derived v_adm(ω) = ω / K.

**What K is.** In the current clock-driven trajectory model, body speed only **rescales time**:
the sampled geometric path, and with it the joint-angle series, does not change (§3, §6.1). K
is therefore a **model-derived joint-rate-per-distance indicator** of the reference trajectory.

**What K is not.** It is **not** a measured motor, servo or hardware capability. It does not
include tracking, load, torque or actuator dynamics.

**The 0.5 rad/s value.** It is kept **only** as a clearly labelled **secondary, provisional
screening flag**. It is the `SIMULATION_PLACEHOLDER` from `spiderx_legs.yaml:24`. It must never
be described as:
- a physical actuator limit;
- a validated hardware limit;
- a primary scientific conclusion.

**Screening failures.** Cases that exceed the flag remain in every table as results. They are
never discarded.

### D3 — Study design: option (A), staged

**The design.** As in §6.2: **285 evaluations, 264 unique** for Stages 0–3, and 293 / 270 with
Stage 4. The exact overlaps are in §6.2.

**The gate.** Stages 0 and 1, with the assumption-verification runs of §6.3, verify the two
assumptions behind separating the blocks:
1. time scaling / speed effects;
2. phase-pattern-dependent support effects.

Stages 2–4 are interpreted only if these checks pass.

**Escalation.** A full factorial (§15.2 D3 (B)) is a **future escalation only**. It is triggered
if the staged results expose an interaction that this design cannot explain, for example:
- a failed invariance check;
- a pattern × (L, h) effect on per-leg metrics;
- a support result that Stage 3's L levels cannot resolve.

Escalation needs a new `study_id` and a recorded reason.

### D4 — Stage 4 included: "stance-centre translation sensitivity"

**What it is.** A fixed, **constant** forward/back translation of the nominal foot/stance centre
(`stance_center_offset_m` y ∈ {−10, −5, 0, +5} mm). It is applied for β ∈ {0.75, 0.85}, with
L = 0.04 m and h = 0.015 m. It is used to examine the **static-support-margin mechanism** behind
the observed crawl-gait support failures (H1: the COM sits behind the foot centre).

**Size.** Exactly the **eight** planned evaluations (6 unique).

**What it is not.** It is **not**:
- body sway;
- time-varying body motion;
- dynamic balance;
- gait playback;
- walking.

**What it claims.** It tests the mechanism without claiming to solve the crawl-gait failures.
Any margin change it shows is a model observation for a static translation, not a remedy.

### D5 — References: option (B), with a citation acquisition protocol

**Status.** No external citation, bibliography entry or paper claim is added in Phase 0 or in
this update.

**Concepts that need literature grounding:**
1. static stability and support polygons (stability-margin definition, quasi-static assumptions);
2. quadruped gait terminology (wave, crawl, amble, pace, trot; lateral vs diagonal sequence);
3. duty-factor and phase-offset conventions;
4. kinematic trajectory evaluation (swing-trajectory shape, workspace and IK feasibility
   assessment, discretisation);
5. the limits of proxy energy metrics (why path, joint-travel and lift proxies are not energy or
   cost of transport).

**Acquisition protocol [PLAN]:**
- **Separate step.** A later **citations-only review step** produces candidate references, one
  set per concept.
- **Verified against the original.** Each candidate must be checked against the **original
  source** (the publisher or DOI record, or the original text), never a secondary citation. It
  must record:
  - the full bibliographic data;
  - the exact statement it supports;
  - the page, section or equation where available;
  - how it was verified.
- **Owner approval first.** The candidates go to the owner for approval **before** any of them
  enters tracked documentation. Rejected or unverifiable candidates are dropped, not paraphrased.
- **Until then,** M5 documents use only repository-defined terms (§3) and make no
  literature-comparison claims.

### Scientific-precision rules (owner requirements, binding for Phase 1)

- **Negative controls.** `wave` and `tripod_crawl` are explicit negative controls in **every**
  baseline and reproduction analysis. Failed configurations are **never filtered** from tables or
  figures.
- **Separate claim levels.** IK feasibility is kept separate from:
  - the static-support approximation;
  - dynamic stability;
  - real energy efficiency;
  - walking, navigation and hardware capability.

  No output combines them into one "feasible gait" claim.
- **N/A, not zero.** Pace and trot (and any configuration with no three-contact support) report
  **N/A** for static support, never zero stability or a stability failure (§10.4).
- **Resolution.** Sample-count sensitivity stays explicit (§9). The **200-sample result is a
  defined numerical resolution, not ground truth.**
- **Energy proxies.** The heuristic energy metrics stay **descriptive secondary proxies** only.
  They must not be called:
  - energy consumption;
  - electrical power;
  - actuator work;
  - efficiency;
  - torque cost;
  - battery or runtime estimates.
- **K.** K is explained as in D2 wherever it appears.

---

*Phase 0 ends with this document and its decision addendum. No M5 code, YAML, data, artifacts or
figures exist. Phase 1 Batch A starts only on the owner's instruction.*
