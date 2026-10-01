# SpiderX M5 Evaluation Guide – Offline Evaluation Study

```text
OFFLINE model analysis of gait configuration classes on the SpiderX URDF kinematic model, using
the unchanged M4.5 evaluator. Not walking, not dynamic stability, not energy or power, not
navigation, not real-time execution, not actuator capability, not hardware readiness.
```

This guide covers:
- how the M5 study turns `config/m5_study.yaml` into evaluated, provenance-tracked records;
- how to run and debug it;
- how to read its result categories, including the negative ones.

The design is in [M5_EVALUATION_PLAN.md](M5_EVALUATION_PLAN.md) and the results are in
[M5_TEST_RESULTS.md](M5_TEST_RESULTS.md). The M4.5 metrics M5 reuses are explained in
[SPIDERX_GAIT_FRAMEWORK.md](SPIDERX_GAIT_FRAMEWORK.md).

## 1. Data flow

```text
config/m5_study.yaml ──► eval_study ─────────► eval_runner ───────────────────► eval_records + eval_tables
  (declarative study)    validate, expand,     Stage 0/1 → GATE → Stages 2-4     <out>/<study_id>/
                         canonical IDs,        (each evaluation = unchanged      manifest.json, raw/, derived/
                         accounting            M4.5 gait_metrics.evaluate_gait)
config/m4_5_gaits.yaml ─┘ (read only: patterns and baselines; every variant passes the M4.5 validator)
```

| Module (`spiderx_controller/`) | Role |
|---|---|
| `eval_study.py` | Strict study schema; experiment matrix; `config_id` / `eval_id`; accounting check |
| `eval_runner.py` | Evaluation records and result categories; the Stage 0/1 gate; gate enforcement; `verify_result` |
| `eval_records.py` | Raw CSV/JSON, provenance, and the manifest with SHA-256 of every compared file |
| `eval_tables.py` | Derived tables, the pre-registered H1–H3 evaluation, `summary.json` / `summary.md` |
| `m5_offline_evaluation.py` | The command-line interface |

**What M5 does not compute.** M5 adds no kinematics or metric mathematics. Every number comes
from M4.5's `evaluate_gait`. M5 only:
- classifies the results;
- divides peak joint speed by body speed to get K;
- tabulates.

## 2. The study

| Stage | What | Planned | New unique |
|---|---|---|---|
| S01 (Stage 0 + 1) | The six M4.5 gaits at n ∈ {40, 100, 200, 400, 800}. Stage 0 is n = 200 | 30 | 30 |
| S2, kinematic block | Lateral-sequence pattern × β {0.50, 0.55, 0.65, 0.75, 0.85} × L {20…120 mm} × h {5…35 mm} | 210 | 206 |
| S3, support block | {lateral sequence, diagonal pairs, lateral pairs} × β × L {20, 40, 60 mm}, h = 15 mm | 45 | 28 |
| S4, stance-centre translation sensitivity | Lateral sequence, β {0.75, 0.85} × constant stance-centre y {−10, −5, 0, +5 mm} | 8 | 6 |
| **Total** | 285 planned / 264 unique for Stages 0–3; **293 / 270** for Stages 0–4 (240 / 246 distinct physical configurations) | | |
| Speed checks (separate) | The six baselines at 2 v | 6 | 6 |

**Identity rules.**
- **config_id** hashes the canonical physical configuration: β, phase offsets, L, h, stance
  offsets, v and swing profile.
- **eval_id** adds the sample count n.
- **Duplicates.** A later stage that plans an already-evaluated (configuration, n) reuses it.
  The plan table maps each duplicate to its first entry, so nothing is evaluated twice and
  nothing is lost.

**Stage 4.** This is a fixed, constant translation of the stance centre, used to examine the
static-support-margin mechanism. It is **not** body sway, time-varying motion, balance control,
gait playback or walking, and it does not claim to solve the crawl-gait failures.

## 3. The Stage 0/1 gate

Stages 2–4 run **only** if every gate check passes. `_gate_allows_later_stages` is the single
enforcement point, and a mutation test proves that removing it is detected.

| Gate check | Verifies |
|---|---|
| `baseline_valid_and_ik_feasible` | All 30 baseline evaluations are valid and sampled-IK feasible |
| `baseline_reproduction_n200` | M4.5 verdicts, failed checks and pinned metrics at n = 200 |
| `negative_controls_reproduced` | wave and tripod_crawl fail the static-support approximation at **every** n |
| `static_support_na_preserved` | pace and trot are N/A at every n (never 0, never fail) |
| `m45_policy_matches_shipped_flags` | The derived `requires_static_stability` equals the shipped flags |
| `speed_scale_invariance` | At 2 v: joint series identical, geometric metrics unchanged, joint speed × 2, K unchanged. Assumption (a): speed only rescales time |
| `pattern_separation` | Per-leg path metrics are equal across patterns (pace vs trot) at every n, and support fractions do not depend on n. Assumption (b) |

**If the gate fails:**
- the CLI exits **3**;
- every Stage 2–4 record is written as `blocked_gate_failed`;
- `raw/gate.json` names the failed checks;
- all Stage 0/1 evidence is kept.

**Example.** The gate is not a formality: on its first real run it found a floating-point
phase-boundary artifact in M4.5's `gait_phase`. The owner approved fixing it in `ece1e23`; see
the results document.

## 4. Result categories

Every planned unique evaluation has **exactly one** record. Categories are explicit columns in
both CSV and JSON, and nothing is filtered.

**`evaluation_category`** (kinematic checks only):

| Value | Meaning |
|---|---|
| `valid_pass` | Valid configuration, sampled-IK feasible at every sample, every M4.5 kinematic check passed |
| `kinematic_check_fail` | Feasible, but a kinematic check failed. Example: `joint_continuity` at coarse n, such as trot at n = 40 |
| `ik_infeasible` | At least one sample has no joint-safe IK solution. The reasons are in `raw/ik_reasons.csv` |
| `invalid_configuration` | The M4.5 validator refused the variant. The message is in `config_error` |
| `blocked_gate_failed` | Not evaluated because the Stage 0/1 gate failed |

`valid_pass` is **not** a stability, walking or hardware claim.

**`static_support_status`** (static-support approximation):

| Value | Meaning |
|---|---|
| `pass` / `fail` | The minimum COM-to-support-edge margin over the samples with ≥ 3 stance feet is ≥ / < 5 mm |
| `not_applicable` | No sample has three stance feet (pace, trot). Shown as **N/A** in derived tables, never 0 and never a failure |
| `not_evaluated` | IK infeasible, invalid or blocked |

`requires_static_stability_m45_policy` says whether static support is *required* for the
pattern. It is required only when the pattern guarantees ≥ 3 stance feet; ripple and amble may
show `fail` over their three-contact samples while not requiring static support.

**Joint speed:**
- **`k_rad_per_m`** (primary, continuous). K = peak joint speed / body speed. In this
  clock-driven model, speed only rescales time, so K is a model-derived joint-rate-per-distance
  indicator, **not** a motor or servo capability.
- **`joint_speed_screen`** (secondary). `pass` / `fail` against the 0.5 rad/s
  SIMULATION_PLACEHOLDER. It is a provisional screening flag: not an actuator limit, never part
  of `evaluation_category`, and failures are kept.
- **`v_admissible_at_reference_m_s`** = 0.5 / K. It is a model-derived figure, not an
  achievable robot speed.

**Heuristic motion proxies.** `foot_path_per_m`, `joint_travel_rad_per_m` and
`lift_work_proxy_j_per_m` are descriptive only. They are not energy consumption, electrical
power, actuator work, efficiency, torque cost, or battery or runtime estimates.

## 5. Running it (offline; no Gazebo, no hardware)

```bash
cd ~/spiderx_ws && colcon build --symlink-install && source install/setup.bash
ros2 run spiderx_controller m5_offline_evaluation --check-only   # validate + accounting (exit 0)
ros2 run spiderx_controller m5_offline_evaluation --dry-run      # 293 + 6 planned lines, writes nothing
ros2 run spiderx_controller m5_offline_evaluation                # full study (about 11-15 min)
ros2 run spiderx_controller m5_offline_evaluation --out log/m5_second_run   # a second, comparable run
```

| Exit | Meaning |
|---|---|
| 0 | Completed. Failing, infeasible and N/A outcomes are results, not errors |
| 2 | Refused: invalid study, dirty git tree without `--allow-dirty`, or an existing study directory (never overwritten) |
| 3 | Gate failed; Stages 2–4 are blocked and the evidence is written |

Results go to `<out>/<study_id>/`; the default `--out` is the git-ignored
`log/m5_offline_evaluation`. Start with `derived/summary.md`, then `derived/gate.csv`, then the
stage tables.

**Comparing two runs.** Every file except `environment.json` must be byte-identical. The manifest
lists their SHA-256 hashes:

```bash
diff <(python3 -c "import json;print(json.dumps(json.load(open('A/manifest.json'))['files_sha256'],indent=1,sort_keys=True))") \
     <(python3 -c "import json;print(json.dumps(json.load(open('B/manifest.json'))['files_sha256'],indent=1,sort_keys=True))")
diff -rq --exclude=environment.json A B          # no output = identical
```

On a different machine, compare the CSV/JSON values (10 significant digits) rather than expecting
byte identity. Cross-machine byte identity is not claimed.

## 6. Debugging

| Symptom | Likely cause | Where to look |
|---|---|---|
| `REFUSED: … expected_counts` | The levels changed but the declared accounting did not | `m5_study.yaml` `expected_counts`; `--dry-run` shows the real matrix |
| `REFUSED: … not on the 1/n sample grid` | A β or phase offset puts transitions between samples | Choose levels with (1 − β)·n and φ·n integers |
| `REFUSED: … uncommitted changes` | Dirty git tree | Commit, or `--allow-dirty` (recorded in the manifest) |
| `REFUSED: … already exists` | That study already ran into this `--out` | Use another `--out`; results are never overwritten |
| Exit 3, `pattern_separation` FAIL | Support fractions vary with n, or per-leg metrics differ across patterns | `raw/gate.json` detail; `derived/resolution.csv` |
| Exit 3, `baseline_reproduction_n200` FAIL | M4.5 behaviour changed | Compare with `test_gait_regression.py`; explain any change in the results doc |
| `ik_infeasible` rows | The stroke or lift exceeds the reach or joint limits | `raw/ik_reasons.csv`, `derived/feasibility_boundary.csv` |
| `kinematic_check_fail` at n = 40 | `joint_continuity` bounds the per-sample step, which depends on n | `derived/resolution.csv`; compare at equal n only |

## 7. Interpreting negative and null results

- **Negative controls must fail.** wave and tripod_crawl fail the static-support approximation:
  the URDF COM sits ≈ 5.8 mm behind the foot centre, and the FL–FR–RR triangle binds. If they
  ever pass, the study is invalid, not improved.
- **Falsified is a valid outcome.** A falsified hypothesis in `derived/hypotheses.json` is
  reported as such. `not_evaluable` (no level pairs, or a blocked study) is not the same as
  "supported".
- **Infeasible rows are data.** `ik_infeasible` rows define the feasibility boundary (H3); they
  are never dropped.
- **Resolution, not ground truth.** The 200-sample results are a defined numerical resolution.
  `derived/resolution.csv` shows how each metric moves with n.
- **No ranking.** No "best gait" is produced. Orderings such as K against β are model
  predictions over IK-feasible points only.

## 8. Tests

| File | Covers |
|---|---|
| `test_eval_study.py` | Schema, ranges, references, accounting (285/264, 293/270, 6), canonical IDs, refusals |
| `test_eval_runner.py` | Real mini studies: categories, N/A, gate pass, forced gate failures, mutation tests (gate removal, record filtering), CLI exit codes |
| `test_eval_records.py` | Byte-identical repeated writes, manifest hashes, no timestamps, provenance, dirty-tree refusal |
| `test_eval_tables.py` | Derived tables, N/A cells, blocked rows kept, synthetic supported and falsified hypothesis cases |
| `test_eval_regression.py` | Pins of the shipped baselines at n = 200 (negative controls, N/A, K, screen, support fractions) |

The shipped study takes about 11–15 minutes, so the unit tests use small **mini studies** built
from the shipped specification. Every mini-study evaluation still runs the real M4.5 evaluator.
