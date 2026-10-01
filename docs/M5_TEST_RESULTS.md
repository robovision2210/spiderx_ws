# M5 Test Results – Offline Evaluation Study

```text
OFFLINE model analysis of gait configuration classes on the SpiderX URDF kinematic model, using
the unchanged M4.5 evaluator. Not walking, not dynamic stability, not energy or power, not
navigation, not real-time execution, not actuator capability, not hardware readiness.
Static support is an approximation; K is a model-derived indicator; motion "energy" values are
heuristic proxies; 0.5 rad/s is a provisional screening flag only.
```

## Outcome (cloud + local offline verification passed)

- **Cloud + local offline verification passed.** Offline model and evaluation results only: no Gazebo, physics or contact-dynamics validation; no runtime or real-time validation; no hardware. The local byte-identical runs were two runs on the same Ubuntu PC, not a cloud-versus-local comparison.
- **The staged study ran completely, twice, with byte-identical artifacts.** It made 293 stage
  evaluations (270 unique) plus 6 separate speed checks.
- **The Stage 0/1 gate passed** after one real finding was fixed at its source: a phase-boundary
  round-off in M4.5 (see [The gate finding](#the-gate-finding-and-the-m45-fix)).
- **All three pre-registered hypotheses (H1–H3) are supported**, with the limits stated below.
  These are model predictions, not robot results.

The study is **not complete** until the owner reviews and merges the PR.

Statements are labelled as follows:
- **[MEASURED]**: a command run in this cloud environment.
- **[RESULT]**: an offline model output of the study. It is true *of the model*, not of the robot.

The design is in [M5_EVALUATION_PLAN.md](M5_EVALUATION_PLAN.md) and the user guide is
[SPIDERX_M5_EVALUATION_GUIDE.md](SPIDERX_M5_EVALUATION_GUIDE.md).

## Environment [MEASURED]

| Item | Value |
|---|---|
| Branch | `claude/spiderx-m5-offline-evaluation-plan`, from `origin/main` @ `3db1aec` |
| Stack | ROS 2 Humble (RoboStack), Python 3.11.16; setuptools temporarily pinned to 59.6.0 for `colcon build` (see the testing guide) and restored to 84.0.0 afterwards |
| Machine | Cloud VM, 4 cores, no GPU. No simulator was started; no hardware |

## Commits and test evidence [MEASURED]

Every batch had a **clean** `colcon build --symlink-install`, with `build/` and `install/`
removed first and 8 packages built, followed by the full `colcon test`. Each gate had 0 errors and
0 failures.

| Commit | Batch | Full-suite total | New tests (file) |
|---|---|---|---|
| `3db1aec` | M4.5 merged (baseline) | 556 | – |
| `1f8e323` | A: study specification and experiment matrix | 598 | 41 (`test_eval_study`) |
| `5450ab0` | B: runner, categories, Stage 0/1 gate | 621 | 22 (`test_eval_runner`) |
| `39c0f6d` | C: provenance-rich raw records and manifest | 636 | 14 (`test_eval_records`) |
| `1aa33d5` | D: derived tables, hypotheses, summary | 651 | 14 (`test_eval_tables`) |
| `ece1e23` | M4.5 fix: phase-boundary round-off (owner-approved) | 688 | 37 (`test_gait_trajectory` 41 → 78) |
| `d2c32f5` | E: regression pins of the shipped baselines | **700** | 11 (`test_eval_regression`) |

Each total is the earlier total plus the new pytest cases plus one CTest entry per new test file.

**Gate and mutation evidence** (`test_eval_runner`, `test_eval_records`):

| Test | What it shows |
|---|---|
| Forced gate failure | A wrong pinned expectation fails `baseline_reproduction_n200`; no Stage 2–4 evaluation is called; their records are `blocked_gate_failed`; CLI exit 3 |
| Perturbed speed evidence | A 1e-6 rad joint-series change fails `speed_scale_invariance` and blocks |
| **Mutation: gate enforcement removed** | `_gate_allows_later_stages` patched to always allow; the later stages ran, and `verify_result` reports "gate failed but … later-stage evaluations ran" |
| **Mutation: record filtering** | A dropped record, a rewritten category, an N/A margin set to 0, or a falsified status are each detected; `write_study` refuses an inconsistent result |
| CLI | `--check-only` and `--dry-run` exit 0 and write nothing; an invalid study exits 2; a dirty tree or an existing output directory is refused **before** any evaluation |

## The gate finding and the M4.5 fix

The first full run of the shipped study, at `1aa33d5`, **blocked at the Stage 0/1 gate**, as
designed [MEASURED]:
- exit 3;
- 36 gate evaluations (30 Stage 0/1 plus 6 speed checks);
- 240 records `blocked_gate_failed`;
- all evidence written to the ignored `log/m5_gate_probe/`.

Six gate checks passed. `pattern_separation` failed because wave's three-foot support fraction
varied with n: 0.625, 0.61, 0.605, 0.6025 and 0.60125 for n = 40 … 800.

**Cause** (in merged M4.5 code):
- `gait_phase.leg_phase` labels a leg as swing while `theta < 1 − β`.
- For wave, `1 − 0.85` evaluates to `0.15000000000000002` in floating point.
- The rear-left touch-down sample (u = 0.15) has `theta = 0.15` exactly.
- So that one sample was labelled swing at every n, giving a three-foot fraction of 0.6 + 1/n.

**Fix.** The owner chose to fix it at the source. In `ece1e23`, `leg_phase` snaps round-off within
1e-12 onto the phase boundary. Its new regression tests check exact swing counts and exact support
fractions at five resolutions; 12 of them fail on the unfixed code.

**Effect at n = 200.** Only wave changes: three-foot / four-foot fraction 0.605 / 0.395 →
0.600 / 0.400, and static-stability failing samples 61 → 60. Every M4.5 regression pin is
unchanged. A dated erratum is in [M4_5_TEST_RESULTS.md](M4_5_TEST_RESULTS.md).

## Final cloud validation [MEASURED]

| Check | Result |
|---|---|
| Clean build + full suite (at `d2c32f5`) | 8 packages; **700 tests, 0 errors, 0 failures, 0 skipped** |
| `--check-only` / `--dry-run` | exit 0; Stages 0–3 285 planned / 264 unique (240 configurations); Stages 0–4 **293 / 270** (246); **6** speed checks counted separately; the dry run lists 299 entries (23 duplicates mapped to their first entry) and writes nothing |
| Two complete Stage 0–4 runs (`--out log/m5_cloud_run_a`, `--out log/m5_cloud_run_b`), in parallel, clean tree at `d2c32f5` | both **exit 0**, 704 s and 707 s; study `m5_offline_evaluation-0f0034da093c`; 276 records each (270 unique + 6 speed); 276 evaluated; 0 blocked |
| Byte comparison | `diff -rq` of the two study directories: **no differences** (23 files each). The 21 compared files have equal SHA-256 in both manifests; `manifest.json` is identical too (SHA-256 `e7659b12ccd0f7aa…`). `environment.json` was also identical on this machine; it is excluded by design for cross-machine comparison |
| Gate | **all 7 checks PASS** in both runs (list below) |
| Generated files | only under the git-ignored `log/` (`.gitignore:3`); `git status` clean throughout; about 3.2 MB per run |
| Static regressions, no arguments, no simulator | `validate_m1_control.sh`: All M1 checks passed. `validate_m2_posture.sh`: All M2 checks passed. `validate_m3_kinematics.sh`: All M3 checks passed. `validate_m4_all_leg_ik.sh`: All M4 checks passed. No `--runtime` validator was run |

**Gate checks** (both runs): `baseline_valid_and_ik_feasible`, `baseline_reproduction_n200`,
`negative_controls_reproduced`, `static_support_na_preserved`,
`m45_policy_matches_shipped_flags`, `speed_scale_invariance` (joint-speed ratio exactly 2, joint
series identical, K unchanged) and `pattern_separation` (per-leg metrics equal for pace vs trot
at every n; support fractions resolution-invariant).

**Provenance** (in `manifest.json`):
- git commit `d2c32f5000cd7769adfae976206518dca5c26894`, dirty `false`;
- input SHA-256 (first 12 hex digits):

  | Input | SHA-256 |
  |---|---|
  | `m5_study.yaml` | `4a77d7487d7d` |
  | `m4_5_gaits.yaml` | `2ea0266c2d65` |
  | `spiderx_legs.yaml` | `78129b594b1d` |
  | expanded URDF | `2af3fefeb0a1` |

- the SHA-256 of all 16 source modules the study uses.

The manifest's `cli_args` record the absolute path of the installed script (`argv[0]`). So a
`manifest.json` from another machine differs even when every data file matches. Compare
`files_sha256` instead.

## Study results [RESULT]

### Accounting and categories (stage records; never filtered)

| Stage | Planned | Duplicates of earlier | New unique | `valid_pass` | `kinematic_check_fail` | `ik_infeasible` |
|---|---|---|---|---|---|---|
| S01 | 30 | 0 | 30 | 23 | 7 | 0 |
| S2 | 210 | 4 | 206 | 157 | 17 | 36 |
| S3 | 45 | 17 | 28 | 45 | 0 | 0 |
| S4 | 8 | 2 | 6 | 8 | 0 | 0 |
| **Total** (categories over the 270 unique records) | **293** | **23** | **270** | 210 | 24 | 36 |

These counts are per stage *membership*, so a duplicate entry is counted in each stage that
planned it. No record was `invalid_configuration` or `blocked_gate_failed`.

| Static-support status (unique records) | pass | fail | not_applicable (N/A) | not_evaluated |
|---|---|---|---|---|
| | 42 | 148 | 44 | 36 (IK infeasible) |

| Provisional joint-speed screen (unique records) | pass | fail | not_evaluated |
|---|---|---|---|
| | 192 | 42 | 36 |

### Baseline reproduction and negative controls (n = 200)

| Gait | Category | M4.5 verdict (failed checks) | Static support | Min margin | K (rad/m) | Screen | Reproduced |
|---|---|---|---|---|---|---|---|
| wave | valid_pass | FAIL (static_stability, joint_speed) | fail | −2.58 mm | 108.10 | fail | yes |
| tripod_crawl | valid_pass | FAIL (static_stability) | fail | −4.07 mm | 60.95 | pass | yes |
| ripple | valid_pass | PASS | fail (not required) | −2.10 mm | 40.58 | pass | yes |
| amble | valid_pass | PASS | fail (not required) | +0.84 mm | 29.29 | pass | yes |
| pace | valid_pass | PASS | **N/A** | N/A | 25.37 | pass | yes |
| trot | valid_pass | PASS | **N/A** | N/A | 25.37 | pass | yes |

**Negative controls.** `wave` and `tripod_crawl` fail the static-support approximation at
**every** resolution. The minimum margin is the same at all five n (−2.582 mm and −4.073 mm), and
the binding triangle is always LF|RF|RR. The number of failing three-contact samples scales
exactly with n: wave 12 / 30 / 60 / 120 / 240, tripod_crawl 22 / 54 / 108 / 217 / 434.

**Static support required vs reported.** "fail (not required)" means the pattern has two-foot
phases, so `m45_policy` does not require static support. The status is reported over its
three-contact samples only. N/A rows: all 46 not-applicable records (44 stage records + 2 speed
checks) have an empty margin, and the derived tables print **N/A**, never 0.

### Resolution (Stage 1)

The 200-sample results are a defined numerical resolution, not ground truth.

| Metric | Behaviour from n = 40 to n = 800 |
|---|---|
| min static margin, support fractions | identical at every n (aligned grids) |
| K | converges: wave 90.12 → 108.10 → 108.93 rad/m (n = 40 / 200 / 800); 200 → 400 change ≤ 0.67 % for every baseline (the H2 discretisation band) |
| max joint step | ∝ 1/n. `joint_continuity` fails for all six gaits at n = 40, and for wave also at n = 100. These are the 7 Stage 1 `kinematic_check_fail` records |

### Stage 2 – kinematic block (lateral sequence; 210 points)

**IK feasibility** (`derived/feasibility_boundary.csv`). All 36 infeasible points fail on
`joint_limits`. The swing never left the geometric reach; it needed joint angles beyond the URDF
limits minus 0.05 rad.

| | h ≤ 30 mm | h = 35 mm |
|---|---|---|
| β = 0.50 | feasible up to L = 100 mm; L = 120 mm infeasible | infeasible at every L |
| β ≥ 0.55 | feasible at every tested L, up to 120 mm (the grid ceiling) | infeasible at every L |

**Other kinematic checks.**
- **`joint_continuity`.** The 17 `kinematic_check_fail` points all fail this check, at
  β ∈ {0.75, 0.85} with large L and h: the per-sample step reaches 0.051–0.082 rad at n = 200.
  This is a resolution-dependent flag caused by fast, short swings, not evidence of an IK branch
  flip.
- **Margins.** The minimum singularity margin over feasible points is 1.11 rad. The minimum
  joint-limit margin is 0.0024 rad, close to the soft limit.

**K at L = 40 mm, h = 15 mm:** 25.37 / 29.29 / 40.58 / 60.95 / 108.10 rad/m for
β = 0.50 / 0.55 / 0.65 / 0.75 / 0.85. The derived v_adm at the 0.5 rad/s placeholder is
19.7 / 17.1 / 12.3 / 8.2 / 4.6 mm/s. The provisional screen fails on 30 Stage 2 points: 26 at
β = 0.85, 3 at 0.75 and 1 at 0.65.

**Static support.** 18 Stage 2 points pass, all lateral sequence at β = 0.55 with L ≥ 80 mm
(margins 6.4–12.9 mm). They pass **only over the 20 % of the cycle with three stance feet**.
Static support is not required for that pattern, so this is not a statically stable gait.

### Stage 3 – support block

- **Lateral sequence.** At β ≥ 0.55 every point fails static support (binding LF|RF|RR). At
  β = 0.55 the margin rises with stroke, −1.72 / +0.84 / +3.57 mm at L = 20 / 40 / 60 mm, but
  stays below 5 mm. At β = 0.5 the static support is N/A.
- **Diagonal and lateral pairs.** At β = 0.5 static support is N/A. At β ≥ 0.55 the
  three-contact samples are **four-foot** double-support phases (10–70 % of the cycle). Their
  margins are 75.6–84.5 mm, so they "pass" over those samples only; the two-foot phases remain
  statically unsupported.
- **Pattern invariance.** Per-leg metrics are equal across the three patterns at all 15
  (β, L) cells; the largest relative difference is 8.7e-15. This confirms assumption (b) beyond
  β = 0.5.

### Stage 4 – stance-centre translation sensitivity (mechanism check, not body sway)

| β | y = −10 mm | −5 mm | 0 | +5 mm |
|---|---|---|---|---|
| 0.75: min margin (binding) | −2.35 mm (RF\|LR\|RR) | −0.89 mm (LF\|RF\|RR) | −4.07 mm (LF\|RF\|RR) | −7.24 mm (LF\|RF\|RR) |
| 0.85: min margin (binding) | −0.85 mm (RF\|LR\|RR) | +0.59 mm (LF\|RF\|RR) | −2.58 mm (LF\|RF\|RR) | −5.74 mm (LF\|RF\|RR) |

**What this shows.**
- Moving the stance centre backward, toward the COM, raises the LF|RF|RR margin. Moving it
  forward lowers it.
- Beyond about −5 mm the opposite triangle (RF|LR|RR) becomes binding.
- **All eight points still fail the 5 mm screening threshold.** This confirms the mechanism behind
  the crawl-gait failures (the COM sits behind the foot centre). It does not solve them, and it is
  not body sway.

### Pre-registered hypotheses (`derived/hypotheses.json`; model predictions only)

| | Outcome | Evidence | Limits |
|---|---|---|---|
| **H1** LS crawl (β 0.75 / 0.85) never reaches 5 mm, binding LF\|RF\|RR | **supported** | 70 evaluable configurations, 0 exceptions | 12 configurations not evaluable (IK infeasible) |
| **H2** K rises strictly with β, does not fall with h | **supported** | 265 pairs; 0 reversals; 0 within the 0.67 % band | Only IK-feasible cells |
| **H3** L_max non-increasing in h, non-decreasing in β | **supported** | 58 pairs, 0 violations | Weak: the tested L grid tops out at 120 mm, which is feasible in 24 of 35 cells, so the boundary is only observed at β = 0.5 and h = 35 mm |

## What these results do not show

- Walking, a working gait, dynamic stability or balance.
- Energy consumption, power, efficiency, actuator work, torque cost, or battery or runtime. The
  proxies in the tables are heuristic.
- Any servo or actuator capability. K and v_adm are model-derived; 0.5 rad/s is a placeholder.
- Hardware readiness, terrain performance, navigation or real-time execution.
- Byte identity across machines. It was verified only between two runs on the same machine.

## Local verification (owner's Ubuntu PC) – passed

The owner ran these checks on the M5 branch on their Ubuntu PC and reported the results below.
Everything was offline.

| Check | Result |
|---|---|
| Git working tree | Clean before and after verification |
| Clean build | 8 packages finished in 12.7 s; no warnings or errors |
| Full test suite | **700 tests, 0 errors, 0 failures, 0 skipped**; `colcon test` exit 0 and `colcon test-result` exit 0 |
| New M5 tests | `test_eval_study` 41, `test_eval_runner` 22, `test_eval_records` 14, `test_eval_tables` 14, `test_eval_regression` 11 |
| M4.5 trajectory tests | `test_gait_trajectory` 78, including the phase-boundary regression tests |
| `--check-only`, `--dry-run` | Both passed and wrote no files |
| Full study run A, `--out log/m5_local_a` | Exit 0 in 937 s; 276 records, 23 files |
| Full study run B, `--out log/m5_local_b` | Exit 0 in 925 s; 276 records, 23 files |
| Determinism | The two local output directories were **byte-identical across all 23 files**, including `environment.json` and `manifest.json` (same machine) |
| Stage 0/1 gate | All seven gate checks passed in both runs |
| Static regressions (no simulator) | M1, M2, M3 and M4 validators all passed |

**Scope of the verification itself.** None of the following happened:
- Gazebo, ROS launch, runtime validation or hardware activity;
- edits to tracked files outside documentation;
- dynamic-walking claims;
- commits, pushes or PR updates.

**One background process** was found: a stale Bash wait loop. It was stopped safely after the test
output had already shown 700/700 passing.

As in the cloud, these are offline model results. They show that the study runs, passes its
gate and reproduces byte-for-byte between two runs on the owner's machine. Local outputs were not
byte-compared with the cloud outputs. They do not show walking, dynamic stability, real-time
behaviour, hardware validity, energy or power performance, or navigation.

These are the commands, all offline; none starts Gazebo or touches hardware.

```bash
cd ~/spiderx_ws && colcon build --symlink-install && source install/setup.bash
colcon test --packages-select spiderx_controller && colcon test-result --verbose     # 0 failures
ros2 run spiderx_controller m5_offline_evaluation --check-only                      # 285/264, 293/270, 6
ros2 run spiderx_controller m5_offline_evaluation --dry-run | grep -c " eval "      # 299
ros2 run spiderx_controller m5_offline_evaluation --out log/m5_local_a              # exit 0, ~12-25 min
ros2 run spiderx_controller m5_offline_evaluation --out log/m5_local_b              # exit 0
diff -rq --exclude=environment.json --exclude=manifest.json log/m5_local_a log/m5_local_b   # no output
cat log/m5_local_a/*/derived/summary.md
./scripts/validate_m4_all_leg_ik.sh && ./scripts/validate_m3_kinematics.sh
```

Expected:
- every gate check passes;
- the category counts and hypothesis outcomes above;
- the local `files_sha256` should equal the cloud hashes in
  [Artifact checksums](#artifact-checksums-cloud-run-a--run-b). If any differ, report the
  per-metric differences rather than assuming an error; cross-machine byte identity is not
  claimed.

## Artifact checksums (cloud, run A = run B)

Study `m5_offline_evaluation-0f0034da093c`, commit `d2c32f5`. SHA-256, first 16 hex digits:

| File | SHA-256 | File | SHA-256 |
|---|---|---|---|
| derived/accounting.csv | `325a93d3fbb3c2f7` | raw/checks.csv | `3a298e03a17f470d` |
| derived/baseline_reproduction.csv | `ab7857a0baf156d6` | raw/configs.json | `e071c49c62bb44c0` |
| derived/feasibility_boundary.csv | `110253632536db61` | raw/evaluations.csv | `ae257d921272c2fa` |
| derived/gate.csv | `7c67930c1946b5d5` | raw/evaluations.json | `46675b0329d358f8` |
| derived/hypotheses.json | `4b56f2b43bdf2d62` | raw/gate.json | `482302b6c1369cc5` |
| derived/metric_labels.csv | `b9ed175139fa178b` | raw/ik_reasons.csv | `d11155016bf17a79` |
| derived/negative_controls.csv | `02985b6f3b3fff98` | raw/plan.csv | `f2c6f2a7380997b7` |
| derived/pattern_invariance.csv | `4616a18bf4cd755b` | derived/summary.json | `4ee29b4099704d25` |
| derived/resolution.csv | `a64fa05c4bbe3824` | derived/summary.md | `ca8fbce2d2be873a` |
| derived/stage2_kinematic.csv | `579a46910fe3e0a4` | manifest.json (this machine) | `e7659b12ccd0f7aa` |
| derived/stage3_support.csv | `d6522f3978835840` | | |
| derived/stage4_translation.csv | `efd480dcbe55716e` | | |
