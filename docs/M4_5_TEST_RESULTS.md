# M4.5 Test Results – Offline Multi-Gait Configuration and Trajectory Validation

```text
OFFLINE kinematic analysis only. No Gazebo gait playback, no dynamic walking, no hardware.
Not walking, not a validated gait, not balance control, not navigation, not hardware validation.
Stability values are quasi-static APPROXIMATIONS; "energy" values are heuristic PROXIES.
```

## Outcome (cloud + local verified)

- **Offline gait configuration and trajectory validation framework implemented.**
- **Six gait configurations are kinematically evaluated** against the URDF-derived M3/M4 IK, the
  URDF joint limits and a quasi-static stability approximation. Each one gets a PASS/FAIL verdict
  and the exact list of failed checks.

What this does **not** show:
- that SpiderX can walk;
- that any gait is dynamically stable;
- any energy, power or servo-load figure;
- anything about hardware.

Nothing was played back in Gazebo or sent to a controller.

The owner repeated the checks on their Ubuntu PC and they passed, with identical results. See
[Local verification](#local-verification-owners-ubuntu-pc--passed).

Statements are labelled as follows:
- **[MEASURED]**: produced by a command in the cloud environment. The owner's local results are
  in their own section.
- **[REPORTED]**: a result the tool computes from the model. It is true *of the model*, not of the robot.
- **[ASSUMPTION]**: believed, not proven.
- **[OPEN]**: the owner must decide.

## Environment [MEASURED]

| Item | Value |
|---|---|
| Branch | `claude/spiderx-m45-offline-gait-framework`, from `main` @ `c4a993c` (M4.1 merged) |
| Commits | plan `2d8b07d`; Batches A `6dca08e`, B `fd03a90`, C `c39f501`, D `f0a3245`, E `ab9f08e`, F `3e59f33` |
| Stack | ROS 2 Humble (RoboStack), Python 3.11.16, matplotlib 3.10.9, setuptools 59.6.0 for the build (as the testing guide notes) |
| Machine | Cloud VM, no GPU. No simulator was needed or started by M4.5 |
| Configuration | `spiderx_controller/config/m4_5_gaits.yaml`, `config_version: 1`, 200 samples per cycle |

## Build and unit tests [MEASURED]

A full `colcon build --symlink-install` (8 packages) plus `colcon test` was run at the end of
**every** batch. There were 0 errors and 0 failures each time.

| After | `colcon test-result --all` | New test file (pytest cases) |
|---|---|---|
| main @ `c4a993c` | 385 | – |
| Batch A | 438 | `test_gait_config.py` (52) |
| Batch B | 480 | `test_gait_trajectory.py` (41) |
| Batch C | 493 | `test_gait_kinematics.py` (12) |
| Batch D | 515 | `test_gait_metrics.py` (21) |
| Batch E | 533 | `test_gait_report.py` (17) |
| Batch F | **556**, 0 errors, 0 failures, 0 skipped | `test_gait_regression.py` (20); `test_gait_report.py` grew to 19 |

Each new test file adds its pytest cases plus one CTest entry, so 556 = 385 + 165 + 6.

### Checks that bite (acceptance criterion 3)

| Mutation or bad input | Caught by |
|---|---|
| Over-long stroke (`step_length_m: 0.30`) | `ik_feasible` fails; energy proxies become `None` (unknown), not 0 |
| `swing_order` disagreeing with `phase_offsets` | Configuration error, refused before any evaluation |
| A trot flagged `requires_static_stability: true` | `static_stability` fails with "only 2 feet in stance" |
| A tenfold body speed | `joint_speed` fails |
| An injected 1 m/s velocity jump in swing | `transition_velocity_jump` fails at all 8 transitions (4 legs × 2) |
| Three hand-injected bugs in the phase and trajectory code (Batch B) | 16, 8 and 23 tests failed respectively; then reverted |
| `offline_only: false` in the YAML | The CLI prints `REFUSED`, exits 2 and writes nothing |

### One failure found during Batch F, and its root cause

The first Batch F run had one failing test, `test_comparison_tables`. The test expected trot to
PASS, but it evaluated trot at a coarse **40** samples per cycle to save time.

`joint_continuity` bounds the joint change **per sample** (0.05 rad, which detects IK branch
flips), so it depends on the sample count:

| Samples per cycle | Largest trot step | Result |
|---|---|---|
| 40 | 0.0504 rad | fails |
| 200 (configured) | 0.0101 rad | passes |

The test now uses the configured resolution, and the code is unchanged. The cause was the test's
setup, not a flake.

**Lesson:** compare `joint_continuity` results only at the same `samples_per_cycle`.

## Offline analysis run [MEASURED] / results [REPORTED]

```bash
ros2 run spiderx_controller m4_5_gait_analysis          # writes log/m4_5_gait_analysis/
```

- **Run:** exit 0, about 20 s for all six gaits.
- **Output:** 40 files:
  - 6 × (`report.json`, `samples.csv` and 4 PNGs);
  - `comparison/{summary.csv, report.json, metrics.png}`;
  - `run_info.json`.
- **Determinism:** a second run into another directory produced **byte-identical files: all 40
  SHA-256 hashes matched**, including the PNGs.
- **Not committed:** `log/` is git-ignored, so the outputs are not in the repository.

**Shared parameters.** All six gaits use the same parameters, so the comparison isolates the
footfall pattern:
- stroke L = 40 mm;
- step height h = 15 mm;
- body speed v = 5 mm/s;
- stance offsets 0;
- cycloid swing.

The period T = L / (βv) and the stride S = L / β follow from these.

| Gait | β | T (s) | Stride (mm) | Min. feet | Verdict | Failed checks | Min static margin* | Statically stable* | Max joint speed |
|---|---|---|---|---|---|---|---|---|---|
| `wave` | 0.85 | 9.41 | 47.1 | 3 | **FAIL** | `static_stability`, `joint_speed` | −2.58 mm | 84.5 % | 0.541 rad/s |
| `tripod_crawl` | 0.75 | 10.67 | 53.3 | 3 | **FAIL** | `static_stability` | −4.07 mm | 77.0 % | 0.305 rad/s |
| `ripple` | 0.65 | 12.31 | 61.5 | 2 | PASS | – | −2.10 mm (information) | 50.5 % | 0.203 rad/s |
| `amble` | 0.55 | 14.55 | 72.7 | 2 | PASS | – | +0.84 mm (information) | 20.0 % | 0.146 rad/s |
| `pace` | 0.50 | 16.00 | 80.0 | 2 | PASS | – | n/a (never 3 feet) | 0 % | 0.127 rad/s |
| `trot` | 0.50 | 16.00 | 80.0 | 2 | PASS | – | n/a (never 3 feet) | 0 % | 0.127 rad/s |

\* These are quasi-static **approximations**. They assume URDF CAD (steel-density placeholder)
masses, point feet at the derived tips, flat ground and no dynamics. Static stability is
**required** only for `wave` and `tripod_crawl`. For the others it is reported as information and
does not affect the verdict. "PASS" for ripple, amble, pace and trot means their *kinematic* checks
passed. It does **not** mean they are stable or that they walk.

**All six gaits are kinematically feasible** [REPORTED]. At every sample, for every leg:
- IK solved, with FK residual ≤ 1e-6 m;
- minimum singularity margin 1.21 rad (threshold 0.2);
- minimum joint-limit margin 0.205 rad, beyond the 0.05 rad soft margin;
- no branch flips;
- foot velocity is continuous at every stance/swing transition (jump < 1e-6 m/s).

The vertical acceleration jump at lift-off and touch-down is non-zero. That is expected for a
cycloid and is reported as information.

### Findings [REPORTED]

1. **The crawl gaits fail static stability while the rear-left foot swings.**
   - Where: wave fails at 61 of 200 samples and tripod_crawl at 108. In every listed failure, the
     support triangle is FL–FR–RR.
   - Why: the URDF whole-robot COM is (0.0510, −0.0576) m. That is ≈ 5.8 mm behind the centre of
     the neutral feet (y = −0.0518 m), so the COM's ground projection falls outside the FL–FR–RR
     triangle.
   - Status: this is a property of the current model and parameters. **It was not tuned away.**
   - What a fixed shift does: in an exploratory scratch run (not committed), shifting all tripod_crawl
     feet 5 mm back (`stance_center_offset_m: [0, -0.005]`) improved the margin to −0.89 mm
     (94.5 % stable). It still fails the 5 mm requirement, and a 12 mm shift overshoots to
     −3.63 mm.
   - Remedies (future work): body sway or better mass data. See the owner decisions below.
2. **`wave` exceeds the joint-speed placeholder.**
   - Measured: 16 samples exceed 0.5 rad/s during the short swing (1 − β = 0.15 of the cycle).
     They are spread across all four thigh joints, four samples each (`lf_`, `rf_`, `lr_` and
     `rr_thigh_joint`). The maximum, 0.541 rad/s, was on `lf_thigh_joint`.
   - The threshold, 0.5 rad/s, is the `SIMULATION_PLACEHOLDER` in `spiderx_legs.yaml`. It is
     **not** a servo specification.
   - Deviation from the plan: plan §5.2 proposed choosing T so the placeholder is met. Batch A
     instead fixed **equal body speed** for all gaits, so that the comparison is fair. The report
     states the failure instead of slowing wave down.
3. **Energy proxies rank the gaits by stride, not by footfall pattern.**
   - Foot path per metre, joint travel per metre and the lift-work proxy all fall as β falls. A
     lower β means a longer stride (L / β), so there are fewer steps per metre.
   - `pace` and `trot` have **identical** proxies: each leg follows the same trajectory, just at
     a different phase.
   - These are heuristic proxies, **not energy**. A paper comparison needs dynamics and measured
     data.

| Gait | Steps / m | Foot path (m / m) | Joint travel (rad / m) | Lift-work proxy (J / m) |
|---|---|---|---|---|
| `wave` | 85 | 5.10 | 90.7 | 5.12 |
| `tripod_crawl` | 75 | 4.90 | 81.4 | 4.64 |
| `ripple` | 65 | 4.72 | 72.3 | 4.17 |
| `amble` | 55 | 4.54 | 63.4 | 3.72 |
| `pace` | 50 | 4.47 | 59.0 | 3.50 |
| `trot` | 50 | 4.47 | 59.0 | 3.50 |

`test_gait_regression.py` pins these verdicts and key metrics. Any code or config change that moves
them fails the tests and has to be explained here.

## Regression and scope checks [MEASURED]

| Check | Result |
|---|---|
| `./scripts/validate_m1_control.sh` (static) | `All M1 checks passed.` |
| `./scripts/validate_m2_posture.sh` (static) | `All M2 checks passed.` |
| `./scripts/validate_m3_kinematics.sh` (static) | `All M3 checks passed.` |
| `./scripts/validate_m4_all_leg_ik.sh` (static) | `All M4 checks passed.` |
| `git diff --name-only origin/main...HEAD` | Only new `gait_*` modules, the CLI, its tests, `m4_5_gaits.yaml`, `CMakeLists.txt` (install and test lines), `package.xml` (`python3-matplotlib`) and docs. **No** URDF, mesh, controller YAML, launch, bringup, M1–M4.1 code or simulation change |
| `m4_5_gait_analysis --check-only` | Exit 0, nothing evaluated |
| Invalid configuration (CLI) | Exit 2 (`REFUSED`) |
| `--strict` with a failing gait | Exit 1 |

**Runtime regressions not re-run.** The Gazebo runtime regressions (`--runtime`) were **not**
re-run, because M4.5 changes no runtime file. Plan §9 lists them as optional.

## Local verification (owner's Ubuntu PC) – passed

The owner ran these checks on their Ubuntu PC and reported the results below. Everything was
offline: no Gazebo was started and no hardware was touched.

**Starting state**
- Branch `claude/spiderx-m45-offline-gait-framework` at
  `4a6553cbb5cedd48de81658fb227f701bc58e5a8`, tracking `origin`.
- The tree was clean at the start and at the finish.
- `log/` was confirmed git-ignored at `.gitignore:3`.

**Build**
- `colcon build --symlink-install`: 8 packages finished in 11.2 s.
- Exit 0, with no warnings or errors.

**Automated tests**
- 556 tests: 0 errors, 0 failures, 0 skipped.
- New M4.5 coverage is 165 tests in six groups:

| Test file | Tests |
|---|---|
| `test_gait_config` | 52 |
| `test_gait_trajectory` | 41 |
| `test_gait_kinematics` | 12 |
| `test_gait_metrics` | 21 |
| `test_gait_report` | 19 |
| `test_gait_regression` | 20 |

**Offline analysis**
- The owner read the real CLI help first and used `--out`; there is no `--output-dir` option.
- `--check-only` passed with exit 0.
- There were two full runs:

  ```bash
  ros2 run spiderx_controller m4_5_gait_analysis --out log/m4_5_local_a
  ros2 run spiderx_controller m4_5_gait_analysis --out log/m4_5_local_b
  ```

  - **Exit code.** Each run exited 0 after about 25 s. That is expected without `--strict`, even
    though some gait verdicts are FAIL.
  - **Files.** Each run produced 40 files: 8 JSON, 7 CSV and 25 PNG.
- **Determinism.**
  - Sorted SHA-256 manifests (relative path plus hash) matched for all 40 files, and `diff -rq`
    agreed.
  - The PNGs and `run_info.json` were byte-identical.
- **Plots.** matplotlib 3.5.1 was available and the plots were generated; the `--no-plots`
  fallback was not needed.
- **Location.** The generated files stayed under the git-ignored `log/` directory.

**Results, cross-checked from the machine-readable reports**

These match the cloud results above exactly.

| Gait | Verdict | Min static margin | Statically stable | Max joint speed | Notes |
|---|---|---|---|---|---|
| `wave` | **FAIL**: `static_stability`, `joint_speed` | −2.58 mm | 84.5 % (61 of 200 samples fail) | 0.541 rad/s vs the 0.5 rad/s reference | 16 joint-speed threshold breaches |
| `tripod_crawl` | **FAIL**: `static_stability` | −4.07 mm | 77.0 % (108 of 200 samples fail) | 0.305 rad/s | |
| `ripple` | PASS | −2.10 mm (information) | 50.5 % | 0.203 rad/s | |
| `amble` | PASS | +0.84 mm (information) | 20.0 % | 0.146 rad/s | |
| `pace` | PASS | n/a: never three or more stance feet | 0 % | 0.127 rad/s | |
| `trot` | PASS | n/a: never three or more stance feet | 0 % | 0.127 rad/s | |

- **Feasibility.** All six gaits are feasible under sampled IK:
  - worst FK residual 9.0e-17 m;
  - singularity margin 1.21 rad;
  - joint-limit margin 0.205 rad;
  - no IK branch flips;
  - largest transition velocity jump 1.4e-9 m/s.
- **Stability failures.** Every listed stability failure used the FL–FR–RR support triangle.

**Static regressions** (no arguments, no simulator)

| Validator | Result |
|---|---|
| `validate_m1_control.sh` | `All M1 checks passed.` |
| `validate_m2_posture.sh` | `All M2 checks passed.` |
| `validate_m3_kinematics.sh` | `All M3 checks passed.` (84 unit tests) |
| `validate_m4_all_leg_ik.sh` | `All M4 checks passed.` (136 unit tests) |

**Environment and safety**
- No Gazebo was started and no hardware was touched.
- A process check found only unrelated system processes and the checking shell.
- No tracked file changed locally. No commits, pushes or PR updates were made locally.

**Commands, for repeating the check**

```bash
cd ~/spiderx_ws && colcon build --symlink-install && source install/setup.bash
colcon test --packages-select spiderx_controller && colcon test-result --verbose   # 0 failures
ros2 run spiderx_controller m4_5_gait_analysis --help
ros2 run spiderx_controller m4_5_gait_analysis --check-only                       # exit 0
ros2 run spiderx_controller m4_5_gait_analysis --out log/m4_5_local_a             # exit 0, 6 gaits
ros2 run spiderx_controller m4_5_gait_analysis --out log/m4_5_local_b
diff -rq log/m4_5_local_a log/m4_5_local_b                                        # no output
./scripts/validate_m4_all_leg_ik.sh && ./scripts/validate_m3_kinematics.sh
```

`colcon test-result --all` totals depend on the machine's earlier test runs, so compare the
per-file counts.

## Owner decisions

These replace the earlier open questions.

- **Q1. Gait naming.** Keep the current YAML names for M4.5. The quadruped meaning of each name
  is in the terminology mapping in
  [SPIDERX_GAIT_FRAMEWORK.md §2.1](SPIDERX_GAIT_FRAMEWORK.md#21-gait-name-terminology-mapping).
- **Q2. Roadmap.** The old walking goals stay as an unscheduled "Future — gait playback" item.
- **Q3. Body sway.** Body sway is documented future work only and is not modelled. That sway would
  widen the crawl gaits' static margins is still an [ASSUMPTION].
- **Q4. matplotlib.** Approved as an `exec_depend`. `--no-plots` is kept for headless,
  tables-only output.

## Erratum (2026-10-01): phase-boundary round-off in `gait_phase` (found by M5, fixed in `ece1e23`)

The results above are kept as originally recorded. This section corrects them.

**What was wrong.**
- `gait_phase.leg_phase` labelled a leg as swing while `(u − φ) mod 1 < 1 − β`.
- For `wave` (β = 0.85), `1 − 0.85` evaluates to `0.15000000000000002` in floating point.
- The rear-left touch-down sample (u = 0.15, φ = 0) gives exactly `0.15`, so that one sample was
  labelled swing instead of stance at every sample count.

**How it was found.** The M5 Stage 0/1 gate check `pattern_separation` found it. Wave's
three-foot support fraction varied with the sample count (0.6 + 1/n instead of 0.6). The owner
approved fixing it at the source.

**The fix.** `leg_phase` now snaps round-off within 1e-12 onto the phase boundary, so lift-off is
swing and touch-down is stance. Regression tests check exact swing counts and exact wave support
fractions at n ∈ {40, 100, 200, 400, 800}.

**Corrected values (n = 200).** Only `wave` changes.

| Quantity | Recorded above | Corrected |
|---|---|---|
| wave three-foot / four-foot support fraction | 0.605 / 0.395 | **0.600 / 0.400** |
| wave samples failing `static_stability` | 61 of 200 | **60 of 200** |

**Unchanged (re-verified by the unchanged `test_gait_regression.py` pins).**
- every verdict and failed-check list;
- every minimum static margin;
- `fraction_statically_stable` (wave 84.5 %);
- every joint speed, including wave's 16 placeholder breaches;
- every energy proxy and kinematic margin;
- all values of the other five gaits.

The M4.5 conclusions are unaffected. See [M5_TEST_RESULTS.md](M5_TEST_RESULTS.md).
