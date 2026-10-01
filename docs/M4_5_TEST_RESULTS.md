# M4.5 Test Results – Offline Multi-Gait Configuration and Trajectory Validation

```text
OFFLINE kinematic analysis only. No Gazebo gait playback, no dynamic walking, no hardware.
Not walking, not a validated gait, not balance control, not navigation, not hardware validation.
Stability values are quasi-static APPROXIMATIONS; "energy" values are heuristic PROXIES.
```

## Outcome (cloud only; local verification pending)

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

Statements are labelled as follows:
- **[MEASURED]**: produced by a command in this environment.
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
   - Remedies (future work): body sway or better mass data. See Q3.
2. **`wave` exceeds the joint-speed placeholder.**
   - Measured: 0.541 rad/s on `lf_thigh_joint` (16 samples above 0.5 rad/s) during its short swing
     (1 − β = 0.15 of the cycle).
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

## Local verification (owner's Ubuntu PC) – pending

Suggested steps. All of them are offline, and none starts Gazebo or touches hardware.

```bash
cd ~/spiderx_ws && colcon build --symlink-install && source install/setup.bash
colcon test --packages-select spiderx_controller && colcon test-result --verbose   # 0 failures
ros2 run spiderx_controller m4_5_gait_analysis --check-only                       # exit 0
ros2 run spiderx_controller m4_5_gait_analysis                                    # exit 0, 6 gaits
cat log/m4_5_gait_analysis/comparison/summary.csv
./scripts/validate_m4_all_leg_ik.sh && ./scripts/validate_m3_kinematics.sh
```

Expected: the verdict table above.
- If `python3-matplotlib` is missing, install it with
  `sudo apt install python3-matplotlib`, or use `--no-plots`. Tables are still written.
- `colcon test-result --all` totals depend on the machine's earlier test runs. Compare the
  per-file counts instead.

## Open questions [OPEN]

- **Q1. Gait naming.** The duty-factor mapping of the hexapod terms tripod, ripple and wave onto a
  quadruped is unconfirmed. Plan §5.2 defines it. The names are YAML data and can be changed without
  code changes.
- **Q2. Roadmap.** The old M4.5 goals ("static walk, then trot; walks 1 m in simulation") are now
  listed as a separate, unscheduled future item. The owner should place them: before M5, inside
  M5, or in a new milestone.
- **Q3. Body sway.** Lateral and longitudinal body sway would likely widen the crawl gaits' static
  margins [ASSUMPTION]. It is not modelled.
- **Q4. matplotlib.** matplotlib is now an `exec_depend`. Figures are optional (`--no-plots`).
