# M4.5 Plan – Offline Multi-Gait Configuration and Trajectory Validation

> This plan was written and committed **before** any M4.5 implementation.
> Branch `claude/spiderx-m45-offline-gait-framework`, created from `main` @ `c4a993c` (PR #10, M4.1,
> merged). `main`'s tree is identical to the M4.1 head verified in the cloud and locally (`b979b03`).

```text
OFFLINE analysis only: pure-Python kinematic and geometric evaluation of gait configurations.
No Gazebo gait playback, no dynamic walking, no navigation, no hardware.
No change to the URDF, meshes, controller YAML, launch files or any simulation behaviour.
```

**Claims M4.5 may make**, and only if the acceptance criteria in §9 pass:
- "Offline gait configuration and trajectory validation framework implemented."
- "Gait configurations are kinematically evaluated against the URDF-derived IK, joint limits and a
  static-stability approximation."

**M4.5 must never claim:** walking, a working gait on the robot, balance, dynamic stability,
energy or power measurements, navigation, or hardware feasibility.

Statements are labelled **[FACT]** (read in this repository or measured by an audit probe),
**[DECISION]** (a design choice made here; changeable), **[ASSUMPTION]** (believed, not proven) or
**[OPEN]** (needs the owner).

---

## 1. Repository state [FACT]

- `main` @ `c4a993c` merges M4.1. `git diff b979b03 c4a993c` is empty, and the working tree is
  clean.
- **M1–M4 scope** (`src/spiderx_controller/`):

  | Milestone | Code | Config | Tests |
  |---|---|---|---|
  | M1 | `joint_safety.py`, `trajectory_client.py`, `scripts/test_one_joint.py`, `scripts/test_neutral_pose.py` | `spiderx_legs.yaml`, `spiderx_poses.yaml`, `spiderx_ros2_controllers.yaml` | `test_joint_safety.py`, `test_config_check.py` |
  | M2 | `posture_config.py`, `posture_metrics.py`, `scripts/run_posture_hold_test.py` | `m2_simulation_postures.yaml` | `test_posture_config.py`, `test_posture_metrics.py` |
  | M3 | `leg_kinematics.py`, `kinematics_targets.py`, `kinematics_validation.py`, `scripts/validate_leg_kinematics` | `m3_kinematics_targets.yaml` | `test_leg_kinematics.py`, `test_kinematics_targets.py` |
  | M4 | `leg_kinematics.py` (opt-in all legs), `m4_pose_targets.py`, `m4_pose_validation.py`, `scripts/m4_pose_validation` | `m4_pose_targets.yaml` | `test_all_leg_kinematics.py`, `test_m4_pose_targets.py`, `test_m4_pose_validation.py` |
  | M4.1 | `launch/controller.launch.py` | – | `test_controller_launch.py`, `test_validator_state_checks.py` |

- **The M4 pipeline**, which M4.5 mirrors:
  1. `m4_pose_targets.yaml` declares poses as per-leg **offsets** from each leg's CAD-neutral
     derived foot tip (`m4_pose_targets.yaml:6-12`).
  2. `m4_pose_targets.validate_config()` (`m4_pose_targets.py:82-163`) checks the YAML strictly.
     Required are `simulation_only: true`, `frame: base_link`, units `{m, rad}`, and the
     reference `cad_neutral` / `derived_foot_tip`. The margin must equal the M1 margin, and the
     IK tolerance must equal `leg_kinematics.IK_POSITION_TOL_M`.
  3. `evaluate_pose()` (`m4_pose_targets.py:172-233`) solves IK atomically for all four legs and
     applies the safety bounds and M1's `check_pose`. It then checks **necessary** support
     geometry only: coplanar support feet, ≥ 3 support legs, and other feet lifted.
  4. `load_and_evaluate()` (`m4_pose_targets.py:261-286`) loads the YAML with
     `posture_config.load_yaml_strict()` (`posture_config.py:58`), which refuses duplicate keys.

## 2. APIs M4.5 will reuse unchanged [FACT]

All of these are in `src/spiderx_controller/spiderx_controller/leg_kinematics.py`, which is pure
Python and imports no ROS:

| API | Line | Use in M4.5 |
|---|---|---|
| `FRAME = 'base_link'`, `ALL_LEGS` | 33, 37 | Frame and leg order of every gait config |
| `IK_POSITION_TOL_M = 1e-6`, `DEFAULT_MARGIN_RAD = 0.05` | 40, 44 | Same tolerances as M3/M4 |
| `resolve_leg(leg, allow_all_legs=True)` | 187 | Leg names and aliases in gait YAML |
| `LegGeometry` (`.limits`, `.tip0`, `.joint_names`, `.axes`, `.axis_points`) | 299–327 | Joint limits and the CAD-neutral tip of each leg |
| `load_all_geometries(urdf_root=None, legs_cfg=None, config_dir=None)` | 603 | Geometry of all four legs from one expanded URDF |
| `forward(geom, q)` | 411 | FK check of every IK sample |
| `inverse(geom, target, frame='base_link', reference=None, margin=0.05)` | 465 | Per-leg, per-sample IK. It never clamps and returns `reason` ∈ {`ok`, `unreachable_hip`, `unreachable_knee`, `joint_limits`, `singular`, `numerical`, …}, every candidate, and the hip/knee singularity margins. `reference` selects the branch nearest the previous sample |
| `inverse_all(geoms, targets, frame, reference, margin)` | 629 | All-leg atomic IK (no partial solutions) |
| `check_limits(geom, q, margin)` | 449 | Limit-violation strings |
| `parse_urdf_joints(urdf_root)`, `_joint_transform(j, q)` | 210, 290 | Whole-robot COM from the URDF `<inertial>` data (§5) |

Other reused helpers:

| Helper | Location | Use |
|---|---|---|
| `config_check.load_urdf()` | `config_check.py:29` | Expands the installed xacro, as M3/M4 do |
| `joint_safety.load_limits()`, `check_pose()` | `joint_safety.py:19, 62` | The M1 12-joint rule |
| `posture_config.load_yaml_strict()` | `posture_config.py:58` | Strict YAML loading |
| `kinematics_targets._num()`, `_vec3()` | `kinematics_targets.py:29, 35` | Numeric validation helpers, as M4 reuses them |

**Rule:** M4.5 adds new modules only. It does **not** edit `leg_kinematics.py`,
`m4_pose_targets.py` or any M1–M4.1 file except the `CMakeLists.txt` test and script registration.

## 3. Joint limits and robot data sources [FACT]

| Data | Source of truth | How M4.5 reads it |
|---|---|---|
| Joint limits | URDF `<limit>`, mirrored in `spiderx_legs.yaml:29-63` and checked by `validate_controller_config` | `LegGeometry.limits` (URDF) |
| Safety margin | `spiderx_legs.yaml:17`, `soft_limit_margin_rad: 0.05` | Gait YAML must restate it; the loader checks equality, as M4 does |
| Joint speed reference | `spiderx_legs.yaml:24`, `max_joint_velocity_rad_s: 0.5`, labelled **SIMULATION_PLACEHOLDER** | Used as a reported comparison threshold, explicitly labelled a placeholder |
| Foot tip | Derived: the lowest point of each foot's collision mesh (`leg_kinematics.py:16-19`) | `LegGeometry.tip0` |
| Masses and COM | URDF `<inertial>`: CAD, steel density, 7.46 kg (`docs/SPIDERX_URDF_AUDIT.md`, `docs/M3_SIMULATION_LIMITATIONS.md:23`) | Whole-robot COM computed per sample (§5) |

**Audit measurements** (scratch probes using the existing IK and URDF; nothing committed):
- **CAD-neutral tips** in `base_link`: front_left (−0.0335, 0.0423, −0.0545), front_right
  (0.1359, 0.0423, −0.0545), rear_left (−0.0335, −0.1459, −0.0545), rear_right
  (0.1359, −0.1459, −0.0545) m.
- **Reach from the neutral tip** with the 0.05 rad margin:
  - along y (forward/back) at stance height, from about **−122 mm to +69 mm**;
  - vertically, from **−43 mm to +33 mm**.

  The ranges are the same for all four legs to the probe's 1 mm resolution. Each leg was extracted
  from its own URDF chain; nothing was mirrored.
- **Whole-robot COM** at q = 0, computed from the URDF `<inertial>` data (root `dummy_link`, which
  coincides with `base_link`): **(0.0510, −0.0576, 0.0717) m**, total mass **7.458 kg**. This
  reproduces the M3 value (`docs/M3_LEG_KINEMATICS_PLAN.md:264`) exactly.

## 4. Where gait configurations live [DECISION]

- **File:** `src/spiderx_controller/config/m4_5_gaits.yaml`. It is installed with the existing
  `config` directory (`CMakeLists.txt:9-12`) and versioned like the M2–M4 configs.
- **Header:** `config_version`, `date`, `simulation_only: true`, `offline_only: true`,
  `frame: base_link`, `units`, the reference `cad_neutral` / `derived_foot_tip`, and
  `validation_margin_rad` (must equal 0.05).
- **Gait parameters:** every parameter lives in this YAML; none is hard-coded in the logic. The
  stability threshold and sampling density are declared in the same file.
- **No new robot data.** The YAML holds no servo IDs, calibration, offsets, ports, clocking or
  hardware settings. Robot geometry always comes from the URDF.

## 5. Proposed design

### 5.1 Conventions [DECISION]

- **Frame and direction.** Everything is in `base_link`: +x right, +y front, +z up. This is NOT
  REP-103. The body moves along **+y**, with constant velocity, no turning, no lateral sway and a
  level body.
- **Phase.** The normalised cycle time is u = t / T ∈ [0, 1). Each leg i has a phase offset φᵢ.
  The leg is in **swing** while (u − φᵢ) mod 1 < 1 − β and in **stance** otherwise, where β is the
  duty factor (fraction of the cycle in stance).
- **Stroke ("step length").** `step_length_m` = L is the foot's travel **relative to the body**
  during stance. The derived quantities are:
  - body speed v = L / (β·T);
  - stride length S = v·T = L / β, the body's distance per cycle.

  This definition is stated in every report, because "step length" and "stride length" are often
  confused.
- **Stance trajectory.** The foot moves along −y at constant speed v (planted in the world). It
  goes from +L/2 to −L/2 about the leg's stance centre.
- **Swing trajectory.** It is defined in the world frame as a **cycloid**:
  - horizontal displacement x(s) = S·(s − sin 2πs / 2π);
  - lift z(s) = h·(1 − cos 2πs)/2;
  - s ∈ [0, 1].

  It is then converted to the body frame by subtracting the body motion. The world-frame foot
  velocity is zero at lift-off and touch-down, so velocity is continuous at stance/swing
  transitions. Vertical acceleration is not continuous there; that is reported as a metric, not
  hidden.
- **Stance height.** `stance_height_offset_m` is a +z offset of all feet from the neutral tip, which
  is M4's convention: +z lowers the body. The geometric body height = −(stance-plane z). This is a
  labelled geometric expectation, as in M4.
- **Stance centre.** `stance_center_offset_m: [x, y]` is applied to all legs, by default (0, 0).
  The forward reach (+69 mm) is the binding limit.
- **Leg order.** `swing_order`, the "leg-order priority", is checked against the phase offsets: the
  sorted offsets must give the declared order. Phase offsets are given explicitly, never inferred.

### 5.2 Gait set [DECISION] [OPEN]

Every name below is **data** in the YAML, so renaming is free.
- **Duty-factor family.** The names follow the wave-gait family used for hexapods, where tripod,
  ripple and wave differ by how many legs swing at once. They are mapped here onto a
  **four-legged** robot.
- **Phase offsets.** They use the lateral-sequence footfall order LR → LF → RR → RF for the
  one-leg-at-a-time gaits.

| Name | β | Phase offsets (LF, RF, LR, RR) | Support pattern | Static stability required? |
|---|---|---|---|---|
| `wave` | 0.85 | 0.25, 0.75, 0.00, 0.50 | 3 or 4 feet always; long four-foot overlap | yes |
| `tripod_crawl` | 0.75 | 0.25, 0.75, 0.00, 0.50 | the "tripod-support" boundary: always ≥ 3 feet, one swinging | yes |
| `ripple` | 0.65 | 0.25, 0.75, 0.00, 0.50 | alternates 3- and 2-foot support | no (2-foot intervals are statically unstable by definition) |
| `amble` | 0.55 | 0.25, 0.75, 0.00, 0.50 | four-beat, mostly 2-foot support | no |
| `pace` | 0.50 | 0.00, 0.50, 0.00, 0.50 | lateral pairs | no |
| `trot` | 0.50 | 0.00, 0.50, 0.50, 0.00 | diagonal pairs (reference gait) | no |

- **Static-stability rule.** For gaits marked "no", the static margin is reported only as
  information: the fraction of the cycle with ≥ 3 feet and a positive margin. Their dynamic
  stability is **out of scope** and is never claimed.
- **Example parameters.** All six start with the same values, so the comparison isolates the
  gait pattern: L = 0.04 m, h = 0.015 m, T chosen so that the placeholder joint-speed reference is
  met, stance-height offset 0, and N = 200 samples per cycle. Batch A sets the final values, and
  §9 requires the tool itself to show they are feasible. If a gait fails, the report states that
  failure; it is not tuned away.

### 5.3 Modules (new files only) [DECISION]

Data flows: YAML → `gait_config` → `gait_phase` → `gait_trajectory` → `gait_kinematics` →
`gait_metrics` → `gait_report`.

| Batch | Module (`spiderx_controller/`) | Responsibility | Tests |
|---|---|---|---|
| A | `gait_config.py` + `config/m4_5_gaits.yaml` | Strict schema and loader; normalised immutable config; no geometry | `test_gait_config.py` |
| B | `gait_phase.py`, `gait_trajectory.py` | Phase engine (swing/stance, support sets, support timeline); foot targets(u) per leg; deterministic sampling | `test_gait_trajectory.py` |
| C | `gait_kinematics.py` | Sampled IK via `leg_kinematics.inverse` (branch-continuous `reference`); FK residual; joint-limit and singularity margins; joint-jump check | `test_gait_kinematics.py` |
| D | `gait_metrics.py` | Whole-robot COM from URDF inertials; support polygon; static margin; heuristic energy proxies; smoothness and continuity; per-gait verdict with the exact failed checks | `test_gait_metrics.py` |
| E | `gait_report.py`, `m4_5_gait_analysis.py`, `scripts/m4_5_gait_analysis` | CSV, JSON and PNG writers (matplotlib, Agg backend, deterministic paths); CLI runner (`--config`, `--out`, `--no-plots`, `--check-only`) | `test_gait_report.py` |
| F | `gait_report.py` (aggregate) | Cross-gait comparison table and plots; regression test on the committed configs (verdicts and pinned metrics) | `test_gait_regression.py` |

**No ROS runtime.** Modules A–D import nothing from rclpy or ROS messages. Only the loader uses
`ament_index`/xacro, exactly as M4 does, to read the installed URDF and config. The analysis core
takes plain geometry objects, so it can be unit-tested without a ROS graph.

### 5.4 Metrics and their honesty labels [DECISION]

| Metric | Definition | Label |
|---|---|---|
| IK feasibility | Every sample, every leg: `inverse(...).ok` and FK residual ≤ 1e-6 m | exact (kinematic model) |
| Joint-limit margin | min over samples of the distance to (URDF limit − 0.05 rad) | exact (model) |
| Singularity margin | min hip/knee margin of the selected IK candidate | exact (model) |
| Joint continuity | max \|Δq\| between consecutive samples; flags a branch flip | sampled |
| Static stability margin | Signed distance from the COM ground projection to the support-polygon edge, at samples with ≥ 3 stance feet. Uses the URDF COM, whose masses are CAD steel-density | **approximation**: quasi-static, flat ground, point feet at the derived tip, no dynamics |
| Joint speed | max \|dq/dt\| by finite differences, compared with the 0.5 rad/s **SIMULATION_PLACEHOLDER** | sampled; the threshold is a placeholder |
| Smoothness | Velocity jump at stance/swing transitions (should be ~0); acceleration jump reported | sampled |
| Energy proxies | Foot path length per metre travelled; Σ\|Δq\| per metre; leg lift work proxy Σ m_leg·g·Δz⁺ per metre (leg masses from the URDF) | **heuristic proxies, not energy or power** |
| Cadence | steps per metre = 4 / S | exact (definition) |

**Verdict.** Each gait passes or fails. The report lists every failed check by name, leg, sample
and value.

## 6. Artifacts [DECISION]

- **Location:** `log/m4_5_gait_analysis/` (git-ignored by `.gitignore` `log/`, as M4's
  `log/m4_all_leg_ik/`).
- **Naming is deterministic:**
  - `<gait>/samples.csv`, `<gait>/report.json`;
  - `<gait>/phase_diagram.png`, `<gait>/foot_trajectories.png`, `<gait>/stability_margin.png`,
    `<gait>/joint_angles.png`;
  - `comparison/summary.csv`, `comparison/metrics.png`.
- **Not committed.** Generated outputs are never committed. The docs record the run's command, its
  config version and the key numbers.
- **Plots** use matplotlib's non-interactive Agg backend and save to files only.
- **Dependency.** matplotlib becomes an `exec_depend` (`python3-matplotlib`). The core and CSV/JSON
  output work without it: `--no-plots` is available, and a clear message is printed if it is
  missing.

## 7. Naming and consistency issues for paper-scale comparisons [FACT]

1. **Non-REP-103 frame.** In `base_link`, +y is forward and +x is right. Any paper figure must
   state this, or convert to x-forward.
2. **`foot` joint = knee.** The `*_foot_joint` joints are knee flexion, and `*_foot_1` links are
   shanks. The "foot tip" is a derived mesh point, not a URDF frame or a contact model.
3. **Leg names.** The prefixes `lf/rf/lr/rr` mean left-front, right-front, left-rear and
   right-rear. Gait YAML uses the canonical names (`front_left`, …) and accepts aliases.
4. **Hexapod gait names on a quadruped** (§5.2). Tripod and ripple are hexapod terms, so their
   quadruped meaning is defined explicitly.
5. **Step length vs stride length** (§5.1). These are defined explicitly.
6. **Masses are CAD steel-density placeholders**, so the COM and stability margin are only as good
   as these values.
7. **Joint speed reference.** 0.5 rad/s is a placeholder, not a servo specification.
8. **Roadmap.** The current M4.5 section reads "Gait generator… walks forward 1 m in simulation
   (video)" (`docs/SPIDERX_DEVELOPMENT_ROADMAP.md:91-94`). That conflicts with the new offline-only
   scope. See [OPEN] Q2.

## 8. Open questions [OPEN]

- **Q1. Gait naming.** Is the duty-factor mapping in §5.2 acceptable? It covers tripod-support
  crawl, wave, ripple, amble, pace, and trot as a reference. It is implemented as data, so it can be
  renamed without code changes.
- **Q2. Roadmap.** Where does the old M4.5 goal ("static walk then trot; walks 1 m in simulation")
  move? The request calls the next milestone "four-channel/motion evaluation (M5)", while the
  roadmap's M5 is "`/cmd_vel` → gait bridge".

  **Proposal:** mark M4.5 "implemented (offline) — paper-evaluation pending", and move the
  Gazebo-walking items to a clearly labelled future item for the owner to place.
- **Q3. Lateral sway.** Body sway, which would widen static margins for crawl gaits, is out of
  scope for M4.5. Should it be a future parameter?
- **Q4. matplotlib as `exec_depend`.** Is that acceptable on the owner's PC? The fallback is
  `--no-plots`.

None of these blocks Batches A–D. Q1 and Q2 must be settled before the docs call the gait names or
the roadmap final.

## 9. Acceptance criteria

1. Batches A–F are each committed with their unit tests. A full `colcon build` and `colcon test`
   passes at the end of every batch, with the previous 385 tests still passing.
2. Every committed gait config is evaluated, with a pass/fail verdict and exact failed checks. A
   failing gait is reported as failing, not tuned silently.
3. **Negative tests prove the checks bite:**
   - an over-long stroke gives `unreachable` / `joint_limits`;
   - wrong phase offsets versus `swing_order` give a config error;
   - a non-crawl β flagged `requires_static_stability` gives a stability failure.
4. Artifacts are generated deterministically in the cloud, and two runs give byte-identical CSV
   and JSON.
5. **No changes** to the URDF, meshes, controller YAML, launch files, M1–M4.1 code or Gazebo
   behaviour. Verified by `git diff --name-only main...`.
6. The M4 and M3 static validators still pass.

   No Gazebo runtime run is required. The M4.5 changes cannot touch runtime behaviour, so the
   runtime regressions remain optional and are listed in the results doc.

## 10. Work order

Audit and this plan (docs only) → A → B → C → D → E → F, each with build and tests → artifacts run →
regression → docs (`M4_5_TEST_RESULTS.md`, `SPIDERX_GAIT_FRAMEWORK.md`, roadmap, `STATUS.md`,
testing guide) → draft PR.

Pauses happen only if a key project fact turns out to be missing. Never merge, and never push to
`main`.
