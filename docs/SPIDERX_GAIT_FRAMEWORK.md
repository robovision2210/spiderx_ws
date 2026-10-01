# SpiderX Gait Framework (M4.5) – Design, Data Flow and How to Use It

```text
OFFLINE kinematic analysis of gait configurations. Nothing here moves the robot, starts Gazebo or
talks to hardware. Not walking, not a validated gait, not balance control.
```

This guide teaches how the M4.5 framework turns a gait written in YAML into a PASS/FAIL report:
- what each step computes;
- why it was designed that way;
- how to debug it.

The results are in [M4_5_TEST_RESULTS.md](M4_5_TEST_RESULTS.md) and the plan is in
[M4_5_PLAN.md](M4_5_PLAN.md).

## 1. The big picture

```text
config/m4_5_gaits.yaml ──► gait_config ──► gait_phase ──► gait_trajectory ──► gait_kinematics ──► gait_metrics ──► gait_report
   (data only)            strict loader   swing/stance    foot target(u)      IK per sample      COM, stability,    CSV / JSON / PNG
                          + cross-checks  support sets    per leg             (M3/M4 IK,          speed, proxies,    + comparison
                                                                               unchanged)          verdict
URDF (xacro) ─────────────────────────────────────────────────► geometry (tips, axes, limits) and masses
```

There is one command-line entry point, `m4_5_gait_analysis`, and no ROS node: the core modules
import nothing from `rclpy`. Each module is a pure function of its inputs. That is why every stage
can be unit-tested on its own, and why two runs give byte-identical files.

**Teaching point.** Separating *data* (YAML) from *geometry* (URDF) from *policy* (thresholds in
`analysis:`) means:
- a reviewer can see every number that decided a verdict;
- nobody can tune one stage to hide a failure in another.

## 2. Coordinates and words you must keep straight

- **Frame.** Everything is in `base_link`: **+x right, +y front, +z up**. This is NOT REP-103's
  x-forward. The body moves along **+y**.
- **Foot tip.** It is the lowest point of each foot's collision mesh at CAD neutral, derived as
  in M3. It is not a URDF frame and not a contact model.
- **Foot joint.** `*_foot_joint` is the **knee**.
- **Normalised time.** u = t / T ∈ [0, 1).
- **Duty factor β.** The fraction of the cycle a foot is on the ground (stance).
- **Phase offset φ.** A leg **swings** while (u − φ) mod 1 < 1 − β.
- **Step length L** (the stroke). How far the foot moves **relative to the body** during stance.
- **Stride S = L / β.** How far the **body** moves in one cycle.
- **Body speed v = L / (βT).** All six shipped gaits use the same v (5 mm/s), so each gait's
  period T comes out different.

Worked example (trot, β = 0.5, L = 40 mm, v = 5 mm/s):
- T = 0.04 / (0.5 × 0.005) = **16 s**;
- S = 80 mm;
- each foot is down for 8 s, sliding back 40 mm under the body, then swings 80 mm forward over
  the ground (40 mm relative to the body) in 8 s.

### 2.1 Gait-name terminology mapping

The owner decided to keep these names for M4.5. Some are borrowed from **hexapod** usage, so their
**four-legged** meaning in this repository is defined here.
- **Footfall order.** The one-leg-at-a-time gaits use the lateral sequence LR → LF → RR → RF,
  with phase offsets LF 0.25, RF 0.75, LR 0.00, RR 0.50.
- **Where the names live.** The names are YAML data; no code depends on them.

| YAML name | β | Quadruped meaning here | Usual source of the term | Static stability required? |
|---|---|---|---|---|
| `wave` | 0.85 | One leg swings at a time with a long four-foot overlap | Hexapod "wave gait" (one leg at a time, highest duty factor) | yes |
| `tripod_crawl` | 0.75 | One leg swings at a time, so the robot always stands on a **support tripod** of three feet. This is the β = 0.75 boundary | Hexapod "tripod" means 3 legs swinging at once, which is impossible for a quadruped. Here "tripod" names the three-foot support, not the swing group | yes |
| `ripple` | 0.65 | Successive swings overlap, alternating three- and two-foot support | Hexapod "ripple gait" (overlapping swings) | no |
| `amble` | 0.55 | Four-beat lateral-sequence gait, mostly on two feet | Quadruped term (four-beat, faster than a walk) | no |
| `pace` | 0.50 | Lateral pairs (LF+LR, RF+RR) swing together | Quadruped term | no |
| `trot` | 0.50 | Diagonal pairs (LF+RR, RF+LR) swing together; the reference gait | Quadruped term | no |

"Static stability required? no" means the static margin is reported only as information. It
does **not** mean the gait is dynamically stable; that is never claimed.

## 3. Stage by stage

### 3.1 `gait_config` – the YAML is the only source of gait parameters

`load_gait_config()` reads `m4_5_gaits.yaml` strictly. Unknown keys, wrong types or out-of-range
values raise `GaitConfigError`, and **nothing is evaluated**; the CLI exits 2. It also
cross-checks values that exist elsewhere, so they cannot drift apart:

| YAML value | Must equal | Why |
|---|---|---|
| `validation_margin_rad` | `spiderx_legs.yaml` soft-limit margin | The same joint-safety rule as M1–M4 |
| `ik_position_tol_m` | `leg_kinematics.IK_POSITION_TOL_M` | The same IK acceptance as M3/M4 |
| `analysis.joint_speed_reference_rad_s` | `spiderx_legs.yaml` `max_joint_velocity_rad_s` (0.5, a placeholder) | No invented servo data |
| `swing_order` | the order of the sorted `phase_offsets` | A "leg-order priority" that can be read and checked |
| `simulation_only`, `offline_only` | `true` | The file cannot be used to imply hardware use |

**Design decision.** You give exactly one of `body_speed_m_s` or `cycle_period_s`, and the other
is derived. Giving both could contradict L = βvT.

### 3.2 `gait_phase` – who is on the ground?

- `leg_phase(spec, leg, u)` returns `SWING` or `STANCE` with the progress 0 → 1 through that
  phase.
- `support_set(spec, u)` lists the stance legs.
- `sample_us(n)` gives u_k = k / n. The samples are uniform and exclude u = 1, so the cycle wraps
  cleanly.

### 3.3 `gait_trajectory` – where is each foot?

The foot target is `tip0 + stance offsets + offset(u)`. The offset depends on the phase:
- **Stance:** the foot slides from +L/2 to −L/2 along y at speed −v in the body frame, so it is
  fixed in the world.
- **Swing:** a **cycloid in the world frame**, converted to the body frame by subtracting the body
  motion:
  - horizontal: x = S(s − sin 2πs / 2π);
  - lift: z = h(1 − cos 2πs)/2.

**Why a cycloid.** Its world velocity is zero at lift-off and touch-down, so the foot velocity is
**continuous** where stance meets swing. Test this with `check_transition_velocity`; it is below
1e-6 m/s for every shipped gait. Its vertical *acceleration* is not continuous there. The tool
reports that jump (`max_transition_acceleration_jump_m_s2`) instead of hiding it.

### 3.4 `gait_kinematics` – can the legs actually reach it?

`solve_cycle()` calls the **unchanged** M3/M4 `LegGeometry.inverse()` for every leg at every
sample:
- It passes the previous sample's solution as `reference`, so the solver stays on one IK branch.
- The URDF limits, minus the 0.05 rad margin, are enforced. Nothing is clamped: an unreachable
  target is a failure with its reason (for example `unreachable_knee`).

It reports five checks:

| Check | Pass condition |
|---|---|
| `ik_feasible` | IK solved at every sample |
| `fk_residual` | FK of the solution matches the target within 1e-6 m |
| `singularity_margin` | ≥ 0.2 rad |
| `joint_limit_margin` | margin to the limits |
| `joint_continuity` | the largest per-sample joint step is ≤ 0.05 rad, so a branch flip shows up as a jump |

**Pitfall.** `joint_continuity` is **per sample**, so it depends on `samples_per_cycle`. At 40
samples, trot's step is 0.0504 rad and fails; at the configured 200 it is 0.0101 rad and passes.
Compare runs only at the same resolution.

### 3.5 `gait_metrics` – stability, speed, proxies and the verdict

**Mass model.** `MassModel` rebuilds the whole-robot centre of mass from the URDF link inertials
for any set of joint angles. At q = 0 it reproduces the documented COM (0.051, −0.0576, 0.0717) m.

**Static margin.** It is the signed distance from the COM's ground projection to the nearest edge
of the support polygon (the convex hull of the stance feet):
- positive means inside;
- it exists only with ≥ 3 stance feet.

It is labelled an **approximation**: quasi-static, CAD placeholder masses, point feet, flat ground,
no dynamics.

**When stability is required.** `requires_static_stability: true` (wave, tripod_crawl) makes it a
required check. Otherwise it is reported as information only.

**Joint speed.** This is the finite-difference |dq/dt|, compared with the 0.5 rad/s
**placeholder**.

**Energy proxies.** These are per metre travelled: steps, foot path, joint travel, and a lift-work
proxy Σ m_leg·g·Δz⁺. They are labelled **heuristic proxies, not energy**.

**Verdict.** A gait PASSES only if every *applicable* check passes. `failed_checks` names each
failing one. Each failure lists the leg, sample, u and value, capped at 20 lines with a total count.

### 3.6 `gait_report` and the CLI – files a reviewer can open

```bash
ros2 run spiderx_controller m4_5_gait_analysis                       # all gaits -> log/m4_5_gait_analysis/
ros2 run spiderx_controller m4_5_gait_analysis --gait trot --no-plots --out /tmp/g
ros2 run spiderx_controller m4_5_gait_analysis --check-only          # validate YAML only
ros2 run spiderx_controller m4_5_gait_analysis --strict              # exit 1 if any gait fails
```

| File | What to look at |
|---|---|
| `<gait>/report.json` | `verdict`, `failed_checks`, every check with its honesty label, metrics, `metric_labels` |
| `<gait>/samples.csv` | One row per sample: support, COM, margin, and per leg the phase, target, q and IK reason |
| `<gait>/phase_diagram.png` | Footfall bars (stance) per leg |
| `<gait>/foot_trajectories.png` | Body-frame y(u) and z(u) |
| `<gait>/joint_angles.png` | Hip, thigh and knee angles per leg |
| `<gait>/stability_margin.png` | Margin over the cycle; grey bands mark < 3 feet |
| `comparison/summary.csv`, `report.json`, `metrics.png` | All gaits side by side; FAIL is marked |

**Format.** Numbers have 10 significant digits, JSON keys are sorted and nothing carries a
timestamp, so files are byte-identical between runs. Plots use the Agg backend and are written to
files only. Without matplotlib, the tables are still written.

**Exit codes:**
- 0: the analysis ran. A FAIL verdict is a *result*.
- 1: `--strict` was given and a gait failed.
- 2: the configuration was invalid and nothing was evaluated.

## 4. Adding or changing a gait (exercise)

1. Copy the YAML to a scratch file and add a gait. For example, a slower trot:
   ```yaml
     slow_trot:
       description: trot at half speed (exercise)
       duty_factor: 0.5
       phase_offsets: {front_left: 0.0, front_right: 0.5, rear_left: 0.5, rear_right: 0.0}
       swing_order: [[front_left, rear_right], [front_right, rear_left]]
       body_speed_m_s: 0.0025
       requires_static_stability: false
   ```
2. Run `ros2 run spiderx_controller m4_5_gait_analysis --config /path/to/copy.yaml --gait slow_trot --out /tmp/ex`.
3. Predict before you look: T doubles to 32 s and the joint speed roughly halves. Check
   `max_joint_speed_rad_s` in `report.json`.

**More exercises.**
- **Find the reach limit.** Set `step_length_m: 0.30` and read which check fails and why. Hint:
  `ik_feasible`, with reason `unreachable_*`.
- **Watch the config guard.** Swap two entries in `swing_order` without changing `phase_offsets`.
  The loader refuses the file.
- **Move the feet under the COM.** Copy `tripod_crawl` and add `stance_center_offset_m`. Predict
  first: the COM is ≈ 5.8 mm *behind* the foot centre, so which way should the feet move? Then
  check your prediction against these cloud results:

  | Offset (mm) | Min margin (mm) | Stable samples | Verdict |
  |---|---|---|---|
  | +5 | −7.24 | 61.5 % | still fails |
  | −5 | −0.89 | 94.5 % | still fails |
  | −12 | −3.63 | 81.0 % | still fails; it overshoots and another triangle becomes the limit |

  **Lesson:** a body shift helps one support triangle and hurts the opposite one. That is why body
  sway (future work only, per the owner's decision), not a fixed offset, is the usual remedy. These are *reported* model
  numbers, not a claim about the robot.

**Shipped gaits.** If you change the shipped `m4_5_gaits.yaml`, `test_gait_regression.py` fails on
purpose. Update the pins **and** explain the change in `M4_5_TEST_RESULTS.md`.

## 5. Debugging guide

| Symptom | Likely cause | Where to look |
|---|---|---|
| CLI prints `REFUSED`, exit 2 | YAML schema or cross-check error | The message names the key; see §3.1 |
| `ik_feasible` fails | Target outside the reach (y −122…+69 mm, z −43…+33 mm from the neutral tip) or the joint limits | `samples.csv` columns `XX_ik_reason`, `XX_target_*` |
| `joint_continuity` fails | IK branch flip, or too few samples | Raise `samples_per_cycle` first; then plot `joint_angles.png` |
| `static_stability` fails | COM outside the support triangle | `stability_margin.png`; the `support_legs` column; the failure text names the triangle |
| `joint_speed` fails | Swing too short (high β) or too fast (high v) | The failure names leg, joint and sample; lower v or β |
| No PNGs | matplotlib missing, or `--no-plots` | The CLI prints a NOTE; install `python3-matplotlib` |
| New module "not found" after editing | Package not rebuilt | `colcon build --packages-select spiderx_controller` |

## 6. What this framework does not do (and what would be needed)

| Not done | Needed first |
|---|---|
| Walking in Gazebo | A trajectory streamer for `leg_trajectory_controller`, contact sensing, a fall criterion and a video, as the unscheduled "Future — gait playback" item |
| Dynamic stability (ZMP or CoP), balance | Dynamics, real masses, IMU feedback |
| Energy or power | Servo models and measured current. The proxies only rank geometry |
| Turning, sideways motion, `/cmd_vel` | M5.5 — Command-velocity bridge (future work; not implemented) |
| Hardware | Calibration, servo IDs and limits that do not exist in this repository. They must not be invented |
