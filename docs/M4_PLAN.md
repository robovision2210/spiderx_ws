# M4 Plan – All-Leg Kinematics and Static Pose Hold via IK (Gazebo Fortress, simulation only)

> This plan was written and committed **before** any M4 implementation.
> Branch `claude/spiderx-m4-all-leg-ik`, created from `main` @ `ce5c27e` (PR #8, M3, merged).
> `main`'s tree is identical to the M3 head verified locally and in the cloud (`aeb5c51`).

Claims that M4 may make, and only if every acceptance criterion in §9 passes:
- "All-leg forward kinematics verified for the current URDF/TF/Gazebo model."
- "All-leg inverse kinematics verified for documented, joint-safe, simulation-only static poses."
- "Static multi-leg pose hold via IK validated in Gazebo (no walking, no gait, no hardware)."

Otherwise the result is: "M4 all-leg kinematics validation not verified."

**Not in M4:**
- walking, gait cycles, or foot trajectories over time;
- body motion planning;
- balance recovery or disturbance handling;
- odometry, SLAM or navigation;
- any hardware operation.

This page labels its statements as **[FACT]** (inspected in this repository or measured), **[PLAN]** (what will be built) or **[ASSUMPTION]** (believed, not yet proven).

---

## 1. Phase 0 audit – the four leg chains

**Source.** The audit expanded the URDF with
`xacro src/spiderx_description/urdf/spiderx.urdf.xacro` (default `sim_backend`). It then walked each
chain from `base_link` to the `foot_link` named in `spiderx_legs.yaml`, parsing the XML directly
rather than reusing M3's code.

### 1.1 Correction to the M4 request: joint names **[FACT]**

The request used `{leg}_hip_joint`. **That name does not exist.** The real names are:

| Leg | Hip | Thigh | Knee ("foot") |
|---|---|---|---|
| front-left `lf` | `lf_hip` | `lf_thigh_joint` | `lf_foot_joint` |
| front-right `rf` | `rf_hip` | `rf_thigh_joint` | `rf_foot_joint` |
| rear-left `lr` | `lr_hip` | `lr_thigh_joint` | `lr_foot_joint` |
| rear-right `rr` | `rr_hip` | `rr_thigh_joint` | `rr_foot_joint` |

M4 uses the real names from the URDF and `spiderx_legs.yaml`. The M1 controllers use the same names.

### 1.2 Chains **[FACT]**

Every origin in every chain has `rpy = 0 0 0`, so at q = 0 every link frame is parallel to `base_link`
(+x robot right, +y robot front, +z up; not REP-103). Units are metres and radians.

```
front-left  base_link ─Rigid 1 (0.005, 0.0024, 0.091)─► b_s4_1
            ─lf_hip (0.005, 0.0137, -0.01375)  axis [0 1 0]   lim [-0.610865, 0.698132]─► b_s4_c_horn_1
            ─Rigid 12 (0, -0.001, 0)─► lf_holder_suppor1_1 ─Rigid 16 (-0.02, 0.01, 0.0165)─► lf_holder_1
            ─lf_thigh_joint (-0.0137, 0.01375, -0.005)  axis [-1 0 0]  lim [-0.610865, 0.785398]─► lf_c_horn_1
            ─Rigid 25 (-0.001, 0, 0)─► lf_thigh_1
            ─lf_foot_joint (-0.012, -0.077782, -0.077782)  axis [1 0 0]  lim [-0.698132, 0.436332]─► lf_foot_1

front-right base_link ─Rigid 3 (0.097, 0.0024, 0.091)─► b_s2_1
            ─rf_hip (-0.005, 0.0137, -0.01375)  axis [0 1 0]  lim [-0.698132, 0.610865]─► b_s2_c_horn_1
            ─Rigid 11 (0, -0.001, 0)─► rf_holder_suppor1_1 ─Rigid 14 (0.02, 0.01, 0.0165)─► rf_holder_1
            ─rf_thigh_joint (0.0137, 0.01375, -0.005)  axis [1 0 0]  lim [-0.785398, 0.610865]─► rf_c_horn_1
            ─Rigid 23 (0.001, 0, 0)─► rf_thigh_1
            ─rf_foot_joint (0.012, -0.077782, -0.077782)  axis [-1 0 0]  lim [-0.436332, 0.698132]─► rf_foot_1

rear-left   base_link ─Rigid 4 (0.015, -0.0924, 0.091)─► b_s3_1
            ─lr_hip (-0.005, -0.0137, -0.01375)  axis [0 -1 0]  lim [-0.698132, 0.610865]─► b_s3_c_horn_1
            ─Rigid 10 (0, 0.001, 0)─► lr_holder_suppor1_1 ─Rigid 15 (-0.02, -0.01, 0.0165)─► lr_holder_1
            ─lr_thigh_joint (-0.0137, -0.03425, -0.005)  axis [-1 0 0]  lim [-0.610865, 0.785398]─► lr_c_horn_1
            ─Rigid 24 (-0.001, 0, 0)─► lr_thigh_1
            ─lr_foot_joint (-0.006, -0.077782, -0.077782)  axis [-1 0 0]  lim [-0.436332, 0.698132]─► lr_foot_1

rear-right  base_link ─Rigid 2 (0.097, -0.0924, 0.091)─► b_s1_1
            ─rr_hip (-0.005, -0.0137, -0.01375)  axis [0 -1 0]  lim [-0.610865, 0.698132]─► b_s1_c_horn_1
            ─Rigid 9 (0, 0.001, 0)─► rr_holder_suppor1_1 ─Rigid 13 (0.02, -0.01, 0.0165)─► rr_holder_1
            ─rr_thigh_joint (0.0137, -0.03425, -0.005)  axis [1 0 0]  lim [-0.785398, 0.610865]─► rr_c_horn_1
            ─Rigid 18 (0.001, 0, 0)─► rr_thigh_1
            ─rr_foot_joint (0.012, -0.077782, -0.077782)  axis [-1 0 0]  lim [-0.436332, 0.698132]─► rr_foot_1
```

**Joint-axis points and foot tips at q = 0** (`base_link`, m):

| Leg | Hip axis point H | Thigh axis point T | Knee axis point K | Foot tip (mesh rule, §3) |
|---|---|---|---|---|
| lf | (0.010, 0.0161, 0.07725) | (−0.0237, 0.03885, 0.08875) | (−0.0367, −0.038932, 0.010968) | (−0.033548, 0.042264, −0.054543) |
| rf | (0.092, 0.0161, 0.07725) | (0.1257, 0.03885, 0.08875) | (0.1387, −0.038932, 0.010968) | (0.135852, 0.042264, −0.054543) |
| lr | (0.010, −0.1061, 0.07725) | (−0.0237, −0.14935, 0.08875) | (−0.0307, −0.227132, 0.010968) | (−0.033548, −0.145936, −0.054543) |
| rr | (0.092, −0.1061, 0.07725) | (0.1257, −0.14935, 0.08875) | (0.1387, −0.227132, 0.010968) | (0.135852, −0.145936, −0.054543) |

Each tip is the mean of 13 unique mesh vertices. All four tips lie on one plane, z = −0.054543, and
they are mirror-symmetric about x = 0.051152, the body centre.

### 1.3 Answers to the audit questions **[FACT]**

**Knee axis +x vs −x.** Only **LF** has +x (`lf_foot_joint [1 0 0]`); RF, LR and RR have **−x**. The
relation between knee and thigh axes also differs by leg:

| Leg | Thigh axis | Knee axis | knee · thigh |
|---|---|---|---|
| lf | −x | +x | **−1** |
| rf | +x | −x | **−1** |
| lr | −x | −x | **+1** ← the odd one |
| rr | +x | −x | **−1** |

M3's IK already handles this case generally: it uses σ = sign(knee · thigh). M4 must test it on LR
explicitly.

**Hip and thigh axes.** On every leg the hip axis (±y) is **perpendicular** to the thigh axis (±x),
with hip·thigh = 0, and the thigh axis is **parallel** to the knee axis, with |thigh × knee| = 0.
The analytic-IK preconditions from M3 therefore hold on all four legs.

**Joint limits, left/right symmetry.** Each limit pair is mirrored together with its axis sign, so
the *physical* ranges match. Physical ranges below are expressed about +y (hip) or +x (thigh, knee):

| Joint | Physical range |
|---|---|
| hip, left legs (LF and LR) | [−0.611, 0.698] |
| hip, right legs (RF and RR) | [−0.698, 0.611] |
| thigh, all four | about +x: [−0.785, 0.611] (LF/LR axis −x with limits [−0.611, 0.785]; RF/RR axis +x with limits [−0.785, 0.611]) |
| knee, all four | about +x: [−0.698, 0.436] |

In words, on every leg:
- outward abduction is allowed up to 0.698 rad;
- the thigh rotates in the same physical range about +x;
- the knee can swing the foot forward and up by up to 0.436 rad.

**Front and rear legs are *not* mirror images of each other.**
- **Thigh offsets differ.** The front thigh axis point T sits 0.02275 m *ahead* of H; the rear one sits 0.04325 m *behind* H.
- **The LR knee origin differs.** It is offset −0.006 in x, where the others use ±0.012.
- **All thighs point the same way.** Every thigh points rear-down, along (∓0.013, −0.077782, −0.077782).
- **The in-plane geometry still matches.** Thigh and knee motion happens in the plane ⟂ x, and that planar geometry is identical on all four legs. That is why the same foot offset gives the same |q| on every leg (§4).

Because of these asymmetries, M4 derives **each leg from the URDF**; nothing is mirrored by assumption.

**Nothing in the audit needs a URDF change.** **[FACT]**

---

## 2. Scope and design **[PLAN]**

### 2.1 Reuse M3 without changing M3's behaviour

`leg_kinematics.py` already has everything the math needs:
- a URDF extractor;
- FK by transform chain and by product of exponentials;
- an analytic IK that takes σ from the URDF.

The only obstacle is that M3 deliberately restricts `resolve_leg()` to `front_left`. That restriction
is M3's validated scope, and M3's tests and target config (for example "rear_right is refused")
depend on it.

**Design:**
- **Opt-in in M3.** Add an *opt-in* parameter to `leg_kinematics` (for example `allowed_legs`). The default stays `('front_left',)`, so every M3 behaviour, test and config refusal stays the same.
- **New `all_leg_kinematics.py`.** This new module enables all four legs, with:
  - `AllLegGeometry`: 4 × `LegGeometry` from one expanded URDF;
  - `forward_all(q12)`: 12 angles → 4 foot poses and tips;
  - `inverse_all(targets)`: 4 foot targets → 12 angles;
  - one structured result per leg, plus an overall `ok`/`reason`.
- **Stable order.** Joint order follows `spiderx_legs.yaml` (legs `front_left, front_right, rear_left, rear_right`, each `[hip, thigh, foot]`). This is the same 12-joint order used by the M1 controller YAML.
- **Checks and refusals.** Limit and margin checks use the M1 `joint_safety.check_pose` rule on all 12 values. Failures are never clamped and always carry a reason.

### 2.2 Foot-tip reference **[PLAN, following M3]**

The rule is the one M3 uses: the unique vertices of each foot's own collision mesh
(`meshes/{leg}_foot_1.stl`) within 0.1 mm of the lowest z at q = 0, giving (mean x, mean y, min z).
The tip is then fixed in `{leg}_foot_1`. It is a derived point, not a measured contact point.

### 2.3 Static poses **[PLAN]**

The poses go in `config/m4_pose_targets.yaml` (`simulation_only: true`). Each pose gives a **per-leg
offset** from that leg's CAD-neutral tip. No absolute coordinates are typed in, so no geometry is duplicated.

| Pose | Offsets (base_link, m) | Intent |
|---|---|---|
| `neutral_stance` | all legs (0, 0, 0) | CAD neutral through IK: all 12 angles ≈ 0 |
| `crouch_10mm` | all legs (0, 0, **+0.010**) | feet move **up** toward the body, so the body lowers by about 10 mm |
| `lift_lf_15mm` | LF (0, 0, +0.015), others (0, 0, 0) | same idea as M3's single-leg lift, now inside a 12-joint command |

**Correction to the request: crouch direction.** The request described `crouch_10mm` as "all feet
moved 10 mm **downward** in base_link z". Downward feet (−z) push the body **up**, which is a
"stand tall" pose, not a crouch. M4 uses +z for `crouch_10mm`. The feasibility check below also
covers `stand_tall_10mm` (−z), which can be added if you want it.

**Scratch feasibility check** **[FACT]**. For this check only, M3's IK was run on all four legs by
patching the leg restriction in memory; no repository file was changed:

| Pose | lf [hip, thigh, knee] | rf | lr | rr |
|---|---|---|---|---|
| neutral_stance | [0, 0, 0] | [0, 0, 0] | [0, 0, 0] | [0, 0, 0] |
| crouch_10mm | [0, **+0.0555, +0.1223**] | [0, **−0.0555, −0.1223**] | [0, **+0.0555, −0.1223**] | [0, **−0.0555, −0.1223**] |
| stand_tall_10mm (optional) | [0, −0.0596, −0.1294] | [0, +0.0596, +0.1294] | [0, −0.0596, +0.1294] | [0, +0.0596, +0.1294] |
| lift_lf_15mm | [0, 0.0819, 0.1812] | [0, 0, 0] | [0, 0, 0] | [0, 0, 0] |

- **Every solution is joint-safe.** Each one is inside the limits minus 0.05 rad, with exactly 1 valid candidate out of 4 and singularity margins ≥ 1.2 rad.
- **The crouch row shows the sign table in action.** The *same* physical motion needs different signs per leg, and LR's knee sign differs from LF's even though both are left legs.

**Pose safety rules** (checked statically by the loader):
- every per-leg offset is ≤ 20 mm;
- every angle is within the URDF limits minus 0.05 rad;
- singularity margins are ≥ 0.2 rad;
- the joint change from CAD neutral is ≤ 0.35 rad.

**Ground contact.** In `base_link` there is no fixed ground height, because the body moves when all
feet move together. Instead, each pose declares which feet are **stance** feet; they must share one
z, so the body stays level. Any **lifted** foot must be ≥ 5 mm above that stance plane. This rule
replaces a per-foot "above the ground" check, which is meaningless for whole-body poses.

### 2.4 Runtime validation **[PLAN]**

**Launch.** It reuses `fortress_posture_hold.launch.py`, the existing M1 launch plus the M2 Gazebo
ground-truth bridge, exactly as M3 does. **No new launch file** and no change to any launch file.

**Files, following the M3 split:**
- `spiderx_controller/pose_validation.py`: pure, unit-tested decision and report logic;
- executable `scripts/m4_pose_validation.py`: the ROS node.

**Sequence:**
1. **Preconditions:**
   - both controllers are `active`;
   - `/joint_states` has exactly 1 publisher;
   - `/clock`, TF and Gazebo poses for all 4 `{leg}_foot_1` links are available;
   - all 12 joints are within 0.05 rad of CAD neutral.
2. **FK check** for all 4 legs against TF (at the `/joint_states` stamp) and against Gazebo.
3. **Negative poses:** they must be refused without any goal being sent.
4. **Each safe pose:**
   - solve IK for all legs and run `check_pose` on all 12 values;
   - send **one** 12-joint trajectory (the M1 trajectory client);
   - **hold for 5 s of simulation time**, sampling joints, body pose and tilt;
   - run the FK check on all 4 legs;
   - compare the observed tip with the target on all 4 legs;
   - compare body height with the geometric expectation for the pose;
   - return to `neutral_stance`.
5. **Report:** a JSON report (per pose, per leg: FK errors, IK solution, tip error, body tilt and height, pass/fail) plus a terminal summary.
6. **Ctrl-C:** cancels the goal, commands `neutral_stance`, then exits.

**Report location.** The tool writes `~/.ros/spiderx_m4/all_leg_ik_report.json` by default (same
convention as M2/M3). `validate_m4_all_leg_ik.sh --runtime` copies it to
`log/m4_all_leg_ik/latest_report.json`, which git ignores.

**Hold criteria** (simulation-only thresholds, fixed in the YAML before the first run):

| Criterion | Threshold | Category |
|---|---|---|
| FK vs TF | ≤ 1e-6 m and ≤ 1e-6 rad | M3 value |
| FK vs Gazebo | ≤ 1 mm and ≤ 0.005 rad | M3 value |
| Observed tip vs target | ≤ 2 mm | M3 value |
| \|roll\|, \|pitch\| during the hold | ≤ 0.10 rad | M2 value |
| Body height vs the geometric expectation | within ±3 mm | new, simulation-only |

The geometric expectation for body height is −(stance-foot z). That gives 0.0545 m for neutral,
0.0445 m for the crouch, and 0.0545 m for the LF lift. It is a geometric indicator, **not** a
contact measurement.

### 2.5 Files **[PLAN]**

| File | Change |
|---|---|
| `spiderx_controller/leg_kinematics.py` | Opt-in `allowed_legs` parameter; the default keeps M3 behaviour exactly |
| `spiderx_controller/all_leg_kinematics.py` | **New.** All-leg FK/IK wrappers |
| `config/m4_pose_targets.yaml`, `spiderx_controller/m4_pose_targets.py` | **New.** Poses plus a strict loader (rejects duplicate keys, malformed entries, unreachable or unsafe poses, and `simulation_only` not true) |
| `spiderx_controller/pose_validation.py`, `scripts/m4_pose_validation.py` | **New.** Runtime tool |
| `config_check.py`, `validate_controller_config` | Add `check_m4_all_leg_ik` |
| `test/test_all_leg_kinematics.py`, `test/test_m4_pose_targets.py` | **New.** Tests: independent numpy walk for 4 legs, IK round trips, sign tests (LR especially), limits, refusals, decision logic |
| `scripts/validate_m4_all_leg_ik.sh` | **New.** Static checks plus `--runtime`, with an ancestor-safe leftover check (M3 pattern) |
| `CMakeLists.txt` | Install and test entries |
| Docs | New `M4_TEST_RESULTS.md`. Updates to `STATUS.md`, `README.md`, the roadmap, the testing guide and the controller README |

**Left untouched:**
- the URDF, meshes and worlds;
- all launch files;
- the controller, leg, pose, M2 and M3 YAML;
- the M1 and M2 tools, the M3 runtime tool;
- `validate_fortress.sh`, `validate_m1_control.sh`, `validate_m2_posture.sh` and `validate_m3_kinematics.sh`.

`git diff main` will confirm this.

---

## 3. How the data flows **[PLAN]**

```
spiderx.urdf.xacro ──xacro──► expanded URDF ──► LegGeometry ×4 (axes, origins, limits, mesh tips)
                                                        │
m4_pose_targets.yaml (per-leg offsets) ──► offsets + CAD-neutral tips = 4 absolute targets
                                                        │ inverse_all()
                                                        ▼
                         12 angles ──check_pose (M1 limits − 0.05)──► FollowJointTrajectory (M1 JTC)
                                                        │
               /joint_states ─► forward_all() ─┬─ compare ─ TF base_link→{leg}_foot_1
                                               └─ compare ─ Gazebo {leg}_foot_1 pose (M2 bridge)
               Gazebo model pose ─► body height and tilt ─► hold criteria ─► JSON report
```

---

## 4. Assumptions and risks **[ASSUMPTION]**

1. **The lift pose may rock.** With the LF foot lifted, the robot stands on 3 feet, and the COM is only 3.8 mm inside the RF–LR edge (measured in M3). The body may rock. Tilt is a pass criterion (≤ 0.10 rad); M3 measured only 1.8e-5 rad.
2. **Crouch height is an estimate.** The crouch should lower the body by about 10 mm (expected height ≈ 0.0445 m). Friction and contact in simulation may shift the resting height slightly, which is why the tolerance is ±3 mm.
3. **FK vs Gazebo should match closely, as in M3.** This assumes Gazebo reports link poses relative to the model for all four feet, which was verified for LF only.
4. **The controller start-up race may reappear.** It was seen once during M3, in the cloud. The tool fails on it, and it is not fixed in M4 (it has its own suggested task).

---

## 5. Limitations

- **Same URDF everywhere:** Gazebo, TF and the M4 code share it, so M4 shows model consistency, not physical truth.
- **Idealised physics:** the joints use 100 N·m placeholders; masses use steel density (7.46 kg); friction µ = 0.2; there is no damping; DART runs a 1 ms step.
- **Derived foot tips:** mesh points, not measured contact points.
- **Narrow coverage:** 3 static poses with small offsets. No walking, gait, balance recovery or disturbance tests.
- **No hardware.**

## 6. Verification plan

1. **Unit tests:**
   - FK on all 4 legs against an independent numpy URDF walk;
   - transform-chain FK against PoE;
   - IK round trips with random joint-safe 12-vectors;
   - per-leg sign tests (LR knee +1 vs −1);
   - limits and margin;
   - unreachable and out-of-limit refusals;
   - M3 behaviour unchanged (the existing M3 tests keep passing untouched).
2. **Static checks:** `validate_controller_config` including M4, plus `validate_m4_all_leg_ik.sh`, including refusals for:
   - an unreachable pose;
   - an out-of-limit pose;
   - a pose with `simulation_only: false`;
   - a malformed or duplicate-key YAML.
3. **Runtime:** `validate_m4_all_leg_ik.sh --runtime`, as described in §2.4.
4. **Regressions:** the M3, M2, M1 and Fortress scripts, static and `--runtime`.

## 7. Rollback

M4 only adds files, plus an opt-in parameter whose default keeps M3 identical. Reverting the M4
commits restores M3 exactly.

## 8. Commands

```bash
source /opt/ros/humble/setup.bash
cd ~/spiderx_ws && rm -rf build install log && colcon build --symlink-install && source install/setup.bash
colcon test && colcon test-result --verbose
./scripts/validate_m4_all_leg_ik.sh
./scripts/validate_m4_all_leg_ik.sh --runtime
./scripts/validate_m3_kinematics.sh --runtime
./scripts/validate_m2_posture.sh --runtime
./scripts/validate_m1_control.sh --runtime
./scripts/validate_fortress.sh --runtime
```

## 9. Acceptance criteria

1. This plan is committed before any implementation.
2. All unit tests pass, and every existing M3 test passes **without modification**.
3. The static validation passes, and the refusals fail for their *expected* reasons.
4. **Runtime:**
   - FK within tolerance on all 4 legs, at CAD neutral and at every pose;
   - every safe pose is reached (tip ≤ 2 mm on all legs) and held for 5 s within the tilt and height criteria;
   - each pose ends with a return to neutral;
   - negative poses are refused with 0 goals sent;
   - `/joint_states` publishers stay at [1, 1];
   - the shutdown is clean.
5. All regressions pass.
6. The docs label everything simulation-only. The PR is a draft, nothing is merged, and M5 is not started.
