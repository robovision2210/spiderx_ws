# M3 Plan – Single-Leg Forward and Inverse Kinematics (Gazebo Fortress, simulation only)

> Written and committed **before** any M3 implementation or configuration change.
> Branch `claude/spiderx-m3-leg-kinematics`, created from `main` @ `d9426c9`. That commit
> contains the merged PR #4, #5, #6 and #7, and its tree is identical to the verified M2 head
> `3b1763d`. The working tree was clean.

Permitted outcomes (no others):
- "Single-leg FK verified against the current URDF/TF/Gazebo model."
- "Single-leg IK verified for documented reachable, joint-safe simulation targets."
- "M3 kinematics validation not verified."

M3 is **not**:
- walking or a gait;
- balance or standing control;
- locomotion;
- real-world leg control;
- hardware validation.

## 1. Verified baseline

| Milestone | Verified (local + cloud) |
|---|---|
| M0 | Passive Fortress simulation: `/clock`, `/scan` on `lidar_link`, `/joint_states` (12 joints) and TF |
| M1 | `gz_ros2_control`: `joint_state_broadcaster` + `leg_trajectory_controller` (JTC, 12 joints, position command). Safe one-joint and neutral-pose tools with limit/margin refusal |
| M2 | Simulation-only CAD neutral posture hold. Gazebo ground-truth pose bridge `/world/spiderx_fortress/pose/info` → `/spiderx/sim/world_poses` (`tf2_msgs/TFMessage`, **not** `/tf`), started by `spiderx_bringup/launch/fortress_posture_hold.launch.py` |

Observed during this audit:
- On one cloud start-up, `joint_state_broadcaster` failed to activate (`Switch controller timed out after 5 s`), yet `controller.launch.py` still started the trajectory controller. A relaunch activated both.
- This is an intermittent M1 launch race on a slow machine. It is **out of M3 scope** and was queued as a separate task (raise `--switch-timeout`; gate the second spawner on success). M3 does not change the M1 launch files.
- The M3 runtime tool **fails** if either controller is not `active`.

## 2. Selected leg: front-left (`lf`, `legs.front_left`)

Evidence from the expanded URDF (`src/spiderx_description/urdf/spiderx.urdf.xacro`), cross-checked at runtime:
- **Complete chain.** The chain is complete and unbranched from `base_link` to `lf_foot_1`: 3 revolute joints and 4 fixed joints. It matches `spiderx_legs.yaml` `front_left`, which `validate_controller_config` already checks against the URDF.
- **No fixed rotations.** Every origin in the chain has `rpy="0 0 0"`. At q = 0, every link frame is parallel to `base_link`.
- **Structure.** The hip axis (+y) is perpendicular to the thigh axis (−x), and the thigh axis is parallel to the foot axis (+x). This is the classic abduction + two-parallel-pitch-axes structure, so an **analytic** IK exists (§11).
- **Sign convention.** LF is the one leg whose knee axis is **+x**; RF, LR and RR are −x. Its limits are mirrored to match. Choosing LF therefore tests sign handling explicitly, instead of hiding it (roadmap M3 item "per-joint sign table … `lf_foot_joint` +x").
- **CAD neutral.** CAD neutral (0, 0, 0) is well inside every LF limit.
- **Runtime evidence.** In the running controlled simulation:
  - at q = 0, TF `base_link→lf_foot_1` and the Gazebo link pose of `lf_foot_1` both equal the URDF chain sum, (−0.0367, −0.038932, 0.010968);
  - with `lf_thigh_joint = +0.2`, both report quaternion x = −0.099833, i.e. a rotation of −0.2 rad about +x, as axis [−1 0 0] predicts, and the knee moves to (−0.0367, −0.052834, 0.027971), matching a hand calculation.

## 3–4. Exact chain, joints, axes, origins, limits (URDF source lines)

| # | URDF element (line) | Type | Parent → child | Origin xyz (m), rpy | Axis | Limits (rad) |
|---|---|---|---|---|---|---|
| 0 | `dummy_joint` (L45) | fixed | `dummy_link` → `base_link` | none (identity) | – | – |
| 1 | `Rigid 1` (L696) | fixed | `base_link` → `b_s4_1` | (0.005, 0.0024, 0.091), 0 | – | – |
| 2 | **`lf_hip`** (L744) | revolute | `b_s4_1` → `b_s4_c_horn_1` | (0.005, 0.0137, −0.01375), 0 | **[0 1 0]** | [−0.610865, 0.698132] |
| 3 | `Rigid 12` (L770) | fixed | `b_s4_c_horn_1` → `lf_holder_suppor1_1` | (0, −0.001, 0), 0 | – | – |
| 4 | `Rigid 16` (L794) | fixed | `lf_holder_suppor1_1` → `lf_holder_1` | (−0.02, 0.01, 0.0165), 0 | – | – |
| 5 | **`lf_thigh_joint`** (L830) | revolute | `lf_holder_1` → `lf_c_horn_1` | (−0.0137, 0.01375, −0.005), 0 | **[−1 0 0]** | [−0.610865, 0.785398] |
| 6 | `Rigid 25` (L850) | fixed | `lf_c_horn_1` → `lf_thigh_1` | (−0.001, 0, 0), 0 | – | – |
| 7 | **`lf_foot_joint`** (L880) | revolute | `lf_thigh_1` → `lf_foot_1` | (−0.012, −0.077782, −0.077782), 0 | **[1 0 0]** | [−0.698132, 0.436332] |

- **Effort and velocity limits:** 100 N·m and 100 rad/s on every joint. These are exporter placeholders.
- **Joint order** used everywhere: `[lf_hip, lf_thigh_joint, lf_foot_joint]`, matching `spiderx_legs.yaml` `joint_order: [hip, thigh, foot]`.
- **Allowed range** = URDF limits pulled in by the existing **0.05 rad** margin:

| Joint | Allowed range (rad) |
|---|---|
| `lf_hip` | [−0.560865, 0.648132] |
| `lf_thigh_joint` | [−0.560865, 0.735398] |
| `lf_foot_joint` | [−0.648132, 0.386332] |

**Sign conventions** (right-hand rule; `base_link`: +x robot right, +y robot front, +z up):

| Joint | Positive rotation | Effect on the LF foot tip |
|---|---|---|
| `lf_hip` | about +y | tip swings to −x: **outward** (abduction) for the left leg |
| `lf_thigh_joint` | about −x | tip swings **rearward** (−y) and slightly up |
| `lf_foot_joint` | about +x | tip swings **forward** (+y) and **up** |

Unit tests will check each sign against the FK.

## 5. Frame diagram (every transform in the chain)

```
dummy_link ──dummy_joint (identity)──► base_link                     [body frame for all M3 results]
base_link ──Rigid 1  T(0.005, 0.0024, 0.091)──► b_s4_1               (hip servo body)
b_s4_1 ──lf_hip  T(0.005, 0.0137, -0.01375)·R(+y, q_hip)──► b_s4_c_horn_1
b_s4_c_horn_1 ──Rigid 12  T(0, -0.001, 0)──► lf_holder_suppor1_1
lf_holder_suppor1_1 ──Rigid 16  T(-0.02, 0.01, 0.0165)──► lf_holder_1
lf_holder_1 ──lf_thigh_joint  T(-0.0137, 0.01375, -0.005)·R(-x, q_thigh)──► lf_c_horn_1
lf_c_horn_1 ──Rigid 25  T(-0.001, 0, 0)──► lf_thigh_1
lf_thigh_1 ──lf_foot_joint  T(-0.012, -0.077782, -0.077782)·R(+x, q_foot)──► lf_foot_1
lf_foot_1 ──(DERIVED, not a URDF/TF frame) constant offset t_tip──► foot tip point
```

Joint-axis points at q = 0 in `base_link` (sums of the origins above):

| Point | Position (m) | Where it is |
|---|---|---|
| H | (0.010, 0.0161, 0.07725) | on the hip axis; origin of `b_s4_c_horn_1` |
| T | (−0.0237, 0.03885, 0.08875) | on the thigh axis; origin of `lf_c_horn_1` |
| K | (−0.0367, −0.038932, 0.010968) | on the knee axis; origin of `lf_foot_1` |

Derived link vectors:

| Vector | Value (m) | Notes |
|---|---|---|
| H→T | (−0.0337, 0.02275, 0.0115) | |
| T→K | (−0.013, −0.077782, −0.077782) | length 0.1100 in the y–z plane, pointing 45° rear-down |
| K→tip | ≈ (0.0032, 0.0812, −0.0655) | from the mesh (§7); computed in code, not hard-coded |

## 6. Leg base and end-effector frames

- **Reference frame for FK/IK inputs and outputs:** `base_link`, the SpiderX body frame. It is not REP-103, and no new TF frame is published.
- **FK outputs:**
  - the full pose (position + rotation) of the URDF link frame `lf_foot_1`, whose origin is on the knee axis;
  - the **foot-tip point**.
- **IK input:** a foot-tip **position** in `base_link`. Three joints cannot also set foot orientation, so IK is position-only. The FK orientation is reported but not used as an IK target.

## 7. Foot tip: derived, not a URDF frame

- The URDF has **no** foot-tip frame. The `lf_foot_1` frame origin is the knee axis point K, so the knee angle does not move it.
- The M3 foot tip is a **derived, non-physical reference point**, fixed in `lf_foot_1`. It is computed from the leg's own URDF collision mesh (`meshes/lf_foot_1.stl`, scale 0.001, collision origin (0.0367, 0.038932, −0.010968), link block L654–673):
  1. transform every mesh vertex into `base_link` at q = 0 (CAD neutral);
  2. take the vertices within **0.1 mm** of the lowest z;
  3. the tip is (mean x, mean y of those vertices, lowest z), expressed in the `lf_foot_1` frame.
- This gives ≈ (−0.03355, 0.04226, −0.05454) in `base_link` at q = 0: the audit's CAD foot contact point (feet bottom out at z = −0.0545).
- It is a body-fixed point. The real contact point on the rounded 5 mm pad moves slightly as the foot rotates.
- Nothing is added to the URDF or TF.

## 8. Geometry source of truth

- `spiderx_controller/leg_kinematics.py` **extracts** every joint origin, rpy, axis and limit, and the foot mesh, from the **expanded URDF**. The URDF is expanded with xacro, exactly as `validate_controller_config` already does.
- No link length or offset is typed into code or YAML.
- The chain's joint names come from `spiderx_legs.yaml`, which is already validated against the URDF.
- Tests re-derive the chain independently, with a generic URDF transform walk (4×4 matrices), and fail if the extracted geometry, or the FK built from it, disagrees.

## 9. Mathematical conventions

- **Units:** metres and radians. Right-handed frames.
- **Positive rotation:** counter-clockwise about the joint's URDF axis vector (right-hand rule). The axis sign is taken from the URDF; it is never assumed.
- **Transform order (URDF semantics):** `T_parent_child(q) = Trans(xyz) · Rot_rpy(rpy) · Rot(axis, q)`, where `Rot_rpy = Rz(yaw)·Ry(pitch)·Rx(roll)`.
  - Chain: `T_base_foot = T_R1 · T_hip(q1) · T_R12 · T_R16 · T_thigh(q2) · T_R25 · T_foot(q3)`.
  - Point: `p_base = T_base_foot · [t_tip; 1]`.
- **Fixed transforms** have no q. **Actuated transforms** rotate about their own child-frame axis.

## 10. FK design

Two implementations, which must agree:
1. **Transform chain** (the primary FK): the 4×4 homogeneous product above. It returns the `lf_foot_1` pose and the tip point.
2. **Product of exponentials** (used inside the IK), with zero-configuration axes a_i and axis points (H, T, K) in `base_link`:
   `p(q) = H + R(a1,q1)·[T − H + R(a2,q2)·[K − T + R(a3,q3)·(tip0 − K)]]`

**Input validation:**
- exactly 3 values, ordered [hip, thigh, foot];
- finite numbers of type `int` or `float`, not `bool` or strings;
- only the known leg identifier `front_left` (or `lf`).

FK never commands anything and does not import ROS.

## 11. IK design: analytic, justified by the geometry

**Preconditions,** checked at load time from the extracted geometry. If any fails, IK raises `KinematicsError`; there is no silent numeric fallback in M3.
- `|a1 · a2| < 1e-9` (hip axis ⟂ thigh axis at q = 0);
- `|a2 × a3| < 1e-9` (thigh axis ∥ knee axis);
- non-degenerate link lengths.

**Step 1 – hip (q1).** Rotations about a2 ∥ a3 preserve the a2-component of the tip. Let w = a2, u = a1, v = u × w and r = p − H. Then:

- r·w·cos q1 + r·v·sin q1 = d, where d = (tip0 − H)·w is a constant.
- Solutions: q1 = atan2(r·v, r·w) ± acos(d/ρ), with ρ = √((r·w)² + (r·v)²).
- It is unreachable if ρ < |d|. That means the target is too close to the hip axis, and the solver returns failure.

**Step 2 – thigh and knee (q2, q3).** Undo the hip rotation (e = R(a1,−q1)·r − (T − H)), then project onto the plane ⟂ a2. This is a planar two-link problem:

- The link vectors are L1 = K − T and L2 = tip0 − K, both projected onto that plane.
- The knee rotates by σ·q3, where σ = a3·a2 = −1 for LF.
- The standard law of cosines gives the elbow-up and elbow-down solutions.
- It is unreachable if |cos| > 1.

**Candidates and selection.**
- There are up to **4 candidates**: 2 for the hip × 2 for the knee.
- Each is wrapped to (−π, π] and checked against the URDF limits minus the 0.05 rad margin.
- Each is accepted only if its FK reproduces the target within **1e-6 m**.
- **Selection rule:** of the accepted candidates, pick the one with the smallest Euclidean joint-space distance to a reference. The reference defaults to CAD neutral (0, 0, 0); the caller may pass the current joint state. Ties go to the lowest candidate index (deterministic).

**Failure behaviour.** The result is a structured `IKResult` carrying `ok`, `reason` and the candidates with their rejection reasons. The reason is one of:
- `unreachable_hip`;
- `unreachable_knee`;
- `singular`;
- `joint_limits` (geometrically reachable, but every candidate is outside the limits minus the margin);
- `invalid_input`.

**Never clamped.** No solution is ever clamped into range and then reported as valid.

**Singularity margins** are reported per candidate:
- hip: acos(|d|/ρ), which is 0 where the two hip solutions merge;
- knee: the distance of the planar knee angle from full extension or fold, 0 or π.

The safe-target config requires both to be ≥ 0.2 rad.

**Numerical tolerance:** acos arguments are clipped only within ±1e-12 of ±1. Anything beyond that is unreachable.

## 12. Reachability definition

A target is **reachable and joint-safe** when all of these hold:
1. at least one analytic candidate exists;
2. it is inside the URDF limits minus 0.05 rad;
3. its FK residual is ≤ 1e-6 m.

A target is **reachable but unsafe** when candidates exist but none satisfies the limits, in which case the reason is `joint_limits`. **Unreachable** means no real candidate exists.

## 13. Joint limits and margin

- The margin comes from the M1 `joint_safety.check_target` rule: URDF limits ∓ 0.05 rad (`spiderx_legs.yaml` `soft_limit_margin_rad`).
- The runtime tool re-checks all 12 commanded values with the M1 `check_pose` before sending anything.

## 14. Validation methodology

**Unit tests** (deterministic, no ROS graph):
- FK equals an independent generic URDF walk for random in-limit configurations.
- FK at CAD neutral equals the mesh-derived tip.
- Sign tests (§4).
- Transform-chain FK equals PoE FK.
- IK→FK round trip for CAD neutral and many in-limit configurations.
- Every IK candidate is checked against the limits.
- Rejection tests:
  - an unreachable target;
  - an out-of-limit-only target;
  - NaN or malformed targets;
  - a wrong leg or frame.
- Geometry-mutation tests: an altered origin or axis must be detected.

**Runtime** (`validate_leg_kinematics`, with `fortress_posture_hold.launch.py`, which is M1 plus the M2 ground-truth bridge; no new launch file is needed):
1. **Preconditions:**
   - both controllers are `active`;
   - exactly 1 `/joint_states` publisher;
   - `/clock` is received;
   - `/spiderx/sim/world_poses` contains `lf_foot_1`;
   - TF `base_link→lf_foot_1` is available.
2. **FK check at the current configuration.** Read the LF angles from `/joint_states` and compute the FK. Compare it with two independent observations:
   - **TF** (robot_state_publisher/KDL from the same URDF): position and orientation;
   - **Gazebo ground truth**: the `lf_foot_1` link pose relative to the model. The model frame is `dummy_link` = `base_link`; the runtime check found link poses in `pose/info` to be model-relative.

   Report per-axis and Euclidean errors for the `lf_foot_1` origin, the orientation error angle, and the tip.

   The observed tip is the observed link pose composed with the derived `t_tip`, so only the link pose is truly independent. The link-origin comparison involves no derived quantity.
3. **Only if step 2 passes, the IK targets.** For each safe target:
   - solve IK;
   - validate the solution;
   - command one trajectory in which the **3 LF joints** take the IK angles and the **other 9 are held at their measured positions** (CAD neutral at start);
   - settle, then compare the observed tip (Gazebo) with the target, and FK(observed q) with the observed link pose;
   - return to the start configuration.

   Return also happens on any failure or on Ctrl-C.
4. **Negative targets:** IK must reject them. The tool verifies that no goal was sent (a goal counter stays unchanged) and that the joints did not move.
5. **Report:** a JSON report plus a terminal summary. Exit 0 = all verified, 1 = not verified, 2 = refused/invalid config.

## 15. Safe target strategy

`spiderx_controller/config/m3_kinematics_targets.yaml` is `simulation_only: true`, with leg `front_left` and frame `base_link`.
- **Offsets, not coordinates.** Targets are **offsets** from the reference tip = FK(CAD neutral), which is computed and never hard-coded.
- **All targets lift.** Every safe target raises the foot by **≥ 5 mm** (no ground penetration), with small forward, rear, outward or inward shifts of ≤ 10 mm.
- **Static checks.** For every safe target, the static check requires:
  - an IK solution inside the limits minus the margin;
  - singularity margins ≥ 0.2 rad;
  - a joint change from CAD neutral of ≤ 0.35 rad.
- **Stance risk.** Lifting LF leaves a 3-foot stance. The whole-robot COM is (0.051, −0.0576, 0.0717) m, only **3.8 mm** inside the RF–LR support edge, so the body may rock or tilt.
  - Body tilt is **recorded** (informational).
  - Kinematics are compared in `base_link`, so body motion does not affect the check.
- **Negative-test-only entries,** clearly marked:
  - a far-away unreachable target;
  - a target generated as FK of an out-of-limit joint configuration, which IK must reject with `joint_limits`.
- **Timing.** Commands last ≥ 3 s, and ≥ 2·max|Δq|/0.5 rad/s, which respects `max_joint_velocity_rad_s` for a cubic profile. Timing is in simulation time.

## 16. Simulation-only limitations

- The joints are stiff and idealised, with 100 N·m placeholder efforts, so tracking is near-perfect.
- Contact, friction (µ = 0.2), damping (none), masses (steel density) and the physics step (DART, 1 ms) are assumptions.
- The foot tip is a derived mesh point, not a measured contact point.
- Gazebo and TF use the **same URDF**. Agreement therefore validates the implementation of the math, not the physical dimensions of the real robot.
- There is no real hardware, servo backlash, frame flex or sensor data.
- One leg only.

## 17. Rollback

M3 changes nothing in the existing packages' behaviour. It only adds:
- new files (the module, config, tool, script and docs);
- install and test entries in `spiderx_controller`;
- a new check function called by `validate_controller_config`.

Reverting the M3 commits restores M2 exactly. The URDF, meshes, launch files, controller YAML, posture configs and M1/M2 tools stay untouched; `git diff` against `main` will be checked for these paths.

## 18. Acceptance criteria and commands

The M3 outcomes may be claimed only if all of these hold:
1. This plan is committed first.
2. The static checks pass:
   - `validate_controller_config`, including the M3 geometry and target checks;
   - `validate_m3_kinematics.sh`;
   - all unit tests (the existing ones plus the new FK/IK/geometry tests, including negative tests).
3. **FK verified.** At CAD neutral **and** at every IK target configuration, FK vs TF agrees to ≤ 1e-6 m and ≤ 1e-6 rad, and FK vs Gazebo to ≤ 1 mm and ≤ 0.005 rad (simulation-only tolerances, fixed in the target YAML before the first run).
4. **IK verified.** Every safe target:
   - is reached with an observed tip error ≤ 2 mm;
   - has an FK(q_cmd) residual ≤ 1e-6 m;
   - has all angles inside the limits minus the margin;
   - ends with a return to the start configuration.

   Every negative target must be rejected without any command being sent.
5. `validate_m3_kinematics.sh --runtime` passes with a complete JSON report, 1 `/joint_states` publisher and a clean shutdown.
6. The regression runs pass: `validate_fortress.sh --runtime`, `validate_m1_control.sh [--runtime]` and `validate_m2_posture.sh [--runtime]`.
7. The docs state "simulation only" and list the limitations. The PR is a draft, nothing is merged, and M4 is not started.

```bash
source /opt/ros/humble/setup.bash
cd ~/spiderx_ws && rm -rf build install log && colcon build --symlink-install && source install/setup.bash
colcon test && colcon test-result --verbose
ros2 run spiderx_controller validate_controller_config
./scripts/validate_m3_kinematics.sh
./scripts/validate_m3_kinematics.sh --runtime
# manual
ros2 launch spiderx_bringup fortress_posture_hold.launch.py          # terminal 1
ros2 run spiderx_controller validate_leg_kinematics                  # terminal 2
# regressions
./scripts/validate_fortress.sh --runtime
./scripts/validate_m1_control.sh --runtime
./scripts/validate_m2_posture.sh --runtime
```
