# M3 Guide – Inverse Kinematics (IK) of the SpiderX Front-Left Leg

> Single-leg simulation kinematics validation (Gazebo Fortress). This is not walking, a gait,
> balance control, locomotion, real-world leg control or hardware validation.

Read with: [frame conventions](M3_FRAME_CONVENTIONS.md), [FK guide](M3_FORWARD_KINEMATICS_GUIDE.md),
[results](M3_TEST_RESULTS.md), [limitations](M3_SIMULATION_LIMITATIONS.md).

## What IK is

**Inverse kinematics** asks the reverse question of FK: *given where the foot should be, which
joint angles put it there?*

- **Input:** a foot-tip position `[x, y, z]` in `base_link` (metres).
- **Output:** the angles `[lf_hip, lf_thigh_joint, lf_foot_joint]` (radians), or a structured
  failure.
- **Position-only.** Three joints can place the foot tip, but they cannot also choose the foot's
  orientation.

## Why an analytic solution fits this leg

The URDF itself shows three properties, and `leg_kinematics.py` checks them at load time:
- **Hip axis ⟂ thigh axis.** `lf_hip` is about +y and `lf_thigh_joint` is about −x.
- **Thigh axis ∥ knee axis.** `lf_foot_joint` is about +x, parallel to the thigh axis with the opposite sign.
- **No fixed rotations.** Every fixed origin in the chain has rpy = 0.

This is the classic "abduction + two parallel pitch joints" leg, so the IK can be solved in
closed form, with no iteration. If a future URDF breaks these properties, `inverse()` returns
`analytic_preconditions` and refuses to answer. It does not silently fall back to another method.

### The two steps

1. **Hip.** Rotating the thigh or knee never changes the tip's component along the thigh axis w.
   The hip angle must therefore bring that component to its fixed value d = (tip₀ − H)·w = 0.043548 m:
   `r·w cos q₁ + r·v sin q₁ = d` (with `r = p − H`, `v = u × w`).
   - This gives **two** hip solutions: `q₁ = atan2(r·v, r·w) ± acos(d/ρ)`.
   - If ρ < |d|, the target is too close to the hip axis and the result is `unreachable_hip`.
   - If ρ = 0, the target lies on the hip axis and the result is `singular`.
2. **Thigh and knee.** Undo the hip rotation. What remains is a planar two-link problem, with
   link lengths l₁ = 0.110000 m (thigh axis to knee axis) and l₂ = 0.104330 m (knee axis to tip),
   both measured in the plane perpendicular to w.
   - The law of cosines gives **two** knee solutions: bent one way or the other.
   - If the distance is outside [0.0057, 0.2143] m, the result is `unreachable_knee`.

## Why several solutions exist, and which one is chosen

- **Up to 4 candidates.** Two hip branches times two knee branches gives up to **4 candidates** for one foot position.
- **Each candidate is checked.** Each one is wrapped to (−π, π], passed through FK, and kept only if it reproduces the target within
  **1e-6 m** *and* lies inside the URDF limits minus the existing **0.05 rad** margin.
- **Selection rule.** Of the valid candidates, the one with the smallest joint-space distance to a *reference* is returned.
  - The reference is CAD neutral (0, 0, 0) by default. The runtime tool passes the current joint state.
  - Ties go to the lowest candidate index, so the choice is deterministic.
- **In practice.** For every SpiderX safe target, exactly one of the 4 candidates is valid; the other three need joint angles far outside the limits.

## Unreachable targets and joint-limit violations

| `reason` | Meaning | What happens |
|---|---|---|
| `ok` | A valid solution was found | `solution` = 3 angles |
| `unreachable_hip` / `unreachable_knee` / `singular` | No real solution exists | `solution = None` |
| `joint_limits` | Solutions exist, but every one violates the URDF limits minus 0.05 rad | `solution = None`; the rejected candidates are listed with their violations |
| `invalid_input` | Bad target, frame, reference or margin | `solution = None` |
| `analytic_preconditions` | The URDF geometry no longer allows closed-form IK | `solution = None` |

A solution is **never clamped** into range and then reported as valid. The runtime tool also
re-checks all 12 commanded values with the M1 `check_pose` rule before it sends anything.

## Targets used for validation

All targets live in `spiderx_controller/config/m3_kinematics_targets.yaml` (`simulation_only: true`).
They are written as **offsets** from the reference tip, FK(CAD neutral); the reference is computed, never typed. Every safe target lifts
the foot by at least 5 mm, so it never pushes into the ground.

| Target | Offset (m) | IK solution [hip, thigh, foot] (rad) |
|---|---|---|
| `lift_10mm` | (0, 0, +0.010) | [0.0, 0.0555, 0.1223] |
| `lift_15mm_forward_10mm` | (0, +0.010, +0.015) | [0.0, 0.0007, 0.1736] |
| `lift_15mm_rearward_10mm` | (0, −0.010, +0.015) | [0.0, 0.1591, 0.1798] |
| `lift_15mm_outward_8mm` | (−0.008, 0, +0.015) | [0.0675, 0.0650, 0.1435] |
| `lift_15mm_inward_8mm` | (+0.008, 0, +0.015) | [−0.0693, 0.0960, 0.2130] |

The static checks require every safe target to meet three bounds:
- singularity margins ≥ 0.2 rad (the measured margins are ≥ 1.2 rad);
- a joint change from CAD neutral of ≤ 0.35 rad (the largest is 0.213 rad);
- offsets of ≤ 20 mm.

Negative-test-only targets must be **rejected without any command being sent**:
- `unreachable_300mm_below`: a point 0.30 m below the reference tip. Result: `unreachable_knee`.
- `outside_joint_limits`: FK of `[0, 1.2, 0]`, where `lf_thigh_joint = 1.2` exceeds its 0.785 limit. Result: `joint_limits`, even though the point is geometrically reachable.

## Runtime test procedure (`validate_leg_kinematics`)

1. **Preconditions.**
   - Both M1 controllers are `active`.
   - `/joint_states` has exactly 1 publisher.
   - `/clock` and the Gazebo ground-truth pose are available.
   - All 12 joints are within 0.05 rad of CAD neutral.
2. **FK check at the start** (see the [FK guide](M3_FORWARD_KINEMATICS_GUIDE.md)). No motion happens unless it passes.
3. **Negative targets.** IK must reject them. The tool confirms that 0 goals were sent and that the joints did not move.
4. **Each safe target.**
   - Solve IK and check all 12 commands with the M1 rule.
   - Send **one** trajectory: the 3 LF joints go to the IK angles and the other 9 are held at their measured positions. It lasts ≥ 3 s, and ≥ 2·max|Δq|/0.5 rad/s.
   - Settle for 1 s of simulation time.
   - Check the observed tip (Gazebo link pose ∘ tip offset) against the target, allowing ≤ 2 mm.
   - Run the FK check against TF and Gazebo.
   - Return to the start configuration.
5. **Final checks.** The leg is back at the start, and the controllers are still active.

Ctrl+C cancels the current goal and sends the leg back to the start positions.

**Three-foot stance.** Lifting the LF foot leaves the robot on three feet. The whole-robot COM is
only 3.8 mm inside the RF–LR support edge. Body tilt is recorded as information; the comparisons
are all in `base_link`, so body motion does not affect them.

## Commands

```bash
ros2 launch spiderx_bringup fortress_posture_hold.launch.py      # terminal 1
ros2 run spiderx_controller validate_leg_kinematics              # terminal 2
./scripts/validate_m3_kinematics.sh                              # static
./scripts/validate_m3_kinematics.sh --runtime                    # automated runtime
```

Exit codes:
- `0`: FK and IK are both verified.
- `1`: `M3 kinematics validation not verified.`
- `2`: the configuration was refused and nothing was sent.

The report is written to `~/.ros/spiderx_m3/leg_kinematics_report.json`, or to the path given by `--output`.
