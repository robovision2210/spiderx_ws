# M3 Frame Conventions – SpiderX Front-Left Leg

> Single-leg simulation kinematics validation. This is not walking, balance, locomotion or
> hardware validation.

Every value on this page comes from the expanded URDF, `src/spiderx_description/urdf/spiderx.urdf.xacro`.
`spiderx_controller/leg_kinematics.py` reads the values from that file at load time, and unit
tests check them against an independent URDF walk.

## Body frame: `base_link` (not REP-103)

| Axis | Direction on the robot |
|---|---|
| +x | robot **right** |
| +y | robot **front** |
| +z | up |

- `base_link` is the CAD origin, near a corner of the body footprint (see `SPIDERX_URDF_AUDIT.md` §3).
- `dummy_link` is the URDF root. It is joined to `base_link` by `dummy_joint`, which has no offset, so the two frames coincide.
- In Gazebo, the model frame of `spiderx` is `dummy_link`, and link poses in `/world/spiderx_fortress/pose/info` are relative to that model frame. This was checked at runtime.
- **M3 does not publish any new or alternative frame.** All FK/IK inputs and outputs are in `base_link`.
- For the **left** legs, "outward" is −x and "inward" is +x.

## Front-left chain

```
base_link
 └─ Rigid 1        fixed     (0.005, 0.0024, 0.091)                    → b_s4_1
     └─ lf_hip     revolute  (0.005, 0.0137, -0.01375)   axis [0 1 0]    → b_s4_c_horn_1
         └─ Rigid 12  fixed  (0, -0.001, 0)                            → lf_holder_suppor1_1
             └─ Rigid 16 fixed (-0.02, 0.01, 0.0165)                   → lf_holder_1
                 └─ lf_thigh_joint revolute (-0.0137, 0.01375, -0.005) axis [-1 0 0] → lf_c_horn_1
                     └─ Rigid 25 fixed (-0.001, 0, 0)                  → lf_thigh_1
                         └─ lf_foot_joint revolute (-0.012, -0.077782, -0.077782) axis [1 0 0] → lf_foot_1
                             └─ (derived, not a frame) foot tip  t_tip ≈ (0.003152, 0.081196, -0.065511)
```

- Every origin has `rpy = 0`, so at q = 0 every link frame is parallel to `base_link`.
- The origins are in metres, relative to the parent frame.
- Source lines in `spiderx.urdf.xacro`: L696, L744, L770, L794, L830, L850 and L880 (joints); L654–673 (`lf_foot_1` link and collision mesh).

## Joint-axis points at q = 0 (base_link, m)

| Point | Value | Meaning |
|---|---|---|
| H | (0.010, 0.0161, 0.07725) | on the hip (`lf_hip`) axis; origin of `b_s4_c_horn_1` |
| T | (−0.0237, 0.03885, 0.08875) | on the thigh axis; origin of `lf_c_horn_1` |
| K | (−0.0367, −0.038932, 0.010968) | on the knee (`lf_foot_joint`) axis; origin of `lf_foot_1` |
| tip₀ | (−0.033548, 0.042264, −0.054543) | derived foot tip at CAD neutral |

## Joint signs (right-hand rule about the URDF axis)

| Joint | URDF axis | Limits (rad) | Allowed with 0.05 margin | Positive rotation moves the LF foot tip |
|---|---|---|---|---|
| `lf_hip` | [0 1 0] | [−0.610865, 0.698132] | [−0.560865, 0.648132] | **outward** (−x) |
| `lf_thigh_joint` | [−1 0 0] | [−0.610865, 0.785398] | [−0.560865, 0.735398] | **rearward** (−y), slightly up |
| `lf_foot_joint` | [1 0 0] | [−0.698132, 0.436332] | [−0.648132, 0.386332] | **forward** (+y) and **up** |

Two points about the signs:
- **LF is the odd leg.** Its knee axis is +x; the knees of RF, LR and RR are −x. Its knee limits are mirrored to match, so the physical range is the same. M3 takes every sign from the URDF and never assumes it.
- **The signs were confirmed at runtime.** With `lf_thigh_joint = +0.2`, TF and Gazebo both reported the quaternion (−0.099833, 0, 0, 0.995004), which is −0.2 rad about +x, as axis [−1 0 0] predicts.

## Foot tip (derived)

The URDF has no foot-tip frame: the `lf_foot_1` frame origin sits on the knee axis, K. M3 therefore uses a **derived, non-physical reference point**, fixed in `lf_foot_1`:
- **Derivation:**
  1. Take the `lf_foot_1` collision mesh (`meshes/lf_foot_1.stl`, scale 0.001).
  2. Transform it into `base_link` at q = 0.
  3. Keep the unique vertices within 0.1 mm of the lowest z.
  4. The tip is (mean x, mean y, lowest z) of those vertices. This currently uses 13 vertices.
- **Result:** tip₀ = (−0.033548, 0.042264, −0.054543) m, which matches the audit's "feet at z = −0.0545".
- **Not a contact point.** It is a point fixed in the foot. The real contact point on the rounded pad moves slightly as the foot rotates.

## Conventions used by the math

- Units: metres and radians.
- Joint vector order: `[lf_hip, lf_thigh_joint, lf_foot_joint]`.
- Joint transform: `T(q) = Trans(xyz) · Rz(yaw)·Ry(pitch)·Rx(roll) · Rot(axis, q)`.
- Chain: `T_base_foot = T_R1 · T_hip · T_R12 · T_R16 · T_thigh · T_R25 · T_foot`, and the tip is `T_base_foot · t_tip`.
- Fixed transforms have no joint variable. Actuated transforms rotate about their own axis in the child frame.
