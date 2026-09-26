# M3 Guide – Forward Kinematics (FK) of the SpiderX Front-Left Leg

> Single-leg simulation kinematics validation (Gazebo Fortress). This is not walking, a gait,
> balance control, locomotion, real-world leg control or hardware validation.

Read with: [frame conventions](M3_FRAME_CONVENTIONS.md), [IK guide](M3_INVERSE_KINEMATICS_GUIDE.md),
[results](M3_TEST_RESULTS.md), [limitations](M3_SIMULATION_LIMITATIONS.md), [plan](M3_LEG_KINEMATICS_PLAN.md).

## What FK is

**Forward kinematics** answers one question: *given the three joint angles of a leg, where is the foot?*

For SpiderX's front-left leg, FK takes the angles
`[lf_hip, lf_thigh_joint, lf_foot_joint]` (radians) and returns:
- the full pose (position and orientation) of the URDF link `lf_foot_1`, whose origin sits on the knee axis;
- the position of the **derived foot tip** (see [frame conventions](M3_FRAME_CONVENTIONS.md#foot-tip-derived)).

All results are in `base_link`.

## Why kinematics comes before a gait

- **A gait is built from foot positions.** A gait says *where each foot should be* at every moment, but the controllers take *joint angles*.
- **So a gait needs IK, and IK needs FK.** IK converts foot positions into angles, and IK can only be trusted once FK, its inverse, is proven correct against the robot model.
- **M3 proves both for one leg.** It does so in simulation, before anyone builds foot trajectories (M4) on top of them.

## How SpiderX FK is computed

`spiderx_controller/leg_kinematics.py` builds the chain from the **expanded URDF** at load time. No link length is typed into the code. It has two implementations:

1. **URDF transform chain.** This is the main FK. It multiplies the seven transforms from
   `base_link` to `lf_foot_1`: 4 fixed and 3 joints.
2. **Product of exponentials.** This form is used inside the IK:
   `p(q) = H + R(a₁,q₁)·[T − H + R(a₂,q₂)·[K − T + R(a₃,q₃)·(tip₀ − K)]]`

Unit tests require the two implementations to agree to 1e-12 m. They also require the transform
chain to agree with a separate numpy URDF walk, written independently with quaternion rotations
and its own XML and STL parsing.

Input rules:
- exactly 3 finite numbers, in the order above;
- anything else raises `KinematicsError`: NaN, strings, booleans, or the wrong length.

FK never commands the robot and does not use ROS.

```python
from spiderx_controller import leg_kinematics as lk
g = lk.load_geometry('front_left')          # expands the installed URDF
fk = lk.forward(g, [0.0, 0.2, 0.0])
fk['foot_link_position']   # (-0.0367, -0.052834, 0.027971) m in base_link
fk['foot_link_quaternion'] # (-0.099833, 0.0, 0.0, 0.995004) (x, y, z, w)
fk['tip_position']         # derived foot tip in base_link
```

## How FK was validated

| Check | Against | Tolerance | Result |
|---|---|---|---|
| Unit | Independent numpy URDF walk, 300 random joint-safe configurations | 1e-12 m / 1e-9 (quaternion components) | pass |
| Unit | Product of exponentials | 1e-12 m | pass |
| Unit | Mesh-derived tip, recomputed independently from the STL | 1e-9 m | pass |
| Unit | Sign conventions (hip outward, thigh rearward, foot forward and up) | – | pass |
| Runtime | **TF** `base_link → lf_foot_1` (robot_state_publisher), looked up at the same `/joint_states` stamp | 1e-6 m, 1e-6 rad | see [results](M3_TEST_RESULTS.md) |
| Runtime | **Gazebo** ground-truth `lf_foot_1` link pose (`/spiderx/sim/world_poses`, model-relative) | 1 mm, 0.005 rad (simulation-only) | see [results](M3_TEST_RESULTS.md) |

The runtime checks run at CAD neutral **and** at every IK target configuration, so FK is also
tested at non-zero angles on all three joints.

## How to run it

```bash
# terminal 1: M1 controllers + the M2 Gazebo ground-truth pose bridge
ros2 launch spiderx_bringup fortress_posture_hold.launch.py
# terminal 2: FK checks, then the IK targets (see the IK guide)
ros2 run spiderx_controller validate_leg_kinematics
```

In the summary, each `FK check [...]` line prints the link-origin error, the orientation error
and the tip error, against both TF and Gazebo.

## Reading the result

| Line | Meaning |
|---|---|
| `Single-leg FK verified against the current URDF/TF/Gazebo model.` | Every FK check passed |
| `M3 kinematics validation not verified.` | Some check failed; every reason is printed as `FAILED: ...` |

These checks show that the **implementation** matches the **model**. They cannot show that the
model's dimensions match the real robot: Gazebo and TF use the same URDF.
