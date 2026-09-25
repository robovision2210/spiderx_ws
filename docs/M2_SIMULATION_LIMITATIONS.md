# M2 Simulation Limitations

```text
Simulation-only posture hold.
Not dynamic balance control.
Not walking or gait control.
Not inverse kinematics.
Not hardware validation.
Not real-servo torque validation.
Not battery/current validation.
Not proof of real-world stability.
```

M2 asks one narrow question: **in the current Gazebo Fortress model, do the M1 joint-position
controllers hold SpiderX in its CAD neutral pose for 10 s of simulation time?** Any M2 result,
pass or fail, is limited by every item below. None of the heights, angles or tracking errors should
be read as predictions for the real robot.

## What limits the result

| # | Limitation | Current value | Effect on M2 |
|---|---|---|---|
| 1 | **Placeholder actuator effort/torque** | URDF `effort="100"` N·m and `velocity="100"` rad/s on every joint. These are fusion2urdf exporter placeholders (audit K4) | The simulated "servos" can apply far more torque than a real hobby servo, so they hold their angles almost perfectly. Tracking errors and body height are **idealised** |
| 2 | **Simulation contact model** | Gazebo Fortress default contact on DART; mesh collisions from the CAD | How the feet sit, slide or penetrate depends on the solver, not on real foot materials |
| 3 | **Friction assumptions** | `mu1 = mu2 = 0.2` on every link, a template default, not measured | Changes whether the feet slip; real rubber or plastic feet would differ |
| 4 | **Damping assumptions** | No joint damping or friction in the URDF (`<dynamics>` absent) | Oscillation and settling are not realistic |
| 5 | **Link mass/inertia assumptions** | CAD values at steel density; total 7.29 kg, not weighed | Load on each joint, and so the torque needed, is almost certainly wrong |
| 6 | **Physics timestep/solver** | 1 ms step with DART. `gz_ros2_control` applies position commands as velocity commands (proportional gain 0.1); controller update rate 100 Hz | Results can change with a different step, solver or plugin version |
| 7 | **No hardware validation** | No real actuator, battery, electronics, structural flexibility or IMU in the loop | Current draw, brown-outs, servo deadband, gear backlash, frame flex and sensor noise are all absent |

## What M2 did not measure

- **Foot contact.** The model and world have no contact sensor, so contact is reported as
  **unavailable** and never faked. Body height compared with the CAD foot plane (0.0545 m) is
  a geometric indicator only.
- **Disturbance rejection.** Nothing pushes the robot. M2 holds joints; it does not balance.
- **Balance feedback.** No controller uses body pose. The ground-truth pose is **observed** for
  the test only.
- **Other postures.** Only the existing `cad_neutral` pose is tested. By owner decision, no
  posture is searched for or tuned.

## How body pose is measured

- **Source:** Gazebo ground truth. The topic `/world/spiderx_fortress/pose/info` is bridged to
  `/spiderx/sim/world_poses`, which is **not** `/tf`.
- **Frame:** the model `spiderx` in the world frame. That is `dummy_link`, which equals
  `base_link` because `dummy_joint` has no offset.
- **Height:** the `base_link` origin z above the ground plane.
- **Angles:** roll and pitch are ZYX Euler angles of `base_link` in the world. Because
  `base_link` has +y forward and +x right:
  - "roll" (about x) is **nose up/down**;
  - "pitch" (about y) is **side tilt**.
- **Status:** this is simulation-only ground truth. There is no equivalent on the real robot
  until an IMU and odometry exist (M6/M9).

## What would make the results more meaningful (future work, not M2)

- Servo datasheet torque and speed limits, joint damping and friction (M9).
- Weighed part masses, or real material densities.
- Measured foot friction.
- An IMU in the model, and later on the robot.
- Contact sensors on the feet, if a later milestone needs contact evidence.
