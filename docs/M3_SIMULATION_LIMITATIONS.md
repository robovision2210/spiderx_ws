# M3 Simulation Limitations

```text
Single-leg simulation kinematics validation.
Not walking or a gait.
Not balance or standing control.
Not locomotion or /cmd_vel control.
Not real-world leg control.
Not hardware validation.
```

M3 answers two narrow questions about the **front-left** leg in the **current Gazebo Fortress
model**. The answers are limited by everything listed below.
- Does our FK match the URDF, as evaluated by TF and by the Gazebo physics model?
- Does our IK put the foot where it is asked, for small, joint-safe targets?

| # | Limitation | Effect on M3 |
|---|---|---|
| 1 | **Same URDF everywhere.** Gazebo, TF and our code all use `spiderx.urdf.xacro` | Agreement proves the *implementation* matches the *model*. It cannot show that the model's dimensions match the physical robot |
| 2 | **Placeholder actuators** (URDF effort 100 N·m, velocity 100 rad/s) | The simulated joints are nearly rigid and track the IK angles to about 1e-9 rad. Real hobby servos will show larger errors, backlash and sag |
| 3 | **Derived foot tip** | The tip is the lowest collision-mesh point at q = 0, fixed in `lf_foot_1`. It is not a measured contact point, and the true contact point on the rounded pad moves as the foot rotates |
| 4 | **Contact, friction and damping assumptions** (µ = 0.2, no joint damping) | They affect how the body rests while one foot is lifted. The body-frame kinematics are not affected |
| 5 | **Mass and inertia assumptions** (steel-density CAD masses, 7.46 kg) | The 3-foot stance margin (COM 3.8 mm inside the RF–LR edge) is only as good as these values |
| 6 | **Physics step and solver** (DART, 1 ms; gz_ros2_control position → velocity command) | The Gazebo link pose comes from DART's generalized coordinates, so it agrees with FK almost exactly |
| 7 | **One leg only** | RF, LR and RR are not validated. Their sign conventions differ: LF's knee axis is +x, the other three are −x |
| 8 | **Small targets only** | The targets move the foot ≤ 20 mm from CAD neutral, with joint changes ≤ 0.213 rad. The full workspace, the limits and the singularities are exercised only in unit tests |
| 9 | **No hardware** | No real actuators, encoders, battery, electronics, structural flexibility or IMU are involved |
| 10 | **Ground-truth bridge timing** | `/spiderx/sim/world_poses` carries no stamps, so Gazebo samples are taken only after the leg has settled. TF is looked up at the exact `/joint_states` stamp |

## What M3 did not do

- **No motion on its own:** no walking, gait, balance, body-height control, `/cmd_vel` handling, odometry, SLAM or Nav2 motion.
- **No model changes:** no change to the URDF geometry, axes, origins, limits, masses, collisions, the CAD-neutral pose or the M1/M2 behaviour.
- **No new frames:** no new TF frame is published. The foot tip exists only inside the kinematics code.

## Environment issue observed during M3 (not caused by M3)

On one slow cloud start-up, `joint_state_broadcaster` failed to activate (`Switch controller
timed out after 5 s`), yet `controller.launch.py` still started the trajectory controller. A
relaunch activated both. Two things follow:
- **M3 fails on it.** The M3 runtime tool and script report this condition as a failure; they never pass silently.
- **The fix is out of M3 scope.** Raising `--switch-timeout` and starting the second spawner only when the first succeeds is suggested as a separate task.
