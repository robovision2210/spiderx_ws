# SpiderX Development Roadmap

A legged robot has to be built bottom-up. Each milestone depends on the one before it and has a
test that proves it works. Do not start Nav2 or SLAM work until M6 is done: they need a robot that
moves when commanded (`/cmd_vel`) and reports its motion (`/odom`).

```
M0 model + sim  ─►  M1 joint control  ─►  M2 sim posture hold  ─►  M3 single-leg FK/IK  ─►  M4 gait
                                                                          │
      M9 hardware ◄─ M8 Nav2 ◄─ M7 SLAM/AMCL ◄─ M6 odometry ◄─ M5 /cmd_vel bridge
```

## M0 – Model and simulation ✅ done

- [x] CAD URDF audited (`SPIDERX_URDF_AUDIT.md`)
- [x] Gazebo Fortress world, spawn, lidar `/scan`, `/joint_states`, `/clock`, TF
- [x] Leg and joint groups validated against the URDF (`spiderx_controller`)
- [x] Validated locally on Ubuntu 22.04

## M1 – Joint position control in simulation ✅ done (cloud + local)

See [`M1_JOINT_POSITION_CONTROL_GUIDE.md`](M1_JOINT_POSITION_CONTROL_GUIDE.md) and [`M1_TEST_RESULTS.md`](M1_TEST_RESULTS.md).

1. Add a Fortress `<ros2_control>` block to `spiderx_fortress.gazebo.xacro`, using the `gz_ros2_control/GazeboSimSystem` hardware plugin and the `gz_ros2_control-system` Gazebo plugin.
2. Load `spiderx_controller/config/spiderx_ros2_controllers.yaml` (`joint_state_broadcaster` and `leg_trajectory_controller`).
3. Replace the URDF effort/velocity placeholders (100 N·m, 100 rad/s) with servo datasheet values, and add joint damping.

- [x] `ros2 control list_controllers` shows both controllers active (cloud)
- [x] A 0.2 rad step on one joint is tracked; steady-state error recorded: 0.0000 rad, idealised by the placeholder effort limit (cloud)
- [x] All 12 joints follow one trajectory to `cad_neutral` (cloud)
- [x] The same checks on the owner's Ubuntu PC
- [ ] With the robot lifted (fixed base), every joint reaches both soft limits
- [ ] Servo datasheet effort/velocity limits and joint damping (moved to M9: hardware evidence needed)

## M2 – Simulation-only CAD neutral posture hold ✅ verified (cloud + local)

Simulation only. This is not balance control, walking, IK or hardware validation. See
[`M2_SIMULATION_POSTURE_GUIDE.md`](M2_SIMULATION_POSTURE_GUIDE.md),
[`M2_TEST_RESULTS.md`](M2_TEST_RESULTS.md) and
[`M2_SIMULATION_LIMITATIONS.md`](M2_SIMULATION_LIMITATIONS.md).

- [x] One canonical posture config (`m2_simulation_postures.yaml`), validated before sending: 12 joints, URDF limits −/+ 0.05 rad, equal to `cad_neutral`
- [x] One slow 12-joint trajectory at ≤ `max_joint_velocity_rad_s`
- [x] 10 s hold (simulation time): joint error, controller states, body height, roll and pitch within the documented thresholds; JSON report (cloud)
- [x] Negative tests: invalid configs refused before sending; failed measured conditions report "not verified"
- [x] The same checks on the owner's Ubuntu PC
- [ ] Longer holds, disturbance tests and a tuned stand height: **not M2**. Only after real actuator limits exist (M9), and only as new, separately tested poses

Balance / standing control (body feedback) is not scheduled yet; it needs an IMU and real actuator data.

## M3 – Single-leg forward and inverse kinematics ✅ front-left verified (cloud + local)

Simulation only. This is not walking, a gait, balance, locomotion or hardware validation. See the
[plan](M3_LEG_KINEMATICS_PLAN.md), [frames](M3_FRAME_CONVENTIONS.md),
[FK guide](M3_FORWARD_KINEMATICS_GUIDE.md), [IK guide](M3_INVERSE_KINEMATICS_GUIDE.md),
[results](M3_TEST_RESULTS.md) and [limitations](M3_SIMULATION_LIMITATIONS.md).

- [x] Front-left geometry extracted from the URDF (no typed link lengths); derived foot tip from the collision mesh
- [x] FK matches an independent URDF walk (unit), and TF + Gazebo `lf_foot_1` at 6 configurations (runtime, ≤ 5.4e-11 m)
- [x] Analytic IK round-trips FK over 500 random joint-safe configurations; out-of-limit and unreachable targets refused, not clamped
- [x] 5 lifted safe targets reached in simulation and returned; the 2 negative targets were not commanded
- [x] Per-joint sign table for LF (`lf_foot_joint` +x) validated
- [x] The same checks on the owner's Ubuntu PC
- [ ] RF, LR and RR legs (including `lr_foot_joint` −x); a 4-leg kinematics API for the gait

## M4 – Gait generator

- [ ] Static walk (3 feet down), then trot, with the stance COM kept inside the support polygon
- [ ] Walks forward 1 m in simulation without falling (a video is required before claiming it)

## M5 – `/cmd_vel` → gait bridge

- [ ] Stepping velocity follows `/cmd_vel` (vx, vy, ωz) within limits
- [ ] A timeout stops the robot safely

## M6 – Odometry

- [ ] Leg odometry publishes `/spiderx/leg_odometry`
- [ ] The EKF (`spiderx_localization`) fuses it with the IMU and publishes `odom → dummy_link`
- [ ] Drift is measured against Gazebo ground truth
- [ ] Add a REP-103 base frame (x forward) for Nav2

## M7 – SLAM and localization

- [ ] `spiderx_mapping slam.launch.py` builds a map of `spiderx_fortress.sdf`
- [ ] AMCL localizes on that map

## M8 – Nav2

- [ ] Replace the velocity placeholders with gait-measured limits
- [ ] Reach a goal in simulation

## M9 – Real hardware

- [ ] Actuator interface (`SPIDERX_HARDWARE_INTERFACE.md`), then repeat M1–M8 on the robot
- [ ] Real masses (weighed parts) replace the steel-density CAD values
