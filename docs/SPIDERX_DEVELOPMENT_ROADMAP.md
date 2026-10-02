# SpiderX Development Roadmap

A legged robot has to be built bottom-up. Each milestone depends on the one before it and has a
test that proves it works. Do not start Nav2 or SLAM work until M7 is done: they need a robot that
moves when commanded (`/cmd_vel`) and reports its motion (`/odom`).

```
M0 model + sim  ─►  M1 joint control  ─►  M2 sim posture hold  ─►  M3 single-leg FK/IK  ─►  M4 all-leg IK + static poses  ─►  M4.5 offline gait analysis  ─►  M5 offline evaluation study  ─►  M6.0 playback safety layers  ─►  (future: gait playback)
                                                                          │
      M10 hardware ◄─ M9 Nav2 ◄─ M8 SLAM/AMCL ◄─ M7 odometry + state estimation ◄─ M5.5 command-velocity bridge
```

**Milestone numbering (owner decision, 2026-10-02).** M6 is **Gait playback safety** (M6.0, the
safety implementation; M6.1, future protected replay, not implemented). The former future
"M6 – Odometry" is now **M7 – Odometry and state estimation (future work)**, and the later future
milestones move down by one: SLAM/localization M7 → **M8**, Nav2 M8 → **M9**, real hardware
M9 → **M10**. Older documents keep their original numbers as historical records: there, "M6
odometry" means M7, "M7" SLAM means M8, "M8" Nav2 means M9 and "M9" hardware means M10.

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
- [ ] Servo datasheet effort/velocity limits and joint damping (moved to M10, formerly M9: hardware evidence needed)

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
- [ ] Longer holds, disturbance tests and a tuned stand height: **not M2**. Only after real actuator limits exist (M10, formerly M9), and only as new, separately tested poses

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

## M4 – All-leg kinematics and static pose hold via IK ✅ verified (cloud + local)

Simulation only. This is not walking, a gait, balance control or hardware validation. See the
[plan](M4_PLAN.md) and [results](M4_TEST_RESULTS.md).

- [x] All four leg chains audited from the URDF: per-leg axis signs, with LR's knee parallel to its thigh
- [x] Opt-in all-leg FK/IK (M3 default unchanged); unit tests against an independent URDF walk
- [x] Three static poses validated before sending (atomic 12-joint commands, no clamping)
- [x] Gazebo: FK vs TF and Gazebo on all four legs; 3 poses reached and held for 5 s; body height and tilt within the thresholds (cloud)
- [x] The same checks on the owner's Ubuntu PC (`fd777fe`; all passed)
- [x] Fix the intermittent M1 controller start-up race → done in **M4.1** below (cloud + local verified)

## M4.1 – Controller start-up robustness ✅ verified (cloud + local)

Simulation only. This covers start-up ordering and failure handling of the M1 controllers. There
is no gait, walking or hardware work. See the [plan](M4_1_PLAN.md) and
[results](M4_1_TEST_RESULTS.md).

- [x] `leg_trajectory_controller` starts only after the `joint_state_broadcaster` spawner exited with code 0; nothing starts during Ctrl+C
- [x] A failed broadcaster spawner logs one clear error and the trajectory controller is not started; the launch is not shut down automatically, so Ctrl+C stays the shutdown path
- [x] Simulation-only spawner options `--switch-timeout 60` and `--service-call-timeout 75`
- [x] The M1/M2 validators report a controller as active only if it is exactly `active`
- [x] Launch tests with a mutation check; a forced broadcaster failure; Ctrl+C during start-up; the M0–M4 runtime regressions (cloud)
- [x] The same checks on the owner's Ubuntu PC (`35b83b5`; all passed)

## M4.5 – Offline multi-gait configuration and trajectory validation ✅ implemented (offline), verified (cloud + local) — paper-evaluation pending

OFFLINE kinematic analysis only. There is no Gazebo gait playback, no walking and no hardware. See the
[plan](M4_5_PLAN.md), [results](M4_5_TEST_RESULTS.md) and [framework guide](SPIDERX_GAIT_FRAMEWORK.md).

- [x] Six YAML gait configurations: wave, tripod_crawl, ripple, amble, pace, trot. Duty factor, phase offsets, swing order, stroke, step height, stance offsets and speed/period are all checked by a strict loader
- [x] Clock-driven foot trajectories: cycloid swing, velocity-continuous transitions
- [x] Sampled IK feasibility through the unchanged M3/M4 IK, plus joint-limit, singularity and continuity margins
- [x] Static-stability margin (an approximation), joint speed against the placeholder, energy proxies (heuristic)
- [x] Per-gait PASS/FAIL with the exact failed checks; deterministic CSV, JSON and PNG; cross-gait comparison (cloud)
- [x] Reported result: wave and tripod_crawl FAIL static stability (the COM is ≈ 5.8 mm behind the foot centre); the other four pass their kinematic checks
- [x] The same checks on the owner's Ubuntu PC (`4a6553c`; all passed, 40 byte-identical artifacts, same verdicts)
- [x] Owner decisions: keep the YAML gait names (terminology mapping in the framework guide); matplotlib approved, with `--no-plots` kept
- [ ] Paper evaluation: better mass data and comparison against published gait results; body sway stays future work only

## M5 – Offline evaluation study — cloud + local offline verification passed

OFFLINE model analysis only: a staged, reproducible evaluation of gait configuration classes on the
SpiderX URDF model, using the unchanged M4.5 evaluator. There is no Gazebo, no gait playback, no
walking and no hardware. See the [plan](M5_EVALUATION_PLAN.md) and its owner-decision addendum (§16).

- [x] Phase 0: read-only audit and evaluation protocol (`4c1bd1d`), approved by the owner in principle; decisions D1–D5 recorded
- [x] Phase 1: study specification, runner with Stage 0/1 gate, provenance-rich records, derived tables, regression pins (cloud; 700 tests)
- [x] Study run (cloud): two complete, byte-identical runs; gate passed; H1–H3 supported (model predictions) — [results](M5_TEST_RESULTS.md)
- [x] The same checks on the owner's Ubuntu PC: offline only; two full local runs exit 0 and byte-identical with each other; 700 tests passing
- [ ] Citations-only review step: original-source-verified candidate references for owner approval (not started)

## M6.0 – Gazebo gait-playback safety layers — cloud + local offline/mock verified; M6.0-B graph-mode read-only preflight passed locally; M6.0-D implementation cloud + local verified in offline/mock/isolated-domain testing; live dispatch hard-disabled; local live playback pending separate approval

A trajectory-execution and observability check only. It replays **one** bounded neutral →
`crouch_10mm` → neutral trajectory, after approval. It is not gait playback and not walking. See
the [plan](M6_GAIT_PLAYBACK_SAFETY_PLAN.md) (§14 owner decisions D1–D7 and option (i)), the
[results](M6_TEST_RESULTS.md) and the [guide](SPIDERX_M6_PLAYBACK_GUIDE.md).

**Naming.** M6 is Gait playback safety: M6.0 is the safety implementation and M6.1 is future
protected replay (not implemented). Odometry is now M7 (see the numbering note at the top).

- [x] Phase 0: read-only audit and safety plan (`cc6974d`); decisions D1–D7 (`3536575`); crouch envelope option (i), 0.1223 rad for M6.0-D only (`bef3c25`)
- [x] M6.0-A: offline conversion and preflight (`0a9d2db`)
- [x] M6.0-C: single-goal action client, mock-only safety and mutation tests (`d6ebaad`)
- [x] M6.0-B tool: live read-only preflight, mock-tested; cloud ran `--interface-only` only (`e9b1565`)
- [x] Cloud validation: 912 tests, then 932 after the provenance portability fix (`fd7a9de`); 0 failures; M1–M4 static validators pass
- [x] Local offline/mock verification on the owner's Ubuntu PC (`f49a6e0`): 932 tests, 0 failures; same `trajectory_id` as the cloud; byte-identical outputs; M1–M4 static validators pass
- [x] M6.0-B: live GUI graph-mode read-only preflight against the running stack on the owner's PC (`b64217d`): READY, five evidence tiers passed, classification `warning`, clean shutdown; observation only, no goal or command ([plan](M6_GRAPH_PREFLIGHT_PLAN.md), [results](M6_GRAPH_PREFLIGHT_RESULTS.md))
- [x] M6.0-D plan adopted with owner decisions D1–D17 (`311a849`, [plan](M6D_LIVE_PLAYBACK_PLAN.md))
- [x] M6.0-D implementation (`28b72a4`, `e87c81a`, `d7fb6a2`, `7fd0152`): goal fingerprint, readiness classification, single-goal live state machine, rclpy transport, CLI (`--dry-run`, `--mock`); cloud-verified in offline/mock/isolated-domain tests only (1116 tests, 0 failures); **live dispatch hard-disabled**
- [x] M6.0-D local offline/mock/isolated-domain verification on the owner's Ubuntu PC (`412eb45`): 1116 tests, 0 failures; 180 M6.0-D tests; identical fingerprint; dry-run/mock reports byte-identical to the cloud; `--live` exit 3 ([results §12](M6D_LIVE_PLAYBACK_RESULTS.md)) ([results](M6D_LIVE_PLAYBACK_RESULTS.md), [guide](SPIDERX_M6D_LIVE_PLAYBACK_GUIDE.md))
- [x] M6.0-D live-enabling design: gated `--live` wiring, gate still `False`; the exact two-line enabling commit and the manual checklist are documented ([live-enabling design](M6D_LIVE_ENABLING_DESIGN.md))
- [ ] M6.0-D enabling commit (two lines; separate owner approval after a final audit)
- [ ] M6.0-D local live playback: one valid neutral → `crouch_10mm` → neutral goal on the owner's PC (needs a separate owner approval, an enabling change and the manual runtime checklist; no goal has been sent)
- [ ] M6.1: protected replay of an offline-validated gait cycle (future work, not implemented; deferred by D2; needs a separate plan)

## Future — Gait playback and walking in simulation (unscheduled; formerly the M4.5 goal)

By the owner's decision, this stays an unscheduled future item. It is not part of M4.5.

- [ ] Static walk (3 feet down), then trot, with the stance COM kept inside the support polygon
- [ ] Walks forward 1 m in simulation without falling (a video is required before claiming it)

## M5.5 – Command-velocity bridge (future work)

Renamed from "M5 – `/cmd_vel` → gait bridge" by owner decision D1 (see [M5 plan §16](M5_EVALUATION_PLAN.md#16-owner-decision-addendum)). Not implemented and not scheduled.


- [ ] Stepping velocity follows `/cmd_vel` (vx, vy, ωz) within limits
- [ ] A timeout stops the robot safely

## M7 – Odometry and state estimation (future work)

Formerly "M6 – Odometry". Not implemented and not scheduled.


- [ ] Leg odometry publishes `/spiderx/leg_odometry`
- [ ] The EKF (`spiderx_localization`) fuses it with the IMU and publishes `odom → dummy_link`
- [ ] Drift is measured against Gazebo ground truth
- [ ] Add a REP-103 base frame (x forward) for Nav2

## M8 – SLAM and localization (formerly M7)

- [ ] `spiderx_mapping slam.launch.py` builds a map of `spiderx_fortress.sdf`
- [ ] AMCL localizes on that map

## M9 – Nav2 (formerly M8)

- [ ] Replace the velocity placeholders with gait-measured limits
- [ ] Reach a goal in simulation

## M10 – Real hardware (formerly M9)

- [ ] Actuator interface (`SPIDERX_HARDWARE_INTERFACE.md`), then repeat M1–M9 on the robot
- [ ] Real masses (weighed parts) replace the steel-density CAD values
