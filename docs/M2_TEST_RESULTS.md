# M2 Test Results – Simulation-Only CAD Neutral Posture Hold

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

## Outcome

**Simulation posture hold verified.** (cloud; local run pending)

The existing `cad_neutral` pose was reached and held for 10 s of simulation time within every
documented threshold, in three separate runs. The posture was **not** tuned or changed. The
thresholds were fixed in `m2_simulation_postures.yaml` before the first run.

This result is **idealised**. Every joint may use the 100 N·m placeholder effort, so the
simulated joints are almost rigid, and foot contact was **not measured**. See
[M2_SIMULATION_LIMITATIONS.md](M2_SIMULATION_LIMITATIONS.md).

## Environment

| Item | Value |
|---|---|
| Branch | `claude/spiderx-m2-posture-hold`, stacked on `claude/spiderx-m1-joint-control` (PR #6) |
| Gazebo | Ignition Gazebo 6.16 (Fortress), DART, 1 ms step |
| Control | `gz_ros2_control` 0.7.15; controller_manager 2.51.0; joint_trajectory_controller / joint_state_broadcaster 2.48.0 |
| Bridge | ros_gz 0.244.20 (`Pose_V → tf2_msgs/TFMessage` for `/spiderx/sim/world_poses`) |
| Machine | Cloud VM, no GPU, Xvfb and Mesa software rendering; real-time factor ≈ 0.25 |
| Build | `colcon build --symlink-install`, **8 packages**. Setuptools was temporarily pinned to Ubuntu's 59.6.0 for the build (the known setuptools ≥ 80 `--symlink-install` issue) and restored afterwards |

## Runtime results (three runs)

| Metric | Threshold | Run A: from spawn | Run B: 3 joints displaced (manual) | Run C: `validate_m2_posture.sh --runtime` |
|---|---|---|---|---|
| Joints moved by the trajectory (> 0.01 rad) | informational | 0/12 (already at the pose) | 3/12 | 3/12 (`rr_hip`, `rf_thigh_joint`, `lr_foot_joint`, each from +0.2 rad) |
| Peak commanded joint speed | ≤ 0.5 rad/s | 0.000 | 0.033 | 0.033 rad/s |
| Action result | `error_code` 0 | 0 | 0 | 0 (SUCCESSFUL) |
| Controllers at start, every 2 s of the hold, and at the end | all `active` | all active (7 checks) | all active (7 checks) | all active (7 checks) |
| `/joint_states` publishers (start, end) | [1, 1] | [1, 1] | [1, 1] | [1, 1] |
| Hold duration (simulation time) | ≥ 10 s | 10.00 s | 10.00 s | 10.00 s (t = 22.722 → 32.722 s) |
| Joint-state / body-pose samples in the hold | ≥ 20 / ≥ 20 | 1000 / 2355 | 1000 / 2225 | 1000 / 2255 |
| Max \|joint error\| during the hold | ≤ 0.05 rad | 6e-10 | 0.00025 | **0.00028 rad** (`rf_thigh_joint`) |
| RMS joint error during the hold | ≤ 0.02 rad | 3e-10 | 0.000009 | **0.000010 rad** |
| Final \|joint error\| (max over 12) | informational | 5.8e-10 | 5.8e-10 | 5.8e-10 rad |
| Body height, min | ≥ 0.045 m | 0.05450 | 0.05449 | **0.05448 m** |
| Body height, range during the hold | ≤ 0.005 m | 0.00001 | 0.00006 | **0.00007 m** |
| Body height, final | informational | 0.0545 | 0.05450 | 0.05450 m |
| \|roll\| max (about `base_link` x: nose up/down) | ≤ 0.10 rad | 1.5e-7 | 0.00018 | **0.00018 rad** |
| \|pitch\| max (about `base_link` y: side tilt) | ≤ 0.10 rad | 2e-12 | 0.00020 | **0.00022 rad** |
| Total tilt, max | informational | 1.5e-7 | 0.00021 | 0.00024 rad |
| xy drift / yaw drift during the hold | informational | 1e-10 m / 5e-14 rad | 0.00001 m / 0.00009 rad | 0.00001 m / 0.00010 rad |
| Final height − CAD foot plane (0.0545 m) | informational, **not contact** | +4.3e-6 m | −3.3e-6 m | −1.9e-6 m |
| Foot contact | – | unavailable | unavailable | unavailable (no contact sensor; not measured) |
| **Outcome** | | **verified** | **verified** | **verified** |

- **Frame.** Body pose is Gazebo ground truth: the model `spiderx` in the world frame. That is
  `dummy_link`, which equals `base_link`. Height is the `base_link` origin above the ground plane.
  Roll and pitch are ZYX Euler angles of `base_link` in the world.
- **Report.** Every run wrote the full JSON report. Run C's copy is at `log/m2_posture_hold/latest_report.json` (not committed; `log/` is ignored by git).
- **Run A** exercised no motion, because gz_ros2_control already holds every joint at 0 from
  spawn. The tool printed a NOTE saying so. Runs B and C start from 3 displaced joints, so the
  single 12-joint trajectory really moves joints.
- **Why this counts as a hold.** The final height minus the CAD foot plane is about −2 µm. That
  is consistent with the feet resting on the ground in the CAD zero pose. It is a geometric
  indicator, **not** a contact measurement.

## Static and unit tests

| # | Check | Result |
|---|---|---|
| 1 | `validate_controller_config` (URDF, `<ros2_control>`, M2 posture config) | ✅ valid |
| 2 | `validate_m2_posture.sh` (static) | ✅ All M2 checks passed |
| 3 | Refusals before sending (exit 2, no report written): `lf_foot_joint` 0.6 (above the URDF limit); `lf_foot_joint` 0.40 (inside the limit but inside the 0.05 margin); unknown `left_wheel_joint`; `simulation_only: false` | ✅ all REFUSED |
| 4 | Unit tests, `spiderx_controller`: `test_config_check` 5, `test_joint_safety` 13, `test_posture_config` 39, `test_posture_metrics` 23 | ✅ 80 passed |
| 5 | Unit tests, `spiderx_scripts` | ✅ 2 passed |
| 6 | Total | **82 pytest cases.** `colcon test-result` prints **86** because it also counts the 4 CTest wrapper entries of `spiderx_controller`. The full-workspace total is in the regression table |

The negative tests include:
- **Config:** missing, extra, duplicate and unknown joints; out-of-limit targets; margin violations; wrong types; degrees mistaken for radians; missing or invalid thresholds; missing or false `simulation_only`; malformed YAML; a safe but altered posture.
- **Measured failures:** each of 17 failed conditions returns "not verified", including low body height, roll, pitch, joint error, an inactive controller, 2 publishers, action timeout, a short hold and a missing clock.
- **Refusal:** the tool refuses before ROS is initialised.

## Regression

Regression runs (`colcon test`, M1 static and runtime, passive Fortress runtime) are in progress; results will be added here.

## Problems found during M2

| Problem | Cause | Handling |
|---|---|---|
| A Gazebo server start hung once (spawn repeated "Requesting list of world names"; the server was idle with no services) | A cloud-environment start-up flake, seen after a container restart. Not reproduced in any later launch | Stopped and relaunched. Documented in the guide's troubleshooting table |
| The ground-truth bridge leaves per-pose header stamps at 0 | ros_gz `Pose_V` conversion of SceneBroadcaster poses | Every sample is stamped with the node's `/clock` time |

## Known limitations

- **Idealised:** placeholder 100 N·m effort and 100 rad/s velocity; µ = 0.2; no joint damping; steel-density masses; DART with a 1 ms step. See [limitations](M2_SIMULATION_LIMITATIONS.md).
- **Not measured:** foot contact; disturbance rejection; balance.
- **Not tested:** holds longer than 10 s; `headless:=true`.
- **Local Ubuntu verification of M2 is still required.**
