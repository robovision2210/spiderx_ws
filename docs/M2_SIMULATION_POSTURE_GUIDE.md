# M2 Guide – Simulation-Only CAD Neutral Posture Hold (Gazebo Fortress)

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

Read with: the [plan](M2_SIMULATION_POSTURE_PLAN.md), [results](M2_TEST_RESULTS.md) and
[limitations](M2_SIMULATION_LIMITATIONS.md).

## What M2 adds beyond M1

| | M1 | M2 |
|---|---|---|
| Question | Does each joint go where it is commanded? | Does the robot reach **and stay in** the CAD neutral pose in simulation? |
| Measures | Final joint error once | Joint error over a 10 s hold (max and RMS), controller states during the hold, **body height, roll and pitch** from Gazebo ground truth |
| Pose | Any safe target | Only the existing `cad_neutral` pose (owner decision; no tuning) |
| Output | Terminal PASS/FAIL | Terminal summary, a JSON report, and exactly one of two outcome strings |

The only possible outcomes are **"Simulation posture hold verified."** and
**"Simulation posture hold not verified."**

## Command and feedback path

```
run_posture_hold_test.py ──FollowJointTrajectory──► leg_trajectory_controller (M1, unchanged)
                                                      └─► gz_ros2_control ─► Gazebo joints (DART)
feedback:  /joint_states            (joint_state_broadcaster, exactly 1 publisher)
           /controller_manager/list_controllers
           /clock                   (simulation time; every duration is simulated seconds)
           /spiderx/sim/world_poses (Gazebo ground-truth model pose; NOT /tf, NOT odometry)
```

`fortress_posture_hold.launch.py` is the M1 launch (`fortress_control.launch.py`, unchanged) plus
one bridge: the Gazebo topic `/world/spiderx_fortress/pose/info` is bridged to
`/spiderx/sim/world_poses`.

## Where every value comes from

`src/spiderx_controller/config/m2_simulation_postures.yaml` is the single source.

| Value | Kind |
|---|---|
| Joint names, axes, limits | From the URDF, via `spiderx_legs.yaml`, which is checked against the URDF |
| Targets (all 0.0 rad) | Existing config: must equal `spiderx_poses.yaml` `cad_neutral` (checked) |
| 0.05 rad safety margin | Existing M1 config |
| 6 s command duration, 0.05 rad max joint error | Existing M1 values |
| 10 s hold; RMS 0.02 rad; height ≥ 0.045 m; height range ≤ 0.005 m; \|roll\|, \|pitch\| ≤ 0.10 rad; ≥ 20 samples | Simulation-only thresholds, fixed before the first run |
| 100 N·m effort, µ = 0.2, no damping, steel-density masses | Placeholders and assumptions ([limitations](M2_SIMULATION_LIMITATIONS.md)) |
| Heights, angles and errors in the report | Measured at runtime |

## Setup and build (Ubuntu 22.04, ROS 2 Humble)

```bash
sudo apt install ros-humble-gz-ros2-control ros-humble-ros2-control ros-humble-ros2-controllers \
  ros-humble-ros2controlcli ros-humble-ros-gz-bridge ros-humble-tf2-msgs
cd ~/spiderx_ws
git fetch origin && git checkout claude/spiderx-m2-posture-hold && git pull
source /opt/ros/humble/setup.bash
rm -rf build install log && colcon build --symlink-install
source install/setup.bash
```

Expected: `Summary: 8 packages finished`.

## Run it manually

**Terminal 1** starts the simulation:

```bash
ros2 launch spiderx_bringup fortress_posture_hold.launch.py
```

Wait until the log shows both `Configured and activated joint_state_broadcaster` and
`Configured and activated leg_trajectory_controller`.

**Terminal 2** (after sourcing both setup files):

```bash
# optional: start away from the pose so the trajectory really moves joints (M1 tool)
ros2 run spiderx_controller test_one_joint.py --joint rr_hip --target 0.2
# the M2 test (takes about 1-2 minutes of wall time on a slow machine)
ros2 run spiderx_controller run_posture_hold_test.py
```

- The last lines print the outcome and the report path. The default report path is `~/.ros/spiderx_m2/posture_hold_report.json`; use `--output <file>` to choose another.
- Exit codes:
  - `0`: verified;
  - `1`: not verified, with every reason printed as `FAILED: ...`;
  - `2`: the configuration was refused, and nothing was sent.

## Automated validation

```bash
./scripts/validate_m2_posture.sh             # static: no simulator
./scripts/validate_m2_posture.sh --runtime   # launches, displaces 3 joints, runs the test, checks the report, shuts down
```

The runtime run copies its report to `log/m2_posture_hold/latest_report.json`.

## How to read the metrics

| Report field | Meaning | Pass if |
|---|---|---|
| `controller_checks` | Both controllers' states at start, every 2 s of the hold, and at the end | All `active` |
| `joint_state_publishers` | Publisher count on `/joint_states` at start and at the end | `[1, 1]` |
| `action.error_code` | Trajectory result (`0` = SUCCESSFUL) | 0 |
| `metrics.joints.max_abs_error_rad` | Largest \|target − measured\| over 12 joints × every hold sample | ≤ 0.05 |
| `metrics.joints.rms_error_rad` | RMS of the same errors | ≤ 0.02 |
| `metrics.body.height_m.min` | Lowest `base_link` origin height above the ground | ≥ 0.045 m |
| `metrics.body.height_m.range` | Max − min height during the hold (sag or bounce) | ≤ 0.005 m |
| `metrics.body.roll_rad.max_abs` | About `base_link` x (robot right), i.e. **nose up/down** | ≤ 0.10 |
| `metrics.body.pitch_rad.max_abs` | About `base_link` y (robot front), i.e. **side tilt** | ≤ 0.10 |
| `metrics.hold_duration_s` | Simulation seconds actually held | ≥ 10 |
| `joints_moved_more_than_0_01_rad` | Joints the trajectory actually moved | Informational |
| `xy_drift_m`, `yaw_drift_rad`, `tilt_rad_max` | Sliding, turning and total tilt during the hold | Informational |
| `geometric_indicator_height_minus_cad_foot_plane_m` | Final height − 0.0545 m (the CAD foot plane). Near 0 is consistent with the feet resting on the ground | Informational; **not a contact measurement** |
| `foot_contact` | Always `unavailable`: no contact sensor exists | – |

`roll` and `pitch` use the non-REP-103 `base_link` axes (+y front, +x right). That is why "roll"
is the nose-up/down angle here.

**Why the errors are so small.** Every joint may use up to the 100 N·m placeholder torque, so the
simulated joints are almost rigid. Tiny errors are expected and say nothing about the real servos.

## Stopping safely

- Press **Ctrl+C** in terminal 2. The test cancels its trajectory goal; nothing else commands the joints.
- Press **Ctrl+C** in terminal 1. The `gz_ros2_control` destructor stack trace at exit is known upstream behaviour.
- If anything keeps running: `pkill -f "ign gazebo"`, then check `pgrep -af "gazebo|parameter_bridge"`.

## Troubleshooting

| Symptom | Cause / fix |
|---|---|
| `REFUSED: invalid M2 posture configuration` (exit 2) | The YAML was edited or is malformed. Run `ros2 run spiderx_controller validate_controller_config` to see why. Nothing was sent |
| `no body pose for model "spiderx" on /spiderx/sim/world_poses` | You started `fortress_control.launch.py`; start `fortress_posture_hold.launch.py` instead, which adds the ground-truth bridge |
| `no /clock received` | The simulation is not running, or is paused (press play in Gazebo) |
| `controllers not active before the command` | Wait for both "Configured and activated" lines, then check `ros2 control list_controllers` |
| `/joint_states publisher counts [2, 2]` | A passive simulation is also running. Stop everything (`pkill -f "ign gazebo"`) and relaunch |
| `hold did not complete within the wall-clock limit` | The simulation is extremely slow or stalled. Check the real-time factor in the Gazebo window |
| `Failed to load system plugin [gz_ros2_control-system]` | `ros-humble-gz-ros2-control` is not installed (see [M1 guide](M1_JOINT_POSITION_CONTROL_GUIDE.md)) |
| Gazebo window stays grey, or spawn repeats "Requesting list of world names" | The Gazebo server did not start. Ctrl+C, run `pkill -f "ign gazebo"`, then relaunch |

## Why M2 comes before kinematics and gait

- Every later milestone assumes the robot can reach a known posture and stay there.
  - **IK (M3)** computes joint angles *relative to* a stance.
  - **A gait (M4)** is a sequence of postures.
- If the neutral pose cannot be held in simulation, no gait built on it can work.
- M2 also creates the reusable measurement tools (body height and tilt, tracking error, reports)
  that later milestones need, and it documents how far those measurements can be trusted.

## Send back after a local run

1. The last 25 lines of `run_posture_hold_test.py` output, from `SpiderX M2 posture hold` to `Report:`.
2. The `./scripts/validate_m2_posture.sh --runtime` summary lines.
3. Optionally, `log/m2_posture_hold/latest_report.json`.
4. A Gazebo-window screenshot showing no terminal.
