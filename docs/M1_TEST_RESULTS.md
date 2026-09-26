# M1 Test Results – Joint Position Control

**Scope:** joint-position control only. None of these results show standing, balance or walking.

## Environment

| Item | Version |
|---|---|
| Gazebo | Ignition Gazebo **6.16.0** (Fortress) |
| gz_ros2_control | **0.7.15** (Humble; links `libignition-gazebo6`) |
| controller_manager / ros2controlcli | 2.51.0 |
| joint_state_broadcaster / joint_trajectory_controller | 2.48.0 |
| control_msgs | 4.8.0 |
| ros_gz | 0.244.20 |
| Machine | Cloud VM, **no GPU**, Xvfb + Mesa software rendering, real-time factor ≈ 0.25 |
| Build | `colcon build --symlink-install`, 8 packages, with Ubuntu 22.04's setuptools 59.6 |

The packages are RoboStack builds of the same Humble packages that Ubuntu apt ships.
## Local verification (owner's Ubuntu 22.04 PC)

The owner ran the full 26-step sequence from the M1 test instructions and reported that **every step passed**:
- build (8 packages);
- `validate_m1_control.sh`, `validate_controller_config` and `colcon test`;
- `fortress_control.launch.py` with both controllers active and one `/joint_states` publisher;
- `test_one_joint.py` on `lf_hip`, including the return;
- both refusals: `lf_foot_joint` at 0.6 and `left_wheel_joint`;
- displacing `rr_hip`, then `test_neutral_pose.py`;
- `validate_m1_control.sh --runtime` and `validate_fortress.sh --runtime`.

Numeric values from the local run were not recorded in this file. The detailed numbers below are from the cloud run.

## Results

| # | Test | Result |
|---|---|---|
| 1 | `validate_controller_config` (12 joints, axes, limits, chain order, `<ros2_control>`: `gz_ros2_control/GazeboSimSystem`, command=position, state=position+velocity, plugin present, no second `/joint_states` source; passive description has no `<ros2_control>`) | ✅ valid |
| 2 | Unit tests: `spiderx_controller` 18 (config vs URDF, `<ros2_control>` checks, joint-safety rejections), `spiderx_scripts` 2 | ✅ 20/20 passed (`colcon test-result` prints `22 tests`: it also counts the 2 CTest wrapper entries) |
| 3 | Passive description unchanged (`sim_backend:=fortress` default, and `classic`/`none`) | ✅ byte-identical XML to the previous branch |
| 4 | `validate_fortress.sh` (static) and `--runtime` on the passive path | ✅ All checks passed |
| 5 | `validate_m1_control.sh` (static) | ✅ All M1 checks passed |
| 6 | Controllers after `fortress_control.launch.py` | ✅ `joint_state_broadcaster` **active**, `leg_trajectory_controller` **active** |
| 7 | Hardware interfaces | ✅ 12 `…/position` command interfaces **claimed**; 24 state interfaces (position + velocity) |
| 8 | `/joint_states` publishers in control mode | ✅ exactly **1** (joint_state_broadcaster), 12 joints |
| 9 | Unknown joint `left_wheel_joint` | ✅ **REFUSED** before sending (exit 2) |
| 10 | Out-of-limit target `lf_foot_joint` = 0.6 rad (allowed [−0.6481, 0.3863]) | ✅ **REFUSED** before sending (exit 2) |
| 11 | `test_one_joint.py --joint lf_hip --target 0.2 --return-to-initial` | ✅ before +0.0000 → after **+0.2000** rad, error 0.0000 ≤ 0.05, action `SUCCESSFUL`; the 11 held joints moved 0.0000; returned to +0.0000 |
| 12 | Displace `rr_hip`, `rf_thigh_joint`, `lr_foot_joint` → +0.2 rad (one at a time) | ✅ each reached +0.2000, `SUCCESSFUL` |
| 13 | `test_neutral_pose.py` (`cad_neutral`, all 12 joints, one 6 s trajectory) | ✅ 3/12 joints moved back (the displaced ones), max error **0.0000** rad, `SUCCESSFUL` |
| 14 | `validate_m1_control.sh --runtime` (tests 6–13 automated) | ✅ All M1 checks passed, 0 leftover processes |
| 15 | No hardware driver in any control launch | ✅ static grep and runtime node list |

## Observations (facts, not capability claims)

- **Position commands are physical, not kinematic.** The `gz_ros2_control` runtime log confirms `The position_proportional_gain has been set to: 0.1`. Position commands are therefore applied through the physics engine as velocity commands proportional to the error. They are **not** teleports.
- **Tracking error of 0.0000 rad is idealised.** The engine may use up to the URDF effort limit, which is the **100 N·m exporter placeholder** (audit K4), so the joints are far stiffer than real hobby servos. Expect larger errors once real servo limits are entered (M9).
- **The robot does not fold in control mode.** `gz_ros2_control` holds every joint at its initial value (0 rad) from the moment it loads. The model therefore rests on its feet: root z = 0.054 m with no tilt, instead of folding to z ≈ 0.012 m as in passive mode. This is joint position holding with stiff placeholder limits. **It is not a standing controller**: there is no balance feedback and no disturbance test.
- **Displaced joints affect the body.** With `rr_hip`, `rf_thigh_joint` and `lr_foot_joint` at +0.2 rad the body pitched 0.137 rad and rose to z = 0.068 m. After `cad_neutral` it returned to z = 0.0545 m with a 0.035 rad yaw drift.

## Problems found and fixed during validation

| Problem | Cause | Fix |
|---|---|---|
| `Failed to load system plugin [gz_ros2_control-system]` | `gz_sim.launch.py` searches `LD_LIBRARY_PATH`, which was empty in the cloud environment. It is set on Ubuntu after sourcing ROS | `fortress_control.launch.py` appends the `gz_ros2_control` `lib/` directory to `IGN_GAZEBO_SYSTEM_PLUGIN_PATH` and `GZ_SIM_SYSTEM_PLUGIN_PATH` explicitly |
| First single-joint run reported FAIL (0.086 of 0.2 rad) | The test tool waited on the wall clock while the simulation ran at about 25 % real time, so it measured mid-motion | Test nodes now use `/clock` (`use_sim_time`), wait for the action result, and settle in simulation time |
| First neutral-pose run "passed" without moving anything | All joints were already at 0 | The validation displaces 3 joints first. The tool reports how many joints moved and warns when none did |
| **Intermittent controller start-up race** (seen once on 2026-09-26 during M3 work, cloud VM, real-time factor ≈ 0.25; every earlier M1/M2 run was fine). The controller manager logged `Switch controller timed out after 5.000000 seconds!`, the `joint_state_broadcaster` spawner logged `Failed to activate controller` and exited with code 1, and the `leg_trajectory_controller` spawner was **still started** and activated. Result: `joint_state_broadcaster` **inactive**, `leg_trajectory_controller` **active**, no `/joint_states` messages | (1) The Humble spawner waits for an activation for `--switch-timeout`, which defaults to 5 s of **wall** time. The controller manager applies the switch only in its update loop, which `gz_ros2_control` drives from simulation time, and that is slow while the simulation is still starting. (2) `OnProcessExit` fires on **every** exit code, including failure | See [Controller start-up race fix](#controller-start-up-race-fix-2026-09-26) below |

## Controller start-up race fix (2026-09-26)

Changes, all in `spiderx_controller/launch/controller.launch.py`. Controller names, the controller
YAML, joint limits, the safety tools and the success path are unchanged.

- **Longer timeouts on both spawners (simulation-only tuning):** `--switch-timeout 60` and
  `--service-call-timeout 75`. The call timeout must be longer than the switch timeout. Otherwise
  the spawner re-sends the pending switch request after its default 10 s call timeout, and the
  controller manager rejects the duplicate request as "already active". The sources show this
  spawner behaviour in both controller_manager 2.51.0 (cloud VM) and 2.54.2 (local PC).
- **The chain stops on failure:** the `leg_trajectory_controller` spawner starts only when the
  `joint_state_broadcaster` spawner exits with code 0. If a spawner fails, the launch logs
  `[ERROR] [spiderx_controller]: spawner for <controller> failed (exit code N) … The controllers are
  not usable: stop this launch (Ctrl-C) and start it again.` and starts nothing else. Nothing is
  reported during a normal Ctrl-C teardown.
- **Why the launch is not shut down automatically:** this was tried with `EmitEvent(Shutdown)`.
  After the launch shut itself down, `ign gazebo server` and `ign gazebo gui` kept running as
  orphans, and the server kept simulating. They run in their own process groups, and a
  launch-initiated shutdown signals only the `ign gazebo` launcher process. Ctrl-C and the
  validation scripts send SIGINT to the whole launch process group, which stops them cleanly.

Verification, run by Claude Code on the owner's Ubuntu 22.04 PC (Mesa llvmpipe software
rendering; Ignition Gazebo 6.18.0, controller_manager 2.54.2, gz_ros2_control 0.7.21):

| Check | Result |
|---|---|
| `colcon test` (whole workspace), including the new `test_controller_launch.py` (the timeouts are passed; on exit code 0 the second spawner starts; on exit code 1 it does not start and one error is logged) | ✅ 90 tests, 0 errors, 0 failures (85 pytest cases + 5 CTest wrappers). A copy of the launch file that starts the next spawner on any exit code fails the new test |
| `./scripts/validate_m1_control.sh --runtime` | ✅ All M1 checks passed (24/24) |
| `./scripts/validate_m2_posture.sh --runtime` | ✅ All M2 checks passed (29/29: "Simulation posture hold verified.", clean shutdown). In one run started from a wrapper shell, the leftover-process check matched that shell, whose command line contained the check's search strings. A standalone re-run passed |
| `./scripts/validate_fortress.sh --runtime` (passive path) | ✅ All checks passed (67/67) |
| Success-path order in the launch log (final M1 run) | ✅ The `joint_state_broadcaster` spawner finished cleanly 4.5 s after launch. The `leg_trajectory_controller` spawner started in the same instant and finished cleanly at 9.5 s |
| Forced failure: `fortress_control.launch.py` with the switch timeout temporarily set to 0.001 s (restored afterwards) | ✅ `Switch controller timed out after 0.001000 seconds!`, the broadcaster spawner exited with code 1, the error line above was logged and `leg_trajectory_controller` was **not loaded**. `list_controllers` showed only `joint_state_broadcaster inactive`. Group SIGINT (Ctrl-C) then stopped everything with no leftover processes |

## Known limitations (still open)

- **Shutdown stack trace.** On Ctrl-C, the `gz_ros2_control` plugin prints a stack trace from its destructor (`GazeboSimROS2ControlPlugin::~GazeboSimROS2ControlPlugin`). This is upstream behaviour at shutdown, after all tests have finished, and it does not affect operation.
- **Transient startup error.** `[gz_ros2_control]: robot_state_publisher service not available, waiting again...` is logged until `robot_state_publisher` is up.
- **Untested modes.** `headless:=true` is not tested (the cloud ogre-next build lacks EGL).
- **Not implemented:** standing stabiliser, gait, `/cmd_vel` locomotion, odometry, Nav2 movement and a real servo driver.
