# M4.1 Test Results – Controller Start-up Robustness (Gazebo Fortress, simulation only)

```text
Controller start-up ordering and robustness only, in simulation.
No gait, walking, balance control, navigation or hardware change.
```

## Outcome (cloud + local verified)

- **Ordering.** `leg_trajectory_controller` is started only after the `joint_state_broadcaster`
  spawner exited with code 0, which means the broadcaster was configured and activated.
- **Failure.** If the broadcaster spawner fails or is killed, exactly one error line is logged and
  the trajectory controller is never started.
- **Shutdown.** While the launch is shutting down (Ctrl+C), no further spawner is started.
- **Evidence.**
  - 72 new unit tests, backed by a mutation check;
  - a forced broadcaster failure in Gazebo;
  - Ctrl+C during start-up;
  - the success-path order;
  - the M4, M3, M2, M1 and Fortress runtime regressions.
- **Verified in the cloud and on the owner's Ubuntu PC** (`35b83b5`). See
  [Local verification](#local-verification-owners-ubuntu-pc--passed).

## Environment

| Item | Value |
|---|---|
| Branch | `claude/spiderx-controller-spawn-race-v2`, from `main` @ `2f03b3c` (M4 merged) |
| Implementation commits | `82178a0`: launch gate, spawner options and launch tests. `fbd2d2d`: M1/M2 validator state check and tests |
| controller_manager | **2.51.0**, together with controller_manager_msgs 2.51.0 and ros2controlcli 2.51.0 (RoboStack `ros-humble-*`) |
| Other packages | launch 1.0.9, launch_ros 0.19.10, rclpy 3.3.16, gz_ros2_control 0.7.15, ros_gz 0.244.20, joint_state_broadcaster and joint_trajectory_controller 2.48.0, Ignition Gazebo 6.16.0 |
| Machine (cloud) | Cloud VM, no GPU, Xvfb and Mesa software rendering |
| Machine (local) | The owner's Ubuntu PC. See [Local verification](#local-verification-owners-ubuntu-pc--passed) |

## What changed

**`src/spiderx_controller/launch/controller.launch.py`.** Each spawner's `OnProcessExit` handler is
now a callable gate rather than a static action list:

| Condition | What the gate does |
|---|---|
| Launch shutting down | Nothing starts and nothing is logged. This is checked **first**, because a spawner interrupted by Ctrl+C can exit with 0 |
| Exit code 0 | Starts the next spawner |
| Any other exit, including a signal | Logs one error line and starts nothing |

The error line for a broadcaster failure is:

```text
[ERROR] [spiderx_controller]: joint_state_broadcaster startup failed; leg_trajectory_controller
was not started; press Ctrl+C and relaunch. (spawner exit code 1; see its output above)
```

**Spawner options.** Both spawners get two new simulation-only options, `--switch-timeout 60` and
`--service-call-timeout 75`. `--controller-manager-timeout 120` is unchanged.

**No automatic shutdown on failure.** Ctrl+C stays the single shutdown path (see
[Findings](#findings)).

**Validators.** `scripts/validate_m1_control.sh` and `scripts/validate_m2_posture.sh` changed by one
line each: the check went from `grep -qE "^$c .*active"` to `"^$c .* active"`, the pattern the M3
and M4 validators already use. No other validator line changed.

**Tests.** Two new test files, `test/test_controller_launch.py` (28 cases) and
`test/test_validator_state_checks.py` (44 cases), plus two lines in `CMakeLists.txt`.

**Unchanged:**
- the URDF, meshes and controller YAML;
- `fortress.launch.py` and `fortress_control.launch.py`;
- the kinematics, the M4 pose code, and every M1–M4 tool.

## Switch-timeout semantics (read from the installed controller_manager 2.51.0)

- **Spawner defaults** (`controller_manager/spawner.py:129-151`):

  | Option | Default |
  |---|---|
  | `--controller-manager-timeout` | 0.0 s |
  | `--switch-timeout` | 5.0 s |
  | `--service-call-timeout` | 10.0 s |

- **How the spawner activates a controller** (`spawner.py:256-270`). It calls
  `switch_controllers(..., strict=True, activate_asap=True, switch_timeout, service_call_timeout)`.
  If that fails, it logs `Failed to activate controller : <name>` and returns **1**. It returns 0
  only after `Configured and activated <name>`, or when interrupted: `KeyboardInterrupt` is caught
  (`spawner.py:355-356`), and the entry script runs `sys.exit(main())`.
- **The request** (`controller_manager_services.py:273-297`). `switch_controllers` sends
  `SwitchController.Request.timeout = switch_timeout`. `service_caller` then waits
  `call_timeout` for each answer and **re-sends the request up to 3 times**
  (`controller_manager_services.py:87-152`). So the call timeout must be longer than the switch
  timeout. Otherwise a slow but successful switch is requested again, and the strict duplicate is
  rejected (`… since it is already active`, a string in `libcontroller_manager.so`).
- **Where the switch happens.** The installed `controller_manager_msgs/srv/SwitchController.srv`
  says the switch happens "all in one single timestep of the controller manager's control loop".
  The timeout field is "the timeout before aborting pending controllers".
  - `controller_manager.hpp:518-541` shows the service waiting on a condition variable with a
    `std::chrono` (wall-clock) timeout.
  - `gz_ros2_control` 0.7.15 is a Gazebo system with `PreUpdate`/`PostUpdate` callbacks
    (`gz_ros2_control_plugin.hpp:35-58`). The controller manager is updated from them, that is,
    from the simulation step. That last part is the upstream design; the installed header declares
    the callbacks but does not show the update call.
- **Confirmed at runtime.** With the switch timeout forced to 0.001 s, the controller manager logged
  `Switch controller timed out after 0.001000 seconds!` (see R2). It uses exactly the value the
  spawner sends.

## Unit tests

| Check | Result |
|---|---|
| Clean `colcon build --symlink-install` | ✅ 8 packages finished in 18.7 s; no warnings or errors |
| `colcon test` + `colcon test-result --all` | ✅ **385 tests, 0 errors, 0 failures, 0 skipped**: 374 pytest cases + 11 CTest wrappers. That is M4's 311, plus 28 + 44 new cases and 2 new wrappers |
| `test_controller_launch.py` | ✅ 28 passed |
| `test_validator_state_checks.py` | ✅ 44 passed |

**What `test_controller_launch.py` covers.** It needs no Gazebo and no controller manager.
- **Arguments.** The exact spawner arguments for both controllers; the call timeout is longer than
  the switch timeout; the installed spawner's `--help` lists all three options.
- **The real launch description.** Only the broadcaster spawner is started directly, and there are
  two exit handlers. A `ProcessExited` event was delivered to them, as `launch` would:

  | Exit event | Result |
  |---|---|
  | Broadcaster exits 0 | Returns the trajectory spawner; no error |
  | Broadcaster exits 1, 2, −15 or −9 | Returns nothing; exactly one error that starts with the specified sentence and names the exit code |
  | Any exit during shutdown (0, 1, −2, −15, both spawners) | Returns nothing; no error |
  | Trajectory spawner exits 0 | Returns nothing |
  | Trajectory spawner fails | One error line |

- **A real `LaunchService` with stand-in processes:**
  - exit 0 starts the next process;
  - exit 1, exit 2 and death by SIGTERM do not, and log one error each;
  - a shutdown while the first process runs (it exits 0 on SIGINT, like the spawner) starts nothing
    and logs nothing.

**Mutation check.** Scratch copies of the launch file were run against the same test file. They are
not committed.

| Mutant | Result |
|---|---|
| A: the original `main` file (unconditional start on any exit, no timeouts) | 8 failed, 19 errors, 1 passed (the one that passed is the installed-spawner `--help` check, which does not read the launch file) |
| B: the gate kept, but the broadcaster handler reverted to the old unconditional action list | 8 failed (all broadcaster-failure and broadcaster-shutdown cases) |
| C: exit-code check removed | 11 failed |
| D: shutdown guard removed | 7 failed (the shutdown cases) |
| E: the new timeouts removed | 2 failed (argument tests) |
| F: error logged twice | 11 failed |
| Unmutated control | 28 passed |

For mutant D, the end-to-end shutdown test alone still passes, because `launch` itself refuses to
start a process once shutdown has begun (`launch/actions/execute_local.py:633-635`). The explicit
guard is caught by the direct handler tests. The guard also keeps a killed spawner from logging a
misleading error during Ctrl+C.

**What `test_validator_state_checks.py` covers.**
- **Method.** It runs each validator's own check block in bash, the `ctrls=$(ros2 control
  list_controllers …)` line plus the loop. `ros2` is replaced by a function that prints listings in
  ros2controlcli's layout, colours included.
- **Cases**, for all four validators:
  - both controllers `active` pass;
  - the race state (broadcaster `inactive`, trajectory controller `active`) reports the broadcaster
    as not active;
  - these states never pass: `inactive`, `inactive_pending`, `unconfigured`, `finalized`,
    `reactive`, `xactive`, `notactive`.
- **Against the previous M1/M2 scripts:** 14 failures, all of them M1 or M2 accepting `inactive` or
  another value with characters before `active`. After the fix: 44 passed.

## Runtime (cloud)

### Test setup

**Where the logs are.** Every launch log of these checks is kept, git-ignored, under
`log/m4_1_controller_startup/`. Each check has two logs:
- `*_launch.log`, the screen output with node timestamps;
- `*_ros_launch.log`, the launch's own timestamped log from `~/.ros/log/`.

**How Ctrl+C was reproduced.** The M4.1 checks send SIGINT to the launch's process group, which is
what Ctrl+C in a terminal does. The checks were run in two modes (see [Findings](#findings), item 4):
- **Interactive-equivalent (main runs).** `ros2 launch` starts with SIGINT at its default
  disposition, as in a terminal.
- **Validator mode (`*.sigint_ignored_harness.log`).** `setsid ros2 launch … &` from a
  non-interactive script, which is how the validators start it. Here SIGINT is inherited as ignored
  by the Python processes.

### R1 – success path: order of the two spawners

**Command.** `ros2 launch spiderx_bringup fortress_control.launch.py`.

This timeline merges the node log timestamps with the launch's own timestamps:

```text
 0.000 s  launch: spawner-5 (joint_state_broadcaster) process started
 2.766 s  [controller_manager] Loading controller 'joint_state_broadcaster'
 2.793 s  [controller_manager] Configuring controller 'joint_state_broadcaster'
 4.736 s  [spawner_joint_state_broadcaster] Configured and activated joint_state_broadcaster
 4.889 s  launch: spawner-5 process has finished cleanly            (exit code 0)
 4.899 s  launch: spawner-6 (leg_trajectory_controller) process started
 5.485 s  [controller_manager] Loading controller 'leg_trajectory_controller'
 5.568 s  [controller_manager] Configuring controller 'leg_trajectory_controller'
 9.215 s  [spawner_leg_trajectory_controller] Configured and activated leg_trajectory_controller
 9.406 s  launch: spawner-6 process has finished cleanly
```

**Order checks.** All four passed:
1. the broadcaster was activated before its spawner exited;
2. the spawner exited 0 before the trajectory spawner started, which it did 10 ms later;
3. the trajectory spawner started before the trajectory controller was loaded;
4. the trajectory controller was activated last.

**State after start-up.** Both controllers were `active`, with one `/joint_states` publisher.

**Shutdown.** After SIGINT to the process group, the whole group exited in 2 s, with no leftover
processes.

**The validator-mode run.** It gave the same order. There, the broadcaster took **8.94 s** from
`Configuring` to `Configured and activated`, longer than the old 5 s switch timeout. So with the
previous spawner defaults, that start-up would most likely have failed. Between the first `gz_ros2_control`
update (+0.19 s after `Configuring`) and the activation, the log shows only the Gazebo GUI loading.
That is consistent with a stalled simulation loop, which is the D2 hypothesis in the
[plan](M4_1_PLAN.md). This is an inference from the log, not a measurement of the loop.

### R2 – forced broadcaster failure

**How it was forced.** An untracked scratch launch runs the real
`spiderx_bringup/fortress_control.launch.py` unchanged. Only when `launch` loads the installed
`controller.launch.py` is `SWITCH_TIMEOUT_S` set to `'0.001'`, in memory. No tracked or installed
file was edited.

```text
[controller_manager]: Loading controller 'joint_state_broadcaster'
[controller_manager]: Configuring controller 'joint_state_broadcaster'
[controller_manager]: Switch controller timed out after 0.001000 seconds!
[spawner_joint_state_broadcaster]: Failed to activate controller : joint_state_broadcaster
[ERROR] [spawner-5]: process has died [pid 6379, exit code 1, cmd '.../spawner joint_state_broadcaster
  --controller-manager /controller_manager --controller-manager-timeout 120 --switch-timeout 0.001
  --service-call-timeout 75 --ros-args -r __node:=spawner_joint_state_broadcaster'].
[ERROR] [spiderx_controller]: joint_state_broadcaster startup failed; leg_trajectory_controller was
  not started; press Ctrl+C and relaunch. (spawner exit code 1; see its output above)
```

**30 s after the error:**
- `ros2 control list_controllers` listed only `joint_state_broadcaster … inactive`;
- `leg_trajectory_controller` was **never loaded**, and its spawner **never started**;
- there was exactly **1** M4.1 error line.

`/joint_states` still had one publisher, belonging to the inactive broadcaster. So a publisher count
alone does not reveal this state.

**Shutdown.** After SIGINT to the process group, the group exited in 2 s. No Gazebo (server, GUI or
Ruby launcher), controller manager, spawner, bridge or `robot_state_publisher` process remained.

Both modes gave the same result. The logs are `r2_forced_failure_launch.log` and its
`_ros_launch.log`, plus the validator-mode variants.

### R3 – Ctrl+C during start-up

SIGINT went to the launch process group about 1 s after the broadcaster spawner started, while it
was still waiting for `/controller_manager/list_controllers`.

| Mode | What happened | M4.1 error line | Trajectory spawner | Group exit | Leftovers |
|---|---|---|---|---|---|
| Interactive-equivalent | `launch`: `user interrupted with ctrl-c (SIGINT)`. The spawner died with exit code −2 | none | never started | 2 s | none |
| Validator mode | The spawner ignored SIGINT. `launch` stopped it with SIGTERM after its 5 s grace (exit code −15); Gazebo, a `required` process, had already exited on SIGINT | none | never started | 7 s | none |

### R3c – SIGINT to the `ros2 launch` process only: the exit-0 case

`launch` forwarded a single SIGINT to the waiting broadcaster spawner. The spawner caught it and
exited with **code 0** (`process has finished cleanly`) while the launch was shutting down. The gate
started nothing and logged nothing, which is the case its explicit shutdown check exists for.

This launch-only shutdown also left `ign gazebo server` and `ign gazebo gui` running (see
[Findings](#findings), item 3). They were stopped by hand, and each exited within 3 s of a SIGINT.

### Regressions (M0–M4)

Each validator ran in its own simulation, 30 s apart. The logs are in
`log/m4_1_controller_startup/regression/`.

| Check | Result | Final line |
|---|---|---|
| `validate_m4_all_leg_ik.sh` (static) | ✅ exit 0 (13 s) | `All M4 checks passed.` (136 M4 unit tests) |
| `validate_m4_all_leg_ik.sh --runtime` | First run ❌: the Gazebo server hung at start-up and the robot was never spawned ([below](#gazebo-server-start-up-hang-not-caused-by-m41)). **Rerun ✅, exit 0** | `All M4 checks passed.` |
| `validate_m3_kinematics.sh --runtime` | ✅ exit 0 (219 s) | `All M3 checks passed.` |
| `validate_m2_posture.sh --runtime` | ✅ exit 0 (130 s) | `All M2 checks passed.` |
| `validate_m1_control.sh --runtime` | ✅ exit 0 (130 s) | `All M1 checks passed.` |
| `validate_fortress.sh --runtime` (passive M0 path) | ✅ exit 0 (38 s) | `All checks passed.` |
| Leftover simulation or controller processes after each | ✅ none | |

**M4 rerun.** It printed all three M4 outcome lines:
- "All-leg forward kinematics verified for the current URDF/TF/Gazebo model."
- "All-leg inverse kinematics verified for documented, joint-safe, simulation-only static poses."
- "Static multi-leg pose hold via IK validated in Gazebo (no walking, no gait, no hardware)."

It also passed its own checks:
- all trajectory goals ended with `error_code` 0;
- `neutral_stance`, `crouch_10mm` and `lift_lf_15mm` were held for 5 s, with 1344, 882 and 1040
  samples;
- FK vs TF was at most 2.8e-11 m and FK vs Gazebo at most 9.4e-07 m;
- both negative poses were refused and never commanded.

**M2 and M1 under the stricter check.** Both printed `[PASS] joint_state_broadcaster active` and
`[PASS] leg_trajectory_controller active`. M2 printed "Simulation posture hold verified.", and M1's
single-joint and `cad_neutral` moves passed.

**Successful start-ups.** Seven controller start-ups in this session reached both controllers
`active`: the two R1 runs, the M4 rerun, M3, M2, M1, and the state probe (Findings item 2). In all
seven the order was correct and the broadcaster activated, with no race.

### Gazebo server start-up hang (not caused by M4.1)

In 3 of the 13 Gazebo launches that ran past start-up, the robot never appeared: the first R1
attempt, the first M4 runtime run, and the first state-probe run.

**What was observed:**
- `ros_gz_sim create` repeated `Requesting list of world names.` every 5 s. The server never
  offered its world service.
- A `gdb` backtrace of the hung `ign gazebo server` (kept, git-ignored, in
  `log/m4_1_controller_startup/gazebo_server_startup_hang_backtrace.txt`) shows:
  - the main thread in `runServer()` (`libignition-gazebo6-ign.so.6.16.0`), blocked in
    `pthread_cond_wait` **without a timeout**;
  - no simulation threads at all, only ign-transport and ZMQ threads.

**Why the server waits.** In GUI+server mode, the installed Fortress launcher sets `'wait_gui' => 1`
unless `-s` or `-g` is given (`lib/ruby/ignition/cmdgazebo6.rb`). The server then waits for the GUI
to publish the starting world on `/gazebo/starting_world`; the library prints "Waiting for a world
to be set from the GUI...". If that handshake message is missed, the server waits forever.

**Why this is not M4.1.** It happens in Gazebo's own start-up, before any controller or spawner
matters, and M4.1 does not touch it. In all three cases the M4.1 gate behaved as designed:
1. the broadcaster spawner gave up after its 120 s `--controller-manager-timeout`, with
   `Could not contact service /controller_manager/list_controllers` and exit code 1;
2. exactly one M4.1 error line was logged;
3. the trajectory spawner was not started.

A process-group SIGINT then stopped everything, with no leftovers. The Ruby launcher SIGKILLs a
server that does not react to SIGINT within 5 s.

**Earlier runs.** The earlier M1–M4 results do not report this hang, and the owner's local M4.1
runs had none either (zero retries). Its rate here may be specific to this cloud VM. The logs are `r1_attempt_cm_never_available.log` and
`regression/m4_runtime_first_run_launch_log.txt`.

### Excluded run

One probe command of mine was wrong. It briefly started a second simulation while another probe
launch was running, and the two shared the `/controller_manager` name for about 40 s. Both were
stopped, with no leftovers, and nothing from that window is used in this report.

## Findings

1. **Activation can take longer than the old 5 s default.**
   - The validator-mode R1 run needed 8.94 s from `Configuring` to `Configured and activated` for
     `joint_state_broadcaster`. This matches the race's recorded trigger
     (`Switch controller timed out after 5.000000 seconds!`).
   - With `--switch-timeout 60` that start-up succeeded.
   - The race is intermittent, so passing runs are evidence, not proof, that a stall never exceeds
     60 s. What M4.1 guarantees is that any failure is **safe and reported**: the trajectory
     controller is never started without an active broadcaster.
2. **The trajectory controller is briefly `inactive` during start-up, but `list_controllers`
   rarely shows it.**
   - **Timing.** Both R1 runs took 3.54 s and 3.65 s from `Configuring` to
     `Configured and activated` for `leg_trajectory_controller`.
   - **Measurement.** A scratch probe polled `/controller_manager/list_controllers` continuously
     during a start-up and got 219 answers:
     - `inactive` appeared in **one** answer only, in a gap of about 1 s after configuration;
     - a call made while the activation was pending was answered only once it completed: it took
       2.47 s and returned `active`.

     This fits the controller manager answering its service calls one at a time
     (`controller_manager.hpp:408` has a single `best_effort_callback_group_`).
   - **Where the validators could still trip.** All four wait with
     `grep -q "leg_trajectory_controller.*active"`, which also matches `inactive`. M3 and M4 then
     wait 3 s before their strict check; M1 and M2 check at once.
   - **Residual risk of the stricter M1/M2 check.** A false `[FAIL] leg_trajectory_controller not
     active` needs both the wait loop's call and the check's call, about 1 s apart, to be answered
     inside that gap. That is possible, but unlikely. M1 and M2 passed in the cloud and locally,
     both printing the strict `[PASS] … active` lines.
   - **Not done here.** Removing the risk entirely would mean making the wait loops use `.* active`,
     or adding M3/M4's 3 s pause. That is outside the approved change.
3. **A shutdown driven by `launch` alone can orphan the Gazebo server and GUI.** This is pre-existing
   and not changed by M4.1.
   - **Setup.** `ros_gz_sim` starts Gazebo as `/bin/sh -c ruby …/ign gazebo …`. The Ruby launcher
     runs `ign gazebo server` and `ign gazebo gui` in **their own process groups**.
   - **What goes wrong.** When only `launch` initiates the shutdown (R3c), it signals the shell,
     which does not pass SIGINT on. `launch` then escalates to SIGTERM, and the Ruby launcher, server
     and GUI keep running.
   - **Why Ctrl+C is fine.** Ctrl+C in the terminal sends SIGINT to the whole foreground process
     group, which reaches the Ruby launcher directly. It stops the server and GUI, and every
     group-SIGINT run in this report ended with no leftovers.
   - **Consequence for M4.1.** This is why M4.1 does not shut the launch down on failure. Fixing it
     belongs in `fortress.launch.py` or `ros_gz_sim`, as separate maintenance work.
4. **The Gazebo Fortress GUI↔server start-up handshake can hang.** This is upstream behaviour and
   not changed by M4.1; see
   [Gazebo server start-up hang](#gazebo-server-start-up-hang-not-caused-by-m41).
   - In GUI+server mode the server waits, with no timeout, for the GUI's `/gazebo/starting_world`
     message. When that message is missed, no robot and no controller manager appear.
   - M4.1 turns this into one clear error line after 120 s, instead of starting a trajectory
     spawner that would also wait and fail.
   - Server-only mode (`-s`, used by `headless:=true`) does not wait for the GUI. Headless rendering
     is still untested on this VM (`STATUS.md`).
   - On the owner's Ubuntu PC no such hang occurred, and no retries were needed. This remains a
     follow-up item outside M4.1.
5. **Harness note: SIGINT inherited as ignored.**
   - **Cause.** A launch started as a background job of a non-interactive shell inherits SIGINT as
     ignored, and Python then keeps it ignored. The validators start their launch exactly this way,
     so there `ros2 launch` and the spawners ignore SIGINT.
   - **Why shutdown still completes.** The C++ nodes and the Ruby Gazebo launcher handle SIGINT;
     `launch` shuts down when Gazebo (`required`) exits; and a spawner that ignores SIGINT is stopped
     by `launch`'s SIGTERM after 5 s.
   - **How this report handled it.** The M4.1 checks were run in both modes, and the
     interactive-equivalent mode is what a user's Ctrl+C does.
6. **Residual case.** A SIGINT delivered to the broadcaster spawner *alone*, with no launch shutdown,
   makes it exit 0 without activating, so the gate would start the trajectory spawner. Neither Ctrl+C
   nor the validators can produce this, because both also signal `ros2 launch`. The downstream tools
   still refuse unless both controllers are `active`.

## Local verification (owner's Ubuntu PC) – passed

The owner reported these facts; they are recorded here exactly as given. All numbers in this
section are **local**.

| Item | Local result |
|---|---|
| Branch and commit | `claude/spiderx-controller-spawn-race-v2` @ `35b83b5`, identical to `origin` |
| Working tree | Clean at start and finish; no tracked files modified |
| Hardware | None started |
| Build | Clean build: 8 packages finished in 9.45 s; no warnings or errors |
| Tests | **385 tests, 0 errors, 0 failures, 0 skipped**, including the 28 `test_controller_launch` and 44 `test_validator_state_checks` tests |
| Environment hangs | None; zero retries needed |
| Leftover processes | None after any run |

### R1 – success path (local)

- The broadcaster spawner exited 0 at **+3.637 s**, and the trajectory spawner started at
  **+3.641 s**, 4 ms later.
- Both controllers were `active`, with exactly one `/joint_states` publisher.
- The broadcaster activation took **0.59 s**, well under the old 5 s default.
- After Ctrl+C, the group exited in 1 s, with no leftovers.

### R2 – forced broadcaster failure (local)

This used the 0.001 s switch timeout and the untracked scratch launch. The scratch launch was
byte-identical to the [documented source](#how-to-rerun-these-checks), lived only outside the
repository, and was deleted afterwards.

- `Switch controller timed out after 0.001000 seconds!`
- The broadcaster spawner exited with code 1.
- Exactly one error line was logged: `joint_state_broadcaster startup failed; leg_trajectory_controller
  was not started; press Ctrl+C and relaunch. (spawner exit code 1; …)`.
- 30 s later, only `joint_state_broadcaster inactive` was listed; `leg_trajectory_controller` was
  never loaded.
- After Ctrl+C, the launch exited cleanly in 1 s, with no leftovers.

### R3 – Ctrl+C during start-up (local)

- The broadcaster spawner was still waiting for `/controller_manager`.
- No `startup failed` line was logged, and no trajectory spawner was started.
- `launch` escalated to SIGTERM after its 5 s grace. The group exited in 12 s, and no Gazebo server
  or GUI was orphaned.

This is pre-existing shutdown behaviour. Its timing differs from the cloud's interactive-equivalent
run (2 s), but it is not a defect. The cloud's validator-mode run showed the same SIGTERM escalation
after the 5 s grace (7 s).

### Regressions (local, final lines)

| Validator | Final line (local) |
|---|---|
| `validate_m4_all_leg_ik.sh` | `All M4 checks passed.` |
| `validate_m3_kinematics.sh` | `All M3 checks passed.` |
| `validate_m2_posture.sh` | `All M2 checks passed.` (29 PASS, 0 FAIL) |
| `validate_m1_control.sh` | `All M1 checks passed.` (24 PASS, 0 FAIL) |
| `validate_fortress.sh` | `All checks passed.` (67 PASS, 0 FAIL) |

M1 and M2 printed the strict `[PASS] … active` lines from the new grep pattern.

### Cloud and local side by side

| Item | Cloud | Local |
|---|---|---|
| Clean build | 8 packages, 18.7 s | 8 packages, 9.45 s |
| Tests | 385, 0 failures | 385, 0 failures |
| R1: broadcaster spawner exit → trajectory spawner start | 10 ms | 4 ms |
| R1: broadcaster activation | 1.94 s (8.94 s in the validator-mode run) | 0.59 s |
| R1: group exit after Ctrl+C | 2 s | 1 s |
| R2: forced failure | 1 error line, trajectory controller never loaded, exit in 2 s | 1 error line, trajectory controller never loaded, exit in 1 s |
| R3: group exit after Ctrl+C during start-up | 2 s (7 s in validator mode) | 12 s (SIGTERM after the 5 s grace) |
| Gazebo start-up hangs | 3 of 13 launches | none |
| Regressions M4/M3/M2/M1/Fortress | all passed (M4 on rerun) | all passed |
| Leftover processes | none | none |

### How to rerun these checks

These are the commands the local verification followed:

```bash
cd ~/spiderx_ws && git fetch origin && git checkout claude/spiderx-controller-spawn-race-v2 && git pull
rm -rf build install && colcon build --symlink-install && source install/setup.bash
colcon test && colcon test-result --all      # expected: 385 tests, 0 errors, 0 failures, 0 skipped
./scripts/validate_m4_all_leg_ik.sh && ./scripts/validate_m4_all_leg_ik.sh --runtime
./scripts/validate_m3_kinematics.sh --runtime
./scripts/validate_m2_posture.sh --runtime
./scripts/validate_m1_control.sh --runtime
./scripts/validate_fortress.sh --runtime
```

**Manual start-up check.** Run `ros2 launch spiderx_bringup fortress_control.launch.py`, then:
1. Check that `Configured and activated joint_state_broadcaster` appears **before**
   `[spawner-6]: process started`, and that `Configured and activated leg_trajectory_controller`
   follows.
2. Press Ctrl+C. Nothing should remain in `pgrep -af "ign gazebo|spawner|parameter_bridge|robot_state_publisher"`.
3. Launch again and press Ctrl+C within about 2 s. There should be no `startup failed` line, no
   `spawner_leg_trajectory_controller`, and no leftovers.

**Optional forced failure.** Save this as `/tmp/m41_forced_failure.launch.py`, outside the
repository, and run `ros2 launch /tmp/m41_forced_failure.launch.py`. Expected result: the M4.1
error line once, `leg_trajectory_controller` never loaded, and a clean stop with Ctrl+C.

```python
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import python_launch_file_utilities as loader
from launch.launch_description_sources import PythonLaunchDescriptionSource

TARGET = os.path.realpath(os.path.join(get_package_share_directory('spiderx_controller'),
                                       'launch', 'controller.launch.py'))
_load = loader.load_python_launch_file_as_module


def _forced(path):  # in memory only: the broadcaster activation times out on purpose
    module = _load(path)
    if os.path.realpath(path) == TARGET:
        module.SWITCH_TIMEOUT_S = '0.001'
    return module


loader.load_python_launch_file_as_module = _forced


def generate_launch_description():
    return LaunchDescription([IncludeLaunchDescription(PythonLaunchDescriptionSource(os.path.join(
        get_package_share_directory('spiderx_bringup'), 'launch', 'fortress_control.launch.py')))])
```
