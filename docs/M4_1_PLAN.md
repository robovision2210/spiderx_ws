# M4.1 Plan – Controller Start-up Robustness (Gazebo Fortress, simulation only)

> This plan was written and committed **before** any M4.1 implementation (`3e1596e`).
> Branch `claude/spiderx-controller-spawn-race-v2`, created from `main` @ `2f03b3c` (PR #9, M4,
> merged). The owner's local branch of the same name starts at the same commit.
>
> **Status (2026-09-29): implemented and cloud-verified; local verification pending.** See
> [M4_1_TEST_RESULTS.md](M4_1_TEST_RESULTS.md). The owner's review approved this design with these
> decisions:
> - The error text is exactly `joint_state_broadcaster startup failed; leg_trajectory_controller
>   was not started; press Ctrl+C and relaunch.` The spawner exit code is appended.
> - The optional M1/M2 validator fix in §8 is **included**, as the check-line pattern only, with
>   regression tests in `test_validator_state_checks.py`.
> - The forced failure uses an untracked scratch launch. No public timeout override is added.
> - Implementation and documentation are committed separately.

**Goal.** `leg_trajectory_controller` must never be started unless `joint_state_broadcaster` was
spawned and activated successfully. A failure must be reported clearly, the success path must stay
as it is, and Ctrl+C must still stop everything without leaving Gazebo or controller processes.

**Not in M4.1:**
- M4.5 or any gait profile, walking, navigation or hardware work;
- URDF, mesh or world changes;
- a controller YAML redesign or unrelated refactoring.

Statements are labelled **[FACT]** (read in this repository, the installed packages or a recorded
log), **[HYPOTHESIS]** (believed, not proven) or **[PLAN]** (what will be built after review).

---

## 1. Current mechanism on `main` @ `2f03b3c` [FACT]

| Where | What |
|---|---|
| `src/spiderx_bringup/launch/fortress_control.launch.py:54-58` | Includes `spiderx_controller/launch/controller.launch.py` when `enable_control` is true (the default). This is the only file that includes it. |
| `src/spiderx_controller/launch/controller.launch.py:16` | `CONTROLLER_MANAGER_TIMEOUT_S = '120'` |
| `controller.launch.py:19-28` | `_spawner()` runs `controller_manager/spawner <name> --controller-manager /controller_manager --controller-manager-timeout 120`. It passes **no** `--switch-timeout` and **no** `--service-call-timeout`. |
| `controller.launch.py:32-41` | Starts the `joint_state_broadcaster` spawner, then `RegisterEventHandler(OnProcessExit(target_action=joint_state_broadcaster, on_exit=[leg_trajectory_controller]))`. |

- **History.** The file has not changed since M1 (`61460a2`).
- **How the handler fires.** In `launch` 1.0.9, an `on_exit` given as a **list of actions** is
  returned for **every** `ProcessExited` event of the target, whatever the exit code
  (`launch/event_handlers/on_action_event_base.py:89-111`). So the trajectory-controller spawner
  starts after the broadcaster spawner exits with code 0, with code 1, or after it was killed by a
  signal.
- **No other path starts controllers.** Nothing else in the repository loads or activates them. The
  `gz_ros2_control` plugin block (`spiderx_fortress_control.xacro:47-49`) only creates the
  controller manager.

### Spawner behaviour (controller_manager 2.51.0, the version used in every cloud run) [FACT]

**Defaults** (`controller_manager/spawner.py:129-151`):

| Option | Default |
|---|---|
| `--switch-timeout` | 5.0 s |
| `--service-call-timeout` | 10.0 s |
| `--controller-manager-timeout` | 0.0 s |

**Exit codes:**
- **0** only after `Configured and activated <name>` (`spawner.py:255-278, 305-306`), or when it is
  interrupted: `KeyboardInterrupt` is caught (`spawner.py:355-356`), `main()` returns `None`, and
  the entry script calls `sys.exit(main())`.
- **1** for these failures:
  - a failure to load, configure or activate (`spawner.py:216-270`);
  - the controller manager not found in time (`spawner.py:357-359`);
  - an uncaught error, for example the `RuntimeError` after 3 unanswered service calls.

**Re-sent requests.** An unanswered request is re-sent every `call_timeout`, at most 3 times
(`controller_manager_services.py:87-152`).

**The controller-manager side:**
- It waits for a switch on a condition variable with a `std::chrono` timeout, which is wall-clock
  time (`controller_manager/controller_manager.hpp:518-541`).
- It logs `Switch controller timed out after %f seconds!`.
- It rejects a strict request for an already-active controller (`… since it is already active`).

**The plugin.** `gz_ros2_control` 0.7.15 is a Gazebo `PreUpdate`/`PostUpdate` system (symbols in
`libgz_ros2_control-system.so`).

**Owner's PC.** `controller_manager` is 2.54.2 there. This is recorded on the unmerged branch in
§4 and was not re-checked for this plan.

### Launch shutdown (`launch` 1.0.9) [FACT]

- `LaunchService._shutdown()` sets `context.is_shutdown` as soon as a shutdown starts
  (`launch_service.py:381, 405`).
- `ExecuteLocal.execute()` does not start a process once shutdown has begun
  (`execute_local.py:633-635`).

## 2. Evidence of the race-capable trigger path [FACT]

The race was **seen twice, both times in the cloud on 2026-09-26** (VM with software rendering,
real-time factor ≈ 0.25). It was not seen in the other recorded cloud runs or in the owner's 5
local M4 launches.

**1. During M3.** From that session's launch log (t0 = hardware `SpiderXGazeboSimSystem` activated):

```text
t0+0.16 s  [controller_manager]: Loading controller 'joint_state_broadcaster'
t0+1.65 s  [controller_manager]: Configuring controller 'joint_state_broadcaster'
t0+6.96 s  [controller_manager]: Switch controller timed out after 5.000000 seconds!
t0+6.97 s  [spawner_joint_state_broadcaster]: Failed to activate controller : joint_state_broadcaster
           [ERROR] [spawner-5]: process has died [pid 1370, exit code 1, cmd '.../spawner
           joint_state_broadcaster --controller-manager /controller_manager
           --controller-manager-timeout 120 ...']
           [INFO] [spawner-7]: process started with pid [1648]
t0+7.49 s  [controller_manager]: Loading controller 'leg_trajectory_controller'
t0+9.02 s  [spawner_leg_trajectory_controller]: Loaded leg_trajectory_controller
```

End state: `joint_state_broadcaster` **inactive** and `leg_trajectory_controller` **active**
(`docs/M3_TEST_RESULTS.md:159`).

**2. During M4, in the first runtime run.** The end state was the same
(`docs/M4_TEST_RESULTS.md:59-68`). The launch log was not kept; `ee42108` fixed that.

**Why no motion followed.** The existing safety nets refused to command motion:

| Tool | Check |
|---|---|
| M2 | `run_posture_hold_test.py:124` / `posture_metrics.py:174-175` |
| M3 | `validate_leg_kinematics:152-153` |
| M4 | `m4_pose_validation.py:307-308` |
| M1 | `test_one_joint.py:62` checks only `leg_trajectory_controller`, but then refuses because `/joint_states` is incomplete (`test_one_joint.py:66-69`) |

## 3. Root cause

- **D1 – unconditional chain [FACT].** `controller.launch.py:37-40` starts the second spawner on
  *any* exit of the first, as shown in §1. This is the safety defect, and it is what M4.1 must fix.
- **D2 – trigger [HYPOTHESIS, strongly supported].** The broadcaster's activation needs a
  controller-manager update, which `gz_ros2_control` runs from the Gazebo simulation step. The
  spawner, however, waits in wall time, for 5 s by default. Early in start-up on a CPU-only machine,
  the simulation made no controller-manager update for more than 5 s of wall time, so the switch
  timed out.
  - **Unknown:** *why* the simulation stalled (possibly render or sensor initialisation under
    software rendering). The fix does not depend on the answer.
- **D3 – latent [FACT, derived from the code].** Raising only `--switch-timeout` above the default
  10 s `--service-call-timeout` would make the spawner re-send the pending switch request. Once the
  first request succeeded, the controller manager would reject the duplicate as `already active`,
  which is a false failure. The call timeout must therefore be longer than the switch timeout.

## 4. Prior work (unmerged) [FACT]

**Branch.** `claude/spiderx-controller-spawn-race` (`b215ec3`, `898beec`). It is based on `d9426c9`
(the M2 merge), was never opened as a PR, and is not in `main`. It already contains:
- a success-gated `OnProcessExit`;
- `--switch-timeout 60` and `--service-call-timeout 75`;
- no automatic shutdown;
- a new `test/test_controller_launch.py` (3 tests);
- an `M1_TEST_RESULTS.md` entry that reports verification on the owner's PC, including a forced
  failure.

**Applies cleanly.** That branch's `controller.launch.py` starts from the same file content as
`main` (blob `3407cb4`).

**Plan.** M4.1 reuses its design on top of M4, but re-verifies everything and takes nothing on
trust.

## 5. Proposed minimal fix [PLAN]

**Only one code file changes:** `src/spiderx_controller/launch/controller.launch.py`.

1. **Timeouts, simulation only.** Both spawners get `--switch-timeout 60` and
   `--service-call-timeout 75` (call timeout > switch timeout, because of D3).
   `--controller-manager-timeout 120` stays.
   - These limits are upper bounds, so a fast switch still returns at once.
   - The file is included only by the Gazebo launch.
   - 60 s is 12× the default and a quarter of the validators' 240 s wait.
2. **Success gate.** Replace the action list with a callable `OnProcessExit` handler on each
   spawner:

   | Condition | What the handler does |
   |---|---|
   | `context.is_shutdown` | Nothing: normal Ctrl+C teardown, no new process and no error |
   | Exit code 0 | Starts the next spawner (for the broadcaster: `leg_trajectory_controller`) |
   | Any other code, including signals | Logs **one** ERROR line and starts nothing |

   The ERROR line names the controller and the exit code, says that `leg_trajectory_controller`
   was NOT started, and says to stop the launch (Ctrl+C) and start it again, referring to the
   spawner output above.
3. **No automatic shutdown on failure.** The branch in §4 records that `EmitEvent(Shutdown)` left
   `ign gazebo server/gui` running as orphans; that record is not re-verified here. Ctrl+C, the
   shutdown path every validator already uses, stays the only one.
4. **Update the docstring** to describe the new ordering and failure behaviour.

**Unchanged:**
- controller names and the YAML;
- the URDF/xacro, `fortress.launch.py` and `fortress_control.launch.py`;
- all M1–M4 tools.

**Alternatives not chosen:**
- **One spawner for both controllers.** The spawner stops on the first failure, but this changes
  the documented two-spawner structure and log prefixes, and a launch-level error would still be
  needed.
- **Automatic shutdown on failure.** This carries the orphan risk above.

### Expected behaviour

| Situation | Result |
|---|---|
| Success | As today: broadcaster activated → trajectory spawner starts → both `active`, one `/joint_states` publisher |
| Broadcaster spawner fails (e.g. switch timeout) | Error line logged. `leg_trajectory_controller` is **never loaded**. The simulation keeps running in its uncontrolled pre-activation state until Ctrl+C, and the tools refuse because not all controllers are active |
| Trajectory spawner fails | Error line logged; the broadcaster stays active; the tools refuse |
| Ctrl+C at any time | No error line, no new spawner, everything stops, no leftover processes |

**Residual risk [FACT].** A broadcaster spawner that gets SIGINT *on its own*, without the launch
shutting down, exits 0 without activating, so the gate would pass. Ctrl+C in the terminal and the
validators both signal the whole launch process group, which starts the launch shutdown first. The
downstream tools still require both controllers to be `active`.

## 6. Tests to add [PLAN]

**New file:** `src/spiderx_controller/test/test_controller_launch.py`, registered in `CMakeLists.txt`
(+1 line). It uses no Gazebo and no controller manager.

1. **Spawner arguments.** Both spawners have `--controller-manager /controller_manager`,
   `--controller-manager-timeout 120`, `--switch-timeout 60` and `--service-call-timeout 75`, with
   the call timeout longer than the switch timeout.
2. **Structure (regression guard).** `generate_launch_description()` starts only the broadcaster
   spawner directly. Both `OnProcessExit` handlers are callables, not static action lists.
3. **Success starts the next spawner.** A real `LaunchService` with stand-in processes: exit 0
   starts the next spawner and logs no error.
4. **Failure does not start it.** Exit codes 1 and 2, and death by SIGTERM (−15), start nothing and
   log exactly one error with the name, the code, "NOT started" and the Ctrl+C advice.
5. **Shutdown guard.** With `is_shutdown` true, exit codes 0 and 1 give no actions and no error.

**Mutation check.** A scratch copy of the launch file with the old list handler must make test 4
fail. That copy is not committed.

**Existing tests.** None are changed. The expected total is 311 + the new cases.

## 7. Validation and M0–M4 regressions [PLAN]

**Static checks:**
- clean `colcon build --symlink-install`;
- `colcon test` / `colcon test-result`;
- all five `validate_*.sh` scripts without `--runtime`.

**Runtime checks** (cloud; each in its own simulation, 30 s apart; no leftover processes after
each):
- **R1 – success path.** `validate_m1_control.sh --runtime`. The launch log must show the
  broadcaster's `Configured and activated` before the trajectory spawner starts.
- **R2 – forced broadcaster failure.** An untracked scratch launch loads the installed
  `controller.launch.py` and sets the switch timeout to 0.001 s. No tracked file is edited. Expected:
  1. `Switch controller timed out after 0.001000 seconds!`;
  2. the broadcaster spawner exits with code 1;
  3. the M4.1 error line;
  4. `list_controllers` shows only `joint_state_broadcaster inactive`, with `leg_trajectory_controller`
     never loaded;
  5. Ctrl+C to the process group then stops everything.
- **R3 – Ctrl+C during start-up**, while the broadcaster spawner is still waiting: no error line,
  no trajectory spawner, no leftovers.
- **R4 – regressions.** `validate_fortress.sh`, `validate_m1_control.sh`, `validate_m2_posture.sh`,
  `validate_m3_kinematics.sh` and `validate_m4_all_leg_ik.sh`, all with `--runtime`.
- **R5 – repeated start.** 5 consecutive M1 runtime launches.
  - The race is intermittent, so passes here are evidence, not proof. What the fix guarantees is
    that a failure is safe and reported.
- **Owner's PC.** The owner reruns the same checks locally.

**Acceptance.** M4.1 is done only when all of these hold:
- the trajectory controller is never started after a broadcaster failure (tests 3–5 and R2);
- the error line appears on failure;
- the success path is unchanged (R1, R4);
- nothing is left over after Ctrl+C (R1–R4);
- all M0–M4 checks pass;
- nothing outside §8 changed.

## 8. Files M4.1 may change after review [PLAN]

**Code and test:**
- `src/spiderx_controller/launch/controller.launch.py`;
- the new `src/spiderx_controller/test/test_controller_launch.py`;
- `src/spiderx_controller/CMakeLists.txt` (+1 line).

**Docs (after validation):**
- the new `docs/M4_1_TEST_RESULTS.md`;
- this plan;
- `STATUS.md:5`;
- `docs/SPIDERX_DEVELOPMENT_ROADMAP.md:76`;
- `src/spiderx_controller/README.md:14`;
- `docs/M1_GZ_ROS2_CONTROL_IMPLEMENTATION_PLAN.md:80-81` (the design description).

The race is also mentioned in historical records, which stay as they are because they describe
what was true then:
- `docs/M3_LEG_KINEMATICS_PLAN.md:29-30`;
- `docs/M3_SIMULATION_LIMITATIONS.md:38-42`;
- `docs/M3_TEST_RESULTS.md:39, 159`;
- `docs/M4_PLAN.md:300`;
- `docs/M4_TEST_RESULTS.md:59-68, 92, 145-147`.

**Optional, only if approved:** `scripts/validate_m1_control.sh:79` and
`scripts/validate_m2_posture.sh:113` check `^<controller> .*active`. That pattern also matches
`inactive`, so they print "[PASS] joint_state_broadcaster active" for an inactive broadcaster (their
overall verdict still fails through later checks). The M3 and M4 validators use `.* active`.

A draft PR is opened only after implementation and validation.
