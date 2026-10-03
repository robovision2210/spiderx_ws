# M6.0-D First Live Goal – Results (owner's Ubuntu PC, Gazebo Fortress GUI)

```text
One bounded joint trajectory (neutral -> crouch_10mm -> neutral) was sent once to the simulated
controller stack in Gazebo. This is a trajectory-execution and observability result in simulation
with placeholder actuators. It does not show gait playback, walking, locomotion, contact, body
support, balance, dynamic stability, navigation, real-time behaviour, actuator capability or
hardware readiness.
```

## Outcome

**SUCCEEDED.** Exactly one goal was sent; the controller accepted it and reported success; the
independent `/joint_states` tracking check passed. No cancel, no retry, no second goal and no
return-to-neutral goal were sent. The gate was re-disabled immediately afterwards.

## When and where

| Item | Value |
|---|---|
| Date | 2026-10-03 (UTC); local time 19:33–19:38 IST |
| Machine | Owner's Ubuntu 22.04 PC, ROS 2 Humble, Gazebo Fortress in GUI mode, ROS domain 0 |
| Operator | Claude Code, on the owner's explicit instruction for this one run |
| Branch and SHA before the run | `claude/spiderx-m6d-enable-gate` @ `5800c475632fb6d9f80c8c389e75a9e03c4a4110` (gate `True`), one commit on `77fd171` |
| Build | `rm -rf build install`, `colcon build --symlink-install`: 8 packages finished in 16.1 s, exit 0 |
| Launch requested (terminal A) | 14:03:08.593Z, `ros2 launch spiderx_bringup fortress_control.launch.py` |
| Both controllers `active` | observed at 14:03:28.6Z (spawners finished at 14:03:16Z and 14:03:22Z), then 3 s settle |
| Read-only preflight | 14:03:49Z, `READY`, exit 0 |
| Confirmation word sent | 14:05:19.001Z, once |
| Goal received and accepted by the controller | 14:05:23.03Z |
| `Goal reached, success!` | 14:06:01.71Z |
| Tool exit | 0 |
| Gazebo stopped (one Ctrl+C in terminal A) | 14:07:47.6Z; all process groups gone within 11.1 s |

## Identity of the goal

| Item | Value |
|---|---|
| Trajectory ID | `44f0a7ad52e5c330` |
| Goal fingerprint | `0d6ef4171f2d338a01e76b934be94d1a4386e7c0fc68fe74262fb3c65005ff83` |
| Offline checks before the launch | `m6_offline_preflight --check-only` PASS; `m6_live_playback --dry-run --no-write` PASS with the same ID and fingerprint |
| **Goal ID** | **`43b487553a1e4e179473f42a0bba5cd7`** |

## Before the goal

- **Preflight** (`m6_live_preflight --out log/m6d_run`): `READY`. All eight checks passed (`probe`,
  `interface_contract`, `action_server`, `controllers`, `joint_states_source`,
  `joint_states_fresh`, `joint_names_contract`, `start_pose`). Eight package versions differ from
  the cloud reference (classification `warning`, as in M6.0-B).
- **T0 zero-command evidence:**
  - `/leg_trajectory_controller/joint_trajectory`: 0 publishers;
  - action: 0 clients, 1 server;
  - launch log: 0 `Received new action goal`, 0 `Accepted`, 0 cancel lines;
  - nodes: `/controller_manager`, `/gz_ros2_control`, `/joint_state_broadcaster`,
    `/leg_trajectory_controller`, `/robot_state_publisher`, `/spiderx_gz_bridge`.
- The `ros2` CLI daemon was stopped before the tool started.

## The run (`m6_live_playback --live --domain-id 0 --out log/m6d_run`)

From `log/m6d_run/live/20261003T140505Z/live_outcome.json` (phase `final`):

| Field | Value |
|---|---|
| `state` / `reason` | `SUCCEEDED` / none; `passed: true` |
| `goals_sent` | **1** |
| `cancels_sent` | **0** |
| `retries` | **0** |
| `automatic_return_goals` | **0** |
| `dispatch` | `send: sent`, `acceptance: accepted`, `final_result_known: true`, `final_goal_status: 4`, `goal_may_still_be_executing: false`, `cancel_attempted: false` |
| Controller result | `error_code 0` (`SUCCESSFUL`), "Goal successfully reached!" |
| Channels | `action: passed`, `tracking: passed`, `goal_tolerances: passed` |
| Tracking | 1003 samples; maximum in-flight error **0.00603 rad** (limit 0.05 rad) |
| Readiness | before and after confirmation: `ready`, classification `warning`, collection 4.02 s each |
| Freshness at the send | 4.03 s old (limit 10 s) |
| Transport | closed, no close error |
| `errors` | none |

Session events (seconds from session start): preflighted 0.001; confirmation accepted 12.05;
dispatching 16.07; goal sent 16.08; goal accepted 16.13; result status 4 at 54.83; `SUCCEEDED`
after the settle window at 59.90.

## After the goal (T2)

- **Launch log:** `Received new action goal` = 1, `Accepted new action goal` = 1,
  `Goal reached, success!` = 1; `Got request to cancel goal` = 0; no abort line.
- **Controller status topic** (latched, read with reliable QoS): exactly one goal,
  ID `43b487553a1e4e179473f42a0bba5cd7`, equal to the report's `dispatch.goal_id`, status 4
  (succeeded).
- `/leg_trajectory_controller/joint_trajectory`: 0 publishers. Action: 0 clients, 1 server.
  Both controllers still `active`.
- **Wall time.** 38.7 s passed between acceptance and `Goal reached` for the 9 s trajectory; the
  simulation ran at roughly 0.23 of real time.

## Shutdown and cleanup

- One Ctrl+C in terminal A. The launch, Gazebo server and Gazebo GUI process groups were all gone
  within 11.1 s. No signal other than that Ctrl+C was sent.
- `ros2 daemon stop` afterwards. Leftover check: no Gazebo, bridge, `robot_state_publisher`,
  controller manager, spawner, launch, daemon or M6 tool process.
- `git status --short` was empty before the gate was re-disabled; all evidence is under the
  git-ignored `log/m6d_run/` and `log/m6d_run_evidence/`.

## Gate re-disabled

- Commit `de7b0cd3aec14a739d24e21742fbd4b8f2b554cb`, "chore: re-disable M6.0-D gate after one-goal
  run", on `claude/spiderx-m6d-enable-gate`.
- `m6_live_contract.py:18` is `LIVE_DISPATCH_ENABLED = False` and `test/m6d_gate.py:8` is
  `EXPECTED_LIVE_DISPATCH_ENABLED = False`. Both files are byte-identical to `77fd171`.
- The installed (symlinked) module reads `False`.
- **Not pushed from this machine:** `git push` failed for lack of GitHub credentials. The owner
  must push `de7b0cd`.
- The live tool was not run again after the goal.

## Anomalies and deviations

1. **The checklist's status command cannot see a latched goal status.**
   `ros2 topic echo --once --qos-durability transient_local …/_action/status` returned no message
   at T2, although a goal existed. With `--qos-reliability reliable` added it showed the goal.
   So the "no status message" results at T0 in this run, and in the M6.0-B run (tier 3d), were
   not evidence that no goal had existed. The other T0 evidence (0 action clients, 0 goal lines
   in the launch log) is unaffected.
2. **CLI daemon fault.** The first T2 `ros2 action info` and `ros2 control list_controllers`
   failed inside the `ros2` daemon (`!rclpy.ok()`) after the earlier `ros2 daemon stop`. They
   succeeded after another `ros2 daemon stop`. This did not involve the controller or the goal.
3. **How the confirmation was entered.** The tool ran in a terminal tab with its stdin connected
   to a named pipe. After the prompt appeared, the exact word was written to the pipe once. It was
   not typed on the keyboard.
4. **Launch command.** The launch output was piped through `tee` to
   `log/m6d_run_evidence/launch.log`, as in the approved checklist. The Ctrl+C also stopped `tee`,
   so the launch's shutdown lines are only in its own log under `~/.ros/log/`.
5. **Not exercised.** No cancel happened, so the hold-after-cancel behaviour was not observed.
6. **Not observed.** The Gazebo GUI was not watched by the operator. The evidence is the tool
   report, the controller's log lines and the controller's status topic.
7. **Not run in this session.** The full test suite was not re-run on the enabling commit or on
   the re-disabling commit.

## Evidence files (git-ignored, on the owner's PC)

| File | Content |
|---|---|
| `log/m6d_run/live/20261003T140505Z/live_outcome.json` | Final report (SHA-256 `8eca1f4556204e05ae430fa57c94d26a72dff14f268874e1b2d763166f745c1c`) |
| `log/m6d_run/20261003T140349Z/live_preflight.json` | Preflight report |
| `log/m6d_run_evidence/launch.log` | Launch output, including the controller's goal lines |
| `log/m6d_run_evidence/t0.txt`, `t2.txt`, `t2b.txt` | Zero-command and one-goal evidence |
| `log/m6d_run_evidence/build.log`, `offline_checks.txt`, `preflight.txt`, `controllers_active.txt`, `after.txt` | Build, identity, preflight, controller state and cleanup |

**Exactly one goal sent; gate re-disabled; no second goal.**
