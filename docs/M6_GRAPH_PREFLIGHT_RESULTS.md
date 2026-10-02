# M6.0-B Results – Live GUI Graph-Mode Read-Only Preflight (local)

```text
M6.0-B was a live GUI ROS-graph OBSERVATION only, on the owner's Ubuntu PC.
No trajectory/action goal, action cancellation, joint command, /cmd_vel message, controller switch
or lifecycle request, other-node parameter change, playback, contact experiment, balance test,
walking test, navigation or hardware work occurred.
This report does NOT demonstrate action-goal acceptance, trajectory tracking, robot motion,
contact, stability, body support or locomotion.
```

## Outcome

- **Verdict: READY.** The preflight exited 0 with `failure_codes: []`.
- **All five zero-command evidence tiers passed** (§4).
- **Manual installed-stack classification: `warning`.** The local package versions are newer than
  the cloud references; the stack is not `incompatible`.
- **Clean shutdown:** one Ctrl+C, no forced cleanup, no leftover process. No retry.
- **M6.0-D valid playback remains pending.** It needs a separate owner approval.

The plan is [M6_GRAPH_PREFLIGHT_PLAN.md](M6_GRAPH_PREFLIGHT_PLAN.md); the approved decisions
OD-1 to OD-11 are in its
[§11.1](M6_GRAPH_PREFLIGHT_PLAN.md#111-approved-owner-decisions-2026-10-02).

**Labels:**
- **[LOCAL]**: run and reported by the owner on their Ubuntu PC. Everything in this document is
  [LOCAL]; nothing here was run in the cloud.

## 1. Environment [LOCAL]

| Item | Value |
|---|---|
| Machine | The owner's Ubuntu PC only (OD-2) |
| Branch used for the live observation | `claude/spiderx-m6-graph-preflight-plan` at `b64217d` |
| M6 implementation | Merged in `main` at `f5a7252` |
| Display mode | GUI, the normal default; not headless (OD-1) |
| Build | None needed: the installed workspace scripts are symlinked to the unchanged source |
| Generated evidence | Only under the git-ignored `log/m6_graph_preflight/` |

## 2. Commands [LOCAL]

**Terminal A** (the unmodified launch):

```bash
cd ~/spiderx_ws && source /opt/ros/humble/setup.bash && source install/setup.bash && ros2 launch spiderx_bringup fortress_control.launch.py 2>&1 | tee log/m6_graph_preflight/launch.log
```

**Terminal B** (the preflight):

```bash
timeout 60 ros2 run spiderx_controller m6_live_preflight --timeout 10 --window 2 --out log/m6_graph_preflight/live_preflight
```

**Approved logging-location deviation.**
- `--out` placed the report under `log/m6_graph_preflight/` instead of the tool's default path
  (`log/m6_playback/live_preflight/`).
- It did not change any ROS graph or control behaviour.

## 3. Timing [LOCAL]

| Event | Time / offset |
|---|---|
| Launch requested | 14:46:07.968Z |
| Launch log began | ≈ 14:46:12Z |
| `joint_state_broadcaster` active | in 0.45 s; its spawner exited cleanly at +4.23 s |
| Trajectory spawner started | +4.23 s |
| `leg_trajectory_controller` active | in 3.66 s; its spawner exited cleanly at +8.62 s |
| Both controllers active | ≈ 12.7 s after the launch request (limit 240 s) |
| First Terminal B poll | 14:48:01Z; both controllers were already active |
| Settle complete (3 s), **T0** | 14:48:04.135Z |
| Preflight ran | 14:48:19.709Z to 14:48:24.679Z, ≈ 5 s (cap 60 s) |
| Retry | None |

## 4. Preflight result [LOCAL]

- **Exit 0, verdict `READY`, `failure_codes: []`.**
- **Passed checks:**
  - `probe`;
  - `interface_contract`;
  - `action_server`;
  - `controllers`;
  - `joint_states_source`;
  - `joint_states_fresh`;
  - `joint_names_contract`;
  - `start_pose`.
- **Report:** `log/m6_graph_preflight/live_preflight/20261002T144824Z/live_preflight.json`
  (git-ignored).
- **Action server:** exactly one, `/leg_trajectory_controller/follow_joint_trajectory`, type
  `control_msgs/action/FollowJointTrajectory`.
- **Controllers:** both expected controllers were `active`, with the expected types.
- **Tool counters:** `goals_sent` 0 and `messages_published` 0. These are **intent and
  source-level evidence**: the tool sets them as constants. They are **not** a direct measurement
  of traffic.

### Joint-state and start-pose evidence

| Item | Result |
|---|---|
| `/joint_states` publishers | Exactly one: `/joint_state_broadcaster`. `robot_state_publisher` was a subscriber, not a second publisher |
| Freshness | 39 fresh messages in the 2 s observation window |
| Joint contract | All 12 canonical joints present |
| Maximum deviation from neutral | 4.99e-9 rad, at `rr_foot_joint` |
| Start-pose threshold | 0.05 rad |
| Result | Start pose ready |

## 5. Five-tier zero-command evidence [LOCAL]

| Tier | Evidence | Result |
|---|---|---|
| 1. Source / tool | Static scan of `m6_live_preflight.py` at `b64217d`: 0 occurrences of the 18 forbidden command and control terms. `FollowJointTrajectory.Goal` occurs once, only for field introspection | **Passed** |
| 2. Tool report | `goals_sent` 0, `messages_published` 0 | **Passed**, as **intent reporting**, not a direct measurement |
| 3. Graph | See the list below | **Passed**, as **supporting evidence**, not absolute proof by itself |
| 4. Logs | See the list below | **Passed** |
| 5. Processes | See the list below | **Passed** |

**Tier 3 details (graph):**
- `/leg_trajectory_controller/joint_trajectory`: 0 publishers and 1 subscriber (the controller).
- `/cmd_vel`: the topic was absent (unknown).
- At both T0 and T1: 0 action clients and 1 action server.
- No message was received on the transient-local action status topic in 10 s.

**Tier 4 details (logs):**
- 0 goal or cancel log strings in the captured run.
- The controller-lifecycle count was 5 at T0 and 5 at T1. All five were start-up `Loading` and
  `Configuring` entries.
- No `Switching`, `Deactivating`, `unloading`, goal or cancel evidence appeared during the
  preflight.

**Tier 5 details (processes):**
- The T0 and T1 snapshots matched.
- Only these appeared:
  - the launch tree;
  - the `ros2` CLI daemon created by the inspection queries;
  - unrelated desktop services.
- No unexpected process was observed.

**Why tier 3 is supporting, not proof.** A graph snapshot cannot rule out a short-lived client or
publisher between snapshots. It is conclusive only together with tiers 1, 2 and 4.

## 6. Allowed housekeeping (disclosed, OD-4)

These are permitted, node-local ROS housekeeping. They are **not** robot or controller commands:
- the preflight node's own `/parameter_events` announcement of its `use_sim_time` parameter;
- its `/rosout` publisher;
- its parameter services;
- normal discovery and graph traffic.

The read-only `ros2` CLI queries also created temporary nodes and started the `ros2` CLI daemon.
That daemon was explicitly stopped after the run.

**The launch's own behaviour (OD-8).** The launch itself is outside the zero rule:
- its spawners loaded, configured and activated the two controllers at start-up;
- `leg_trajectory_controller` then holds position internally.

This report does not claim that the launch emits no internal controller commands.

## 7. Manual compatibility classification (OD-5) [LOCAL]

**Result: `warning`, not `incompatible`.**

**Contract checks:**
- The interface contract passed.
- The `FollowJointTrajectory` action-file SHA-256 (`31a12442…`) was unchanged.
- The action type, the `JointState` type, the endpoint and the 12-joint contract all matched.

**Versions.** Eight local packages differ from the cloud references, and all are newer:

| Package | Local | Cloud reference |
|---|---|---|
| `joint_trajectory_controller` | 2.54.0 | 2.48.0 |
| `control_msgs` | 4.9.0 | 4.8.0 |
| `controller_manager` | 2.54.2 | 2.51.0 |
| `controller_manager_msgs` | 2.54.2 | 2.51.0 |
| `hardware_interface` | 2.54.2 | 2.51.0 |
| `gz_ros2_control` | 0.7.21 | 0.7.15 |
| `rclpy` | 3.3.19 | 3.3.16 |
| `action_msgs` | 1.2.2 | 1.2.1 |

`sensor_msgs` and `trajectory_msgs` matched, at 4.9.0.

**Rule applied** (M6 plan §14.9; graph-preflight plan §8). Version differences alone give
`warning`, not `incompatible`. Only `incompatible` would block a future valid playback.

## 8. Shutdown and cleanup [LOCAL]

- **One Ctrl+C** was sent in Terminal A.
- `robot_state_publisher` and the bridge exited cleanly ≈ 0.11 s later.
- The Gazebo launcher exited on SIGINT (−2) ≈ 0.79 s after the interrupt.
- The launch, Gazebo server and Gazebo GUI process groups were gone within 9.6 s, inside the 20 s
  grace.
- No SIGTERM, SIGKILL or manual cleanup signal was needed.
- The `ros2` daemon was stopped.
- **No leftover process remained:** no Gazebo, `ign`, `gz`, `controller_manager`, spawner,
  `robot_state_publisher`, bridge or daemon process.
- No retry occurred.

## 9. Non-claims

**Allowed claim:**

> The SpiderX Fortress control stack was observed read-only in graph mode on the owner's Ubuntu PC.
> The controller manager, both controllers, the FollowJointTrajectory action server and a fresh
> 12-joint `/joint_states` were present. The preflight sent no goal, cancel, command, controller
> switch or parameter change. Simulation only.

**Not shown:**
- that an action goal would be accepted, executed or rejected by the live controller;
- trajectory tracking or any robot motion;
- contact, stability, balance, body support or locomotion;
- walking, navigation, odometry or state estimation;
- real-time behaviour, actuator capability or hardware readiness;
- that M6.0-D is approved or safe to run.

## 10. Next steps (not started)

- **M6.0-D:** one valid neutral → `crouch_10mm` → neutral goal. It needs:
  - a separate owner approval;
  - a live adapter, which does not exist yet;
  - the classification labels in code;
  - start-pose blocking in front of goal construction (M6 plan §14.9).
- **M6.1 and M7–M10:** future work, not started.
