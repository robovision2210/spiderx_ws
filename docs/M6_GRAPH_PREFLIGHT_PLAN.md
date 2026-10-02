# M6.0-B Plan – Live Graph-Mode Read-Only Preflight

> **Phase 0 (read-only audit and plan only).** Written before any M6.0-B runtime activity. Branch
> `claude/spiderx-m6-graph-preflight-plan`, created from `origin/main` @ `f5a7252` (M6.0
> offline/mock tooling merged; cloud + local offline/mock verified). During this audit no Gazebo,
> launch file, controller, ROS node, graph query, action goal or command was started or sent.
>
> **Status (2026-10-02): Phase 0 approved by the owner; decisions OD-1 to OD-11 approved
> ([§11.1](#111-approved-owner-decisions-2026-10-02)). The live graph-mode run has NOT been performed.** It is a manual, two-terminal run on
> the owner's Ubuntu PC only (§9); its results go to `docs/M6_GRAPH_PREFLIGHT_RESULTS.md`, created
> only after that run.

```text
M6.0-B only OBSERVES a running, unmodified SpiderX Fortress control simulation and then stops it.
It sends no trajectory goal, no command, no cancel, no controller switch and no parameter change.
A pass shows only that the interfaces a future M6.0-D playback would use are present.
It is NOT evidence of joint motion, tracking, goal acceptance, contact, stability, balance,
locomotion, walking, navigation or hardware capability.
```

**Labels:**
- **[FACT]**: read in this repository or in the installed stack, with a path:line reference.
- **[RESULT]**: an earlier verified result.
- **[HYPOTHESIS]**: believed but not yet observed; checked during the first M6.0-B run.
- **[PLAN]**: a proposed procedure or rule.
- **[OWNER DECISION]**: needed from the owner before any graph-mode run (§11).

## Contents

1. [Scope and non-claim boundary](#1-scope-and-non-claim-boundary)
2. [Audited runtime launch and data-flow map](#2-audited-runtime-launch-and-data-flow-map)
3. [Graph-mode inspection contract](#3-graph-mode-inspection-contract)
4. [Zero-command proof requirements](#4-zero-command-proof-requirements)
5. [Startup, preflight and cleanup timeouts](#5-startup-preflight-and-cleanup-timeouts)
6. [Success evidence](#6-success-evidence)
7. [Failure categories](#7-failure-categories)
8. [Cloud and local version and compatibility procedure](#8-cloud-and-local-version-and-compatibility-procedure)
9. [Local execution checklist](#9-local-execution-checklist)
10. [Test and mutation strategy for future automation](#10-test-and-mutation-strategy-for-future-automation)
11. [Owner decisions required before any graph-mode run](#11-owner-decisions-required-before-any-graph-mode-run)
12. [Out of scope](#12-out-of-scope)

---

## 1. Scope and non-claim boundary

**[PLAN] M6.0-B may:**
- start the **existing, unmodified** normal Fortress control launch (§2.1), only to observe it;
- inspect the ROS graph: nodes, topics, publishers, action servers and clients;
- read controller states with the `list_controllers` query;
- subscribe to `/joint_states` and read it;
- record package versions and the action/message contract;
- stop the launch cleanly and check that no process is left.

**[PLAN] M6.0-B must send, from the start of the preflight window to its end:**
- 0 `FollowJointTrajectory` goals;
- 0 action cancel requests;
- 0 controller commands, and 0 messages on any command topic (for example
  `/leg_trajectory_controller/joint_trajectory`);
- 0 `/cmd_vel` messages;
- 0 controller load, configure, switch or unload requests (no controller-lifecycle activity);
- 0 parameter changes on any other node.

The exact approved definition of "zero", and the node-local housekeeping that is allowed and must
be disclosed, are in [§11.1](#111-approved-owner-decisions-2026-10-02) (OD-4).

**What the normal launch itself does (not M6.0-B activity) [FACT].** These happen before the
preflight window, and they are unchanged since M1/M4.1:
- The two spawners load, configure and activate `joint_state_broadcaster` and then
  `leg_trajectory_controller` through the controller manager's services
  (`src/spiderx_controller/launch/controller.launch.py:36-47, 75-90`).
- Once active, `leg_trajectory_controller` holds its current position. It writes position commands
  to the `gz_ros2_control` command interfaces on every control cycle, internally, inside the Gazebo
  server process. This is not a ROS topic publication and not caused by the preflight. It is still
  a reason why no "zero joint commands" claim may ever be made for the simulation as a whole.
- **[OWNER DECISION OD-8, approved]** The zero-command requirement applies to the **preflight
  window and the preflight tool**, not to the launch's own start-up activation and hold. No claim
  may say that the launch itself emits no internal controller commands.

**Allowed claim after a successful run [PLAN]:**

> "The SpiderX Fortress control stack was observed read-only (graph mode): the controller
> manager, both controllers, the FollowJointTrajectory action server and a fresh 12-joint
> `/joint_states` were present, and the preflight sent no goal, cancel, command, controller switch or
> parameter change. Simulation only."

**Prohibited claims:**
- any joint motion or tracking;
- any goal being accepted, executed or rejected by the live controller;
- contact, stability, balance, body support, locomotion or walking;
- navigation, odometry or state estimation;
- real-time behaviour, actuator capability or hardware readiness;
- that M6.0-D is approved or safe to run. That needs a separate owner approval.

## 2. Audited runtime launch and data-flow map

### 2.1 The normal launch command [FACT]

```bash
ros2 launch spiderx_bringup fortress_control.launch.py            # GUI + server (default)
ros2 launch spiderx_bringup fortress_control.launch.py headless:=true   # server only (EGL)
```

- **Source.** `src/spiderx_bringup/launch/fortress_control.launch.py:3, 70-78`.
- **Defaults.**
  - `enable_control` = `true` (line 72) and `rviz` = `false` (line 74).
  - `headless` = `false` (`src/spiderx_description/launch/fortress.launch.py:52-55`). This
    argument is declared in the included launch and passed through by the M1–M4 validators
    (`scripts/validate_m1_control.sh:72`).
- **Not used by M6.0-B.** `fortress_posture_hold.launch.py`, the M2–M4 launch with the
  ground-truth pose bridge, is not used: M6.0-B needs only the control stack.
- **`headless:=true` is untested.** The cloud ogre-next build lacks EGL (`STATUS.md:110`). The
  owner's local runs used the default GUI mode.

### 2.2 What the launch starts [FACT]

| # | Process | Source | Lifetime |
|---|---|---|---|
| 1 | `ros2 launch spiderx_bringup fortress_control.launch.py` (Python `launch` 1.0.9) | the command | until shutdown |
| 2 | `/bin/sh -c ruby …/ign gazebo -r -v 2 <world> --force-version 6` | `ros_gz_sim/launch/gz_sim.launch.py:151-157` (`shell=True`, `output='screen'`, `on_exit=Shutdown()` because `on_exit_shutdown:='true'`) | until shutdown |
| 3 | Ruby launcher `ign gazebo …`, then `ign gazebo server` and `ign gazebo gui`, in **their own process groups** | `docs/M4_1_TEST_RESULTS.md:376-382` | until shutdown |
| 4 | Inside the Gazebo server: `gz_ros2_control` plugin, `/controller_manager`, and the controllers `joint_state_broadcaster` and `leg_trajectory_controller` | `src/spiderx_description/urdf/spiderx_fortress_control.xacro:29, 47-48` | with the server |
| 5 | `robot_state_publisher` | `fortress.launch.py:94-100` | until shutdown |
| 6 | `ros_gz_sim create` (node `spawn_spiderx`) | `fortress.launch.py:125-138` | exits after spawning |
| 7 | `ros_gz_bridge parameter_bridge` (node `spiderx_gz_bridge`) with `fortress_bridge_control.yaml`: `/clock` and `/scan` only, **no `/joint_states`** | `fortress.launch.py:140-153`; `src/spiderx_description/config/fortress_bridge_control.yaml:5-15` | until shutdown |
| 8 | `spawner joint_state_broadcaster` | `controller.launch.py:36-47, 76, 80` | exits 0 after activation |
| 9 | `spawner leg_trajectory_controller`, started only after #8 exits 0 | `controller.launch.py:56-72, 81-85` | exits 0 after activation |

**Spawner timeouts [FACT].** `--controller-manager-timeout 120`, `--switch-timeout 60`,
`--service-call-timeout 75` (`controller.launch.py:26, 32-33, 42-46`).

**Not started:**
- RViz (`rviz:=false`);
- any hardware driver (`scripts/validate_m1_control.sh:54-60`);
- any `/cmd_vel` consumer.

**Expected ROS endpoints once ready:**
- **[FACT]** `/controller_manager` with `~/list_controllers` and `~/switch_controller` services
  (`libcontroller_manager.so` strings).
- **[FACT]** The action `/leg_trajectory_controller/follow_joint_trajectory` of type
  `control_msgs/action/FollowJointTrajectory` (`m6_envelope.ACTION_NAME`, `ACTION_TYPE`).
- **[FACT]** `/joint_states`, published only by `joint_state_broadcaster`. The control-mode bridge
  excludes it (`fortress_bridge_control.yaml:1-3`).
- **[HYPOTHESIS]** The node names are `/controller_manager`, `/joint_state_broadcaster`,
  `/leg_trajectory_controller`, `/robot_state_publisher`, `/spiderx_gz_bridge`, plus a
  `gz_ros2_control` node. The first run records the observed node list.

### 2.3 What must be absent afterwards [FACT, `scripts/validate_m4_all_leg_ik.sh:190-195`]

```bash
pgrep -af 'ign gazebo|gz sim|parameter_bridge|robot_state_publisher|controller_manager/spawner|ros2 launch spiderx_bringup'
```

- **Exclude the caller.** The calling shell's own ancestors must be excluded, as the M4 validator
  does (`validate_m4_all_leg_ik.sh:191-193`).
- **[PLAN] Also absent:**
  - the `ros2` CLI daemon (`_ros2_daemon`), stopped with `ros2 daemon stop` if any `ros2` CLI
    query started it;
  - the preflight's own node `m6_live_preflight`.

### 2.4 Known start-up and shutdown hazards [FACT / RESULT]

- **Gazebo GUI↔server handshake hang.**
  - In GUI+server mode, the server waits with no timeout for the GUI's starting-world message
    (`docs/M4_1_TEST_RESULTS.md:306-337`).
  - **Symptoms.** `Requesting list of world names.` repeats every 5 s; no `/controller_manager`
    appears; the broadcaster spawner exits 1 after 120 s with
    `Could not contact service /controller_manager/list_controllers`.
  - **Where it was seen.** On the cloud VM; never on the owner's PC (`M4_1_TEST_RESULTS.md:386-398`, item 4).
- **Launch-only shutdown can orphan Gazebo.** A shutdown started by `launch` alone can leave
  `ign gazebo server`/`gui` running. **SIGINT to the whole process group** (Ctrl+C, or
  `kill -INT -- -<pgid>`) is the proven path (`M4_1_TEST_RESULTS.md:376-385`).
- **Script-started launches ignore SIGINT.** A launch started as `setsid … &` from a
  non-interactive script inherits SIGINT as ignored, in the Python processes only. Shutdown still
  completes, because the C++ nodes and Ruby handle SIGINT and `launch` escalates to SIGTERM after
  5 s (`M4_1_TEST_RESULTS.md:399-407`).

## 3. Graph-mode inspection contract

The tool is `ros2 run spiderx_controller m6_live_preflight`, with no `--interface-only`
(`src/spiderx_controller/scripts/m6_live_preflight:1-12`). Module:
`src/spiderx_controller/spiderx_controller/m6_live_preflight.py`.

### 3.1 How graph mode works today [FACT]

**1. Interface and version data (no node).**
- `collect_versions()` reads installed `package.xml` files through `ament_index` (lines 109-121).
- `collect_interface()` imports the `FollowJointTrajectory` and `JointTolerance` **types**, hashes
  the installed `.action` file and lists the goal fields (lines 124-139).
- `FollowJointTrajectory.Goal` is only introspected (`get_fields_and_field_types()`, line 136); no
  goal object is created.

**2. One node.** `_ros_probe()` (lines 531-540) does `rclpy.init()`, then
`create_node('m6_live_preflight')`, then `GraphProbe(...).collect()`, then `destroy_node()` and
`try_shutdown()`.

**3. `GraphProbe.collect()`** (lines 416-426):
1. Spins for `discovery_s` = 2.0 s. This is a fixed default (line 360) and is not a CLI option.
2. `action_servers()` (lines 374-381): for every node from `get_node_names_and_namespaces()`, it
   calls `rclpy.action.get_action_server_names_and_types_by_node()` and keeps the entries for
   `/leg_trajectory_controller/follow_joint_trajectory`. This is a graph query only; no action
   client is created.
3. `controllers()` (lines 383-395): it creates **one** service client for
   `/controller_manager/list_controllers`, makes one `ListControllers` request (a read-only query),
   records `(name, type, state)`, and destroys the client. It waits at most `--timeout` (default
   10 s) for the service and again for the answer.
4. `joint_state_publishers()` (lines 397-399): `get_publishers_info_by_topic('/joint_states')`,
   a graph query.
5. `joint_state_messages()` (lines 401-414): **one** subscription to `/joint_states` for
   `--window` (default 2 s) of wall time. It keeps at most 200 messages (line 47), then destroys
   the subscription.
6. Every exception is recorded as `probe_error`; none is hidden (lines 421-425).

**4. `evaluate()`** (lines 173-288) is pure Python and applies:
- `interface_contract`: the goal fields, `JointTolerance` fields and error codes 0 to −5;
- `action_server`: exactly one server, of the exact type;
- `controllers`: both required controllers present and `active`, and
  `leg_trajectory_controller` of type `joint_trajectory_controller/JointTrajectoryController`;
- `joint_states_source`: exactly one publisher, of type `sensor_msgs/msg/JointState`;
- `joint_states_fresh`: at least 2 messages, with header stamps positive and strictly increasing;
- `joint_names_contract`: exactly the 12 canonical names, with finite positions;
- `start_pose`: every joint within 0.05 rad of CAD neutral.

**5. Report and exit.**
- The report is written to `log/m6_playback/live_preflight/<UTC>/live_preflight.json` and is never
  overwritten (lines 472-481).
- Exit 0 means READY, 1 NOT READY, 2 refused (line 41).

### 3.2 Source proof that graph mode cannot dispatch, command, cancel, switch or set parameters [FACT]

- **Static scan of `m6_live_preflight.py`.** These strings each occur **0 times**:
  - `send_goal`, `ActionClient`, `create_publisher`, `publish(`, `cancel`;
  - `switch_controller`, `load_controller`, `unload`;
  - `set_parameters`, `SetParameters`;
  - `subprocess`, `Popen`, `os.system`;
  - `cmd_vel`, `m6_action_client`;
  - `create_timer`, `create_service`, `create_action`.

  `FollowJointTrajectory.Goal` occurs once, as the field introspection at line 136. `ros2 launch`
  occurs once, in the docstring (line 4).
- **The only ROS objects created** (lines 384, 408, 535):
  - one node;
  - one `ListControllers` client;
  - one `JointState` subscription.
- **The only service request** is a `ListControllers.Request()` (line 388).
- **Guarding tests [RESULT]** (`src/spiderx_controller/test/test_m6_live_preflight.py`, 42 tests):
  - a fake node raises on any method outside a read-only allow-list;
  - a static scan rejects the words above;
  - importing the module loads neither `rclpy` nor `control_msgs`.

  These passed in the cloud and locally (`docs/M6_TEST_RESULTS.md:219-276, 397-504`).

### 3.3 Side effects that do exist and must be stated exactly [FACT]

- **One `/parameter_events` message.** Creating any rclpy node creates a `/parameter_events`
  publisher (`rclpy/node.py:199-200`; rclpy 3.3.16). Its `TimeSource` declares the node's **own**
  `use_sim_time` parameter (`node.py:221-225`), which publishes **one** `ParameterEvent` announcing
  that new parameter (`node.py:860-881`).
- **This is not a parameter change on another node.** It cannot be avoided with rclpy Humble's
  `create_node`. Every `ros2` CLI command (`ros2 node list`, `ros2 control list_controllers`) does
  the same.
- **A `/rosout` publisher and parameter services are also created.**
  - `/rosout` is created with `enable_rosout=True` (`node.py:124`). The probe logs with `print`,
    not the node logger, so **[HYPOTHESIS]** it publishes nothing on `/rosout`.
  - The parameter **services** (`start_parameter_services=True`, `node.py:125, 227-228`) are
    servers that nobody calls.
- **Consequence for the claims.** The zero-publication requirement must be stated as zero
  **command, control, action, controller-switch and other-node parameter** activity. That is not
  "zero bytes on the wire". **[OWNER DECISION OD-4].**

### 3.4 Gaps in the current tool relative to the M6.0-B goal [FACT]

| Gap | Effect | Proposal |
|---|---|---|
| No installed-stack class label (`compatible` / `warning` / `incompatible`); owner decision §14.9 of the [M6 plan](M6_GAIT_PLAYBACK_SAFETY_PLAN.md#149-final-owner-decisions-2026-10-02) | The class must be derived by hand from the report | **OD-5**: manual classification by the documented mapping (§8) for M6.0-B, or a small approved code batch first |
| Zero-command evidence is **source-level only**; the tool does not observe other nodes' command activity | Runtime evidence must come from graph counts and the launch log (§4) | **OD-6**: manual evidence tiers 1–3, or an approved observer tool |
| `discovery_s` is fixed at 2.0 s | On a slow graph, the action server may be missed (a false NOT READY, which fails safe) | Accept; rerun once if `action_server_missing` but the server appears in a manual `ros2 action list -t` |
| The tool never starts or stops Gazebo | Start-up and cleanup are manual (§9) or need a wrapper | **OD-3** |

## 4. Zero-command proof requirements

A pass needs **all** of the following tiers. The window is **T0**, just before the preflight
starts, to **T1**, just after it exits. **[PLAN]**

| Tier | Evidence | Pass condition | Source |
|---|---|---|---|
| 1. Source | The §3.2 static scan, re-run on the exact commit used | Every forbidden word occurs 0 times | `grep -c` on `m6_live_preflight.py` |
| 2. Tool report | `live_preflight.json` fields `goals_sent` and `messages_published` | Both 0 (lines 302-303 of the module set them as constants; they record intent, not measurement) | the report |
| 3a. Graph, after T1 | Publishers on `/leg_trajectory_controller/joint_trajectory` | 0 | `ros2 topic info -v` |
| 3b. Graph, after T1 | Publishers on `/cmd_vel` | 0 | `ros2 topic info /cmd_vel` (the topic may not exist; "unknown topic" counts as 0) |
| 3c. Graph, after T1 | Action **clients** of `/leg_trajectory_controller/follow_joint_trajectory` | 0 | `ros2 action info /leg_trajectory_controller/follow_joint_trajectory` |
| 3d. Graph, after T1 | Last action status | No goal status ever published | **[HYPOTHESIS]** The status topic is `TRANSIENT_LOCAL`, depth 1 (`include/rcl_action/rcl_action/default_qos.h:26-31`), so a late subscriber receives the last status if any goal ever existed. Read with `ros2 topic echo --once --qos-durability transient_local …/_action/status`, a short timeout and `--include-hidden-topics` if needed. Expected: no message, or an empty `status_list` |
| 4a. Launch log, whole run | JTC goal and cancel strings: `Received new action goal`, `Accepted new action goal`, `Got request to cancel goal`, `Goal reached, success!`, `Canceling active action goal` | 0 occurrences | `strings libjoint_trajectory_controller.so` (installed 2.48.0) |
| 4b. Launch log, T0 to T1 | Controller-manager lifecycle strings: `Loading controller`, `Configuring controller`, `Deactivating controller`, `Switching controllers:`, `unloading service called` | No **new** occurrence after T0. The count at T1 equals the count at T0 | `strings libcontroller_manager.so` (installed 2.51.0). **[HYPOTHESIS]** which of these appear at INFO during the spawners' start-up; the first run records the counts |
| 5. Process | No process other than the launch tree, the preflight and the read-only `ros2` CLI queries ran | `ps` snapshot at T0 and T1 | `pgrep -af` |

**Not proof by itself.** A tool that merely *claims* zero is not proof: tiers 3–4 are independent
runtime observations. Tier 4 depends on the controller's log level; if `INFO` lines are not
visible in the launch log, tier 4 is `unavailable` and must be reported as such, never as passed.

## 5. Startup, preflight and cleanup timeouts

| Phase | Limit | Basis | On expiry |
|---|---|---|---|
| Start-up: launch until both controllers are `active` | **240 s** wall | `validate_m1_control.sh:75-76`; `validate_m4_all_leg_ik.sh:127-128` | Failure **F1** or **F2–F4** (§7); go straight to shutdown |
| Settle after `active` | **3 s** | `validate_m4_all_leg_ik.sh:129` (M4.1 finding item 2) | – |
| Preflight tool | `--timeout 10` s per query, `--window 2` s sampling, plus 2 s discovery. Overall wall cap **60 s** (**[PLAN]**, via `timeout 60`) | `m6_live_preflight.py:360, 436-437` | Treat as NOT READY (`probe_error`); go to shutdown |
| SIGINT | `kill -INT -- -<pgid>` of the launch's process group (manual: Ctrl+C in the launch terminal) | `M4_1_TEST_RESULTS.md:174-180, 376-385` | – |
| Grace | Poll `kill -0 -- -<pgid>` once per second for up to **20 s** | `validate_m4_all_leg_ik.sh:186-187` | Escalate |
| Escalation | `kill -KILL -- -<pgid>`, then wait 2 s | `validate_m4_all_leg_ik.sh:188-189` | – |
| Leftover check | The §2.3 `pgrep` (excluding the caller's ancestors), plus `pgrep -af _ros2_daemon` | `validate_m4_all_leg_ik.sh:190-195` | Failure **F9**; next step below |
| Direct cleanup (only if leftovers remain) | `kill -INT <pid>` per leftover PID (the Ruby launcher, then server and GUI each exited within 3 s of a SIGINT in M4.1), then `kill -KILL <pid>` after 5 s; then `ros2 daemon stop`; then re-run the leftover check | `M4_1_TEST_RESULTS.md:268-269` | Report every PID and command; the run is FAIL even if cleanup then succeeds |

**[PLAN] Never** stop the launch by signalling only the `ros2 launch` PID: that is the orphaning
path. Never use `pkill -f gazebo` broadly. Only the PIDs listed by the leftover check are
signalled.

## 6. Success evidence

| Evidence | Pass condition | Where it comes from |
|---|---|---|
| Launch reached ready | Both spawners printed their configured-and-activated line and exited 0; no `spiderx_controller` error line | Launch log (`controller.launch.py:50-53`) |
| Controller manager available | `list_controllers` answered within `--timeout` | Report `observed.controllers` is not null |
| `joint_state_broadcaster` | `active` | Report check `controllers` |
| `leg_trajectory_controller` | `active`, type `joint_trajectory_controller/JointTrajectoryController` | Report check `controllers` |
| Action endpoint | Exactly one server for `/leg_trajectory_controller/follow_joint_trajectory`, type `control_msgs/action/FollowJointTrajectory` | Report check `action_server` |
| `/joint_states` source | Exactly one publisher, `/joint_state_broadcaster` (**[HYPOTHESIS]** name), type `sensor_msgs/msg/JointState` | Report check `joint_states_source` |
| `/joint_states` fresh | ≥ 2 messages in the window; stamps positive and strictly increasing | Report check `joint_states_fresh` |
| 12-joint contract | Exactly the 12 canonical names (any order), finite positions | Report check `joint_names_contract` |
| Start pose | All joints within 0.05 rad of CAD neutral (**[HYPOTHESIS]**: the held spawn pose; M2 measured ≤ 0.00028 rad joint error in hold, `STATUS.md:118`) | Report check `start_pose` |
| Interface contract | Goal, `JointTolerance` and error-code contract matches | Report check `interface_contract` |
| Version classification | `compatible` or `warning` (§8) | Report `versions` and `version_check`, plus the §8 mapping |
| Zero-command proof | Tiers 1–5 pass (§4) | §4 |
| Clean shutdown | Group exited within 20 s of SIGINT, or after the KILL escalation (recorded); leftover check empty; no `ros2` daemon | §5 |
| Verdict | Tool exit 0 `READY` **and** every row above passes | – |

## 7. Failure categories

| ID | Category | Detection | Report as |
|---|---|---|---|
| F1 | Gazebo/world start-up hang | Not `active` within 240 s, and the launch log shows repeated `Requesting list of world names.` or `Waiting for a world to be set from the GUI`, or the broadcaster spawner exits 1 with `Could not contact service /controller_manager/list_controllers` | FAIL `gazebo_startup_hang`; shut down per §5; the tool is not run |
| F2 | Controller manager unavailable | Tool `controller_manager_unavailable`, or the spawner timeout above without the hang symptoms | FAIL |
| F3 | Broadcaster inactive | Tool `controller_missing` / `controller_not_active` for `joint_state_broadcaster`, or the M4.1 error line `joint_state_broadcaster startup failed` | FAIL |
| F4 | Trajectory controller inactive | Same codes for `leg_trajectory_controller`, or `controller_type_mismatch` | FAIL |
| F5 | Action server unavailable | `action_server_missing`, `action_server_ambiguous`, `action_type_mismatch` | FAIL (`incompatible` if the type differs) |
| F6 | Joint states missing, stale, multiple publishers, incomplete or wrong names | `joint_states_no_publisher`, `joint_states_multiple_publishers`, `joint_states_type_mismatch`, `joint_states_no_messages`, `joint_states_stale`, `joint_states_incomplete`, `joint_names_mismatch` | FAIL (`incompatible` for a names or type mismatch) |
| F6b | Start pose not neutral | `start_pose_not_neutral` | FAIL for M6.0-B readiness; never "fixed" by M6.0-B (no command is allowed) |
| F7 | Interface incompatible | `interface_contract_mismatch` | FAIL, `incompatible` |
| F8 | Unexpected command, action or publisher activity | Any §4 tier 3 or 4 condition violated, or an unexpected node or process | FAIL `unexpected_activity`. Stop immediately (§5), keep every log, and report |
| F9 | Cleanup failure | Leftover check not empty after the KILL escalation, or a `ros2` daemon still running | FAIL `cleanup_failure`, even if the preflight itself passed |
| F10 | Tool or probe error | `probe_error`, tool exit 2, or the 60 s wall cap hit | FAIL; record the exception text |

**[PLAN] No automatic retry.** One manual rerun is allowed only for F1 or a discovery-timing
`action_server_missing`, and both attempts are reported (**OD-10**).

## 8. Cloud and local version and compatibility procedure

**1. Collect versions and the interface contract with no ROS node.**
- Run `ros2 run spiderx_controller m6_live_preflight --interface-only` before the graph run.
- **[RESULT]** Cloud: versions match the reference, contract passed
  (`docs/M6_TEST_RESULTS.md:267-275`).
- **[RESULT]** Local: exit 0, contract passed, **8 package versions differ**; not classified
  (`docs/M6_TEST_RESULTS.md:474-484`).

**2. Reference versions [FACT]** (`m6_live_preflight.py:51-62`), with the cloud's installed values:

| Package | Value |
|---|---|
| `joint_trajectory_controller` | 2.48.0 |
| `control_msgs` | 4.8.0 |
| `trajectory_msgs` | 4.9.0 |
| `controller_manager`, `controller_manager_msgs`, `hardware_interface` | 2.51.0 |
| `gz_ros2_control` | 0.7.15 |
| `rclpy` | 3.3.16 |
| `sensor_msgs` | 4.9.0 |
| `action_msgs` | 1.2.1 |

Also recorded in this audit [FACT]:
- `joint_state_broadcaster` 2.48.0;
- `ros_gz_sim` and `ros_gz_bridge` 0.244.20;
- `rcl_action` 5.3.9;
- `launch` 1.0.9 and `launch_ros` 0.19.10;
- `ros2cli` 0.18.12 and `ros2controlcli` 2.51.0;
- Gazebo (Ignition) 6.16.0.

**3. Classification [OWNER DECISION, M6 plan §14.9; applied by hand per OD-5].** The tool
reports the facts; the owner's manual review applies the label to the **graph-mode** report,
because `--interface-only` cannot observe the endpoint or the joints:
- **`incompatible`** if any of these appear: `interface_contract_mismatch`,
  `action_server_missing`, `action_server_ambiguous`, `action_type_mismatch`,
  `joint_names_mismatch`, `joint_states_type_mismatch`;
- otherwise **`warning`** if any `version_check` package is not `matches`;
- otherwise **`compatible`**.

**4. Effect.** Only `incompatible` blocks any future valid playback. Every installed version value
is copied into the M6.0-B results, whatever the class.

## 9. Local execution checklist

This is a **[PLAN]** for the owner's Ubuntu PC, to run **only after** the §11 decisions. It uses
the existing tool unchanged. It is written for two terminals (manual mode, **OD-3**).

```bash
# 0. Clean, known state
cd ~/spiderx_ws && git status --short && git rev-parse HEAD        # expect a clean tree at the approved commit
colcon build --symlink-install && source install/setup.bash
pgrep -af 'ign gazebo|gz sim|parameter_bridge|robot_state_publisher|controller_manager/spawner|ros2 launch spiderx_bringup' || echo "no leftovers before start"
ros2 daemon stop
grep -c -E 'send_goal|ActionClient|create_publisher|publish\(|cancel|switch_controller|set_parameters' \
  src/spiderx_controller/spiderx_controller/m6_live_preflight.py   # tier 1: expect 0

# 1. Interface-only (no node)
ros2 run spiderx_controller m6_live_preflight --interface-only

# 2. Terminal A: the unmodified launch, its output saved
mkdir -p log/m6_graph_preflight
ros2 launch spiderx_bringup fortress_control.launch.py 2>&1 | tee log/m6_graph_preflight/launch.log

# 3. Terminal B: wait for both controllers (<= 240 s), then 3 s
ros2 control list_controllers                     # both lines must read "active"
sleep 3
grep -c -E 'Loading controller|Configuring controller|Deactivating controller|Switching controllers:|unloading service called' \
  log/m6_graph_preflight/launch.log               # T0 count (tier 4b)

# 4. Terminal B: the graph-mode preflight (window T0..T1), wall-capped (OD-7)
timeout 60 ros2 run spiderx_controller m6_live_preflight --timeout 10 --window 2; echo "exit=$?"

# 5. Terminal B: independent zero-command evidence (tiers 3-4), read-only
ros2 topic info -v /leg_trajectory_controller/joint_trajectory   # 0 publishers
ros2 topic info /cmd_vel                                          # 0 publishers or unknown topic
ros2 action info /leg_trajectory_controller/follow_joint_trajectory   # 0 action clients, 1 server
timeout 10 ros2 topic echo --once --qos-durability transient_local \
  /leg_trajectory_controller/follow_joint_trajectory/_action/status   # no message or empty status_list
grep -c -E 'Loading controller|Configuring controller|Deactivating controller|Switching controllers:|unloading service called' \
  log/m6_graph_preflight/launch.log               # T1 count == T0 count
grep -c -E 'Received new action goal|Accepted new action goal|Got request to cancel goal|Goal reached, success!|Canceling active action goal' \
  log/m6_graph_preflight/launch.log               # 0
ros2 node list                                    # record the node list

# 6. Terminal A: Ctrl+C once (SIGINT to the foreground process group); wait up to 20 s

# 7. Terminal B: cleanup proof
ros2 daemon stop
pgrep -af 'ign gazebo|gz sim|parameter_bridge|robot_state_publisher|controller_manager/spawner|ros2 launch spiderx_bringup' || echo "no leftovers"
pgrep -af _ros2_daemon || echo "no ros2 daemon"
```

**Notes:**
- **Step 5's CLI queries are read-only.** They create their own short-lived nodes, each with the
  §3.3 `use_sim_time` event; they send nothing else.
- **The `_action/status` topic is hidden.** If `ros2 topic echo` refuses it, add
  `--include-hidden-topics`.
- **[HYPOTHESIS]** Whether the `ros2 topic echo` CLI accepts `--qos-durability` is confirmed on the
  owner's `ros2cli` before relying on it. If it does not, tier 3d is reported as `unavailable`.
- **Results.** The report JSON, `launch.log` and the outputs of steps 5 and 7 are kept under
  `log/m6_graph_preflight/`, which is git-ignored, and summarised in a later results document.

## 10. Test and mutation strategy for future automation

This applies only if the owner approves code (**OD-3**, **OD-5**, **OD-6**). Each item would be a
separate, reviewed batch. None of it is part of this plan's commit. **[PLAN]**

| Possible automation | Tests without Gazebo | Mutation tests (must turn red) |
|---|---|---|
| Installed-stack class label in `m6_live_preflight` | Pure `evaluate()` cases for each §8 rule, including the local 8-difference case giving `warning` | Mapping `joint_names_mismatch` to `warning`; letting a version difference block |
| Launch-log evidence parser (tiers 4a–4b) | Synthetic logs: clean; one `Received new action goal`; a new `Switching controllers:` after T0; INFO lines absent (gives `unavailable`) | The parser ignoring the goal string; counting T0 lines after T0; reporting `passed` when the lines are absent |
| Graph evidence collector (tiers 3a–3d), read-only | Fake node or graph as in `test_m6_live_preflight.py`; allow-list proof that it creates no publisher or action client | A collector that creates an `ActionClient` or a publisher must fail the allow-list test |
| Wrapper script (start, wait, preflight, evidence, SIGINT group, leftover check) | A process-group test with dummy `sleep` children, no Gazebo; timeout paths; the leftover parser excluding the caller's ancestors | Signalling only the leader PID instead of the group; skipping the leftover check; a non-empty leftover list reported as pass |
| Any new module | The static forbidden-word scan from §3.2, extended to the new files | Inserting `send_goal`, `create_publisher`, `switch_controller` or `set_parameters` must be detected |

Every batch would also need:
- a clean build and the full suite;
- the M1–M4 static validators;
- an unchanged M1–M6 behaviour scope audit;
- no live run in the cloud.

## 11. Owner decisions required before any graph-mode run

The table below is the Phase 0 proposal, kept for the record. The owner's approved decisions are
in [§11.1](#111-approved-owner-decisions-2026-10-02), which governs.

| ID | Decision | Options | Recommendation |
|---|---|---|---|
| **OD-1** | Display mode | (a) default GUI+server; (b) `headless:=true` | **(a)**. It was proven locally in M1–M4; headless is untested (no EGL in the cloud) |
| **OD-2** | Where to run | (a) the owner's Ubuntu PC only; (b) also the cloud VM | **(a)**. The cloud VM showed the F1 handshake hang (M4.1); the owner's PC never did |
| **OD-3** | Execution mode | (a) manual two-terminal checklist (§9), existing tool unchanged; (b) an approved wrapper script first | **(a)** for the first run: no code change, and Ctrl+C is the proven shutdown path |
| **OD-4** | Meaning of "zero publications" | (a) zero command, control, action, controller-switch and other-node parameter activity, accepting the node's own `use_sim_time` `/parameter_events` message and the `/rosout` and parameter-service objects (§3.3); (b) require a code change first (still cannot remove the `use_sim_time` event in rclpy Humble) | **(a)**, stated exactly in the results |
| **OD-5** | Installed-stack class | (a) classify by hand from the graph report with the §8 mapping; (b) implement the labels first | **(a)** for M6.0-B; **(b)** before M6.0-D |
| **OD-6** | Zero-command evidence | (a) tiers 1–5 by hand (§4, §9); (b) an approved read-only observer and log parser first | **(a)**, with every `unavailable` tier reported as such |
| **OD-7** | Timeouts | 240 s start-up; 3 s settle; tool `--timeout 10`, `--window 2`; 60 s wall cap; 20 s SIGINT grace; KILL escalation | As listed |
| **OD-8** | Scope of "zero controller-switch/commands" | It applies to the preflight window and tool, not to the launch's own spawner activation and the controller's internal position hold (§1) | Confirm |
| **OD-9** | Start pose | Keep `start_pose_not_neutral` as a M6.0-B readiness failure (0.05 rad, §14.9 decision 3) | Keep |
| **OD-10** | Retries | No automatic retry; at most one manual rerun for F1 or a discovery-timing `action_server_missing`; both attempts reported | As listed |
| **OD-11** | Results location | A new `docs/M6_GRAPH_PREFLIGHT_RESULTS.md` (or a section in `docs/M6_TEST_RESULTS.md`), committed docs-only after the run | Owner's choice |

### 11.1 Approved owner decisions (2026-10-02)

**[OWNER DECISION]** Approved after the Phase 0 review. Docs-only record; no code, test, script or
config changed, and no runtime operation occurred.

**OD-1 Display mode: GUI.**
- Use the existing normal GUI default: `ros2 launch spiderx_bringup fortress_control.launch.py`.
- Do not introduce or test `headless:=true` during M6.0-B.

**OD-2 Environment: the owner's Ubuntu PC only.**
- The live graph-mode preflight runs only on the owner's Ubuntu PC.
- The cloud is not used for this live runtime step, because of the documented cloud GUI↔server
  start-up hang (§2.4).

**OD-3 Execution: manual two-terminal checklist.**
- The first live run follows §9 by hand, with the existing tool unchanged.
- No new runtime wrapper script is created for the first live run.

**OD-4 The precise zero rule.**
- **"Zero" means** zero activity of the following kinds **from the M6 preflight tool**:
  - robot commands;
  - trajectory commands;
  - action goals;
  - action cancels;
  - controller switches;
  - controller-lifecycle requests (load, configure, activate, deactivate, unload);
  - `/cmd_vel` messages;
  - parameter changes on other nodes.
- **Allowed node-local housekeeping**, which must be disclosed in the results:
  - the preflight node's one `/parameter_events` update announcing its own `use_sim_time`
    parameter (§3.3);
  - its `/rosout` publisher;
  - its parameter services (servers nobody calls);
  - normal node discovery and graph traffic.
- **These side effects are never described as controller or robot commands.** The same applies
  to the read-only `ros2` CLI queries in §9, which create their own short-lived nodes.

**OD-5 Classification: manual.**
- For M6.0-B, the report collects the package-version, action-interface and joint-contract facts.
- The owner's manual review applies `compatible` / `warning` / `incompatible`, following the
  approved rules (§8; M6 plan §14.9).
- No classification-label code is added on this plan branch. Label support in code remains a
  prerequisite for M6.0-D.

**OD-6 Evidence: all five tiers.**
- All five evidence tiers of §4 are required.
- A tier that cannot be observed is reported as **`unavailable`**, never as passed.
- Missing evidence is never inferred.

**OD-7 Timeouts.**

| Phase | Limit |
|---|---|
| Start-up (both controllers `active`) | 240 s |
| Settle after `active` | 3 s |
| Each graph or controller query (`--timeout`) | 10 s |
| `/joint_states` observation window (`--window`) | 2 s |
| Preflight tool wall-clock cap (`timeout 60`) | 60 s |
| SIGINT grace before forced cleanup | 20 s |

There is no automatic retry.

**OD-8 Scope of the zero rule.**
- The zero-command rule applies to the M6 preflight window and tool.
- It does not prohibit the existing launch's normal start-up: the spawners loading, configuring
  and activating `joint_state_broadcaster` and `leg_trajectory_controller`, and that controller's
  hold-position behaviour.
- No claim may say that the launch itself emits no internal controller commands.

**OD-9 Start pose.**
- The check that every observed joint is within 0.05 rad of neutral is kept.
- A failure is **NOT READY** and blocks any future playback authorization.
- The check sends no command, and M6.0-B never moves the robot to fix it.

**OD-10 Retry.**
- At most **one** manual rerun is allowed.
- It may be used only after full cleanup (§5) and only for one of these:
  - a documented Gazebo start-up hang (F1);
  - a documented discovery-timing miss (`action_server_missing` while `ros2 action list -t`
    shows the server).
- There is no automatic retry, and no rerun for a substantive preflight failure.
- Both attempts are reported.

**OD-11 Results.**
- After the actual live run, create `docs/M6_GRAPH_PREFLIGHT_RESULTS.md`. It records:
  - environment and version facts;
  - the command and timing;
  - all five evidence tiers;
  - the classification;
  - the result;
  - the shutdown and cleanup evidence;
  - the strict non-claims.
- That file is **not** created now, because no live run has occurred.

## 12. Out of scope

- Any `FollowJointTrajectory` goal, valid or invalid; any cancel; M6.0-D playback.
- Any controller command, command-topic message or `/cmd_vel` message; any controller load,
  configure, switch or unload request by M6.0-B; any parameter change on another node.
- Any change to the launch files, URDF/xacro, worlds, meshes, controller YAML, bridge configs,
  package metadata, or M1–M6 code, tests or configs, as part of this plan.
- `fortress_posture_hold.launch.py`, the ground-truth pose bridge, and body-pose or foot
  measurements.
- M6.1 protected replay; M5.5 command-velocity bridge; M7 odometry and state estimation; M8 SLAM;
  M9 Nav2; M10 hardware.
- Contact, stability, balance, locomotion or walking evaluation; hardware, servos, firmware.
- Fixing the upstream Gazebo GUI↔server handshake hang or the launch-only orphaning path
  (`M4_1_TEST_RESULTS.md` findings 3–4).
- A live graph-mode run in the cloud, `headless:=true`, a runtime wrapper script, and
  classification-label code (OD-1, OD-2, OD-3, OD-5).
- Opening a PR or merging. This plan is committed docs-only on its own branch.
