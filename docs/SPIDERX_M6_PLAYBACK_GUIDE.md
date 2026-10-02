# SpiderX M6 Playback Guide – M6.0 Safety Layers

```text
M6.0 is a trajectory-execution and observability check only. It is NOT gait playback, walking,
locomotion, contact, body support, balance, dynamic stability, navigation, real-time behaviour,
actuator capability or hardware readiness. M6.0-D (one neutral -> crouch_10mm -> neutral goal)
has NOT been run; it needs separate owner approval.
```

This guide covers how the M6.0 layers turn the M4-validated `crouch_10mm` pose into **one**
bounded trajectory, and how that trajectory is checked before any goal could exist. It also covers
how to run the offline and live read-only checks, and what to do when they refuse.

- The design and owner decisions D1–D7 are in
  [M6_GAIT_PLAYBACK_SAFETY_PLAN.md](M6_GAIT_PLAYBACK_SAFETY_PLAN.md) §14.
- The evidence is in [M6_TEST_RESULTS.md](M6_TEST_RESULTS.md).

## 1. Data flow and layers

```text
m4_pose_targets.yaml ─┐                                   (M6.0-A, no ROS)
spiderx_poses.yaml ───┤  m6_trajectory.load_sources ──► build_trajectory ──► preflight ──► PASS / REFUSED (exit 2)
spiderx_legs.yaml ────┤   (hashes, canonical order,       neutral → crouch_10mm → neutral     32 failure codes
controller YAML ──────┤    neutral cross-check)           3 points, 9 s, start delay 3 s
expanded URDF ────────┘

preflighted trajectory ──► m6_action_client.build_goal ──► PlaybackSession ──► ActionAdapter
                           (re-runs preflight; 0.05 rad     (one goal max; no     (M6.0-C: a test double only;
                            path/goal tolerances)            retry; cancel+hold)   no live adapter exists)

running controller stack ──► m6_live_preflight (M6.0-B) ──► READY / NOT READY report under log/
(started by the operator)    read-only graph queries; sends no goal; publishes nothing
```

| Module (`spiderx_controller/`) | Layer | ROS graph? |
|---|---|---|
| `m6_envelope.py` | The fixed limits and the three separate quantities | No |
| `m6_trajectory.py` | Sources, conversion, preflight, failure codes, `trajectory_id` | No |
| `m6_offline_preflight.py` | CLI for M6.0-A | No |
| `m6_action_client.py` | `build_goal`, `PlaybackSession`, D3 channels, tracking evaluation | No. It talks to an adapter; only a test double exists |
| `m6_live_preflight.py` | CLI for M6.0-B: read-only probe and pure evaluation | Read-only (`--interface-only`: none) |

`trajectory_client.py` and every M1–M5 tool are unchanged and are not used by M6.

## 2. The exact envelope

| Rule | Value | Source |
|---|---|---|
| **Maximum commanded displacement from neutral** (M6.0-D only) | **0.1223 rad**. Accepted iff \|q − q_neutral\| ≤ 0.1223 + 1e-9 rad | Owner option (i): the exact M4 `crouch_10mm` delta, 0.12229413600889982 rad. It never widens automatically |
| **Tracking tolerance** | **0.05 rad** | D3. Used for the `/joint_states` comparison and the goal's `path_tolerance` / `goal_tolerance` |
| **Joint-limit soft margin** | **0.05 rad** | `spiderx_legs.yaml` `soft_limit_margin_rad`. Every point must lie inside the URDF limits pulled in by the margin; nothing is clamped |
| Start-pose tolerance (M6.0-D only) | 0.05 rad | The existing M4 `START_POSE_TOL_RAD`, owner-approved. An observed `/joint_states` comparison against neutral; a failure must block goal construction and dispatch. Today only the live preflight evaluates it |
| Points | ≤ 5. The approved content is exactly 3: neutral, `crouch_10mm`, neutral | D4, D1 |
| Duration | ≤ 30 s. The shipped trajectory is 9 s | D4 |
| Start delay | > 0 and ≥ the lead-in, max(3.0 s, 2 × 0.05 / 0.5) = 3.0 s | D4; the M3/M4 duration rule |
| Segments | Each ≥ max(3.0 s, 2 × max\|Δq\| / 0.5 rad/s) | M3/M4 rule; 0.5 rad/s is the SIMULATION_PLACEHOLDER |
| Times | Strictly increasing | D4 |
| Velocities | Exactly 0 at every waypoint | Plan §5 |
| Joints | Exactly the 12 joints, unique, in canonical controller order (LF, RF, LR, RR × hip, thigh, foot) | D4 |
| Values | Every time, position and velocity finite and numeric | D4 |
| Mode | `single` only. No cyclic or repeat field exists, and unknown fields are refused | D4 |
| Goals | One per session; no retry; no concatenation; no automatic neutral return | D4, D5 |
| Goal message | `path_tolerance` and `goal_tolerance` of 0.05 rad position per joint; `goal_time_tolerance` 1.0 s; no controller-YAML change | D3. The 1.0 s is owner-approved for M6.0-D only: a finite deadline, not a claim that tracking is validated |

**Do not mix up the four values.** The fourth is the observed start-pose tolerance; see the
table above.

**The three envelope quantities:**
- 0.1223 rad limits what may be **commanded**.
- 0.05 rad tracking judges how closely the joints **followed**.
- The 0.05 rad soft margin keeps commands away from the **mechanical limits**.

## 3. Offline conversion and preflight (M6.0-A)

```bash
cd ~/spiderx_ws && colcon build --symlink-install && source install/setup.bash
ros2 run spiderx_controller m6_offline_preflight --check-only          # convert + preflight, write nothing
ros2 run spiderx_controller m6_offline_preflight                       # also writes log/m6_playback/offline/<id>/
ros2 run spiderx_controller m6_offline_preflight --out log/m6_second   # a second, comparable run
ros2 run spiderx_controller m6_offline_preflight --check-only --trajectory my.json   # preflight a file
```

| Exit | Meaning |
|---|---|
| 0 | PASS. The trajectory satisfies the envelope. This is **not** evidence of motion or tracking |
| 2 | REFUSED: a preflight failure (named codes are printed), an unreadable file or an existing output directory |

**Expected output.** `Preflight: PASS trajectory_id=…`, 3 points, a 3.0 s start delay, 9.0 s
duration, and a maximum displacement of 0.1222941360 rad (`rr_foot_joint`).

**Comparing two runs.** `trajectory.json` and `preflight.json` contain no timestamps:
`diff -r log/m6_playback/offline/<id> log/m6_second/<id>` must print nothing. The
`trajectory_id` hashes the content and the SHA-256 of every source file. Any source edit gives a
new ID, and an old file is refused as `source_stale`.

**Cross-machine identity.** The expanded URDF is hashed after its SpiderX mesh URIs are made
workspace-independent: `file://…/share/spiderx_description/meshes/<rel>` is hashed as
`package://spiderx_description/meshes/<rel>`, for provenance only.
- The same sources therefore give the same `trajectory_id` on any machine. With the current
  sources it is `44f0a7ad52e5c330`.
- Any real change to the description still changes the hash, including a mesh name, a joint limit
  or an inertia value.
- Before `fd7a9de`, IDs differed between workspaces; for example, cloud `b884584ed4aeddf0` versus
  local `1280770cae26aa54`.
- Trajectory files written before the fix are refused as `source_stale`. Regenerate them.

## 4. Mock action-client tests (M6.0-C)

M6.0-C is **mock-only**.
- No invalid goal is ever sent to a live controller.
- No live adapter exists in this repository.
- `PlaybackSession` talks only to an `ActionAdapter`; the tests use `test/m6_mock_action.py`.

```bash
colcon test --packages-select spiderx_controller --ctest-args -R test_m6 && colcon test-result --verbose
```

What the tests prove is listed in [M6_TEST_RESULTS.md](M6_TEST_RESULTS.md#mock-action-client-evidence-m60-c-measured):
- invalid input never reaches goal construction or dispatch;
- rejections, aborts, timeouts and errors become structured outcomes;
- cancel only, then hold;
- no retry, no automatic return goal, no second goal;
- the mutation tests prove the checks can fail.

**D3 channels.** Each outcome records three channels separately, each as `passed`, `failed`,
`timed_out` or `unavailable`:
- `action`;
- `tracking`;
- `goal_tolerances`.

A playback passes only if all three pass. Action success alone is not tracking evidence.

## 5. Live read-only preflight (M6.0-B)

**Not run in the cloud** except for `--interface-only`.
- It is for the owner's Ubuntu PC.
- It needs the owner's approval for the M6.0-B step.

```bash
# terminal 1 (the operator starts the stack; the tool never does):
ros2 launch spiderx_bringup fortress_control.launch.py
# terminal 2, after the controllers are active:
ros2 run spiderx_controller m6_live_preflight                    # full read-only preflight
ros2 run spiderx_controller m6_live_preflight --interface-only   # versions + interface; no ROS node
```

**What it reads:**
- the action server for `/leg_trajectory_controller/follow_joint_trajectory`, found by a graph
  query;
- the controller states, from one `list_controllers` query;
- the `/joint_states` publishers, then a 2 s subscription window, which checks freshness, the 12
  names, completeness and the start pose within 0.05 rad of neutral;
- the installed package versions and the `FollowJointTrajectory` interface contract.

**What it never does:**
- send a goal;
- create an action client or a publisher;
- switch controllers or set parameters;
- start Gazebo, a launch file or any child process.

| Exit | Meaning |
|---|---|
| 0 | READY: every requirement met (`--interface-only`: interface contract passed) |
| 1 | NOT READY: the printed codes name what is missing |
| 2 | Refused: bad arguments or an existing report directory |

- **Report location.** The report is saved to
  `log/m6_playback/live_preflight/<UTC time>/live_preflight.json`.
- **Installed-stack classification (owner decision, [plan §14.9](M6_GAIT_PLAYBACK_SAFETY_PLAN.md#149-final-owner-decisions-2026-10-02)).** Exact version
  equality is not required. The approved classes are:
  - `compatible`: the exact action and interface contract, the controller endpoint and the
    12-joint contract are present and usable;
  - `warning`: a package version differs from the cloud reference, or is unavailable, but the
    action and interface inspection is compatible. It does not block;
  - `incompatible`: a required FollowJointTrajectory, action or message field, the endpoint or
    the joint contract differs or is missing. Only this class blocks a future valid playback.
- **Current output.** The tool does not yet print these labels.
  - It records the actual versions as `matches` / `differs` / `unavailable`, which correspond to
    `warning` when not `matches`.
  - It reports the blocking contract failures (`interface_contract_mismatch`,
    `action_server_*`, `action_type_mismatch`, `joint_names_mismatch`), which correspond to
    `incompatible`.
  - `--interface-only` cannot establish `compatible`, because it observes no endpoint or joints.
- **What READY means.** READY only means that the interfaces M6.0-D would use are present and that
  nothing was sent. It does not prove tracking, movement, contact or walking.

## 6. Debugging

| Symptom | Likely cause | What to do |
|---|---|---|
| `REFUSED source_stale` | A config file or the URDF changed after the trajectory was written | Re-run the offline conversion; never edit the hash |
| `REFUSED trajectory_id_mismatch` | The trajectory file was edited by hand | Regenerate it; hand edits are refused by design |
| `REFUSED displacement_exceeds_cap` | A point moves a joint more than 0.1223 rad from neutral | Stop. The cap is not widened automatically; any change needs an owner decision |
| `REFUSED joint_limit_violation` | A point is within 0.05 rad of a URDF limit | Values are never clamped; fix the source |
| `REFUSED neutral_source_missing` / `_inconsistent` | `spiderx_poses.yaml` `cad_neutral` is missing or incomplete, or disagrees with the M4 `neutral_stance` | Restore the shipped config |
| `REFUSED canonical_order_inconsistent` | The controller YAML, URDF and `spiderx_legs.yaml` joint orders differ | Restore the shipped config; never reorder joints |
| `REFUSED source_pose_unavailable` | M4 refuses its own `crouch_10mm` | Run the M4 static validator |
| Live `controller_manager_unavailable` | The stack is not running, or the service timed out | Start `fortress_control.launch.py`; wait for both controllers to be active; use `--timeout` |
| Live `joint_states_multiple_publishers` | A second `/joint_states` source, such as a stale launch | Stop the extra source (see §7) |
| Live `start_pose_not_neutral` | The robot is not at neutral | Run `test_neutral_pose.py` (M1) first. M6 never moves the robot to neutral itself |
| Live `interface_contract_mismatch` | The installed `control_msgs` differs from the contract | Report the versions; do not proceed to M6.0-D |

## 7. Cleanup

**Offline layers.** They start nothing, so there is nothing to clean up.

**Live preflight.** It creates one node and destroys it on exit. The operator owns the launched
stack. Stop it with **Ctrl+C in the launch terminal**: signalling only the `ros2 launch` process
can leave `ign gazebo` running (see [M4.1 results](M4_1_TEST_RESULTS.md)). Then check:

```bash
pgrep -af 'ign gazebo|gz sim|parameter_bridge|robot_state_publisher|controller_manager/spawner|ros2 launch spiderx_bringup' || echo clean
```

**Generated files.** Everything generated goes under the git-ignored `log/m6_playback/`. Reports
are never overwritten.

## 8. Prohibited claims

**Allowed now:**
- "Offline conversion and preflight are implemented and tested."
- "The M6 client's rejection and failure paths are proven against a deterministic mock."
- "A live read-only preflight tool exists and is mock-tested."

**Not allowed:**
- any claim that a goal was sent to, accepted by or executed by a live controller;
- any claim of tracking in Gazebo;
- any claim of gait playback, walking, locomotion, contact, body support, balance, stability,
  navigation, real-time behaviour, actuator capability or hardware readiness;
- calling `crouch_10mm` playback a gait.

**After M6.0-D**, if the owner approves it and it passes locally, the most that may be claimed is:

> One bounded multi-point joint trajectory was executed in Gazebo through the existing controller
> stack, and joint-space tracking was observed within 0.05 rad (simulation, placeholder actuators).
