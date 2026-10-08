# M6.1 – Protected Trot Gait Replay: Implementation Notes

**Status: IMPLEMENTED for offline, mock and isolated-domain use only. Live dispatch is
HARD-DISABLED** (`m61_live_contract.M61_LIVE_DISPATCH_ENABLED = False`, pinned by
`test/m61_gate.py`).
- No goal has been sent to any simulation.
- No Gazebo, launch or controller was started to build or test this.
- Nothing is merged.

| Item | Value |
|---|---|
| Design | [`M61_TROT_REPLAY_DESIGN.md`](M61_TROT_REPLAY_DESIGN.md) (`7b8798c`) |
| Owner approval (2026-10-03) | 2 cm step, 6 mm lift, 4.0 s cycle, fixed / clamped base, 4.5 cm body-height gate, separate `m61_limits.yaml` |
| Branch | `claude/stoic-shannon-ur2mes` |

---

## 1. What was built

| File | Role |
|---|---|
| `config/m61_limits.yaml` | The M6.1 limits (separate file; M6.0-D limits untouched) |
| `config/m61_trot_cycle.yaml` | The ONE approved trot cycle: metadata, 9 waypoints (positions + velocities), content hash |
| `spiderx_controller/m61_limits.py` | Strict loader for `m61_limits.yaml`. Refuses unknown, missing or duplicate keys, booleans used as numbers, non-finite values, wrong schema, and contact sensing other than `not_measured` |
| `spiderx_controller/m61_live_contract.py` | The **separate** gate `M61_LIVE_DISPATCH_ENABLED = False`, the word `SEND-ONE-TROT-CYCLE`, and the approved limits, gait and content hash pinned in code |
| `spiderx_controller/m61_trot_cycle.py` | Design generator (section 2.3 logic), strict YAML loader, content hash, re-derivation (IK) check, M6.1 offline preflight, cubic-Hermite reference, tracking evaluation |
| `spiderx_controller/m61_goal.py` | The approved `FollowJointTrajectory` goal and its fingerprint, built on the unchanged M6.0-D `m6_goal_fingerprint` |
| `spiderx_controller/m61_gates.py` | Pure gate logic for G1–G7, body-pose freshness and body-pose readiness |
| `spiderx_controller/m61_evidence.py` | Run directory `log/m61_run/<UTC>/`, CSV writers, exclusive (never-overwrite) file creation |
| `spiderx_controller/m6_gait_replay.py` | `M61Session` (subclass of the unchanged M6.0-D `LiveSession`), body-pose readiness wrapper, CLI (`--dry-run`, `--mock`, `--live`; live returns exit 3) |
| `spiderx_controller/m61_mock.py` | `M61FakeTransport`: the M6.0-D fake plus a body-pose stream and 14 scenarios |
| `spiderx_controller/m61_live_adapter.py` | `M61RclpyLiveTransport`: the M6.0-D rclpy transport plus one read-only ground-truth pose subscription |
| `scripts/m6_gait_replay.py` | Thin entry point: `ros2 run spiderx_controller m6_gait_replay.py ...` |
| `test/m61_gate.py`, `test/test_m61_gait_replay.py` | The gate pin, and 118 offline / mock / isolated-domain tests |
| `CMakeLists.txt` | Installs the script and registers the test (the only existing file changed) |

**Not modified:**
- every M6.0-D module (`m6_*.py`) and the M6.0-D gate;
- all launch files, the URDF/xacro and the controller YAML.

The existing pinned-hash tests (`test_m6d_contract`, `test_m6d_live_enabling`) still pass, which
proves the M6.0-D files are byte-identical.

### 1.1 The approved trajectory (`config/m61_trot_cycle.yaml`)

| Item | Value |
|---|---|
| `content_sha256` | `94a492c43fcc046125d6bc71ee7f1d9a8a5d8466f1a1bdd9958a16044dd58e64` |
| `trajectory_id` | `241760e7dfd5ef12` |
| Goal fingerprint | `9dba1a173e212bfc172ffcd7da59124f87e98a321906e914e9d60e55595e5c3a` |

The `content_sha256` uses the design-note scheme (section 2.3). The design-note generator
reproduces it with `STEP 0.020`, `LIFT 0.006`. The other two values use the M6.0-D scheme and are
pinned in the tests.

The table gives positions in rad. All four hip joints are 0 at every point. Times are sim time
from goal acceptance.

| # | t (s) | lf_thigh | lf_foot | rf_thigh | rf_foot | lr_thigh | lr_foot | rr_thigh | rr_foot | Label |
|---|---|---|---|---|---|---|---|---|---|---|
| 0 | 3.00 | 0 | 0 | 0 | 0 | 0 | 0 | 0 | 0 | neutral_start_pair_a_liftoff |
| 1 | 3.50 | +0.0056 | +0.0368 | −0.0018 | −0.0001 | +0.0018 | −0.0001 | −0.0056 | −0.0368 | pair_a_swing_quarter |
| 2 | 4.00 | −0.0282 | +0.0687 | −0.0129 | −0.0004 | +0.0129 | −0.0004 | +0.0282 | −0.0687 | pair_a_apex |
| 3 | 4.50 | −0.0799 | +0.0261 | −0.0368 | −0.0004 | +0.0368 | −0.0004 | +0.0799 | −0.0261 | pair_a_swing_three_quarter |
| 4 | 5.00 | −0.0730 | −0.0074 | −0.0691 | +0.0014 | +0.0691 | +0.0014 | +0.0730 | +0.0074 | pair_a_touchdown_pair_b_liftoff |
| 5 | 5.50 | −0.0379 | −0.0028 | −0.1075 | −0.0339 | +0.1075 | −0.0339 | +0.0379 | +0.0028 | pair_b_swing_quarter |
| 6 | 6.00 | −0.0130 | −0.0007 | −0.0931 | −0.0736 | +0.0931 | −0.0736 | +0.0130 | +0.0007 | pair_b_apex |
| 7 | 6.50 | −0.0018 | −0.0001 | −0.0284 | −0.0377 | +0.0284 | −0.0377 | +0.0018 | +0.0001 | pair_b_swing_three_quarter |
| 8 | 7.00 | 0 | 0 | 0 | 0 | 0 | 0 | 0 | 0 | neutral_end_pair_b_touchdown |

**Offline preflight figures** (this commit, cloud):

| Check | Value | Limit |
|---|---|---|
| Max \|q − neutral\| at the waypoints | 0.1075 rad (`rf/lr_thigh`, point 5) | 0.1223 rad |
| Max \|q − neutral\| on the commanded spline (50 samples/segment) | **0.1114 rad** (`rf_thigh`) | 0.1223 rad (margin 0.0109) |
| Peak spline joint speed | **0.139 rad/s** | 0.25 rad/s planned (0.5 placeholder) |
| Max fraction of a URDF limit | 0.169 (`lr_foot`) | G3 trips at 0.8 |
| Min distance to URDF limit minus the soft margin | 0.313 rad | > 0 |
| Waypoint re-derivation (IK) vs stored | 0.0 (cloud) | ≤ 1e-8 |
| Points, segment, duration | 9, 0.5 s, 7.0 s (3.0 s lead-in + 4.0 s cycle) | ≤ 9, ≥ 0.5 s, ≤ 10 s |

---

## 2. Implementation decisions

1. **Reuse by subclass, never by edit.**
   - `M61Session` subclasses `m6_live_playback.LiveSession` and overrides only the gait-specific
     parts:
     - `_run`: M6.1 trajectory, preflight, spec, confirmation word, goal;
     - `_on_event`: the gates;
     - `_inflight_tracking` and `_tracking`: the Hermite reference;
     - `_finish_cancel` and `_finish_result`: `GATE_TRIPPED`;
     - `outcome`: the extended report fields.
   - It re-uses as is:
     - the one-shot dispatch latch;
     - readiness before and after confirmation;
     - freshness at the send;
     - the supervision loop (stream monitor, controller presence, watchdogs);
     - the single guarded cancel;
     - interrupt handling and `EvidenceFile`.
   - The coupling is to M6.0-D internals, which are hash-pinned. Any M6.0-D change fails those
     tests first, and M6.1 must then be re-verified.
2. **Pose freshness without copying the loop.** A `_TickTransport` wrapper appends one tick event
   to every `poll()`. This lets the re-used loop hand the session a chance to check body-pose
   freshness even when no message arrives. It sends, cancels and publishes nothing.
3. **Identity is layered.**
   - The YAML content must hash to the stored and the approved `content_sha256`.
   - Every waypoint must equal its IK re-derivation from the approved parameters within
     `source_match_rad` (1e-8). This is a tolerance, not a hash, so it is robust to libm
     round-off across machines.
   - The goal fingerprint binds `trajectory_id`, which includes the SHA-256 of `m61_limits.yaml`,
     `m61_trot_cycle.yaml`, the M6.0-D source configs and the expanded URDF.
   - The transport (unchanged M6.0-D adapter) refuses any other goal before any ROS call. That
     includes a return-to-neutral, a second cycle and the M6.0-D crouch goal.
4. **Approved values are pinned in code.** `m61_live_contract.APPROVED_LIMITS`, `APPROVED_GAIT`
   and `APPROVED_CONTENT_SHA256` must equal the YAML files. An edited YAML is loaded and reported,
   but the preflight refuses it (`limits_not_approved`, `gait_not_approved`,
   `content_not_approved`). Loosening a limit therefore needs a reviewed code change.
5. **Re-used M6.0-D monitor values are cross-checked.** G5 (5.0 s), G6 (0.25 s) and the 0.5 s
   joint-state staleness are enforced by the re-used `StreamMonitor`, which reads
   `m6_live_contract`. `m61_limits.yaml` must agree with those values
   (`limits_inconsistent_with_m6d_monitor`).
6. **Tracking reference = the controller's interpolation.** The goal carries positions and
   velocities but no accelerations, so the installed JTC interpolates cubic splines. M6.1 tracks
   against the same cubic Hermite, unlike M6.0-D's zero-velocity form.
7. **Outcome states.**

   | Cancel reason | Terminal state |
   |---|---|
   | G1 `body_too_low`, G2 `body_tilt`, G3 `joint_near_limit`, `body_pose_stale` | New `GATE_TRIPPED` |
   | G5 `sim_time_stalled` (re-used) | `READINESS_LOST` |
   | G6 `sample_gap` (re-used) | `TRACKING_FAILED` |

   - The gates report G5/G6 as tripped.
   - Exactly one cancel is ever sent, for the first reason.
   - Later trips are still recorded in `gates.trips_in_order`.
   - A trip during the post-result settle ends in `GATE_TRIPPED` without a cancel, because the
     goal has already ended.
8. **Readiness needs the body pose.** With `body_pose_required: true`, readiness is NOT READY
   unless a ground-truth pose is fresh (≤ 1.0 s), at least 0.045 m high and within 0.26 rad tilt.
   This applies both before and after the confirmation.
9. **Evidence is never overwritten.** The run directory is created with `mkdir`, and every file
   with `O_EXCL`. The live outcome goes through the M6.0-D `EvidenceFile` (phases `reserved`,
   `pre_send`, `final`). A failed supporting file is recorded in `evidence_files` and never hides
   the outcome.

---

## 3. Deviations from the design or the request (with justification)

| # | Deviation | Why |
|---|---|---|
| V1 | **The trajectory hash is `94a492c4…`, not `d03e80a0…`** | The request quoted `d03e80a0…`, which is the design-note table for **2.5 cm / 10 mm**. That variant was not approved: its continuous path reaches 0.1517 rad, above the 0.1223 rad cap. The approved 2 cm / 6 mm trajectory hashes to `94a492c4…` under the same scheme. Both are recorded in the YAML (`content_sha256`, `design_note_reference_sha256`); verification uses `94a492c4…`. A test proves the 2.5 cm / 10 mm variant exceeds the cap |
| V2 | Peak joint speed is **0.139 rad/s**, not 0.196 | 0.196 rad/s also belongs to the 2.5 cm / 10 mm table. The approved cycle peaks at 0.139 rad/s on the commanded spline |
| V3 | `max_trajectory_points: 9`, not 20 | The request said "20 (or as specified in design)"; the design specifies 9. The owner's names are available as `Limits.MAX_TRAJECTORY_POINTS`, `MIN_SEGMENT_DURATION` and `MAX_JOINT_DISPLACEMENT_RAD`. The YAML keys follow the repo convention: lower case with units |
| V4 | No `m60d_limits.yaml` was touched | That file does not exist. The M6.0-D limits live in `m6_envelope.py` and `m6_live_contract.py`, and both are unmodified (hash-pinned tests pass) |
| V5 | `scripts/m6_gait_replay.py` is a thin wrapper; the logic is in `spiderx_controller/m6_gait_replay.py` | This is the repo pattern (`scripts/m6_live_playback` → `spiderx_controller/m6_live_playback.py`), and it keeps the logic importable and testable |
| V6 | The trajectory YAML stores the waypoints, not only the parameters | The owner asked for the 9-waypoint trajectory in the file. The design said parameters only. Both are stored, and the IK re-derivation ties them together |
| V7 | **The fixed / clamped base is NOT implemented in this batch** | No fixed-base model or launch variant exists. Creating one is a URDF/launch change that can only be validated by running Gazebo, which this batch prohibits. It is design Batch D and needs its own approval. Until it exists, a live run is blocked (section 5) |
| V8 | **No launch file provides the body pose for `fortress_control`** | The ground-truth bridge exists only in `fortress_posture_hold.launch.py`. With `body_pose_required: true`, live readiness against plain `fortress_control.launch.py` would refuse (`body_pose_missing`). A pose-bridging M6.1 launch file is part of Batch D |
| V9 | G1 (body height) stays **active** on the fixed base | The design table marked body gates "N/A (logged only)" on a fixed base, but the owner approved the 4.5 cm gate. On a clamped base it detects a lowered or failed clamp |
| V10 | G2 (0.26 rad) and G7 keep the free-base design values | On a fixed base the expected tilt is about 0, so a tighter tilt limit is possible. That is an owner decision; no value was invented |
| V11 | `joint_states.csv` has `time_s` + 12 positions, no velocities | This is the owner's requested format. The unchanged M6.0-D adapter's joint-state events carry positions only |
| V12 | Mock and dry-run evidence paths | Live: `log/m61_run/<UTC>/`. Mock: `log/m61_run/mock/<UTC>_<scenario>/`. Dry run: `log/m61_run/dry_run/<trajectory_id>/` |
| V13 | One isolated-domain rclpy test was added (pose subscription) | Explicit domain 150–199, `ROS_LOCALHOST_ONLY=1`, no other node, an in-test publisher in place of the bridge. It uses no Gazebo, launch or controller, like the M6.0-D isolated tests. The owner can deselect it if it counts as "runtime" |
| V14 | A body-pose freshness gate was added (1.0 s wall) | Design section 3.2 makes the pose a readiness requirement. Without a freshness check during flight, G1 and G2 could go silently blind |

---

## 4. Testing status

Everything below ran in the cloud at this commit. There was no Gazebo, no launch and no live
goal.

| Check | Result |
|---|---|
| `colcon build --symlink-install` (setuptools 59.6.0, restored to 84.0.0 after) | **8 packages finished, 0 errors**. stderr holds only the existing CMake `cmake_minimum_required` deprecation warning |
| `colcon test` (`PYTEST_DISABLE_PLUGIN_AUTOLOAD=1`) | **1332 tests, 0 errors, 0 failures, 0 skipped** |
| `test_m61_gait_replay` | **118 tests, 0 failures** |
| Existing M6.0-D tests (pins, fingerprint, isolated success, live-enabling) | All pass: M6.0-D files unchanged |
| Leftover processes after the tests | None (no ROS / Gazebo process) |

What `test_m61_gait_replay.py` proves:
- **Gate:**
  - `M61_LIVE_DISPATCH_ENABLED` is a single `False` literal, pinned in `test/m61_gate.py`;
  - no environment variable or flag can enable it;
  - `--live` returns exit 3 before loading the configuration, reading input or importing rclpy
    (checked in a subprocess);
  - the M6.0-D gate is separate and unchanged.
- **Confirmation:** only the exact `SEND-ONE-TROT-CYCLE` is accepted. The M6.0-D word, other
  case, spaces, EOF and interrupts are all refused.
- **Limits:**
  - the file equals the approved values;
  - it agrees with the re-used M6.0-D monitor;
  - malformed files and duplicate keys are refused;
  - an edited limit refuses the goal.
- **Trajectory:**
  - the file loads with its metadata;
  - the hash verifies;
  - any edit to the content or the stored hash is refused;
  - the waypoints re-derive from the approved parameters;
  - the 2.5 cm / 10 mm design variant exceeds the cap.
- **Preflight:**
  - the stored trajectory passes, with the margins in section 1.1;
  - 16 mutations are each refused with their documented code, and no goal or spec is built for
    any of them.
- **Goal:**
  - every message field;
  - `trajectory_id` and fingerprint are pinned;
  - an appended return point, and the M6.0-D crouch goal, both fail the fingerprint.
- **Gates (pure):**
  - G1 debounce;
  - G2 roll and pitch;
  - G3 on both sides of 0, with non-finite values ignored;
  - nothing trips while not gating;
  - pose freshness;
  - pose readiness codes;
  - G7 is report-only;
  - G4 reports `not_measured`.
- **Session (mock):** all 14 scenarios, with the outcomes below.
  - At most one goal and one cancel, and never a retry or return goal.
  - The second `run()` is forbidden.
  - A post-result trip fails without a cancel.
  - Two trips produce one cancel, and both trips are recorded.
  - A low body at readiness sends nothing.
  - A preflight failure sends nothing.

  | Scenario | Final state | Cause |
  |---|---|---|
  | success | SUCCEEDED | – |
  | body_drift_report_only | SUCCEEDED | G7 flags only |
  | interrupt | CANCEL_CONFIRMED | Ctrl+C |
  | tracking_error | TRACKING_FAILED | 0.06 rad offset |
  | stale_joint_states | READINESS_LOST | 0.5 s |
  | controller_lost | HELD_ERROR | – |
  | rejected | REJECTED | – |
  | body_too_low | GATE_TRIPPED | G1 |
  | body_tilt | GATE_TRIPPED | G2 |
  | joint_near_limit | GATE_TRIPPED | G3 |
  | sim_stall | READINESS_LOST | G5 |
  | joint_state_gap | TRACKING_FAILED | G6 |
  | body_pose_stale | GATE_TRIPPED | pose freshness |
  | no_body_pose | REFUSED | 0 goals |

- **Evidence (mock CLI):**
  - the run directory holds all 11 files, with the CSV headers and row counts checked;
  - a refusal still records evidence;
  - `--no-write` writes nothing;
  - dry-run and run directories are never overwritten.
- **Gated live wiring (gate set True inside the test only, in-memory mock transport):**
  - writes `log/m61_run/<UTC>/live_outcome.json` (phase `final`) plus all supporting files;
  - refuses to overwrite an existing run;
  - refuses on a gate trip, a missing pose or the wrong word;
  - still needs `--domain-id` and refuses `--no-write`.
- **Isolated domain:** `M61RclpyLiveTransport` parses the model pose from a `TFMessage`, ignores
  messages without the model, sends no goal or cancel, and releases everything on `close()`.

**Not tested (cannot be, without the prohibited runtime):**
- the real controller's spline;
- real `/joint_states` timing;
- the real Gazebo pose bridge;
- a fixed-base model;
- anything about the robot's motion.

---

## 5. Verification checklist: before ANY live M6.1 run

Every item must be true, and each run needs a separate owner approval. Status at this commit:

| # | Item | Status | How to check |
|---|---|---|---|
| 1 | Build passes (8 packages, no errors) | ✅ cloud | `colcon build --symlink-install`: "8 packages finished" |
| 2 | All tests pass (mock / isolated) | ✅ cloud: 1332 / 0 failures | `PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 colcon test && colcon test-result --all` |
| 3 | Trajectory hash matches | ✅ `94a492c4…` (approved 2 cm / 6 mm; see V1, **not** `d03e80a0…`) | `ros2 run spiderx_controller m6_gait_replay.py --dry-run --no-write` prints content `94a492c43fcc0461`, trajectory `241760e7dfd5ef12`, fingerprint `9dba1a17…` |
| 4 | `M61_LIVE_DISPATCH_ENABLED` is False | ✅ | `grep -n "M61_LIVE_DISPATCH_ENABLED =" src/spiderx_controller/spiderx_controller/m61_live_contract.py` and `test/m61_gate.py` |
| 5 | No leftover processes from previous runs | ✅ cloud (none) / ⏳ owner PC | `pgrep -af "ign gazebo\|gz sim\|ros2\|parameter_bridge\|controller_manager"` is empty, then `ros2 daemon stop` |
| 6 | Controllers active in the launch config | ⏳ runtime only | Static: `spiderx_ros2_controllers.yaml` defines `joint_state_broadcaster` and `leg_trajectory_controller` (hash-pinned). At run time, readiness refuses unless `list_controllers` reports both `active` |
| 7 | Fixed / clamped base variant exists and is verified | 🟡 implemented by M6.1-A, verified offline only ([implementation](M61A_FIXED_BASE_IMPLEMENTATION.md)); ⏳ phase 1 observation | `ros2 run spiderx_controller m61a_observe_fixed_base --domain-id 0 --preflight` READY on `fortress_m61a_fixed_base.launch.py` |
| 8 | Ground-truth pose bridge is in the M6.1 launch | 🟡 in `fortress_m61a_fixed_base.launch.py` (M6.1-A), not yet run | The body pose is now COMPOSED (`T_world_model · T_model_body`); without the bridge readiness still refuses (`body_pose_missing`) |
| 9 | Owner-side offline / mock / isolated verification on the Ubuntu PC | ⏳ | Items 1–4, then `--mock --scenario success` and one gate scenario |
| 10 | Read-only graph preflight on the M6.1 launch (no goal) | ⏳ | Separate approval |
| 11 | Final audit, then the enabling change on a never-merged branch (flips `M61_LIVE_DISPATCH_ENABLED` and `test/m61_gate.py` only) | ⏳ | Separate approval |
| 12 | One owner live run, results document, gate re-disabled | ⏳ | Separate approval |

Offline and mock commands (no simulation is needed, and nothing is sent to any graph):

```bash
cd ~/spiderx_ws && source install/setup.bash
ros2 run spiderx_controller m6_gait_replay.py --dry-run
echo SEND-ONE-TROT-CYCLE | ros2 run spiderx_controller m6_gait_replay.py --mock --scenario success
echo SEND-ONE-TROT-CYCLE | ros2 run spiderx_controller m6_gait_replay.py --mock --scenario body_too_low
ros2 run spiderx_controller m6_gait_replay.py --live --domain-id 0     # REFUSED, exit 3
```

---

## 6. Allowed claim and non-claims

**Allowed now:** "The M6.1 trot-cycle replay tool is implemented and verified offline, with mocks
and on an isolated ROS domain; live dispatch is hard-disabled."

**Not allowed:**
- that any goal was sent;
- that anything moved, stepped, walked or balanced;
- any claim about contact (not measured);
- any claim about the fixed base (not built);
- anything about hardware.
