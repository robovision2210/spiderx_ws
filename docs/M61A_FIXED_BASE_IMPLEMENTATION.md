# M6.1-A – Fixed-Base Simulation and Body-Pose Observability: Implementation

**Status: IMPLEMENTED and verified in Cloud offline / mock / isolated-domain tests only.** Nothing
here has run in Gazebo. Every number marked *provisional* awaits the local launch-only observation
(§10). Both live gates stay `False`:
- `m6_live_contract.LIVE_DISPATCH_ENABLED = False` (M6.0-D);
- `m61_live_contract.M61_LIVE_DISPATCH_ENABLED = False` (M6.1).

Design: [`M61A_FIXED_BASE_POSE_OBSERVABILITY_DESIGN.md`](M61A_FIXED_BASE_POSE_OBSERVABILITY_DESIGN.md)
(saved at `acf36ac`). This document records what was built, why, the evidence and what is still
pending. It does not authorize any launch or replay.

## 0. Status at a glance

| Item | Implemented | Tested offline / mock / isolated (Cloud) | Tested in the simulator | Pending |
|---|---|---|---|---|
| Fixed-base wrapper `spiderx_fixed_base.urdf.xacro` (Approach B) | ✅ | ✅ xacro, `check_urdf`, `ign sdf -p`, converter rule | ❌ | Launch-only observation (§10, phase 1) |
| Dedicated launch `fortress_m61a_fixed_base.launch.py` | ✅ | ✅ loaded with `importlib`, nodes and arguments checked | ❌ | Phase 1 |
| Frame composition `T_world_body = T_world_model · T_model_body` | ✅ | ✅ numeric (general rotations) and against sdformat output | ❌ | Phase 1 confirms the Gazebo side |
| Clearance analysis and mounting height **0.125 m** (provisional) | ✅ | ✅ exact meshes, guaranteed bounds | ❌ | Phase 1: no contact, observed height |
| Body-pose observation, attachment check, G8 | ✅ | ✅ pure + mock + isolated domain | ❌ | Phase 1: noise and gaps decide the tolerances |
| Read-only observer / preflight `m61a_observe_fixed_base` | ✅ | ✅ fake observer + isolated-domain fake peers | ❌ | Phase 1 |
| M6.1 replay integration (fixed-base readiness, composed pose, G8) | ✅ | ✅ 14 + 5 mock scenarios, isolated domain | ❌ | Phase 2 (separate approval) |
| Live dispatch | ❌ hard-disabled | ✅ refusal tested | ❌ | Separate owner approval and enabling change |

---

## 1. What was built

| File | Role |
|---|---|
| `src/spiderx_description/urdf/spiderx_fixed_base.urdf.xacro` | **New.** Includes the unchanged `spiderx.urdf.xacro` and adds `<link name="world"/>` plus ONE fixed joint `spiderx_fixed_base_weld` world → `dummy_link` whose origin is the mounting transform (x, y, z, yaw; roll = pitch = 0). Defaults: (0, 0, **0.125**, 0) |
| `src/spiderx_bringup/launch/fortress_m61a_fixed_base.launch.py` | **New.** Gazebo (unchanged world), `robot_state_publisher` from the wrapper, `create` at the **identity** pose, `spiderx_gz_bridge` (control YAML), the gz_ros2_control plugin path, the unchanged `controller.launch.py`, the ground-truth pose bridge (same as posture hold), optional RViz. Arguments: `headless`, `rviz`, `gz_verbosity` only |
| `src/spiderx_controller/config/m61a_fixed_base.yaml` | **New.** The plant and observation parameters (mount, clearance minimum, tolerances, freshness, graph expectations). Strict loader. Not an input of the M6.1 trajectory identity |
| `spiderx_controller/m61a_fixed_base.py` | **New, pure.** Quaternion poses and composition, the strict config loader, `/robot_description` weld parsing, the SDF pose resolver, pose-entry selection and validation, body composition, attachment/spawn/link checks, `ClockMonitor`, readiness (`assess_plant`, `assess`), and the shared `FixedBasePoseTracker` used by the live transport, the observer and the mock |
| `spiderx_controller/m61a_clearance.py` + `scripts/m61a_clearance` | **New.** Offline collision-geometry clearance analysis (numpy) and its CLI |
| `spiderx_controller/m61a_observer.py` + `scripts/m61a_observe_fixed_base` | **New.** The read-only observer / preflight CLI |
| `spiderx_controller/m61a_live.py` | **New.** `M61AFixedBaseTransport`: the M6.1 rclpy transport plus a read-only `/robot_description` subscription; its body pose is the composed fixed-base body |
| `spiderx_controller/m61_gates.py` | **Changed.** New gate **G8 attachment** (cancel reason `attachment_lost`), inactive without a fixed-base config |
| `spiderx_controller/m6_gait_replay.py` | **Changed.** `with_fixed_base` readiness, `fixed_base_record` evidence, the session handles `fixed_base` events, the live and mock paths use the fixed base |
| `spiderx_controller/m61_mock.py` | **Changed.** Fixed-base mode (the welded plant through the real tracker) and 5 new scenarios |
| `test/test_m61a_fixed_base.py`, `test/test_m61a_clearance.py`, `test/test_m61a_observer.py` | **New** tests (§9) |
| `test/test_m61_gait_replay.py` | **Changed** where the fixed base changes behaviour (§8) |
| `CMakeLists.txt`, `package.xml` | Two scripts installed, three tests registered; `std_msgs`, `rosgraph_msgs`, `python3-numpy` exec dependencies |

**Not changed:** the free-base model (`spiderx.urdf.xacro` and its includes), every existing launch
file, both bridge YAMLs, the world, the controller YAML, every M6.0-D module and both gate pins.
The M6.0-D hash-pin tests prove the M6.0-D files are byte-identical.

---

## 2. Frames and equations

**Frames.**

| Name | Meaning |
|---|---|
| `world` (Gazebo) | World `spiderx_fortress`, ground plane at z = 0, +z up |
| `world` (URDF/TF root) | The wrapper's root link. Because the model is spawned at the identity, it **is** the Gazebo world (Approach B) |
| model | Gazebo model frame `spiderx::__model__` |
| `dummy_link` | The canonical Gazebo link. `base_link` is merged into it by fixed-joint reduction |
| `base_link` | The body frame: +x right, +y front, +z up (not REP-103) |

**Composition order.** `T_a_c = T_a_b · T_b_c` ("pose of c in a"). Rotations are unit quaternions
(x, y, z, w). Euler angles are ZYX (roll about x, pitch about y, yaw about z), as in
`posture_metrics.quat_to_rpy`. Units: m, rad, s.

**What the converter does (verified, sdformat 12.8.0).** `ign sdf -p` turns the weld into a fixed
joint with `<parent>world</parent>` and pose = the weld origin **relative to `__model__`**.
`dummy_link` is placed relative to that joint. The model element has **no** `<pose>`. Therefore:

```
T_model_dummy = T_weld_origin                       (converter rule)
T_model_body  = T_model_dummy · T_dummy_body         (T_dummy_body = dummy_joint origin = identity)
T_world_body  = T_world_model · T_model_body         (never "model z + offset")
```

`test_converter_places_the_weld_origin_between_model_and_body` checks this from real `ign sdf -p`
output for (0, 0, 0.125, 0) and for (0.31, −0.17, 0.43, yaw 0.7). The same test checks that the
body-relative frames (hips, foot joints, lidar) are unchanged by the weld.
`test_converter_rule_also_holds_for_general_rotation` repeats the check with roll and pitch in a
temporary description. The wrapper itself only allows yaw.

**Why the canonical-link name is not enough.** With a non-zero weld the model frame is attached to
`dummy_link` but sits `T_weld` away from it, so the Gazebo model pose is **not** the body pose.
Consumers therefore always compose (§4).

**Spawn.** The launch passes `-x 0 -y 0 -z 0 -R 0 -P 0 -Y 0`. A non-zero spawn would apply the
mount a second time. The default `-z 0.075` of `fortress.launch.py` would put the body 0.075 m too
high. Readiness refuses that case as `frame_spawn_not_identity`.

---

## 3. Mounting height and ground clearance (provisional 0.125 m)

**Question.** If `base_link` is welded at height h with zero roll and pitch, how close can **any**
collision geometry come to the ground? For a body-frame point v, z_world = h + z_body(v); yaw and
x/y do not change heights.

**Method** (`m61a_clearance`, offline):
- **Geometry:** all 31 links' `<collision>` elements from the expanded URDF:
  - every unique STL vertex, with mesh scale and collision origin applied (the lowest point of a
    triangle mesh is always a vertex, so each configuration's minimum is exact);
  - analytic primitives (the lidar cylinder).
- **Kinematics:** the URDF tree. A test checks it against the M3/M4 leg kinematics to 1e-12 m, and
  the neutral foot minima equal the M3-derived foot tips.
- **Sampled versus guaranteed:**
  - "sampled" values are exact only at the evaluated configurations;
  - "bound" values are guaranteed lower bounds over a continuous set, from a Lipschitz argument:
    |dz/dq_j| ≤ R_j, where R_j is a triangle-inequality bound on the distance from joint j's axis;
  - between time samples, the bound uses the exact maximum joint speed of each cubic-Hermite
    segment (its derivative is quadratic);
  - over joint boxes, branch and bound per leg proves the minimum to within 1 mm.

**Envelopes** (metres below `base_link`; trajectory `241760e7dfd5ef12`, content `94a492c4…`):

| Envelope | Sampled | Guaranteed bound | Limiting link |
|---|---|---|---|
| Neutral (start and end posture) | 0.05454 | exact | `rf_foot_1` |
| Commanded cubic-Hermite spline, 200 samples per 0.5 s segment | 0.05459 (t = 5.0025 s) | 0.05463 | `rf_foot_1` |
| Initial transition: any readiness-accepted start (±0.05 rad) plus tracking (±0.05 rad) around neutral | 0.06603 | 0.06703 | `lf_foot_1` |
| Spline ± tracking tolerance 0.05 rad (before the abort) | 0.06091 | 0.06227 | `rr_foot_1` |
| **Every reachable configuration: URDF limits widened by 0.02 rad** (passive, fault hold, controller loss) | 0.10806 | **0.10906** | `rr_foot_1` (knee at its limit) |

The commanded motion and its interpolation stay within 0.1 mm of the neutral foot height. The lowest
**commanded** point is the stance feet. Only joint motion beyond the command (tracking error, or
passive sag up to the limits) brings geometry lower.

**Chosen mount.** The basis is the **reachable-set** bound, so that no joint configuration the
simulator can reach lets the robot touch the ground, whatever the controllers do:

```
h = ceil_5mm( 0.10906 + margin 0.015 ) = 0.125 m
```

| Clearance at h = 0.125 m | Value |
|---|---|
| Neutral | 70.5 mm |
| Commanded spline (bound) | 70.4 mm |
| Initial transition and tracking box (bound) | 58.0 mm |
| Every reachable configuration (bound) | 15.9 mm |

**Margin, 0.015 m.** It covers:
- the 3 mm attachment tolerance (a weld that sags less than that is not flagged);
- the contact surface layer and collision margins of the physics engine (millimetres);
- joint-limit softness beyond the 0.02 rad already included;
- CAD/model error.

The margin is a judgement, not a measurement: phase 1 must show no contact and the expected
height.

**The earlier 0.075 m candidate** clears the commanded motion (20.5 mm at neutral, 8.0 mm worst
bound with tracking) but **not** the reachable set (−34 mm): a passive leg sag could touch the
ground. It was rejected.

**Limitations.**
- Collision geometry is the CAD STL (steel-density masses, unweighed parts) plus the primitive
  lidar.
- The ground is the flat plane z = 0.
- Gazebo contact margins are not modelled, and nothing about contact forces is claimed.
- The commanded envelope is a guaranteed bound; the reachable-set bound depends on Gazebo keeping
  joints within limit + 0.02 rad (an assumption for phase 1 to check).
- The weld is assumed rigid at exactly zero roll and pitch. A weld that sags or tilts within the
  attachment tolerance (3 mm, 0.01 rad) can lower geometry by up to about 3 mm + 0.01 rad × the
  largest horizontal extent of the collision geometry; the 15 mm margin is meant to cover it,
  and G8 trips beyond the tolerance.

Reproduce with `ros2 run spiderx_controller m61a_clearance`; the report is
`log/m61a_clearance/<UTC>/clearance.json`. A preserved copy, regenerated twice from a recorded
revision with its hashes, command and environment, is in
[`docs/evidence/m61a/`](evidence/m61a/README.md) (§14). `--check-config` recomputes only the reachable-set bound
(about 30 s) and compares it with the config. The test `test_config_agrees_with_a_fresh_passive_bound`
does the same.

---

## 4. Body-pose observability

**Source** (code-verified; runtime facts are historical from M2/M3):
- Gazebo `/world/spiderx_fortress/pose/info` (SceneBroadcaster) is bridged to
  `/spiderx/sim/world_poses` (`tf2_msgs/TFMessage`), exactly as in the posture-hold launch.
- The entry with `child_frame_id == "spiderx"` is the **model** pose in the world.
- An entry whose `child_frame_id` is a link name is that **link's pose relative to the model**.
  M3 compared `lf_foot_1` this way to 5.4e-11 m.
- Per-pose stamps are 0 at the bridge.

**What measures the physical body.** The body is
`T_world_model · T_model_dummy(Gazebo entry) · T_dummy_base`:
- It uses Gazebo's own `dummy_link` entry, not the description, so a body that moves relative to
  the model root is seen whichever way Gazebo updates the model pose.
- `base_link` is merged into `dummy_link` (no runtime joint), so there is no internal attachment
  that could fail unseen.
- `/robot_description` is never trusted alone. Its weld must **match** Gazebo's `dummy_link` entry
  (`frame_body_link_inconsistent` otherwise), which associates the description with the spawned
  model.

**The physical body versus a stationary model root.** A welded model can be reported two ways:
Gazebo may keep the model root fixed and move the `dummy_link` entry, or move the model root with
its canonical link. The composition covers both:
- a body that moves while the root stays put changes the `dummy_link` entry:
  `frame_body_link_inconsistent` and `attachment_displaced`;
- a root that moves with the body: `frame_spawn_not_identity` and `attachment_displaced`.

Neither case can pass as "attached", because the attachment check uses the composed body, never
the root alone.

**Association with the spawned model:**
- the entries are selected by name: `spiderx` is the `create -name` of the launch, and
  `dummy_link` is its canonical link;
- two entries with either name make the sample ambiguous (refused);
- the `dummy_link` entry must agree with the description's weld to 0.1 mm;
- `header.frame_id` values are recorded, not gated (their runtime content is unverified).

**What this cannot detect.** If `pose/info` itself stopped reflecting the simulated body (a stale
or wrong scene broadcast while the body moved), every pose check would be blind. Freshness only
proves that messages arrive. Phase 1 therefore adds an independent cross-check: the body-mounted
lidar's `/scan` ranges to the world walls must match the welded pose (M0 measured them within
±8 mm of the world geometry).

**Handling.** Each problem is a separate code, never guessed around:

| Problem | Code | Effect |
|---|---|---|
| No `spiderx` / `dummy_link` entry | `pose_missing_model`, `pose_missing_body_link` | The sample is unusable and does not refresh freshness |
| Two entries with the same name | `pose_ambiguous_model`, `pose_ambiguous_body_link` | Same |
| NaN or Inf | `pose_nonfinite` | Same |
| Quaternion norm off by more than 1e-3 | `pose_bad_quaternion` | Same (within 1e-3 it is normalized) |
| Gazebo link entry ≠ description weld (> 0.1 mm or 1e-4 rad) | `frame_body_link_inconsistent` | Not ready; G8 in flight |
| Model root not at identity (> 3 mm or 0.01 rad) | `frame_spawn_not_identity` | Not ready; G8 in flight |
| Composed body away from the weld pose (> 3 mm or 0.01 rad) | `attachment_displaced` | Not ready; G8 in flight |
| No usable sample for 1.0 s (wall, receipt time) | `body_pose_stale` / `body_pose_missing` | Not ready; pose-freshness cancel in flight |

**Time.**
- Pose and joint-state freshness are judged by **monotonic wall receipt time**, because the bridge
  stamps are zero.
- Simulation-clock progress is judged by how long, in wall time, `/clock` has not advanced:
  `sim_time_stalled` after 5.0 s (pause, freeze, lost bridge).
- `/clock` going back by more than 1 ms is `sim_time_reset` and **latches**, so evidence from
  before a world reset is never reused.
- The two clocks are never mixed in one comparison.

**Separate checks.** Pose freshness, joint-state freshness, clock progress, body height (G1), tilt
(G2) and attachment displacement (G8) are separate checks with separate codes.

---

## 5. Attachment integrity (G8) and the provisional tolerances

G1 (body height ≥ 0.045 m) **cannot** detect a failed weld: a detached body would settle near
0.0545 m, above the threshold. `test_a_detached_body_resting_above_the_height_gate_is_still_caught`
shows that G8 does catch it.

G8 trips (one cancel, `attachment_lost` → `GATE_TRIPPED`) after **2** consecutive samples with
`attachment_displaced`, `frame_spawn_not_identity` or `frame_body_link_inconsistent`.

| Tolerance (provisional) | Value | Measured in | Why this value | How phase 1 decides it |
|---|---|---|---|---|
| Attachment translation | 3 mm | World, composed body vs expected weld pose | A detached body falls about 70 mm (0.125 → about 0.0545 m), so 3 mm detects any real failure with about 20× margin. It stays well below the 15 mm clearance margin. M2's free-base rest noise was 0.07 mm | Over ≥ 120 s stationary, max deviation must stay below 1/3 of the tolerance (≤ 1 mm). Otherwise the owner re-decides |
| Attachment rotation | 0.01 rad | Same | A falling or tipping body rotates far more; 0.57° stays above numerical noise | Same rule (≤ 0.0033 rad) |
| Spawn identity | 3 mm / 0.01 rad | Model root vs identity | Catches double counting (75 mm with the default spawn) | Same |
| Link consistency | 0.1 mm / 1e-4 rad | Gazebo `dummy_link` entry vs description | Both come from the same SDF pose and should agree to float precision | Observed maximum is recorded |
| Debounce | 2 samples | – | Same as G1/G2 | Pose gap statistics |

The historical 0.005 m / 0.10 rad M2 values were **not** adopted. They are the M2 posture-hold
acceptance values for a free body and are too loose to detect a weld creeping.

---

## 6. The read-only observer / preflight

```
ros2 run spiderx_controller m61a_observe_fixed_base --interface-only
ros2 run spiderx_controller m61a_observe_fixed_base --domain-id 0 --preflight      # ~4 s
ros2 run spiderx_controller m61a_observe_fixed_base --domain-id 0 --duration 120   # statistics
```

**What its node has:**
- subscriptions: `/spiderx/sim/world_poses`, `/joint_states`, `/clock` (best effort) and
  `/robot_description` (transient local);
- one service client, `list_controllers`;
- graph queries: publisher counts and names.

**What it has not:** any publisher, action client, goal or command. A static source test enforces
this.

**READY needs all of these:**
- a fixed-base description with the approved mount ≥ 0.125 m;
- link consistency, spawn identity and attachment within tolerance;
- a fresh pose;
- fresh, complete 12-joint `/joint_states` from exactly one publisher;
- a progressing `/clock` with no reset;
- both controllers `active`;
- **no other commander visible**: no publisher on `/leg_trajectory_controller/joint_trajectory`
  and no FollowJointTrajectory client (counted as subscribers of
  `/leg_trajectory_controller/follow_joint_trajectory/_action/status`; the observer has none of
  its own). An unmeasured count is not READY (`command_owner_unknown`).

**Command ownership is a graph observation, not a lock.**
- ROS 2 has no exclusive ownership of a topic or an action: the controller accepts topic commands
  and goals from anyone.
- The counts are a snapshot at the moment of the query. DDS discovery adds a delay before a new
  participant is counted: 0.05–0.06 s on a localhost-only private domain in the Cloud tests
  (`test_observer_sees_a_late_competing_commander_after_discovery`). There is no general bound,
  and it is slower across hosts.
- A commander that appears after the last snapshot, or publishes once and leaves between
  snapshots, is **not** seen.
- What then happens is the controller's behaviour: a later goal or topic command preempts the
  M6.1 goal. M6.1 then ends **not SUCCEEDED** (preempted, cancelled or aborted, or a tracking or
  G-gate trip), which is reported, never silently accepted.
- Codes: `competing_command_publishers`, `competing_action_clients`.

**Statistics (long mode):**
- body z mean, minimum, maximum and range;
- xy and yaw drift;
- maximum roll, pitch and tilt;
- maximum attachment, spawn and link deviation, and the fraction of the tolerance used;
- pose and joint-state receipt gaps (mean, max, p99, p99.9);
- the real-time factor and poses per simulated second;
- sample codes;
- the `header.frame_id` values seen (recorded, not gated: their runtime content is unverified).

**Exit codes and evidence:**
- exit 0 READY, 1 NOT READY or interrupted, 2 usage;
- evidence in `log/m61a_observation/<UTC>/observation.json`, never overwritten;
- the config SHA-256 is recorded.

**Partial initialization** (for example an RMW failure) is reported as `observer_failed`, and
`close()` still releases everything.

The launch's own controllers (`joint_state_broadcaster`, `leg_trajectory_controller`) start as
usual and **hold** their positions. The observer sends them nothing.

---

## 7. M6.1 integration

**Command owner (review correction).** `with_command_owner` wraps the M6.1 readiness provider:
- **Rule.** NOT READY while another commander is visible:
  - a publisher on the controller's topic;
  - a FollowJointTrajectory client other than the transport's own (its own client counts once
    and is subtracted).
- **When it runs.** It is re-evaluated at every readiness observation, including the
  re-observation immediately before dispatch.
- **Mock scenarios:** `competing_publisher`, `competing_client` and `competing_client_late`. In
  the late case a client appears after a clean readiness #1; the pre-dispatch re-check refuses
  it, and 0 goals are sent.
- **Not covered.** A commander appearing after that last re-check is not seen by readiness
  (§6).

- **Readiness** (`m6_gait_replay.with_fixed_base`, before and after the confirmation): adds
  `m61a_fixed_base.assess_plant` to the existing M6.0-D readiness and body-pose readiness. That
  covers the description and mount, link consistency, spawn identity and attachment.
- **Body pose** for G1, G2 and G7 is the **composed** body (live: `M61AFixedBaseTransport`; mock:
  the same tracker).
- **G8** runs in flight. The outcome gains `fixed_base`: the config and its SHA-256, the
  description SHA-256 and the plant checks. This is the **instantiated-plant evidence**, kept apart
  from the trajectory provenance.
- **Trajectory identity is unchanged.** `trajectory_id 241760e7dfd5ef12`, goal fingerprint
  `9dba1a17…` and content `94a492c4…` are unchanged: the plant config is deliberately not an input
  of the trajectory identity, because it describes the plant and not the command. M6.0-D
  `44f0a7ad52e5c330` / `0d6ef417…` is unchanged too.
- **New mock scenarios:**

  | Scenario | Result |
  |---|---|
  | `attachment_drift` | G8 → `GATE_TRIPPED`, 1 cancel |
  | `not_fixed_base` | REFUSED, 0 goals |
  | `spawn_offset` | REFUSED, 0 goals |
  | `description_mismatch` | REFUSED, 0 goals |
  | `mount_not_approved` | REFUSED, 0 goals |

---

## 8. Behaviour changes in existing tests (explained, not weakened)

The CLI `--mock` now plays the welded plant, because M6.1 is only valid on the fixed base:

| Test expectation | Was | Now | Physical reason |
|---|---|---|---|
| Mock body height | 0.0545 m (free base at rest) | 0.125 m (the weld) | The plant changed |
| G1 margin in the success outcome | 0.0095 m | 0.080 m | Same |
| `body_too_low`, `body_tilt` tripped gates | G1 / G2 | G1 then G8 / G2 then G8 (the first reason still ends the run, one cancel) | A body that drops or tilts on a weld has left the weld pose |
| `body_drift_report_only` scenario | SUCCEEDED with G7 flags | Replaced by `attachment_drift` → G8 trip. G7 report-only is still tested on the free-base mock | On a weld any drift G7 could flag is first an attachment failure |
| Test transport factory | `(fp, domain_id)` | `(fp, domain_id, fixed_base)` | The live path needs the fixed-base config |
| `assess` evidence (review correction) | READY without an action-client count | NOT READY unless `action_clients == 0` (`competing_action_clients`); the observer now measures it | Another FollowJointTrajectory client is a competing commander too |
| M6.1 mock scenarios (review correction) | 18 | 21: `competing_publisher`, `competing_client`, `competing_client_late` (all REFUSED, 0 goals) | Readiness now includes the command-owner observation |

---

## 9. Verification (Cloud)

**Environment:**
- RoboStack ROS 2 Humble, Gazebo Fortress (libignition-gazebo6 6.16.0), sdformat 12.8.0, ros_gz
  0.244.20, xacro 2.0.13, urdfdom 4.0.1, Python 3.11.16, numpy 1.26.4;
- build with Ubuntu 22.04's setuptools 59.6 from a private `--target` directory (the shared
  environment keeps 84.0.0; ≥ 80 breaks `--symlink-install` for `ament_python`);
- `PYTEST_DISABLE_PLUGIN_AUTOLOAD=1`.

| Check | Result |
|---|---|
| Baseline before any change (`acf36ac`) | Build 8/8 packages; `colcon test`: **1330 tests, 0 errors, 0 failures, 0 skipped** |
| Clean build at M6.1-A (`rm -rf build install log; colcon build --symlink-install`) | **8 packages finished**, exit 0 |
| `colcon test --packages-select spiderx_controller spiderx_scripts` + `colcon test-result --verbose` | **1448 tests, 0 errors, 0 failures, 0 skipped** (+118: 72 + 11 + 22 new, 10 more in `test_m61_gait_replay` (118 → 128), 3 CTest wrapper entries). The isolated-domain tests ran (none skipped). One test was later moved between files (71 / 129): both files re-run, 200 passed |
| Static validators `validate_fortress.sh`, `validate_m1_control.sh`, `validate_m2_posture.sh`, `validate_m3_kinematics.sh`, `validate_m4_all_leg_ik.sh` | All exit 0 ("All … checks passed.") |
| `m6_live_playback --dry-run --no-write` (M6.0-D, protected) | `Dry run PASS: trajectory 44f0a7ad52e5c330, goal fingerprint 0d6ef417…` |
| `m6_gait_replay.py --dry-run --no-write` (M6.1) | `Dry run PASS: trajectory 241760e7dfd5ef12, content 94a492c43fcc0461, goal fingerprint 9dba1a17…` |
| `m6_gait_replay.py --mock` | `success`: SUCCEEDED, 1 goal, 0 cancels. `attachment_drift`: GATE_TRIPPED `attachment_lost`, 1 goal, 1 cancel. `spawn_offset`: REFUSED, 0 goals |
| `--live` of both tools | Exit 3 (refused; gates `False`) |
| `m61a_observe_fixed_base --interface-only` | Exit 0; no publisher or action client |
| `m61a_clearance` (full report) | The §3 numbers; `config check: consistent`; 2 min 50 s |

**What the new tests prove:**
- **`test_m61a_fixed_base.py`:**
  - composition against independent 4×4 matrices for 200 random general rotations, and Euler/tilt
    conventions against `posture_metrics`;
  - the config strict loader, and the config equal to the wrapper defaults;
  - the wrapper equals the plain model plus exactly world + weld (both control modes), passes
    `check()`, `check_ros2_control` and `check_urdf`, and its mount arguments reach the weld;
  - free-base files contain no weld;
  - nine malformed descriptions are refused;
  - the converter rule from real `ign sdf -p` output (two mounts and a general rotation);
  - entry selection (missing, ambiguous, non-finite, non-unit, normalized), full composition,
    double-counted spawn, description/spawn mismatch, attachment thresholds (2.9 mm passes,
    3.1 mm and 0.0101 rad trip), the detached body above G1, debounce;
  - `/clock` progress, pause, reset latch;
  - wall-receipt freshness with zero source stamps;
  - the tracker;
  - `assess` READY and each failure code.
- **`test_m61a_clearance.py`:**
  - all 31 collision geometries;
  - the neutral minimum equals the M3 foot tips;
  - FK equals the M3/M4 leg kinematics;
  - the Hermite speed bound is exact (0.139 rad/s peak);
  - Lipschitz box bounds and branch and bound are conservative against random sampling;
  - the commanded bound is below dense samples;
  - the config agrees with a fresh reachable-set bound, and 0.075 m is shown to touch.
- **`test_m61a_observer.py`:**
  - the static read-only source scan;
  - `--interface-only` creates no node;
  - usage errors;
  - statistics;
  - the CLI READY and NOT READY paths with never-overwritten evidence, and partial initialization;
  - the launch file (identity spawn, wrapper, pose bridge, control bridge YAML, controller include,
    no mount or spawn arguments, no motion tool);
  - **isolated domain** (150–199, `ROS_LOCALHOST_ONLY=1`, in-test fake peers):
    - observer READY against a fixed-base description;
    - NOT READY against a free-base description and against a 0.075 m spawn offset;
    - `M61AFixedBaseTransport` composes the body and stays read-only;
  - the installed script runs `--interface-only`.
- **`test_m61_gait_replay.py`:** every mock scenario, including the 5 new ones, and the fixed-base
  outcome record. The identity pins are unchanged.

---

## 10. Local acceptance, phases 1 and 2 (owner's Ubuntu PC; each needs explicit approval)

Every phase keeps both gates `False` unless a separate, reviewed enabling change says otherwise.
Record the exact commit (`git rev-parse HEAD`) with every result.

### Phase 1 – launch-only fixed-base observation (dispatch disabled; no goal)

```bash
cd ~/spiderx_ws && git status --short && git rev-parse HEAD     # clean, record the commit
colcon build --symlink-install && source install/setup.bash
ros2 run spiderx_controller m61a_clearance --check-config        # "Config consistent ..."
pgrep -af "ign gazebo|gz sim|ros2|parameter_bridge|controller_manager" || echo "nothing running"
# Terminal A (GUI; Ctrl+C here ends everything):
ros2 launch spiderx_bringup fortress_m61a_fixed_base.launch.py 2>&1 | tee log/m61a_phase1_launch.log
# Terminal B, after both controllers are active plus 3 s:
ros2 control list_controllers
ros2 run spiderx_controller m61a_observe_fixed_base --domain-id 0 --preflight
ros2 run spiderx_controller m61a_observe_fixed_base --domain-id 0 --duration 120
ros2 topic info /leg_trajectory_controller/joint_trajectory      # Publisher count: 0
ros2 action info /leg_trajectory_controller/follow_joint_trajectory   # Action clients: 0
ros2 topic echo /scan --once > log/m61a_phase1_scan.yaml          # independent body cross-check
# repeat the 120 s observation in 3 separate launches; then Ctrl+C in terminal A,
ros2 daemon stop; pgrep -af "ign gazebo|gz sim|parameter_bridge|controller_manager"   # empty
# preserve the evidence BEFORE any clean build (a clean build must not delete log/; §14):
mkdir -p ~/spiderx_evidence && cp -a log/m61a_observation log/m61a_phase1_* ~/spiderx_evidence/
```

**Provisional acceptance criteria** (the owner approves, edits or replaces them before the run):
1. Every preflight is READY.
2. In each 120 s run:
   - body z mean within 1 mm of 0.125 m and z range ≤ 1 mm;
   - maximum attachment deviation ≤ 1 mm and ≤ 0.0033 rad (one third of the tolerances);
   - spawn deviation ≤ 1 mm;
   - link deviation ≤ 0.1 mm;
   - tilt ≤ 0.0033 rad.
3. Pose receipt gaps: p99.9 and maximum recorded; the maximum must be below the 1.0 s pose timeout.
   Joint-state sim gap maximum ≤ 0.25 s.
4. No sample codes other than the expected ones. The `header.frame_id` values are recorded.
5. Visual: the legs hang clear of the ground (screenshot from the GUI). No contact is claimed;
   there is no contact sensor.
6. Independent cross-check of the pose source: the `/scan` ranges to the world walls agree with
   the welded pose (0, 0, 0.125 m, yaw 0) to within the M0 ±8 mm. A disagreement means
   `pose/info` does not describe the simulated body, and nothing else in this list can be
   trusted.
7. Command ownership: `Action clients: 0` and `Publisher count: 0` above, and the observer's
   `action_clients` and `command_publishers` are 0 (a snapshot; see §6).
8. Clean shutdown: no leftover process.

If a criterion fails, record it and stop; do not tune a tolerance from the same run.

### Phase 2 – read-only readiness, then ONE separately approved fixed-base cycle

1. With the phase 1 launch running: `ros2 run spiderx_controller m6_gait_replay.py --dry-run`
   (expects `241760e7dfd5ef12`, `9dba1a17…`) and the observer `--preflight` READY.
2. Only after a separate owner approval, a reviewed enabling change on a never-merged branch
   flips `M61_LIVE_DISPATCH_ENABLED` and `test/m61_gate.py` (the M6.1 implementation notes, §5,
   items 11–12). Then
   `ros2 run spiderx_controller m6_gait_replay.py --live --domain-id 0` and type
   `SEND-ONE-TROT-CYCLE`.
3. Proposed acceptance:
   - one goal, at most one cancel, no retry or return goal;
   - tracking max error < 0.05 rad (G3 never trips);
   - G1/G2/G8 not tripped;
   - attachment maximum below the phase 1 noise rule;
   - evidence in `log/m61_run/<UTC>/`;
   - the gate re-disabled afterwards.

   **A successful fixed-base cycle shows tracking and the safety layer on a welded body. It is not
   walking, balance or locomotion evidence.**

---

## 11. Decisions and open items

| # | Item | Status |
|---|---|---|
| D-M61A-1 | Technique (Approach B weld via the wrapper + dedicated launch) | **Implemented** on the evidence of the M6.1-A frame study; the owner confirms on review |
| D-M61A-1 | Mount 0.125 m | **Provisional**, derived (§3). Confirm in phase 1 |
| D-M61A-2 | Pose source and freshness | Source as designed (composed); freshness 1.0 s kept. **Phase 1 measures the gaps** |
| D-M61A-3 | Implementation | Done for the files in §1. No live run |
| D-M61A-4 | Phase 1 acceptance criteria | Proposed in §10; owner to approve |
| – | Attachment / spawn / link tolerances | **Provisional** (§5) |
| – | `header.frame_id` of the bridged poses | Unverified: recorded, not gated |
| – | Gazebo welds at the SDF joint pose with an identity spawn | Expected (the two possible semantics coincide under Approach B); phase 1 confirms |

**Non-claims.** Nothing here shows a working runtime weld, controller activation, ground clearance
in Gazebo, a replay, balance, walking or hardware capability.

---

## 12. Beginner's guide to the pieces

| Piece | What it does | Why it is needed | What success looks like |
|---|---|---|---|
| Wrapper xacro | Glues `dummy_link` to the world at 12.5 cm with a fixed joint | To test leg motion and the safety layer without the robot falling | `check_urdf` shows root `world`; the observer says READY |
| Dedicated launch | Starts the simulator with the glued model, at the identity spawn | The normal launch spawns 7.5 cm up and would double the height | The robot hangs still; the controllers are active |
| Frame composition | Turns "where is the model" plus "where is the body inside the model" into "where is the body" | With the glue's offset, the model origin is not the body | The composed height is 0.125 m |
| Clearance analysis | Finds the lowest point any collision mesh can reach | So the feet can never touch the ground, whatever the legs do | Every reachable configuration stays ≥ 15.9 mm above ground |
| Attachment check (G8) | Watches that the body stays where it was glued | The height check alone cannot see a broken glue | Deviation stays ≪ 3 mm |
| Observer | Looks, measures and reports; never moves anything | Evidence before any motion is approved | READY plus a statistics file |

---

## 13. Focused Cloud review: corrections (after `252b9c2`)

| Finding | Kind | Correction |
|---|---|---|
| M6.1 readiness had no command-owner check; the observer counted topic publishers but not other FollowJointTrajectory clients | Confirmed gap | `command_owner_codes`; the observer counts action clients; `with_command_owner` in M6.1 readiness (re-evaluated before dispatch); 3 mock scenarios; isolated late-arrival tests |
| Graph checks were described as if they excluded other commanders | Wording | §6: point-in-time observation, discovery delay, races, and what the controller does instead |
| Body versus stationary model root; association with the spawned model | Resolved by inspection | §4: the composition and both Gazebo reporting cases; residual blindness and the `/scan` cross-check |
| Clearance assumptions | Wording | §3: rigid zero-tilt weld and the attachment tolerance's effect |
| The clearance report lived only in `log/` and was deleted by `rm -rf build install log` | Evidence defect | Regenerated twice from a recorded revision and preserved in `docs/evidence/m61a/` (§14) |

The verification record for these corrections is in
[`docs/evidence/m61a/README.md`](evidence/m61a/README.md).

## 14. Evidence preservation and clean builds

- A clean build is `rm -rf build install && colcon build --symlink-install`. **Do not delete
  `log/`**: every tool writes its evidence under `log/<tool>/<UTC>/`, never overwriting, and
  colcon's own logs there are harmless.
- Evidence that matters is copied out of `log/` before anything else touches it: locally to
  `~/spiderx_evidence/`, and reports that the documents quote are committed under
  `docs/evidence/` with a manifest giving the commit, configuration hashes, command, environment
  and output hashes.
- Earlier documents that show `rm -rf build install log` record what was run then; they are not
  the current instruction.
