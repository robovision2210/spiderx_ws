# M6.1-A – Fixed-Base Simulation and Body-Pose Observability: Design Note

**Status: DESIGN ONLY.** The owner asked for this note to be saved as a documentation record
(2026-10-08). Saving it is **not** approval to implement, launch or run anything, and **every
decision in §7 is pending**.
- Nothing in this note is implemented. No source, model, launch, configuration, test or build file
  was changed.
- No Gazebo, ROS 2 launch, ROS node, controller or observer was started for this note.
- No trajectory was dispatched, live or dry.
- `M61_LIVE_DISPATCH_ENABLED` is `False` and stays `False`.
- These are **proposals**, none verified in Gazebo or authorized:
  - the world-to-body weld;
  - the 0.075 m weld height;
  - a readiness check of the fixed-base model;
  - a weld-integrity monitor.
- No fixed-base result may ever be described as free-base walking, locomotion or balance
  validation.

**Purpose.** M6.1 (protected one-cycle trot replay, implemented at `6a7f1f0` with live dispatch
hard-disabled) cannot run live today. Two prerequisites are missing:
1. a fixed or clamped base;
2. a fresh ground-truth body pose in the control launch.

M6.1-A designs both as a separate, reviewable milestone. It does not authorize any replay.

**Labels.** Every factual statement carries one or more of these labels. A claim that rests on both
repository content and a past run carries both labels.

| Label | Meaning |
|---|---|
| **[V]** Repository-verified | Read in a source, model, launch, configuration, test or build file at `6a7f1f0` (path and line given), or the literal content of a committed document (for example, a threshold it lists). Never a runtime measurement |
| **[H]** Historical runtime observation | Measured or seen in an earlier simulator run and recorded in a committed document: the M2 cloud runs, the M2 runtime listings, and the owner's M6.0-D live run. Not re-observed for this note. All of them used the **free-base** model, so they set expectations but are not evidence about a fixed base |
| **[P]** Proposed | A design choice or value for a later implementation. Not approved and not implemented. Every number marked [P] is provisional until the owner decides (§7) |
| **[O]** Unverified, offline check | Expected tool behaviour that can be checked without a simulator (§6 A/B): xacro, the URDF→SDF converter, forward kinematics of the interpolated trajectory. Not yet checked |
| **[G]** Unverified, needs Gazebo | Expected simulator or ROS runtime behaviour. Unproven until observed in an approved launch-only run (§6 C/D) |

---

## 0. Summary

**Recommendation [P]: a world weld, packaged as a launch-selected model variant.** It is pending
D-M61A-1 and D-M61A-3; nothing here is approved.

- **Model.** A NEW wrapper xacro, `spiderx_description/urdf/spiderx_fixed_base.urdf.xacro`. It
  includes the unchanged `spiderx.urdf.xacro` and adds one `world` link and one fixed joint
  `world → dummy_link`.
- **Launch.** A NEW dedicated launch, `spiderx_bringup/launch/fortress_m61_fixed_base.launch.py`,
  is the only file that uses the wrapper. It also starts the same ground-truth pose bridge that the
  posture-hold launch already uses.
- **Weld height.** Provisionally 0.075 m, the existing default spawn height (§2.5). Not decided.
- **Unchanged.**
  - The ordinary free-base files: `fortress_control.launch.py`, `fortress.launch.py` (both
    packages), `spiderx.urdf.xacro`, the bridge YAMLs and the world SDF.
  - All of M6.0-D.

**Pose source.**
- [V] The posture-hold launch bridges Gazebo `/world/spiderx_fortress/pose/info`
  (`ignition.msgs.Pose_V`) to ROS `/spiderx/sim/world_poses` (`tf2_msgs/msg/TFMessage`).
- [V] The consumers take the transform whose `child_frame_id` is `spiderx`, the name of the spawned
  model, and treat it as the Gazebo model pose in the world frame (documented in
  `posture_metrics.py`).
- [H] M2 used it for body height and tilt. [V] The M6.1 adapter already subscribes to it.
- **Open:** whether that model pose is exactly the `base_link` pose has not been established from
  the converted model structure (§3.2, uncertainty U1).
- [P] The proposal reuses the exact bridge definition. No second pose source and no new topic are
  created.

**Findings that shape the design:**

| # | Finding | Consequence |
|---|---|---|
| F1 | [V] M6.1 currently times out a stale body pose after **1.0 s of wall time** (`m61_limits.yaml:41`, `m61_live_contract.py:49`). The **0.25 s** named in the M6.1-A request equals the joint-state gap limit (`m61_limits.yaml:44`, sim time) | Any pose-timeout change is an M6.1 limits change and an owner decision (D-M61A-2). **Undecided** |
| F2 | The 0.045 m body-height gate [V] could not detect a failed weld at the provisional 0.075 m height [P]. A robot that came loose would be expected [G] to settle on its feet near 0.0545 m, the free-base rest height measured in M2 [H], which is still above 0.045 m | A weld-integrity monitor is **proposed** (§4.3, §5.6). **Not authorized** (D-M61A-3); its tolerance is undecided |
| F3 | [V] M6.1 only *labels* the base as fixed (`m61_limits.yaml:49`); no code checks it. From the readiness code [V], a replay started against the free-base posture-hold launch would pass readiness today | A readiness check of the fixed-base model is **proposed** (§5.6). **Not authorized** (D-M61A-3) |
| F4 | [V] `config_check.load_urdf()` expands `spiderx.urdf.xacro`, and its SHA-256 is bound into the pinned M6.0-D and M6.1 trajectory IDs and fingerprints (`m6_trajectory.load_sources`) | Editing `spiderx.urdf.xacro`, even with a default-off argument, risks changing those pinned identities [O]. This is why the recommendation uses a separate wrapper file |

---

## 1. Repository findings and historical observations

§1.1–§1.3 and §1.5 contain only [V] facts. Runtime observations from earlier runs are kept
separate in §1.4 [H].

### 1.1 Repository state [V]

| Item | Value |
|---|---|
| Branch | `claude/stoic-shannon-ur2mes`. The note was written against `6a7f1f0` (`feat(m61): protected trot gait replay, live dispatch hard-disabled`), which was identical to `origin/claude/stoic-shannon-ur2mes` |
| Working tree | Clean at `6a7f1f0`. This note is the only file added after it, in a documentation-only commit |
| `origin/main` | `77fd171` (merge of PR #16). The branch also carries `7b8798c` (M6.1 design note) and `6a7f1f0` (M6.1 implementation); none of this is merged |
| Gates | `M61_LIVE_DISPATCH_ENABLED = False` (`m61_live_contract.py:19`); `LIVE_DISPATCH_ENABLED = False` (`m6_live_contract.py:18`); `test/m61_gate.py` and `test/m6d_gate.py` pin both as False |
| M6.0-D enabling branch | `origin/claude/spiderx-m6d-enable-gate` exists and is not merged (not touched) |

### 1.2 Files and their roles [V]

| Role | File | What it does |
|---|---|---|
| Normal control launch | `src/spiderx_bringup/launch/fortress_control.launch.py` | Appends the gz_ros2_control library dir to `IGN_GAZEBO_SYSTEM_PLUGIN_PATH` / `GZ_SIM_SYSTEM_PLUGIN_PATH` (l. 38–42). Includes `spiderx_description/launch/fortress.launch.py` with `enable_control` and `controllers_file` = `spiderx_controller/config/spiderx_ros2_controllers.yaml` (l. 44–52). Includes `spiderx_controller/launch/controller.launch.py` when `enable_control` (l. 54–58). Optional RViz. Args: `enable_control` (default `true`), `rviz` (default `false`). **No pose bridge, no fixed base** |
| Simulation / spawn launch | `src/spiderx_description/launch/fortress.launch.py` | `ros_gz_sim gz_sim.launch.py` with `-r` (GUI) or `-s -r --headless-rendering` (l. 105–123). `robot_state_publisher` with `xacro spiderx.urdf.xacro sim_backend:=fortress enable_control:=… controllers_file:=…` (l. 38, 88–100). `ros_gz_sim create -name <robot_name> -topic robot_description -x -y -z -Y` (l. 125–138). `parameter_bridge` named `spiderx_gz_bridge` with `fortress_bridge_control.yaml` when control is enabled, else `fortress_bridge.yaml` (l. 140–153). The xacro path is hard-coded: **there is no argument to select another model** |
| Posture-hold launch (pose source) | `src/spiderx_bringup/launch/fortress_posture_hold.launch.py` | Includes `fortress_control.launch.py` with `enable_control:=true` (free base). Adds `ros_gz_bridge parameter_bridge` named `spiderx_sim_ground_truth_bridge` with argument `/world/spiderx_fortress/pose/info@tf2_msgs/msg/TFMessage[ignition.msgs.Pose_V`, remapped to `/spiderx/sim/world_poses`, `use_sim_time: True` (l. 28–30, 46–54). Args: `rviz`, `headless` |
| Controller spawners | `src/spiderx_controller/launch/controller.launch.py` | Spawns `joint_state_broadcaster`, then `leg_trajectory_controller` only after the first spawner exits 0 (M4.1). Hash-pinned in `test_m6d_contract.py` |
| Robot model | `src/spiderx_description/urdf/spiderx.urdf.xacro` (937 lines) | Args `sim_backend` (`classic`/`fortress`/`none`), `enable_control`, `controllers_file`. Root link `dummy_link` (l. 42); fixed `dummy_joint` → `base_link` with **no `<origin>`** (l. 45–48). 32 links; none named `world` or `spiderx`. **No world attachment, no fixed base** |
| Fortress extensions | `src/spiderx_description/urdf/spiderx_fortress.gazebo.xacro` | gpu_lidar on `lidar_link` → `/spiderx/scan`. `JointStatePublisher` system only when control is disabled. `mu1 = mu2 = 0.2` and `self_collide=false` on 30 CAD links. **No pose publisher plugin.** No `disableFixedJointLumping` or `preserveFixedJoint` tag in any Fortress xacro |
| Control extension | `src/spiderx_description/urdf/spiderx_fortress_control.xacro` | `<ros2_control>` with exactly the 12 leg joints (position command; position and velocity state); `gz_ros2_control-system` plugin with `<parameters>$(arg controllers_file)` |
| URDF actuator values | `spiderx.urdf.xacro` | `effort="100" velocity="100"` on all 12 revolute joints (placeholders) |
| World | `src/spiderx_description/worlds/spiderx_fortress.sdf` | World `spiderx_fortress` (l. 14). 1 ms step (l. 247). Systems Physics, UserCommands, **SceneBroadcaster with no parameters (l. 255–256)** and Sensors (ogre2). Static `ground_plane`, `walls`, `box_a`, `box_b`, `pillar`. The robot is spawned by the launch, not defined here |
| Bridge configs | `src/spiderx_description/config/fortress_bridge.yaml`, `fortress_bridge_control.yaml` | `/clock`; `/spiderx/scan` → `/scan`; `/spiderx/joint_states` → `/joint_states` (passive file only). **No pose bridge, nothing on `/tf`** |
| Pose consumers | `src/spiderx_controller/scripts/run_posture_hold_test.py` (M2); `spiderx_controller/m61_live_adapter.py` (M6.1) | Both subscribe to `TFMessage` on `posture_metrics.POSE_TOPIC` = `/spiderx/sim/world_poses` (`posture_metrics.py:26`) and take the transform whose `child_frame_id` equals `MODEL_NAME` = `spiderx` (`posture_metrics.py:25, 87–90`) |
| M6.1 gates and limits | `m61_gates.py` (`on_body_pose` l. 88, `check_pose_stale` l. 144, `pose_readiness` l. 227); `config/m61_limits.yaml` | `body_min_height_m 0.045`, `body_max_tilt_rad 0.26`, `body_pose_required true`, **`body_pose_stale_s 1.0`** (wall), `joint_state_gap_s 0.25` (sim), `base_constraint fixed` (a label only) |
| Build / install | `spiderx_bringup/CMakeLists.txt` installs `launch` and `rviz` wholesale; `spiderx_description/CMakeLists.txt` installs `urdf meshes launch config worlds` wholesale | New files in those directories need **no CMake change**. Neither package has tests; launch, model and xacro tests live in `spiderx_controller/test` (e.g. `test_controller_launch.py` loads a launch file with `importlib`, without running it) |
| Static validators | `scripts/validate_fortress.sh`, `validate_m1_control.sh`, `validate_m2_posture.sh` | Offline-check conventions: `xacro` + `check_urdf`, `ign sdf -p` / `ign sdf -k`, `ros2 launch … --show-args`, grep guards (e.g. the pose bridge is remapped away from `/tf`) |

### 1.3 Frames, topics, message types, launch arguments [V]

| Item | Value | Source |
|---|---|---|
| Gazebo world name | `spiderx_fortress` | `spiderx_fortress.sdf:14`; `fortress_posture_hold.launch.py:28` |
| Gazebo model name | `spiderx` (launch argument `robot_name`, default `spiderx`, passed to `create -name`) | `spiderx_description/launch/fortress.launch.py:60, 131` |
| URDF root and body link | Root link `dummy_link`. `dummy_joint` is fixed (parent `dummy_link`, child `base_link`) with **no `<origin>`**, so `dummy_link` and `base_link` coincide exactly in the URDF and the TF tree. How this relates to the Gazebo *model* frame is **not** established (§3.2, U1) | `spiderx.urdf.xacro:42–48` |
| Gazebo pose topic | `/world/spiderx_fortress/pose/info`, `ignition.msgs.Pose_V`, from the SceneBroadcaster system (loaded with no parameters) | `fortress_posture_hold.launch.py:29, 51`; `spiderx_fortress.sdf:255–256` |
| ROS pose topic | `/spiderx/sim/world_poses`, `tf2_msgs/msg/TFMessage` | `fortress_posture_hold.launch.py:30, 51–52`; `posture_metrics.py:26` |
| Bridge node | `spiderx_sim_ground_truth_bridge` (`ros_gz_bridge parameter_bridge`, `use_sim_time: True`) | `fortress_posture_hold.launch.py:46–54` |
| Consumer time stamping | Each pose sample is stamped on receipt: the node's `/clock` time plus wall time (M6.1); the node's clock time (M2) | `m61_live_adapter.py:88`; `run_posture_hold_test.py` (`_on_pose`, `now_s`) |
| TF sources | The URDF has no `world` or `odom` link, and neither bridge YAML forwards `/tf`. The TF tree comes from `robot_state_publisher`, rooted at `dummy_link`. The pose bridge is remapped away from `/tf`, which `validate_m2_posture.sh` checks | `spiderx.urdf.xacro`; `fortress_bridge*.yaml`; `fortress_posture_hold.launch.py:52`; `validate_m2_posture.sh` |
| RViz fixed frame | `dummy_link` | `spiderx_bringup/rviz/spiderx_fortress.rviz:89` |
| Spawn height | `spawn_z` default 0.075 m. The foot soles are 0.0545 m below the root-link origin at the CAD zero pose (from the URDF audit) | `fortress.launch.py:30–33`; `posture_metrics.py:34` (`CAD_FOOT_PLANE_BELOW_BASE_M`) |
| Ground plane | Static `ground_plane` with plane normal (0, 0, 1) and no `<pose>`, i.e. at world z = 0 | `spiderx_fortress.sdf:285–295` |
| M2 posture thresholds | `min_body_height_m 0.045`, `max_body_height_range_m 0.005`, `max_abs_roll_rad 0.10`, `max_abs_pitch_rad 0.10` | `m2_simulation_postures.yaml:64–71` |

### 1.4 Historical runtime observations [H]

These come from earlier runs recorded in committed documents. They were **not re-observed** for
this note, all used the **free-base** model, and the machines differed. They guide expectations
only.

| Observation | Value | Recorded in |
|---|---|---|
| Gazebo pose topic rate | "about 50 Hz" (`ign topic -e`) | `M2_SIMULATION_POSTURE_PLAN.md` §2 |
| Body-pose samples per 10 s sim-time hold | 2225–2355 at real-time factor ≈ 0.25. That is about 225 per sim-second, or about 56 Hz wall (derived here) | `M2_TEST_RESULTS.md` |
| Model pose at rest (free base, control mode) | z 0.05448–0.05450 m; hold range ≤ 0.00007 m; \|roll\| ≤ 0.00018 rad; \|pitch\| ≤ 0.00022 rad; tilt ≤ 0.00024 rad; xy drift ≤ 0.00001 m; yaw drift ≤ 0.0001 rad; orientation ≈ identity | `M2_TEST_RESULTS.md`; `M2_SIMULATION_POSTURE_PLAN.md` §2 |
| Final height minus the CAD foot plane | +4.3e-6, −3.3e-6, −1.9e-6 m (three runs) | `M2_TEST_RESULTS.md` |
| Per-pose header stamps from the bridge | 0 | `M2_TEST_RESULTS.md` (problems table) |
| Control-mode ROS topics | `/clock`, `/joint_states` (1 publisher), `/dynamic_joint_states`, `/leg_trajectory_controller/*`, `/robot_description`, `/scan`, `/tf`, `/tf_static` | `M2_SIMULATION_POSTURE_PLAN.md` §2 |
| TF frames | `dummy_link → …` only; no `world` or `odom` frame | `M2_SIMULATION_POSTURE_PLAN.md` §2 |
| Real-time factor | ≈ 0.25 (M2, cloud VM, software rendering); ≈ 0.23 (owner's M6.0-D live run) | `M2_TEST_RESULTS.md`; `M6D_FIRST_GOAL_RESULTS.md` on the unmerged `origin/claude/spiderx-m6d-enable-gate` |

### 1.5 What does not exist today [V]

- No `world` link, world joint, fixed base, fixture model or detachable joint anywhere in the
  model, world or launch files.
- No check anywhere that the base is fixed; there is only the label in `m61_limits.yaml:49`.
- No pose bridge in `fortress_control.launch.py` or `spiderx_description/launch/fortress.launch.py`.
- No Gazebo `PosePublisher`, `OdometryPublisher` or contact system in any model or world file.
- No test or CMake test setup in `spiderx_bringup` or `spiderx_description`.

---

## 2. Fixed-base design alternatives

### 2.1 Constraints every option must meet [P]

1. The ordinary free-base path stays **byte-identical**: `fortress_control.launch.py`, both
   `fortress.launch.py`, `fortress_posture_hold.launch.py`, `spiderx.urdf.xacro`, the
   gazebo/control xacros, the bridge YAMLs and the world SDF. The M2 results document already used
   "M1 and passive files unchanged" as an acceptance check [V].
2. Robot kinematics, inertia and joint limits do not change. M6.0-D and M6.1 compute their
   geometry from `load_urdf()` = `spiderx.urdf.xacro sim_backend:=none` [V], so that file must stay
   untouched (F4).
3. The base is fixed **only** when the dedicated M6.1 fixed-base launch is used.
4. No contact with the ground or a fixture during the replay. Contact is not measured by any sensor
   [V: none exists].

### 2.2 Option A: world-to-base weld added to the robot model

| Question | Answer |
|---|---|
| Mechanism | [P] Add `<link name="world"/>` and `<joint name="…" type="fixed"><parent link="world"/><child link="dummy_link"/></joint>` to the URDF. [O] The URDF→SDF converter (sdformat) is expected to treat a link named `world` as the world and turn the joint into a fixed joint to the world. This is the documented Gazebo approach, checkable offline (§6 A3). [G] That the joint actually holds the body in Fortress is **not verified** (§6 C1, D2) |
| Files that change | `spiderx.urdf.xacro`, unless done in a separate file (that is Option C) |
| Geometry / inertia change | None by construction [P]: `world` has no geometry or inertia, and the joint adds no mass |
| Interference: controllers | [G] None expected. gz_ros2_control should drive only the 12 joints in `<ros2_control>` [V], and the weld is not one of them |
| Interference: TF | [G] `robot_state_publisher` would publish a new static `world → dummy_link`. No `world` link exists today [V] |
| Interference: collisions / ground | No new collision geometry [P]. Ground contact depends on the weld height (§2.5) |
| Enabled only in a dedicated launch? | **No**, if added unconditionally to `spiderx.urdf.xacro`: every launch would get a fixed base. A default-off xacro argument fixes that, but still edits a file whose expansion is hashed into the pinned M6.0-D/M6.1 identities (F4). [O] Python's `ElementTree` keeps whitespace text, so even an inert edit could change the hash |
| Removal / isolation | Poor: the weld lives in the shared model file |

### 2.3 Option B: raised fixture or support attached to the world

| Variant | Mechanism | Assessment |
|---|---|---|
| **B1** Pedestal, robot resting on it | A static support model (a box or column) in a NEW world file; the free-base robot lies on it with its feet off the ground | The robot is **not fixed**. Friction on `base_link` is the fusion2urdf default µ = 0.2 [V]. Leg reaction forces during a trot could slide or tip it [G, expected], which breaks the M6.1 owner decision for a base that cannot fall (D-M61-1). It also adds a contact pair between the full CAD `base_link` collision mesh and the pedestal: an unmeasured contact artifact. The legs hang close to the body, so a pedestal wide enough for stability risks leg collisions [G] |
| **B2** Support link welded to the world, robot welded to the support | Option A plus extra geometry | Everything Option A does, plus collision geometry near the legs. No benefit |

- **Files:** a NEW world SDF (B1), plus a launch able to select it (`fortress.launch.py` already
  has a `world` argument [V]).
- **Geometry / inertia:** the robot is unchanged, but a new collision body sits under it.
- **Interference:** contact and friction with the fixture; possible leg-fixture collisions; slip
  (B1).
- **Isolation:** good (a separate world file).
- **Verdict [P]:** rejected. B1 is not fixed; B2 is Option A plus risk.

### 2.4 Option C: launch-selected fixed-base model variant (with Option A's weld)

| Question | Answer |
|---|---|
| Mechanism | **C1 [P, recommended]:** a NEW wrapper `spiderx_description/urdf/spiderx_fixed_base.urdf.xacro`. Its `<robot name="spiderx">` root `xacro:include`s the unchanged `spiderx.urdf.xacro` and adds `world` plus a fixed joint `world → dummy_link` with zero origin. [O] xacro is expected to insert the included file's root children and to treat command-line `xacro:arg` values as global, so `sim_backend`, `enable_control` and `controllers_file` keep working; to be checked offline (§6 A2). **C2 [P, rejected]:** a default-false `fixed_base` argument inside `spiderx.urdf.xacro` (F4 risk) |
| Files that change | **None existing** [P]. New: the wrapper xacro and a dedicated launch file (§5). No CMake change, because the directories are installed wholesale [V] |
| Geometry / inertia change | None by design [P]. A proposed test asserts that the expanded fixed-base URDF equals the default expansion plus exactly one link and one joint [O] |
| Interference: controllers | [G] None expected, as in Option A. The 12-joint `<ros2_control>` contract is unchanged [V], and `joint_state_broadcaster` should keep publishing the same 12 joints [G] |
| Interference: TF | [P] A static `world → dummy_link` would exist **only** under the fixed-base launch, which must never run together with a future odometry stack. RViz keeps fixed frame `dummy_link` [V] |
| Interference: collisions / ground | None intended if the weld is raised so the feet clear the ground. That is unverified (§2.5). The weld has no geometry [P] |
| Enabled only in a dedicated launch? | **Yes** [P]. Only the new launch would expand the wrapper. Every existing launch expands `spiderx.urdf.xacro` through its hard-coded path [V] |
| Removal / isolation | Delete two new files. Nothing else would reference them, and future free-base work never sees the weld [P] |

**Launch packaging for C1.** The description launch hard-codes the xacro path, so there is no
argument to select another model [V]. Two sub-options:

| Sub-option | Change | Assessment |
|---|---|---|
| **L1 [P, recommended]** | A NEW `spiderx_bringup/launch/fortress_m61_fixed_base.launch.py`. It re-states the description launch's four actions with the wrapper xacro: gz_sim include, `robot_state_publisher`, `create`, and `spiderx_gz_bridge` with `fortress_bridge_control.yaml`. It sets the same plugin-path variables as `fortress_control.launch.py`, includes the unchanged `controller.launch.py`, and adds the unchanged ground-truth pose bridge | **Zero diff to existing files.** The cost is duplicated launch logic, mitigated by a static test that compares the new launch's actions and parameters with the originals (§5.4) |
| L2 [P, rejected] | Add one `xacro_file` argument (default = today's path) to `spiderx_description/launch/fortress.launch.py` | A smaller duplicate, but it modifies a file included by the ordinary control launch. Rejected under constraint 1 |

### 2.5 Weld height and ground clearance [P], provisional

**The weld height is not decided (D-M61A-1).** 0.075 m is a provisional proposal.

| Height | Effect |
|---|---|
| **Raised, 0.075 m (provisional proposal)** | Equal to the existing `DEFAULT_SPAWN_Z` [V]. The soles are 0.0545 m below the root-link origin at the CAD zero pose [V], so the nominal clearance at neutral would be ≈ 0.0205 m. That figure is an estimate. It assumes both of these, which are unverified: the welded model frame sits at 0.075 m [G, U4], and the model frame coincides with the `base_link` origin [U1] |
| Nominal, 0.0545 m [P, rejected] | The feet would touch the ground. Stance feet would drag across the ground at µ 0.2 [V] while the body is welded, so unmeasured contact forces would disturb tracking [G, expected], and nothing physically meaningful would be shown |

**Ground clearance throughout the interpolated trajectory is unverified:**
- [V] In the M6.1 trajectory (approved for M6.1 on 2026-10-03), the IK foot **targets** never go
  below the neutral foot plane: stance z offset = `stance_height_offset_m` = 0 and swing z offset
  ≥ 0 (`m61_trot_cycle.foot_offset`; `config/m61_trot_cycle.yaml`).
- [O] **Not verified:** that the joint-space cubic-Hermite interpolation between the 9 waypoints
  (what the controller executes) keeps every foot at or above that plane. The earlier offline
  spline check (`M61_TROT_REPLAY_DESIGN.md` §2.4: stance-foot z error < 0.01 mm) was computed for
  the unapproved 2.5 cm / 10 mm variant, not for the approved 2 cm / 6 mm trajectory.
- [O] **Not verified:** that the feet remain the robot's lowest points at every pose of the cycle.
  Thighs, horns or holders could be lower at some joint angles.
- [G] **Not verified:** the clearance actually achieved in Gazebo, given tracking error and the
  real weld height.

Until all three are checked (§5.4 FK test; §6 C/D), "no ground contact" is a design intent, not a
verified property.

[G] Where the weld holds the model is not proven in the repository either. With the weld origin at
zero, the model is expected to be held at its spawn pose (`create -z`), but Fortress might instead
weld at the URDF joint origin. §6 C/D checks this by confirming the observed pose z equals the
intended height and stays constant (U4).

### 2.6 Other options considered and rejected [P]

| Option | Why rejected |
|---|---|
| `<static>true</static>` model | [G, expected] A static model's links cannot move, so the legs could not be actuated. Its pose may also not appear on `pose/info` |
| A runtime-created joint (e.g. a detachable-joint system plus a service call) | Adds a system plugin and a runtime command that the M6.1 tool would have to trust. More moving parts than a static weld |
| Zero gravity | Not a fixed base: leg reaction forces would still move the body |

### 2.7 Recommendation [P], pending D-M61A-1 and D-M61A-3

**Option C1 + L1 with Option A's mechanism, at a provisional 0.075 m weld height.**
- A new wrapper xacro and a new dedicated launch.
- No existing file changed.
- No geometry, inertia or kinematic change.
- The feet are *intended* to stay clear of the ground (unverified, §2.5).
- Removal is two file deletions.

The ordinary free-base control launch is untouched. Nothing in this recommendation is approved.

---

## 3. Body-pose observability design

### 3.1 End-to-end trace of the existing pose path

```
Gazebo server, world "spiderx_fortress"
  SceneBroadcaster system (spiderx_fortress.sdf:255, no parameters)                    [V]
    publishes /world/spiderx_fortress/pose/info  (ignition.msgs.Pose_V)                [V names]
    "about 50 Hz" seen with ign topic -e; ~56 Hz wall derived from M2 counts            [H]
        │  gz-transport
        ▼
ros_gz_bridge parameter_bridge  "spiderx_sim_ground_truth_bridge"                     [V]
  argument  /world/spiderx_fortress/pose/info@tf2_msgs/msg/TFMessage[ignition.msgs.Pose_V   [V]
            delivered Gazebo poses to ROS in the M2 runs                               [H]
  remap     /world/spiderx_fortress/pose/info -> /spiderx/sim/world_poses              [V]
  use_sim_time True; remapped away from /tf (checked by validate_m2_posture.sh)        [V]
        │  ROS 2
        ▼
/spiderx/sim/world_poses   tf2_msgs/msg/TFMessage                                       [V]
  child_frame_id = entity name                                                          [V doc]
  per-pose header stamps = 0                                                            [H]
        │
        ├─► run_posture_hold_test.py (M2): extract_model_pose("spiderx") → height, roll, pitch   [V]
        └─► m61_live_adapter.M61RclpyLiveTransport: extract_model_pose("spiderx") → quat_to_rpy →
            ('body_pose', wall, /clock stamp, (x, y, z, roll, pitch, yaw)) and latest_body_pose()
            → m6_gait_replay.with_body_pose (readiness) and m61_gates (G1, G2, freshness)        [V]
```

### 3.2 What the source represents

| Question | Answer |
|---|---|
| Ground truth, odometry or TF? | **Simulator ground truth** [V, as documented in `posture_metrics.py` and `M2_SIMULATION_POSTURE_PLAN.md`]: physics state broadcast by SceneBroadcaster, not an estimator. It is not odometry and it is not TF (remapped away from `/tf` [V]) |
| Which entity? | [V] The consumers select the transform whose `child_frame_id` is exactly `spiderx`, the name the launch gives the spawned model (`create -name spiderx`). No link is named `spiderx`. [V, documentation] `posture_metrics.py` describes this entry as the Gazebo **model** pose. [G] Other entries (links) may share the message; the consumers ignore them |
| **Model pose vs `base_link` pose** | 1. [V] In the URDF, `dummy_joint` is fixed (parent `dummy_link`, child `base_link`) with no `<origin>` (`spiderx.urdf.xacro:45–48`). `dummy_link` and `base_link` therefore coincide exactly in the URDF and the TF tree.<br>2. [V, documentation only] `posture_metrics.py` states that the Gazebo model frame is the canonical link `dummy_link`, and hence `base_link`. M2 and M6.1 rely on that statement.<br>3. [H] At rest on the free base, M2 measured model-pose z = 0.05448–0.05450 m with orientation ≈ identity. That matches the CAD sole depth below the `base_link` origin (0.0545 m [V]) within about 0.02 mm. This is **consistent** with model pose = `base_link` pose in z and orientation. It does **not** test an x/y offset.<br>4. **Uncertainty U1 [O, then G]:** the equivalence of the Gazebo model frame and the `base_link` origin has **not** been established from the converted model structure. It depends on where the URDF→SDF conversion places the root link relative to the model frame. It also depends on how the default fixed-joint lumping (no lumping-control tag is present [V]) merges `base_link` into `dummy_link`. Neither has been inspected in this repository. Proposed checks: §6 A3 and A3b (offline: the root link's pose relative to the model is identity in the `ign sdf -p` output, for both the default and the fixed-base description) and §6 D2 (runtime, fixed base).<br>Until U1 is closed, "body height" means the z of the Gazebo **model-frame origin**. It is *assumed*, not proven, to be the `base_link` origin |
| Which frame? | The Gazebo world frame of `spiderx_fortress` [V, documentation]. The ground plane is at world z = 0 [V: `spiderx_fortress.sdf:285–295`], so z is a height above the ground. Measured 0.0545 m at rest in M2 [H] |
| Position and orientation | Both: translation plus quaternion. The consumers compute roll, pitch and yaw (ZYX) [V `posture_metrics.quat_to_rpy`] |
| Time stamps | Per-pose stamps were 0 in M2 [H]. The consumers stamp on receipt [V]. Freshness can therefore only be measured from **receipt time**: wall (`time.monotonic`) or the node's `/clock` |
| Fixed-base behaviour | [G] A welded, non-static model is expected to remain a dynamic entity, so its pose should still appear on `pose/info` (U5; §6 D2, D4) |

### 3.3 Is it sufficient?

All statements in this table rest on **free-base** data [H] and must be re-confirmed on the fixed
base [G].

| Need | Assessment |
|---|---|
| Body height vs 0.045 m | **Expected to be sufficient.** M2 measured 0.05448–0.05450 m with a hold range ≤ 0.00007 m [H]: resolution and noise about 1000× below the margin. On a raised weld z should be about 0.075 m [P, G]; F2 explains what the gate then means. Subject to U1 |
| Tilt monitoring | **Expected to be sufficient.** M2 max tilt at rest was 0.00024 rad [H], three orders of magnitude below the 0.26 rad gate [V] |
| Freshness 0.25 s | **Probably sufficient, not proven.** About 50–56 Hz wall [H] gives a nominal inter-arrival of about 0.02 s, so 0.25 s is about 12 nominal periods (derived from [H]). The **maximum** gap under load (GUI, software rendering, RTF ≈ 0.23–0.25) has never been measured (U6). M6.1 currently uses **1.0 s** wall [V] (F1). Proposal [P]: measure the maximum and the 99.9th-percentile receipt gap during launch-only observation (§4.4) before D-M61A-2 decides. **Undecided** |
| Time base | **Wall time** [P]. The stamps are 0 [H]. A stalled simulator is handled separately by the re-used G5 sim-stall monitor [V]. A wall-time pose timeout also catches a dead bridge process, which a `/clock`-based timeout would miss if `/clock` stalled at the same time |

### 3.4 Recommendation for the dedicated fixed-base launch [P], pending D-M61A-2

1. Reuse the bridge **exactly** as `fortress_posture_hold.launch.py` defines it:
   - package `ros_gz_bridge`, executable `parameter_bridge`;
   - name `spiderx_sim_ground_truth_bridge`;
   - the same argument string, the same remap to `/spiderx/sim/world_poses`, and
     `use_sim_time: True`.

   A static test would assert that the new launch's bridge equals the posture-hold one and
   `posture_metrics.POSE_TOPIC`.
2. Do **not** add a pose bridge to `fortress_control.launch.py`, the bridge YAMLs or the world
   file. Do **not** add `PosePublisher` or `OdometryPublisher`, a second ROS topic, or anything on
   `/tf`.
3. Keep `robot_name` fixed at `spiderx` in the fixed-base launch (no launch argument). The
   consumers match that name [V].
4. No M6.1 adapter change is needed for the topic or type: it already reads
   `/spiderx/sim/world_poses` [V].

### 3.5 Uncertainties and fallbacks

| Uncertainty | If verification fails |
|---|---|
| [G] The welded model's pose is published on `pose/info` (U5) | M6.1 readiness refuses (`body_pose_missing`) [V], which is the safe outcome. Any alternative source would be a new decision with its own design. No topic name is assumed here |
| [G] The maximum receipt gap stays well under 0.25 s (U6) | Keep 1.0 s, or choose from the measured distribution (D-M61A-2) |
| [G] The weld holds the model at the spawn pose (U4) | Set the weld height in the joint origin instead, and re-verify (§6 C/D) |
| [O/G] Model frame = `base_link` origin (U1) | Correct for any measured offset in the consumers (a reviewed code change), or document the offset in every height threshold |

---

## 4. Safety and semantics

### 4.1 What a future fixed-base replay would validate (and what it would not)

**It would validate [P]**, if the run is ever approved and succeeds:
- one offline-validated trot-cycle **joint trajectory** executes through the existing
  `leg_trajectory_controller` in Gazebo;
- the observed joints track the commanded spline within 0.05 rad;
- every M6.1 gate and the cancel path behave correctly on a real graph;
- the base stays welded (pose evidence).

All of this is with placeholder actuators (`effort="100" velocity="100"` [V]), µ 0.2 [V], no
damping and the real-time factor of that run (`M2_SIMULATION_LIMITATIONS.md` [V]).

**It would NOT validate:**
- free-base stability, support or balance, or balance recovery;
- actual forward locomotion or walking. The body cannot move, so the legs swing in the air;
- foot contact, slip or ground reaction (feet intended to be off the ground; no contact sensor);
- odometry, state estimation, SLAM or Nav2;
- real-time performance, actuator capability or hardware readiness.

**No fixed-base result may be described as free-base walking, locomotion or balance
validation**, nor as a stepping or gait success. The only allowed wording is "a trot-cycle joint
trajectory was replayed on a welded base in simulation".

### 4.2 M6.1 protections preserved [V at `6a7f1f0`; unchanged by this design]

| Protection | Where |
|---|---|
| Gate False; `--live` exits 3 before config, input or ROS import | `m61_live_contract.py:19`, `m6_gait_replay.main`, `test/m61_gate.py` |
| One goal, no retry, no automatic return | Re-used `LiveSession` latch; single-use transport; the fingerprint refuses any other goal |
| Explicit typed confirmation `SEND-ONE-TROT-CYCLE` | `m61_live_contract.parse_confirmation` |
| Evidence persistence, never overwritten | `m61_evidence` + `EvidenceFile` (`log/m61_run/<UTC>/`) |
| Stale joint-state cancellation (0.5 s wall), gap (0.25 s sim), sim stall (5 s) | Re-used `StreamMonitor` |
| Stale pose cancellation (1.0 s wall today) | `m61_gates.check_pose_stale` |
| Joint-limit monitoring (80 %) | `m61_gates` G3 |

M6.1-A as designed adds **no** path that dispatches a goal. It would add:
- a model variant;
- a launch file;
- a read-only observer;
- only if the owner explicitly authorizes them (D-M61A-3), two proposed M6.1 checks (§5.6).

This note authorizes none of them.

### 4.3 How a fixed base would change the meaning of the evidence

| Signal | Free base (original design) | Fixed base (proposed weld; provisional 0.075 m) |
|---|---|---|
| Body-height gate (0.045 m) | Detects the belly dropping toward the ground | **Could not detect a weld failure** (F2). A loose robot is expected to settle near 0.0545 m [H rest height; G prediction], which is above 0.045 m. It would still catch a wrong launch only if the body ended lower than 0.045 m. **A weld-integrity monitor is proposed [P], not authorized:** \|z − z_weld\| within a tolerance, and x, y and yaw constant. The tolerance is undecided |
| Tilt gate (0.26 rad) | Catches tipping (expected tilt ≈ 4.5° in trot) | Expected ≈ 0 [G]. Any noticeable tilt would mean the weld or the model is wrong. 0.26 rad stays unchanged until baseline data exists (§4.4). **No new threshold is chosen** |
| Contact | Not measured; G1/G2 are proxies | No contact is *intended* (feet ≈ 2 cm above the ground: unverified, §2.5). Still not measured, and no contact claim is made |
| Body-pose evidence | Shows how the body moved | Would show that the **base did not move** (weld integrity). It says nothing about balance |
| G7 drift (report only) | Lateral and yaw drift | Expected ≈ 0 [G]; non-zero drift would mean the weld failed |

### 4.4 Baseline pose-noise collection (before any tighter fixed-base limit) [P]

**Undecided:** the pose timeout, the weld-integrity tolerance and any tighter tilt threshold. This
section only proposes how to gather the data for those decisions.

It would run only in an approved launch-only observation (§6 C/D): the M6.1 gate stays False and no
goal is sent.

1. With the fixed-base launch running and the controllers active (holding neutral), record
   `/spiderx/sim/world_poses` for at least 120 s wall, plus `/joint_states` and `/clock`, with the
   proposed read-only observer (§5.6). Repeat in three separate launches. Record GUI versus
   headless and the real-time factor.
2. Report per run:
   - z mean, min, max and range;
   - |roll| and |pitch| max;
   - x, y and yaw drift;
   - pose receipt gaps (wall): mean, max, 99th and 99.9th percentile;
   - poses per sim-second;
   - joint-state gap (sim);
   - the number of messages without the `spiderx` entry.
3. Only then, each as a separate owner decision, choose:
   - the weld-integrity tolerance;
   - any tighter tilt limit;
   - the pose freshness timeout.

   M2 thresholds already in the repository may serve only as **provisional acceptance values for
   the observation** (`max_body_height_range_m 0.005`, `max_abs_roll_rad`/`max_abs_pitch_rad 0.10`
   [V `m2_simulation_postures.yaml:66–71`]), not as the new gates.
4. The noise **during** leg motion can only be observed in an eventual approved replay run. It
   would be recorded there (`body_pose.csv`), and no threshold would be tuned from a run in which
   that threshold was enforced.

---

## 5. Proposed implementation boundary (a later PR, only after the decisions in §7)

All items are [P]. No file listed here exists or was changed. **Nothing in §5 is authorized.** It
needs D-M61A-3, and each §5.6 replay-code change needs the owner's explicit approval.

### 5.1 Model / URDF / Xacro

| File | Action | Why |
|---|---|---|
| `src/spiderx_description/urdf/spiderx_fixed_base.urdf.xacro` | **New** | The wrapper: includes `spiderx.urdf.xacro` and adds `world` plus a fixed joint `world → dummy_link` (zero origin). The only place the weld would exist |
| `src/spiderx_description/urdf/spiderx.urdf.xacro`, `spiderx_fortress*.xacro` | **Unchanged** | They keep the free-base model and the pinned URDF provenance hash (F4) |

### 5.2 Launch

| File | Action | Why |
|---|---|---|
| `src/spiderx_bringup/launch/fortress_m61_fixed_base.launch.py` | **New** | Would launch:<br>• Gazebo with the unchanged world;<br>• `robot_state_publisher` and `create` from the wrapper with `sim_backend:=fortress enable_control:=true controllers_file:=<spiderx_ros2_controllers.yaml>`, `-name spiderx -z <weld height>` (0.075 provisional);<br>• `spiderx_gz_bridge` with the unchanged `fortress_bridge_control.yaml`;<br>• the gz_ros2_control plugin-path variables;<br>• the unchanged `controller.launch.py`;<br>• the unchanged ground-truth pose bridge.<br>Args: `headless` and `rviz` only. No spawn-pose, model-name or xacro arguments, and no node that sends a goal |
| Every existing launch file | **Unchanged** | Constraint 1 |

### 5.3 Bridge / configuration

| File | Action | Why |
|---|---|---|
| `fortress_bridge.yaml`, `fortress_bridge_control.yaml`, `spiderx_fortress.sdf` | **Unchanged** | The pose bridge would be defined inline, exactly as in the posture-hold launch |
| `src/spiderx_controller/config/m61_limits.yaml` (+ `APPROVED_LIMITS` in `m61_live_contract.py`) | **Modify, only if D-M61A-2 and D-M61A-3 approve** | New keys for the weld height (provisional 0.075) and the weld-integrity tolerance (undecided); possibly `body_pose_stale_s` (undecided). Any change re-pins the M6.1 `trajectory_id` and fingerprint, because the limits file is hashed into the binding [V]. M6.0-D is unaffected |

### 5.4 Tests (in `spiderx_controller/test`, the repository's test home [V])

| File | Action | What it would prove (offline, no Gazebo) |
|---|---|---|
| `test_m61a_fixed_base_model.py` | **New** | • The wrapper expands for `fortress` + control.<br>• It equals the default expansion plus exactly `world` and one fixed `world → dummy_link` joint.<br>• `<ros2_control>` still lists the same 12 joints.<br>• The `ign sdf -p` output contains a fixed joint with parent `world`, and the root link sits at identity relative to the model for both descriptions (U1, structural part); skipped with a reason if `ign` is missing.<br>• Dense FK of the **interpolated** (cubic-Hermite) trajectory: every foot, and every link collision mesh if practical, stays above the ground at the chosen weld height, with a margin still to be decided (U7) |
| `test_m61a_fixed_base_launch.py` | **New** | Loads the new launch with `importlib`, without running it (like `test_controller_launch.py`), and checks:<br>• the pose bridge is identical to the posture-hold one;<br>• bridge YAML = `fortress_bridge_control.yaml`;<br>• the wrapper xacro is used;<br>• `-name spiderx` and the decided `-z`;<br>• `controller.launch.py` is included;<br>• no remap onto `/tf`;<br>• no `m6_gait_replay`, `m6_live_playback`, `test_one_joint`, `run_posture_hold_test` or action client |
| `test_m61a_unchanged_files.py` (or new rows in an existing pin table) | **New** | SHA-256 pins of every constraint-1 file, so the free-base path provably stays byte-identical |
| `test_m61a_observer.py` | **New** | The read-only observer (§5.6) creates no publisher or action client (static scan), and its statistics functions are correct on synthetic data |
| `test_m61_gait_replay.py` | **Modify, only with an approved §5.6 change** | Mock tests for the fixed-base readiness check and the weld-integrity monitor; re-pinned identities |

### 5.5 Documentation

| File | Action |
|---|---|
| This note | Kept; a status section is added after the decisions |
| `docs/M61A_IMPLEMENTATION_NOTES.md` | **New** (implementation PR) |
| `docs/M61A_OBSERVATION_RESULTS.md` | **New** (after an approved launch-only observation) |
| `docs/M61_IMPLEMENTATION_NOTES.md` | Update checklist items 7 and 8, and the deviations V7, V8 and V10 |
| `docs/SPIDERX_DEVELOPMENT_ROADMAP.md` | One M6.1-A line |

### 5.6 M6.1 replay code: proposed changes, not authorized

Each row below is a **proposal**. None is authorized; each needs D-M61A-3 and the owner's explicit
approval.

| Change | Why it is proposed | File |
|---|---|---|
| A read-only observer tool `spiderx_controller/m61a_observer.py` + `scripts/m61a_observe_fixed_base.py`. It subscribes to the pose, `/joint_states` and `/clock`, calls `list_controllers`, counts publishers and writes JSON to `log/m61a_observation/<UTC>/` | §4.4 baseline data, and §6 D acceptance in one repeatable command. It has no action client and no publisher | New files (not replay code) |
| **Readiness check of the fixed-base model** | F3: today the replay cannot tell a fixed base from a free one. Readiness would require all three:<br>(a) the `robot_description` it reads (read-only) contains a fixed joint `world → dummy_link`;<br>(b) the observed pose z is within a tolerance (undecided) of the weld height decided in D-M61A-1;<br>(c) the z and xy range over the readiness window is within that tolerance | `m6_gait_replay.with_body_pose` (or a sibling wrapper), `m61_live_adapter` (read `/robot_description`, transient-local, read-only), `m61_gates.pose_readiness` |
| **Weld-integrity monitor in flight** | F2: G1 cannot see a weld failure. It would trip when \|z − z_weld\|, the xy drift or the yaw drift exceeds a tolerance still to be decided (§4.4). Debounced, one cancel, `GATE_TRIPPED` | `m61_gates.py`, `m61_limits.yaml`, `m61_live_contract.py` |
| Pose freshness value | Only if D-M61A-2 changes 1.0 s | `m61_limits.yaml` + `APPROVED_LIMITS` |

**M6.0-D stays untouched [P]:**
- no `m6_*.py`, `m6_live_contract`, `m6d_gate`, `controller.launch.py` or controller YAML change;
- M6.0-D keeps using the free-base `fortress_control.launch.py`;
- its pinned URDF hash comes from the unchanged `spiderx.urdf.xacro`.

The existing hash-pin tests (`test_m6d_contract.py`, `test_m6d_live_enabling.py`) [V] would prove
all three.

---

## 6. Validation plan (future; NOT executed; no gait replay; no goal)

**Rules for every phase:**
- `M61_LIVE_DISPATCH_ENABLED` stays `False`.
- No command below sends a trajectory goal, publishes a command, or runs `m6_gait_replay`,
  `m6_live_playback`, `test_one_joint.py` or `run_posture_hold_test.py`.
- Phases C and D start Gazebo, so they need the owner's approval (D-M61A-4) and run on the owner's
  Ubuntu PC.
- None of these commands has been run for this note.

### A. Static / offline (no simulator)

| # | Command | What it does | Expected | How to verify | Debug a failure |
|---|---|---|---|---|---|
| A1 | `git status --short && git diff --stat origin/main -- src/spiderx_description src/spiderx_bringup/launch/fortress_control.launch.py src/spiderx_bringup/launch/fortress_posture_hold.launch.py src/spiderx_controller/launch src/spiderx_controller/spiderx_controller/m6_*.py` | Shows that the free-base and M6.0-D files are unchanged | Only the two new M6.1-A files appear under `spiderx_description/urdf` and `spiderx_bringup/launch`; no modification of existing files | Read the stat lines | Any `M` line on an existing file is a regression: revert it |
| A2 | `xacro $(ros2 pkg prefix spiderx_description)/share/spiderx_description/urdf/spiderx_fixed_base.urdf.xacro sim_backend:=fortress enable_control:=true controllers_file:=$(ros2 pkg prefix spiderx_controller)/share/spiderx_controller/config/spiderx_ros2_controllers.yaml > /tmp/m61a_fb.urdf && check_urdf /tmp/m61a_fb.urdf` | Expands the wrapper and parses the URDF tree | `robot name is: spiderx`, root link `world`, child `dummy_link` | `check_urdf` prints the tree | xacro error: check the include path and args. Root not `world`: check the joint parent and child |
| A3 | `ign sdf -p /tmp/m61a_fb.urdf > /tmp/m61a_fb.sdf && ign sdf -k /tmp/m61a_fb.sdf && grep -n -B2 -A4 "<parent>world</parent>" /tmp/m61a_fb.sdf && grep -n -A3 "link name=.dummy_link." /tmp/m61a_fb.sdf` | Offline URDF→SDF conversion (the `validate_fortress.sh` convention), plus a look at the root link's pose | A valid SDF with one fixed joint (parent `world`, child `dummy_link`), and `dummy_link` with no `<pose>` or an identity pose relative to the model (U1, U2) | The grep output | No joint: check the link name is exactly `world`. Non-identity root pose: record the offset and resolve U1 before any height threshold is used |
| A3b | `xacro …/spiderx.urdf.xacro sim_backend:=fortress > /tmp/m61a_default.urdf && ign sdf -p /tmp/m61a_default.urdf \| grep -n -A3 "link name=.dummy_link."` | The same root-pose look for the unchanged free-base description (U1) | `dummy_link` with no `<pose>` or an identity pose | The grep output | As A3 |
| A4 | `xacro …/spiderx.urdf.xacro sim_backend:=fortress \| sha256sum`, on this branch and on `origin/main` | Default expansion identical | Equal hashes | Compare the two lines | Different: an existing model file was touched |
| A5 | `ros2 launch spiderx_bringup fortress_m61_fixed_base.launch.py --show-args` | Parses the launch file only; starts nothing | The args `headless` and `rviz` are listed | Exit 0 | Python error: fix the launch file before anything else |
| A6 | `grep -n "M61_LIVE_DISPATCH_ENABLED = " src/spiderx_controller/spiderx_controller/m61_live_contract.py` | Gate check | `M61_LIVE_DISPATCH_ENABLED = False` | One line | Anything else: stop |

### B. Build and isolated tests (no simulator)

| # | Command | What it does | Expected | How to verify | Debug a failure |
|---|---|---|---|---|---|
| B1 | `pip install setuptools==59.6.0 && colcon build --symlink-install; pip install setuptools==84.0.0` | Builds with the documented setuptools pin, then restores it | "8 packages finished", 0 errors | Read the summary; only the known CMake deprecation warning on stderr | `colcon build --packages-select <pkg> --event-handlers console_direct+` |
| B2 | `source install/setup.bash && PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 colcon test && colcon test-result --all` | All tests, including the new M6.1-A tests and the existing pins | 0 errors, 0 failures. Isolated-domain tests may only skip with a stated isolation reason | `colcon test-result --all --verbose` | Re-run one file with `python -m pytest <file> -q` |
| B3 | `pgrep -af "ign gazebo\|gz sim\|parameter_bridge\|robot_state_publisher\|controller_manager\|spawner" \|\| echo none` | Leftover processes after the tests | `none` | Output | Kill the leftovers and find the test that did not shut down |

### C. Launch-only observation (gate False; no goal), needs owner approval

| # | Command | What it does | Expected | How to verify | Debug a failure |
|---|---|---|---|---|---|
| C1 | Terminal 1: `ros2 launch spiderx_bringup fortress_m61_fixed_base.launch.py` (add `headless:=true` only if decided) | Starts Gazebo with the welded robot, the bridges and the controllers. Controllers hold neutral; **no goal is sent** | Gazebo shows the robot suspended above the ground (≈ 2 cm if 0.075 m is kept and U1 holds) and not falling. Spawners finish with no errors | Visual check plus the launch log | Robot falls: weld not applied (repeat A3). Controllers fail: read the M4.1 messages in the log; Ctrl+C and relaunch |
| C2 | Terminal 2: `ros2 control list_controllers` | Read-only service call | `joint_state_broadcaster` and `leg_trajectory_controller` both `active` | Both lines say `active` | `ros2 control list_hardware_interfaces`; check the plugin path in the launch log |
| C3 | `ros2 topic info /joint_states` | Publisher count | `Publisher count: 1` | Output | 2 publishers means the passive bridge YAML is in use: check the bridge `config_file` |

### D. Read-only checks (pose, rate, freshness, frames, controllers, leftovers)

| # | Command | What it does | Expected | How to verify | Debug a failure |
|---|---|---|---|---|---|
| D1 | `ros2 topic info -v /spiderx/sim/world_poses` | Type and publisher | `tf2_msgs/msg/TFMessage`, publisher node `spiderx_sim_ground_truth_bridge` | Output | Missing: is the pose bridge node running (`ros2 node list`)? |
| D2 | `ros2 topic echo --once /spiderx/sim/world_poses \| grep -A12 "child_frame_id: spiderx"` | Content of the model entry | z ≈ the decided weld height (0.075 if kept), rotation ≈ identity (w ≈ 1) (U4, U5) | Read the translation and rotation | No `spiderx` entry: compare with `ign topic -e -t /world/spiderx_fortress/pose/info -n 1` (§3.5) |
| D3 | `ros2 topic hz /spiderx/sim/world_poses --window 1000` (60 s, then Ctrl+C) | Wall-time receipt rate and min/max gap | Rate around the M2 reference of about 50 Hz [H]; max gap recorded for D-M61A-2 (U6) | Read the average, min, max and std dev | Low rate: GUI load. Try `headless:=true` in a separate approved run |
| D4 | `ign topic -e -t /world/spiderx_fortress/pose/info -n 1` | Gazebo-side source, read-only | One `Pose_V` containing `name: "spiderx"` | Output | Nothing: SceneBroadcaster not running. Check the world file in the log |
| D5 | `ros2 run tf2_ros tf2_echo world dummy_link` (5 s, then Ctrl+C) | Static TF of the weld | Translation 0 0 0, identity rotation. The weld origin is zero; the height lives in the Gazebo pose, not in TF | Output | "frame does not exist": the wrapper was not used by `robot_state_publisher` |
| D6 | `ros2 topic info -v /tf \| grep -c spiderx_sim_ground_truth_bridge` | Confirms the pose bridge never publishes `/tf` | `0` | Output | Non-zero: remap error in the new launch |
| D7 | `ros2 topic echo --once --qos-durability transient_local /robot_description \| grep -c "<parent link=\"world\"/>"` | Read-only check that the running description is the fixed-base variant (U8) | `1` | Output | `0`: the wrong launch is running |
| D8 | `ros2 run spiderx_controller m61a_observe_fixed_base.py --duration 120` (proposed tool, §5.6) | Read-only baseline: pose z, tilt and drift statistics, receipt gaps, controllers and publisher counts → `log/m61a_observation/<UTC>/` | JSON report; every D-M61A-4 criterion met | Read the report | The report names the failing criterion |
| D9 | Ctrl+C terminal 1, then `pgrep -af "ign gazebo\|gz sim\|parameter_bridge\|robot_state_publisher\|controller_manager\|spawner" \|\| echo none`, then `ros2 daemon stop` | Clean shutdown, no leftovers | `none` | Output | `pkill -f "ign gazebo"` etc., and note the leftover in the results doc |

---

## 7. Decision gates (owner): all pending

**None of these decisions has been made. Saving this note does not approve any of them.** Each is
required before the step it guards. The safe default for every decision is "no": nothing is
implemented or launched.

**D-M61A-1: Fixed-base technique (status: pending).**

| | |
|---|---|
| For a beginner | "How do we hold the robot's body still in the air so that only the legs move?" Options: glue the body to the world (a *weld*), set it on a stand (a *fixture*), or make a special copy of the robot description that is welded, used only by one special launch file |
| Recommendation | Weld via a separate wrapper description used only by a new launch file (C1 + L1), at a provisional 0.075 m |
| Owner decides | The technique, and the weld height (0.075 m, or another value with a reason) |

**D-M61A-2: Pose source and semantics (status: pending).**

| | |
|---|---|
| For a beginner | "Which sensor reading do we trust to say where the body is?" The simulator knows the exact body position (*ground truth*). The posture-hold test already reads it from `/spiderx/sim/world_poses`. Also decide how old a reading may be before we stop |
| Recommendation | Accept `/spiderx/sim/world_poses` (`TFMessage`, entry `spiderx` = Gazebo model pose in the world frame, receipt-time stamped), as used in M2. This is subject to closing U1 (model frame = `base_link` origin). Decide the freshness timeout (keep 1.0 s, or 0.25 s) **after** measuring the gaps (§4.4) |
| Owner decides | Accept the source and its semantics, and the freshness value |

**D-M61A-3: Implement the fixed-base launch (status: pending).**

| | |
|---|---|
| For a beginner | "May code be written now?" This would permit only the files in §5. It would not permit running Gazebo or a replay |
| Owner decides | Approve §5, and separately each §5.6 replay-code change: the readiness check of the fixed-base model and the weld-integrity monitor. Both are proposals and are **not** authorized by this note |

**D-M61A-4: Acceptance criteria for launch-only observation (status: pending).**

| | |
|---|---|
| For a beginner | "What must we see on screen and in the numbers before we trust the setup?" The simulator runs, the robot hangs still, and nothing moves except that the controllers hold the legs |
| Proposed criteria (provisional) | **Structure:**<br>• all of §6 A and B pass;<br>• both controllers `active`;<br>• exactly 1 `/joint_states` publisher;<br>• the pose topic is the `TFMessage` from `spiderx_sim_ground_truth_bridge`, with the `spiderx` entry in every message;<br>• static TF `world → dummy_link` exists;<br>• `/robot_description` contains the weld;<br>• the pose bridge does not publish `/tf`.<br>**Weld:** z within ±0.005 m of the decided weld height (the M2 range threshold [V] as a provisional value).<br>**Tilt:** \|roll\|, \|pitch\| ≤ 0.10 rad (the M2 threshold [V], provisional).<br>**Freshness:** the max pose receipt gap is recorded and is below the timeout chosen in D-M61A-2.<br>**Shutdown:** no leftover process after Ctrl+C.<br>No goal sent; gate False throughout |
| Owner decides | Approve, edit or replace the criteria, and approve the observation run itself |

**D-M61A-5: Separation from any free-base gait milestone (status: pending).**

| | |
|---|---|
| For a beginner | "Promise that results from the welded robot are never presented as walking." The welded setup tests the legs, the controller and the safety system only |
| Proposed rules | The fixed-base launch, model and results stay labelled "fixed base". A free-base gait milestone (future, e.g. M6.2) needs its own design, envelope, approval and results document. It may not reuse the weld or a fixed-base result as evidence, and the fixed-base files are never included by free-base launches |
| Owner decides | Accept these rules |

---

## 8. Risk register

| # | Risk | Likelihood / impact | Mitigation |
|---|---|---|---|
| R1 | Incorrect world attachment (welded at the wrong pose, not welded, or welded in the default model by accident) | Medium / high | New wrapper only; pin tests on every free-base file; §6 A2–A4 offline checks; §6 C1 visual check and D2/D7 read-only checks; the proposed weld-integrity monitor (§5.6, not authorized) |
| R2 | Accidental modification of the default free-base simulation | Low / high | Constraint 1; SHA-256 pins; A1 and A4 diffs; no CMake change needed |
| R3 | The pose topic represents the wrong entity or frame (a link instead of the model, a relative pose, or a model frame offset from `base_link`) | Low–medium / high | Reuse the M2 source and consumer code; close U1 (A3, A3b, D2); D2 and D4 confirm `spiderx` at the decided weld height; the proposed readiness check rejects a z outside tolerance (§5.6) |
| R4 | Stale or missing bridge data (bridge crash, GUI load, low RTF) | Medium / medium | Readiness requires a fresh pose [V]; in-flight freshness cancel (wall time) [V]; measure gaps before choosing a timeout (§4.4); a missing pose refuses (safe) |
| R5 | TF / frame mismatch (a new `world` frame conflicting with future odometry; RViz confusion) | Low / medium | `world` exists only in the fixed-base launch; D-M61A-5 forbids combining it with odometry stacks; the RViz fixed frame stays `dummy_link` |
| R6 | Collision or ground-contact artifacts | Low–medium / medium | No fixture geometry (weld only); intended clearance ≈ 2 cm (unverified, U7); dense FK clearance test of the interpolated trajectory; self-collision stays disabled as today [V] |
| R7 | The fixed base is mistaken for a free base (the replay is run on the posture-hold launch) | Medium / high (today the replay only labels the base [V]) | Proposed readiness check: `robot_description` weld plus z at the weld height (§5.6, not authorized); evidence records `base_constraint` and the observed z |
| R8 | Fixed-base leg motion presented as walking or balance validation | Medium / high (reputational) | §4.1 wording; non-claims in every outcome; D-M61A-5; results-doc review |
| R9 | Regression of M6.0-D or ordinary Fortress control | Low / high | No M6.0-D file touched; M6.0-D keeps the free-base launch; the existing pinned hashes and IDs must stay green |
| R10 | Duplicated launch logic drifts from the original description launch | Medium / low | A static comparison test (§5.4); a future change to the description launch fails that test until it is mirrored or explicitly accepted |

---

## 9. Open uncertainties and undecided items

**Uncertainties.** These are resolved only by the checks named here, never by assumption:

| # | Uncertainty | Label | Resolved by |
|---|---|---|---|
| U1 | The Gazebo model frame (the pose under `child_frame_id: spiderx`) coincides with the `base_link` origin. Established today only for `dummy_link` ↔ `base_link` in the URDF [V]; M2 data is consistent in z and orientation [H]; the converted model structure has not been inspected | [O], then [G] | §6 A3, A3b; D2 |
| U2 | sdformat turns the URDF `world` link plus a fixed joint into a joint to the world | [O] | §6 A3 |
| U3 | The weld actually holds the body in Fortress | [G] | §6 C1, D2, D8 |
| U4 | The weld holds the model at its spawn pose (`create -z`) rather than at the URDF joint origin | [G] | §6 D2 |
| U5 | The welded model's pose still appears on `pose/info` under the name `spiderx` | [G] | §6 D2, D4 |
| U6 | The maximum pose receipt gap under load stays below 0.25 s | [G] | §6 D3, D8 |
| U7 | Ground clearance throughout the **interpolated** trajectory: the cubic-Hermite path keeps the feet at or above the neutral foot plane, and the feet are the lowest points; and the clearance actually achieved in Gazebo | [O], then [G] | §5.4 dense FK test; §6 C1, D8 |
| U8 | `robot_state_publisher` publishes `/robot_description` with transient-local durability. The topic existed in M2 [H], and `create` reads it [V] | [G] | §6 D7 |
| U9 | gz_ros2_control ignores the weld joint, and the 12-joint contract and `/joint_states` are unchanged | [G] | §6 C2, C3 |
| U10 | Behaviour of the new launch with `headless:=true`. The posture-hold launch documents its own headless mode as untested [V] | [G] | A separate approved run |

**Undecided (owner decisions, §7).** Each of these remains open:
- the fixed-base technique and the weld height (0.075 m is provisional);
- the pose source acceptance and the pose freshness timeout (1.0 s today, 0.25 s requested);
- the weld-integrity tolerance;
- any tighter fixed-base tilt threshold;
- the launch-only acceptance criteria;
- whether to implement anything at all;
- whether to add the readiness check of the fixed-base model and the weld-integrity monitor.

---

*Design only. Saved as documentation at the owner's request; no implementation, no PR, no merge, no
launch, no runtime and no live action. `M61_LIVE_DISPATCH_ENABLED` remains `False`. Every decision
in §7 is pending.*
