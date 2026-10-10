# M6.1-A Cloud verification: incident diagnosis, corrected revision, Cloud simulator observation

**Where this ran:** a Claude Cloud VM (Ubuntu 24.04 container, 4 vCPU Xeon 2.8 GHz, 16 GB, no GPU)
with RoboStack ROS 2 Humble, Gazebo Fortress 6.16.0 and Fast DDS (`rmw_fastrtps_cpp`). It is
**not** the owner's Ubuntu 22.04 PC. Every result below is labelled as one of:

| Label | Meaning |
|---|---|
| **Offline** | pytest/colcon tests, static validators, dry runs; no simulator |
| **Isolated domain** | in-process rclpy test doubles on a private, localhost-only ROS domain; no simulator |
| **Cloud simulation** | the real launch file running Gazebo Fortress (GUI on Xvfb, Mesa software rendering) in this VM |
| **Owner PC** | the owner's Ubuntu 22.04 machine: **nothing in this document ran there** |

Both live gates stayed `False` throughout (`m6_live_contract.LIVE_DISPATCH_ENABLED`,
`m61_live_contract.M61_LIVE_DISPATCH_ENABLED`). No goal and no command was sent to any simulated
controller. No hardware was involved.

**Outcome (Cloud simulation, `7f4c30f`).** Every **per-run** criterion of the §10 provisional list
of [`M61A_FIXED_BASE_IMPLEMENTATION.md`](M61A_FIXED_BASE_IMPLEMENTATION.md) passed in three separate
launches (1, 1b, 2a–2e, 3, 4, 7, 8, plus mount and zero commands). The requirement-by-run table is
in §8.1. The remaining items:
- **Criterion 5 (visual):** met by runs 2–3; run 1 is UNMEASURED (inconclusive).
- **Criterion 6 (`/scan`), as written** (version 1: "the `/scan` ranges … within ±8 mm", i.e.
  individual ranges) **FAILED** in all three runs: 84–88 % of the 40-scan means and 52–53 % of the
  single-scan ranges were within ±8 mm (§8.3, §9.6).
  - The fitted-pose agreement (within 0.4 mm) is a supplementary metric (6a). It is **not**
    criterion 6.
  - *Correction:* an earlier version of this summary said criterion 6 "passes as pose agreement".
    That reinterpreted the criterion.
  - A frozen version-2 proposal and its single validation run are in §9.6.
- **Pose-stream liveness and z/roll/pitch:** see §9 (§8.2 is corrected there).
- **Run 1's supplementary controller-hold check:** UNMEASURED.

Two earlier launch attempts did not become observation runs (§4.1). Owner-PC verification is still
outstanding.

**After the closeout (§9, `b982d8d`).**
- **Pose: what is measured.** Only the model entry is measured: the physics pose of the welded
  body. The `dummy_link` entry is the SDF weld value; §8.2 called it measured, and that is
  corrected. z, roll and pitch have **no independent confirmation** (U-L2, unresolved).
- **Readiness correction.** Readiness now requires simulation time to advance while the body pose
  is received. A paused world, whose streams stay "fresh", is refused. Demonstrated live in
  `run_05L`.
- **Gate tests.** Both gate modes pass: enabled build, 1457 passed; disabled, a clean build and
  1497 tests with 0 failures.
- **Criterion 6, `run_04`.** Version 1 **FAIL** again; the frozen version-2 proposal **PASS** on
  its single validation run. Acceptance is the owner's.

**One-cycle prerequisites (§10: `927acb2`, `1732c32`).**
- **E8, the pose source.** It is traced to DART and shown in a fresh no-motion launch (`run_06`):
  the model entry in this launch is DART's pose of the merged body, and the composition cancels
  the SDF factor. That is enough for simulation, without an added sensor. The trust boundary is
  DART.
- **E9.** Corrected to dynamic link/kinematic consistency, which says nothing about body height or
  tilt. Its analysis was frozen before any motion run.
- **E5.** The streams are now checked *at the send*, after a drain, over a 1 s window.
- **E11.** A restore-and-verify script, validated against a running launch (`DISABLED AND
  VERIFIED`) and against an enabled build (`NOT VERIFIED`, `--live` never run).

## 1. The incident (first Cloud attempt, `41fc10f`)

The first Cloud phase-1 attempt stopped before any launch because one required test failed
(evidence preserved verbatim with its manifest in
[`evidence/m61a_cloud/incident_20261009T075335Z/`](evidence/m61a_cloud/incident_20261009T075335Z/README.md)):

- `colcon test`: 1465 tests, 0 errors, **2 failures** (one pytest case and its CTest wrapper),
  0 skipped;
- the case: `test_m6d_isolated_success.py::test_gate_enabled_main_succeeds_end_to_end_with_one_goal`
  (M6.0-D; gate set `True` inside the test only; in-process fake controller stack on isolated
  domain 160);
- readiness #1 passed, the confirmation was read, readiness #2 was **not ready**:
  `('REFUSED', 'readiness_not_ready', [])`; no goal was sent, even to the fake;
- after the pytest summary, rclpy printed `The following exception was never retrieved: cannot use
  Destroyable because destruction was requested`, a line that appears in no passing run;
- the outcome file holding the readiness reports was deleted by pytest's keep-three temporary
  directory rotation before it could be copied. **The exact failed check is unrecoverable.**

### 1.1 What the incident's own evidence rules in and out

| Observation | Consequence |
|---|---|
| The suite took 11.84 s (a passing run takes about 17 s with the goal; about 11–12 s without it) | Readiness #2 took its normal ~4 s. Every 10 s timeout path (`controller_manager_unavailable` by timeout) is excluded |
| Reason `readiness_not_ready`, not `stack_incompatible` | The classification was compatible/warning: the action server and the 12-joint names were observed. Excluded: `action_server_*`, `joint_names_mismatch`, a whole-probe exception, zero samples |
| The fake publishes constant start positions until a goal arrives | `start_pose_not_neutral` and `joint_states_incomplete` are excluded |
| Remaining codes | `joint_states_stale`, `joint_states_multiple_publishers` / `no_publisher` (graph), `joint_states_no_messages` with exactly one sample, `probe_error` in one attribute. Of these only `joint_states_stale` has a mechanism in the test double (§1.3) |
| Session transcript: nothing else ran between 07:59:10 and 07:59:51 UTC; the test ran 07:59:21.6–07:59:33.5 | The CPU-heavy scan self-tests mentioned in the first report had **ended at 07:58:25**. Concurrent load from this session is **not** supported as the cause (the earlier report's suggestion is corrected here) |
| ctest ran the suite sequentially (no `-j`); the isolation probe saw an empty domain 160 | No other test process shared the domain during the run |

### 1.2 Diagnosis method

All diagnosis code is evidence tooling, not package source, preserved in
[`evidence/m61a_cloud/m6d_diagnosis/`](evidence/m61a_cloud/m6d_diagnosis/README.md):

- **Observer plugin** (`diag_plugin.py`, loaded with `pytest -p`): runs the repository test
  unmodified and records both readiness reports in full (every check and failure message, sample
  count, stamp order, sample age against the fake's simulation clock, monotonic and wall times,
  collection duration, the nodes visible to the probe), the isolation snapshots, the fake stack's
  tick overlaps and its publish order. Each run uses its own `--basetemp`, outside pytest's
  rotation. `DIAG_STACK=original` substitutes the pre-fix test double (A/B).
- **Mechanism experiments:** `stress_order.py` (60 s of ticks, single-threaded subscriber in the
  same process), `reorder_demo.py` (0–25 ms between stamping and publishing),
  `stress_teardown.py` (150 start/stop cycles; `TRACE_ORPHANS=1` prints where an orphaned rclpy
  Task failed).
- **Load:** 8 `python3 -c 'while True: pass'` processes on the 4 vCPUs, where stated.

### 1.3 Findings

| # | Experiment (original test double) | Result |
|---|---|---|
| 1 | 31 runs, no competing load (1 smoke + 30) | **31 passed: not reproduced.** Tick overlaps in 1 run; no stamp-order violation; no "never retrieved" line |
| 2 | 5 runs, 8 burners | 4 passed. 1 failed **differently**: `TRACKING_FAILED`, `sample_gap` (the starved fake stopped publishing for longer than the tool's gap limit; the tool failed safe). Tick overlaps in 1 run |
| 3 | 60 s tick stress, no load / 8 burners | 0 / 2 overlapping ticks of ~6000; 0 / 0 stamp-order violations |
| 4 | 0–25 ms delay between stamping and publishing, 5 s | **84 backward stamps in 498; 2 of 2 200-sample windows would fail `joint_states_stale`** |
| 5 | 150 start/stop cycles | **12 orphaned Tasks** `InvalidHandle('cannot use Destroyable because destruction was requested')` raised at `rclpy/executors.py:444` (`gc.trigger()` on the executor's guard condition): **the incident's line, reproduced** |

**Defects demonstrated in the test doubles (not in production code):**

1. **Overlapping ticks.** The 100 Hz `/clock` + `/joint_states` tick of `m6d_isolated_stack.py`
   shared a `ReentrantCallbackGroup`. rclpy 3.3 marks a timer as taken *before* its callback runs
   (`Executor._make_handler`), so a tick slower than 10 ms overlaps the next one. Overlaps occurred
   with and without load (1, 2); when the overlapping tick publishes later than it stamped, the
   received stamps go backwards and the production `joint_states_stale` check (stamps must strictly
   increase) correctly refuses (4). A real `joint_state_broadcaster` publishes from one update
   loop; the double did not.
2. **Teardown race.** Humble's `Executor.shutdown()` waits only for callbacks already running and
   then destroys its guard condition. A pool callback that starts just afterwards still triggers
   it and fails into a Task nobody reads, printed at garbage collection as the incident's line
   (5). A first fix that drained the pool *after* `shutdown()` did not help (8 orphans in 150);
   the fix stops scheduling first (§2).
3. **Stamp rounding.** `_stamp()` rounded only the fractional second, so a time within 0.5 ns
   below a whole second became a stamp one second in the past.

**Root cause, stated with its uncertainty.** The incident's exact failed check cannot be
recovered. Its "never retrieved" line is fully explained by defect 2 (a teardown artifact that
does not by itself change a readiness verdict). Among the checks consistent with the incident's
duration and classification, the only one with a demonstrated mechanism is `joint_states_stale`
through defect 1. That is the **most probable cause, not a proven one**: the failure did not
recur in 36 recorded runs of the original double (31 without competing load, 5 with 8 CPU
burners). The production readiness logic behaved
correctly (it refused on its inputs and sent nothing) and is unchanged.

## 2. Corrections (`7f4c30f`, tests only)

| File | Change |
|---|---|
| `test/m6d_isolated_stack.py` | The tick has its own `MutuallyExclusiveCallbackGroup` (one tick at a time; stamps leave in order). `SpinThread`: spins until `stop()`, then stops scheduling, joins, waits for every scheduled callback (the pool) and only then shuts the executor down and destroys entities. `_stamp()` converts whole nanoseconds. An executing goal ends (aborted, never "succeeded") when the double stops |
| `test/m6d_isolated_action_server.py` | Uses `SpinThread`; a waiting execute callback returns on stop |
| `test/test_m6d_isolated_success.py` | Pass-through `ReadinessRecorder`: every readiness report (checks, failure messages, sample stamps, non-increasing indices, sample age, visible nodes) goes into the assertion message, so a future failure explains itself in the xunit and colcon logs. Three regression tests: stamp rounding at a second boundary; no overlapping ticks and strictly increasing received stamps with a 0–25 ms publish delay; 30 start/stop cycles with no orphaned Task |

No production module, threshold, tolerance or gate changed; no test was skipped or weakened.
**A/B:** the three new tests fail on the original double (`_stamp(117.9999999996)` = 117.0 s;
up to 3 concurrent ticks; one orphaned `InvalidHandle`) and pass on the corrected one. The
teardown stress gives 0 orphans in 150 cycles (12 before); the reorder demo gives 0 backward
stamps (84 before).

**Known environmental limit (not changed):** under heavy CPU contention the end-to-end test can
fail with `TRACKING_FAILED`/`sample_gap` (finding 2) because the in-process fake cannot publish on
time; the tool's gap limit is a production threshold and stays. Run the suite without competing
CPU-heavy work, as required.

## 3. Verification of the corrected revision (Offline / Isolated domain)

Revision `7f4c30f` (branch `claude/stoic-shannon-ur2mes`), clean build, nothing else running
(load average 0.1–0.7). Evidence:
[`evidence/m61a_cloud/verify_7f4c30f/`](evidence/m61a_cloud/verify_7f4c30f/README.md).

| Check | Command | Result |
|---|---|---|
| Clean build | `rm -rf build install && colcon build --symlink-install` (setuptools 59.6) | 8 packages, exit 0, 15.1 s |
| Focused M6.0-D suite | `python3 -m pytest` on the 8 `test_m6d_*` files (unique `--basetemp`) | **276 passed**, 0 failed, 45.8 s; no "never retrieved" line |
| Full suite | `colcon test --packages-select spiderx_controller spiderx_scripts` + `colcon test-result --verbose` | **1468 tests, 0 errors, 0 failures, 0 skipped** (6 min 14 s). 1465 + the 3 new tests; 1430 pytest cases + 38 CTest wrappers; `test_m6d_isolated_success.py` 5/5 in 30.0 s |
| Static validators | `validate_fortress.sh`, `validate_m1…m4` (static mode) | 60 / 14 / 17 / 17 / 18 PASS, 0 FAIL |
| Clearance config | `m61a_clearance --check-config` | `Config consistent with the geometry.` |
| Dry runs | `m6_gait_replay.py --dry-run --no-write`, `m6_live_playback --dry-run --no-write` | trajectory `241760e7dfd5ef12` / fingerprint `9dba1a17…`; trajectory `44f0a7ad52e5c330` / fingerprint `0d6ef417…` (unchanged) |
| Observer interface | `m61a_observe_fixed_base --interface-only` | exit 0; `publishers: {}`, `action_clients: {}` |
| Gates | `--live` of both tools | exit 3, `REFUSED`, nothing sent |

## 4. Cloud simulation: M6.1-A phase-1 launch-only observation

**Setup (identical for every launch).**
- Revision `7f4c30f`, the install of §3.
- Gazebo Fortress 6.16.0 GUI on `Xvfb :77` (1600×900×24), Mesa software rendering.
- Fast DDS; `ROS_DOMAIN_ID=0`, `ROS_LOCALHOST_ONLY=1`, no discovery server, set by one `env.sh`
  sourced in the launch shell and in the observer/CLI shell (`env_A.txt`, `env_B.txt` per run).
- Launch: `ros2 launch spiderx_bringup fortress_m61a_fixed_base.launch.py` (defaults: GUI,
  `gz_verbosity:=2`), in its own process group, output to `launch.log`.
- After both controllers were `active` (`ros2 control list_controllers`) plus 3 s:
  `m61a_observe_fixed_base --domain-id 0 --preflight`, then `--domain-id 0 --duration 120`
  (wall time).
- Then read-only captures: controllers, hardware interfaces, nodes, topics,
  `ros2 topic info -v` of the command topic, `ros2 action info`, one `/scan`, `ign model` (model
  pose, `dummy_link`, weld joint), a 10 s / 40-scan subscriber capture (`capture.py`: no
  publisher of its own besides rclpy's `/parameter_events`) and GUI screenshots (`import -window
  root`). The camera moves are GUI-only (`/gui/move_to`, `/gui/move_to/pose`) and change no
  simulation state.
- Stop: SIGINT to the launch process group (Ctrl+C equivalent), `ros2 daemon stop`, Xvfb stop,
  leftover-process check.

**Controller initialization (launch) versus the observer.** The launch itself loads and
activates `joint_state_broadcaster` and `leg_trajectory_controller`; they then hold their initial
positions. That is the launch's own behaviour. The observer and every capture are read-only: no
publisher on any command topic, no action client, no goal; `goals_sent 0`,
`publishers_created 0` in every observation file.

### 4.1 Launch attempts that did not become observation runs

| Attempt | What happened | Classification |
|---|---|---|
| 09:02 (`attempt_aborted_helper_defect/`) | The launch came up normally (controllers active at about 09:02:55), but my wait helper never recognised it: the auto-started ros2 CLI daemon answered `RuntimeError: !rclpy.ok()`, and `list_controllers` output carries ANSI colour codes that my pattern did not strip. Stopped via the normal Ctrl+C path (group empty after 6 s) | **Evidence-helper defect**, fixed (strip ANSI codes, restart a faulted daemon, wait for Xvfb before the leftover check). No observation |
| 09:05 (`attempt_failed_gazebo_startup_race_0905/`) | The GUI started, found the server's `/gazebo/starting_world` subscription and published once; the server never loaded the world (no console log for this launch). `create` gave up after ~2 min (exit 255), so no controller manager; Ctrl+C needed a SIGKILL of the hung server; cleanup clean | **Gazebo Fortress GUI→server startup handshake race** (`ign gazebo` GUI mode sets `wait_gui=1`, `cmdgazebo6.rb`; the server waits for the starting world before logging). Outside SpiderX; 1 of 5 launches in this session. The helper now fails fast and captures diagnostics if it recurs |

Neither attempt is counted, both are preserved, and the three runs below are separate fresh
launches made after each cause had been identified.

### 4.2 Results per run (provisional criteria of M6.1-A §10)

| Criterion | run_01 | run_02 | run_03 |
|---|---|---|---|
| Launch → both controllers active | 14 s | 11 s | 11 s |
| 1. Preflight READY | ✅ (11/11 checks) | ✅ | ✅ |
| 1b. 120 s observation READY at its end | ✅ | ✅ | ✅ |
| 2a. Body z mean / range (0.125 m ±1 mm; ≤ 1 mm) | 0.125 / 0.0 | 0.125 / 0.0 | 0.125 / 0.0 |
| 2b. Attachment translation / rotation (≤ 1 mm, ≤ 0.0033 rad) | 0 / 0 | 0 / 0 | 0 / 0 |
| 2c–2e. Spawn / link deviation; tilt | 0 / 0; 0 | 0 / 0; 0 | 0 / 0; 0 |
| 3. Pose receipt gaps: max / p99.9 (< 1.0 s), wall | 0.076 / 0.053 s | 0.068 / 0.052 s | 0.061 / 0.054 s |
| 3. Joint-state sim gap max (≤ 0.25 s) | 0.05 s | 0.02 s | 0.01 s |
| Clock: simulated / wall span, real-time factor | 84.2 / 122.0 s, 0.690 | 85.0 / 122.0 s, 0.697 | 85.0 / 120.1 s, 0.708 |
| 4. Unexpected pose sample codes | none | none | none |
| 4. `header.frame_id` of the pose entries (recorded, not gated) | `''` (13 692) | `''` (13 688) | `''` (13 722) |
| 5. Visual: legs clear of the ground | inconclusive (no foot-level view) | ✅ legs clear (foot-level view) | ✅ legs clear (foot-level view) |
| 6. `/scan` vs the weld: fitted lidar dx, dy, dyaw (robust 1σ) | +0.38, +0.08 mm, +0.02 mrad (0.30 mm) | +0.28, +0.03 mm, +0.11 mrad (0.25 mm) | +0.17, +0.02 mm, +0.04 mrad (0.29 mm) |
| 6. `/scan` worst per-surface median residual (±8 mm) | 2.4 mm (box_b) | 1.6 mm (box_b) | 1.9 mm (pillar) |
| 7. Ownership: `ros2 action info` clients / `ros2 topic info` publishers / observer start+end | 0 / 0 / 0 | 0 / 0 / 0 | 0 / 0 / 0 |
| Mount from the running model | `/robot_description` weld world→`dummy_link` (0, 0, 0.125, rpy 0); Gazebo: model at identity, `dummy_link` at (0, 0, 0.125) | same | same |
| Zero motion commands: command-topic messages / goal statuses in the 10 s capture; goal lines in the launch log | 0 / 0; 0 | 0 / 0; 0 | 0 / 0; 0 |
| Holding: controller reference change / tracking error over the capture | not measured (helper defect, `NOTES.txt`); joint states changed ≤ 4.8e-19 rad | 0.0 / 1.3e-11 rad (207 states) | 0.0 / 1.3e-11 rad (290 states) |
| 8. Clean shutdown (group empty; nothing left) | ✅ 3 s | ✅ 3 s | ✅ 4 s |
| **Verdict** | **PASS** (visual inconclusive, holding check not measured) | **PASS** | **PASS** |

Run 2 note: an offline `/scan` re-analysis of run 1 (single-core Python) ran during part of run 2's
observation window; its timing statistics include that load (`run_02/NOTES.txt`). They are within
a few milliseconds of runs 1 and 3.

### 4.3 Interpretation

- **Pose and frames.** The observer composes `T_world_body = T_world_model · T_model_body` from
  `/spiderx/sim/world_poses`. In every run the model sits at the identity spawn pose and the body
  (`dummy_link` = `base_link`) at exactly (0, 0, 0.125) with zero attitude for 120 s: a weld to the
  world in DART produces no motion at all, so the zero ranges show that nothing moved, not that the
  pose source is noise-free. The pose entries carry an empty `header.frame_id`; frame identity
  therefore comes from `child_frame_id` and the bridge configuration, which is recorded for any
  later consumer.
- **Attachment.** Attachment, spawn and link deviations are 0 against tolerances of 1 mm /
  0.0033 rad; the Gazebo model, link and joint queries agree with `/robot_description`.
- **Timing.** The GUI under software rendering ran at a real-time factor of about 0.70 (85 s of
  simulation per 120 s of wall time). Pose receipt gaps stayed below 0.08 s and joint-state
  simulation gaps at 0.01–0.05 s; the 1.0 s pose timeout and 0.25 s gap limit were never
  approached.
- **`/scan` (supporting evidence, not proof of attachment).** The scan plane is
  z = 0.125 + 0.141 = 0.266 m (`lidar_joint`), frame `lidar_link`, 360 beams over 359°, 10 Hz,
  σ = 1 cm noise. The comparison averages 40 scans per beam, ray-casts the same beams against the
  world's collision geometry (four 0.5 m walls, box_a, box_b, the pillar) from the weld-implied
  lidar pose (0.051, −0.045, yaw +90°), and fits the lidar's planar pose robustly. A horizontal
  scan constrains only x, y and yaw, not z, roll or pitch. Five beams differ by more than 2 cm in
  every run, at identical angles: two graze an edge (−62°, +30°); three sit on the `gpu_lidar`
  cube-map seams (lidar-frame −135°, −45°, +45°), where Gazebo returns the range of a
  neighbouring direction. With the eight beams within 1° of the seams excluded (and nothing
  else), the fitted lidar pose is within 0.4 mm and 0.11 mrad of the weld-implied pose (robust
  1σ ≤ 0.31 mm), and every surface's median residual is within 2.4 mm. The **fitted pose**
  agrees with the welded pose well inside ±8 mm; individual ranges do not all stay within ±8 mm
  (§8.3). The raw all-beam result is preserved next to it.
- **Controllers and zero commands.** Both controllers were active at the start and end of every
  observation. Nothing published on `/leg_trajectory_controller/joint_trajectory` and no goal
  status existed; the controller reference stayed constant with a tracking error of about 1e-11
  rad; the launch log contains no goal or trajectory line.

**Screenshots** (actual GUI captures, per run in `captures/`): `screenshot_gui.png` (default
view), `screenshot_gui_move_to_spiderx.png`, `screenshot_gui_side.png` (body height) and, in runs 2
and 3, `screenshot_gui_foot_level.png` (eye 8 mm above the floor, 0.4 m to the side): the feet hang
visibly above the floor. No pixel measurement of the clearance is claimed (the GUI camera is not
calibrated); the clearance itself is the §3 analysis (≥ 15.9 mm for every reachable
configuration at 0.125 m).

## 5. Evidence and hashes

Each folder holds a README and a `SHA256SUMS` over every file (`sha256sum -c SHA256SUMS`):

| Folder | Content | SHA-256 of its `SHA256SUMS` |
|---|---|---|
| [`evidence/m61a_cloud/incident_20261009T075335Z/`](evidence/m61a_cloud/incident_20261009T075335Z/README.md) | The original failure, verbatim (80 files) | `0cfeda98d70d42ac192ead2098dc345c0c4e9508d86c6611fa3425aa719e4a45` |
| [`evidence/m61a_cloud/m6d_diagnosis/`](evidence/m61a_cloud/m6d_diagnosis/README.md) | Diagnosis runs, experiments and A/B (170 files) | `53bf477f3125dd6a95c929324a4ec70a2712b3d5e008761d7de61450e4a61992` |
| [`evidence/m61a_cloud/verify_7f4c30f/`](evidence/m61a_cloud/verify_7f4c30f/README.md) | Build, focused and full tests, static checks (96 files) | `6c0345003b19076d47f27261d4267d7ae1c24bfedfd531f3bc71e616a4b7531f` |
| [`evidence/m61a_cloud/observation_7f4c30f/`](evidence/m61a_cloud/observation_7f4c30f/README.md) | Three runs, two non-counted attempts, tools (190 files) | `4226c0d88525f6061ef7c51673857fc9bf33c0b907973a0005d914980c479e18` |
| [`evidence/m61a_cloud/closeout/`](evidence/m61a_cloud/closeout/README.md) | Requirement-by-run table, scan masks, frozen harness (6 files; derived, read-only on the observation) | `83eaa442c6776e1db87882d287f1a98c6926c924601cac1addf1432975949b61` |

The incident's original archive (outside the repository) has SHA-256
`1f6c9ae4aa9fe5e6492046ac02f09f838d8685c13d8cc03bb7ad1d367d99f85b`.

## 6. Reproducing

```bash
# build and tests (no competing CPU load)
rm -rf build install && colcon build --symlink-install && source install/setup.bash
colcon test --packages-select spiderx_controller spiderx_scripts && colcon test-result --verbose
cd src/spiderx_controller && python3 -m pytest test/test_m6d_isolated_success.py \
  --basetemp=$HOME/spiderx_evidence/m6d_$(date -u +%Y%m%dT%H%M%SZ) -rA   # outcome kept outside rotation

# diagnosis (from docs/evidence/m61a_cloud/m6d_diagnosis/; see its README)
./run_diag.sh LABEL original|repo BURNERS        # one observed run of the repository test
python3 stress_teardown.py original|repo 196 150 # TRACE_ORPHANS=1 prints each orphaned Task
python3 reorder_demo.py original|repo 195 out.json

# Cloud phase-1 observation, one run (from docs/evidence/m61a_cloud/observation_7f4c30f/tools/)
bash run_one.sh <run_dir>     # launch, wait, preflight, 120 s, captures, /scan, views, stop, analysis
python3 summarize_runs.py <observation_dir>
```

On the owner PC, the documented M6.1-A §10 phase-1 commands apply unchanged; the helper scripts
only automate them for an unattended VM (Xvfb display, process-group SIGINT instead of a
keyboard Ctrl+C).

## 7. Limitations and what remains

- **Cloud, not the owner PC.** No result here ran on the owner's Ubuntu 22.04 machine; software
  rendering gave a real-time factor of about 0.7. Phase 1 on the owner PC remains the reference.
- **Not proven:** the exact failed check of the original incident (§1.3); the most probable cause
  is fixed, and any recurrence now reports its failed check in the test output.
- **Gazebo startup race** (§4.1): 1 of 5 launches here; outside SpiderX. If it recurs, Ctrl+C and
  relaunch; the server-only `headless:=true` path avoids the handshake but needs EGL, which this
  conda build lacks.
- **Visual criterion** is qualitative; run 1 has no foot-level view.
- **Criterion 6, per-beam reading** fails systematically (§8.3); the owner decides which reading
  §10 intends.
- **Pose-stream liveness** is unresolved: nothing moved, so a stale but constant pose stream
  would have looked the same (§8.2). The phase-2 plan closes it.
- **Weld exactness.** A world weld cannot show attachment noise; the attachment tolerances still
  need the separately approved phase-2 cycle (motion) for evidence under load.
- **Not shown by any of this:** walking, contact, balance, odometry or hardware behaviour. No goal
  was sent; both gates stay `False`.

**Before a separately approved fixed-base cycle (M6.1 phase 2):**
- owner review of this Cloud phase-1 evidence (or the same observation on the owner PC);
- approval of the provisional §10 criteria as used here, including the criterion-6 reading;
- the separate enabling change, prepared only as an unapplied patch.

The source-verified procedure is [`M61_CLOUD_ONE_CYCLE_PLAN.md`](M61_CLOUD_ONE_CYCLE_PLAN.md).
None of it is done here.

## 8. Closeout of the Cloud observation (after `b910df1`)

The tables are generated from the preserved evidence by
[`evidence/m61a_cloud/closeout/closeout_tables.py`](evidence/m61a_cloud/closeout/README.md), which only
reads `observation_7f4c30f/` (its `SHA256SUMS` still verifies). Per-cell evidence paths are in
`closeout/requirements_by_run.json`.

### 8.1 Requirement by run (PASS / FAIL / UNMEASURED)

| ID | Criterion (M6.1-A §10 and supplementary) | Scope | run_01 | run_02 | run_03 | Evidence in `observation_7f4c30f/run_NN/` |
|---|---|---|---|---|---|---|
| 1 | Preflight READY | per run | PASS | PASS | PASS | `observer/<UTC>/observation.json` (preflight), `preflight.txt` |
| 1b | 120 s observation READY at its end | per run | PASS | PASS | PASS | `observer/<UTC>/observation.json` (observe), `observe.txt` |
| 2a | Body z mean within 1 mm of 0.125 m; z range ≤ 1 mm | per run | PASS (0.125; 0) | PASS | PASS | observation `statistics.body` |
| 2b | Attachment ≤ 1 mm and ≤ 0.0033 rad | per run | PASS (0; 0) | PASS | PASS | `statistics.attachment` |
| 2c/2d/2e | Spawn ≤ 1 mm; link ≤ 0.1 mm; tilt ≤ 0.0033 rad | per run | PASS (0; 0; 0) | PASS | PASS | `statistics` |
| 3 | Pose receipt max gap < 1.0 s (p99.9 recorded) | per run | PASS (0.076; 0.053 s) | PASS (0.068; 0.052 s) | PASS (0.061; 0.054 s) | `statistics.pose_receipt` |
| 3 | Joint-state sim gap ≤ 0.25 s | per run | PASS (0.05 s) | PASS (0.02 s) | PASS (0.01 s) | `statistics.joint_state_sim_gap_max_s` |
| 4 | No unexpected sample codes; `frame_id` recorded | per run | PASS (none; `''`) | PASS | PASS | `statistics.sample_codes`, `header_frame_ids` |
| 5 | Visual: legs clear of the ground (GUI screenshot) | per campaign | **UNMEASURED** (inconclusive: no foot-level view) | PASS (qualitative) | PASS (qualitative) | `captures/screenshot_gui*.png` |
| 6a | `/scan` pose agreement, seam beams masked (gated) | per run | PASS (dx 0.38, dy 0.08 mm; 1σ 0.30 mm) | PASS (0.28, 0.03 mm) | PASS (0.17, 0.02 mm) | `captures/scan_check_seams_excluded.json` |
| 6b | `/scan` per-surface median ≤ ±8 mm, seam beams masked (gated) | per run | PASS (worst 2.4 mm) | PASS (1.6 mm) | PASS (1.9 mm) | same |
| 6c | Every individual range within ±8 mm (strict per-beam reading; reported) | per run | **FAIL** (85.8 %; max 17.4 mm) | **FAIL** (88.4 %; 18.0 mm) | **FAIL** (84.4 %; 18.9 mm) | `captures/capture.json` → `closeout/scan_masks.json` |
| 6-raw | Pose agreement with no seam mask (reported) | per run | **FAIL** (fit 0.40 mm, but 1σ 15.5 mm) | **FAIL** | **FAIL** | `captures/scan_check.json` |
| 7 | Ownership: action clients 0, command publishers 0 (CLI and observer, start and end) | per run | PASS | PASS | PASS | `captures/action_info.txt`, `command_topic_info.txt`, observation `graph` |
| 8 | Clean shutdown | per run | PASS (3 s) | PASS (3 s) | PASS (4 s) | `shutdown.txt` |
| M | Mount from the running model: weld world→`dummy_link` (0, 0, 0.125) in `/robot_description` and Gazebo | per run | PASS | PASS | PASS | `captures/capture.json`, `ign_link_dummy_link.txt`, `ign_model_pose.txt` |
| Z | Zero commands: no goal line, 0 command messages, 0 goal statuses, observer goals 0 / publishers 0 | per run | PASS | PASS | PASS | `launch.log`, `captures/capture.json`, observation |
| H1 | Supplementary: controller reference constant | per run | **UNMEASURED** (helper defect) | PASS (0.0 rad; 207 msgs) | PASS (0.0 rad; 290) | `captures/capture.json`, `analysis.json` |
| H2 | Supplementary: joint states unchanged | per run | PASS (4.8e-19 rad) | PASS | PASS | `captures/capture.json` |

**Note on rows 6a and 6b (added in §9.6).** The label "gated" on 6a and 6b was this closeout's
proposal and was never approved. The criterion of record is version 1, the "individual ranges"
wording, which is row 6c: it **failed** in every run. A versioned replacement is proposed in §9.6.

**Three complete passes?** §10 makes criteria 1–4, 7 and 8 per-run ("every preflight", "in each
120 s run"), and all three runs meet them. Criterion 5 asks for "a screenshot from the GUI", which
runs 2 and 3 supply. H1 and H2 are this session's supplementary checks. Criterion 6 is
discussed in §8.3; another run would not change 6c, because the error is systematic. **No further
observation was run.** The harness was frozen anyway (§8.7).

### 8.2 Physical-body / world-frame binding (with empty `frame_id`)

**What the stream contains.** Each `/spiderx/sim/world_poses` message carries:
- an entry with `child_frame_id` `spiderx`, the model;
- an entry with `child_frame_id` `dummy_link`.

Both have `header.frame_id` `''` and stamp 0 (the bridge does not fill them), in 532–562 captured
messages per run. `base_link` and `lidar_link` have **no** entries: sdformat's fixed-joint
reduction merges them into `dummy_link`. The binding therefore rests on the entry **names**, the
configured `model_name: spiderx` / `body_link: dummy_link` (`config/m61a_fixed_base.yaml`), and the
SceneBroadcaster semantics: a top-level model pose is given in the Gazebo world frame, and a link
pose relative to its model. M3 confirmed the link semantics at run time: Gazebo `lf_foot_1` versus
FK within 5.4e-11 m.

| Transform in `T_world_body = T_world_model · T_model_dummy · T_dummy_body` | Source | Status |
|---|---|---|
| `T_world_model` | Gazebo entry `spiderx` | **Measured** (simulator state): identity in every sample |
| `T_model_dummy` | Gazebo entry `dummy_link` | ~~Measured~~ **Not measured: corrected in §9.1.** Gazebo never writes a canonical link's pose; this is the SDF value of the URDF weld, (0, 0, 0.125), zero rotation in every sample |
| `T_dummy_body` (`base_link`) | URDF `dummy_joint`: fixed, no `<origin>`, so identity (`m61a_fixed_base.parse_robot_description`) | **Assumed** from the description; the conversion rule is checked offline (`test_m61a_fixed_base.py`); not reported by Gazebo |
| `T_body_lidar` (for `/scan` only) | URDF `lidar_joint` (0.051, −0.045, 0.141, yaw π/2) | **Assumed** from the description |

**Independent corroboration.** `/scan` is rendered from the sensor's actual pose in the simulated
world. Its fitted planar pose (x, y, yaw) of `T_world_body · T_body_lidar` agrees with the
weld-implied value to ≤ 0.4 mm and ≤ 0.11 mrad. That ties the planar body pose to the world
geometry independently of `frame_id` and of the pose stream. It checks the measured and the
assumed transforms **jointly**: two errors that cancel exactly are not excluded.

**Unresolved (marked explicitly):**
- **U-L1 Liveness of the pose stream.** Each entry had exactly one distinct value over each 10 s
  capture, and the observer's freshness proves only that messages arrive. A stale but constant
  stream would have looked identical in a motionless phase. Closing it needs motion: plan E9,
  leg-link entries versus FK during the cycle.
- **U-L2 Independent z, roll and pitch.** `pose/info` and `ign model` read the same simulator
  state, so they are not independent of each other. The horizontal scan cannot see z, roll or
  pitch, and the screenshots are qualitative. The body height of 0.125 m is therefore a
  **simulator-state measurement without an independent sensor confirmation**. It is acceptable
  only if the owner accepts the Gazebo state as ground truth for a welded body.

### 8.3 `/scan`: raw versus seam-filtered, masks, fitted pose versus individual ranges

**Frozen masks**, identical in all three runs (`closeout/scan_masks.json`; `closeout/frozen_harness.json`):

| Mask | Indices (lidar angle) | Justification |
|---|---|---|
| Cube-map seams | 45 (−135°), 46 (−134°), 135 (−45°), 136 (−44°), 225 (+45°), 226 (+46°), 315 (+135°), 316 (+136°) | Gazebo renders `gpu_lidar` through 90° camera faces. In every run beams 45, 135 and 225 returned the range of a neighbouring direction (errors −2.56, +2.68 and −2.38 m) while their neighbours were normal. The ±1° float rule also masks one neighbour on one side of each seam and the +135° seam (315), which never deviated; it is frozen as an explicit list |
| Edge grazing (at the weld pose) | 117, 118, 210, 222, 309, 334 | The expected range changes by more than 5 cm within ±0.5°, so a tiny angular offset switches surfaces |

| Result | run_01 | run_02 | run_03 |
|---|---|---|---|
| Raw fit (all 353 usable beams): dx, dy / RMS 1σ (x, y) / max residual | 0.40, 0.16 mm / 15.5, 15.9 mm / 2679 mm | 0.31, 0.11 mm / 15.5, 15.9 mm / 2681 mm | 0.19, 0.08 mm / 15.5, 15.9 mm / 2681 mm |
| Seam-filtered fit (346 beams): dx, dy, dyaw / robust 1σ | 0.38, 0.08 mm, 0.02 mrad / 0.30 mm | 0.28, 0.03 mm, 0.11 mrad / 0.25 mm | 0.17, 0.02 mm, 0.04 mrad / 0.29 mm |
| Individual ranges within ±8 mm (346 beams, both masks) | 85.8 % | 88.4 % | 84.4 % |
| Walls at 0–30° incidence: median / p95 / max \|r\| | 2.6 / 8.8 / 12.5 mm | 2.2 / 8.0 / 15.4 mm | 2.6 / 8.8 / 12.1 mm |
| Walls at 30–45° incidence: median / p95 / max \|r\| | 7.3 / 14.7 / 17.0 mm | 6.4 / 13.4 / 15.2 mm | 6.5 / 13.3 / 14.8 mm |

**How to read this:**
- **The fitted pose is one 3-parameter estimate from 346 beams.** It agrees with the weld to
  sub-millimetre (criterion 6a) and does not depend on any single beam.
- **Individual ranges carry errors that are systematic, not random.** A 40-scan mean should
  scatter by about 1.6 mm (σ = 10 mm per scan). The errors also grow with the incidence angle
  and repeat across runs. They are a property of the simulated `gpu_lidar` (depth sampling at
  oblique incidence), not of the pose.
- **Criterion 6's wording** ("the `/scan` ranges … agree … within the M0 ±8 mm") literally
  describes individual ranges. M0's ±8 mm came from spot checks at expected bearings. Under that
  literal reading criterion 6 is **FAIL**; as pose agreement it is **PASS**. The owner decides
  which reading applies before phase 2. *Superseded by §9.6: version 1, the literal reading, is
  the criterion of record and failed. "As pose agreement" is not a reading of it but a different
  metric, now proposed as version 2.*

### 8.4 Caveats that stay visible

- **Run 2:** an offline `/scan` re-analysis of run 1 ran during part of its observation, and
  `run_one.sh` was edited while it executed. Post-processing was regenerated from unchanged
  captures (`run_02/NOTES.txt`).
- **Run 1:** H1 not measured (helper defect); visual inconclusive (`run_01/NOTES.txt`).
- **Two unsuccessful launch attempts** (§4.1, preserved):
  - 09:02, an evidence-helper defect; the launch itself was fine;
  - 09:05, the Gazebo GUI→server startup race; no world was loaded.

### 8.5 Commits, tests and builds reconciled

| Commit | Branch | Content | Verified on it |
|---|---|---|---|
| `41fc10f` | M6.1-A | docs (previous head) | First Cloud attempt: 1465 tests, **2 failures** (the incident) |
| `7f91a18` | M6.1-A | evidence only | — |
| `7f4c30f` | M6.1-A | **test doubles only** (the only code change) | Clean build 08:52; focused 276 passed; full suite 1468 / 0 / 0 / 0; static checks; **all five launches (09:02–09:25) ran this build** |
| `b910df1` | M6.1-A | docs + evidence only (`git diff 7f4c30f b910df1 -- src/` is empty) | The `7f4c30f` results apply; rebuilt at about 09:42 for the closeout dry run and `--live` exit 3 |
| closeout commit | M6.1-A | docs, evidence, frozen harness, **unapplied** patch (`src/` unchanged) | dry run identity, `--live` exit 3, `git apply --check` only |
| `8561ff6` | M5.5 | merge of `b910df1` into `a808b94` | — |
| `22b8a01` | M5.5 | docs only | Clean build + full suite **1684 / 0 / 0 / 0** |
| `a826da6` | M5.5 | evidence only | — |
| closeout merge | M5.5 | merge of the closeout (no `src/` change) | Not re-tested (no code change) |

### 8.6 Diagnosis (unchanged)

Three test-double defects were demonstrated and corrected in `7f4c30f`:
- overlapping ticks that could reorder stamps;
- the executor teardown race;
- the stamp rounding.

The exact readiness failure of the original incident **remains unproven**. The most probable
cause is `joint_states_stale` through the overlapping ticks (§1.3).

### 8.7 Frozen harness (before any further observation)

[`closeout/frozen_harness.json`](evidence/m61a_cloud/closeout/frozen_harness.json) fixes, before
any further observation:
- the SHA-256 of every harness tool;
- the environment;
- the procedure;
- both scan masks as explicit index lists;
- the fit settings;
- the gated and reported definitions of every criterion (6a and 6b gated; 6c and 6-raw reported).

A future observation uses these exact tools and re-derives the masks with `closeout_tables.py`.
A different mask or threshold is a new, reviewed definition, never a retune from the same run.

### 8.8 Next step

ONE fixed-base M6.1 trot cycle in Cloud:
- the source-verified procedure: [`M61_CLOUD_ONE_CYCLE_PLAN.md`](M61_CLOUD_ONE_CYCLE_PLAN.md);
- the enabling change, unapplied, for review:
  [`patches/m61_enable_one_cycle.UNAPPLIED.patch`](patches/m61_enable_one_cycle.UNAPPLIED.patch).

Nothing is enabled or sent.

## 9. Remaining one-cycle blockers (after `a2c954f`)

What was asked: criterion 6 resolved honestly; pose and liveness established before dispatch;
gate-mode test coverage fixed. Commits:
- `f44c27c`: criterion-6 version-2 proposal, **frozen before** its validation run;
- `b982d8d`: the only code change (readiness correction and mode-explicit gate tests);
- this documentation and evidence commit.

Gates stayed `False` and no goal was sent. The two Cloud launches in this section were no-motion
launches:
- `run_04` (criterion-6 validation, frozen harness);
- `run_05L` (liveness and pause demonstration).

### 9.1 Pose: what Gazebo measures and what is inferred (corrects §8.2)

**Sources, read for this purpose.**
- **gz-sim 6.16.0, the installed version.**
  - `Physics.cc` `UpdateSim`: the body loop at line 2783 writes the pose of every link **except**
    the canonical link.
  - `UpdateModelPose` (line 2567): the model pose is set from the canonical link's physics pose,
    `X_WM = X_WL · X_ML⁻¹`.
  - `ChangedLinks` (line 2556): this happens only in a step in which that link's world pose
    changed (> 1e-6).
- **The fixed-base SDF** (`ign sdf -p`, sdformat 12) and the runtime entities (`ign model`).

| Factor in `T_world_body = T_world_model · T_model_dummy · T_dummy_body` | Simulator entity | Written by | Status |
|---|---|---|---|
| `T_world_model` | model `spiderx`, the `spiderx` entry of `pose/info` | Physics, from the canonical body's world pose, at each change (once at start for an intact weld) | **Measured**: the physics state of the merged body |
| `T_model_dummy` | canonical link `dummy_link` (first link; no `canonical_link` attribute), the `dummy_link` entry | SDF loader: `<pose relative_to='spiderx_fixed_base_weld'>0 0 0</pose>`, with the weld joint at `<pose relative_to='__model__'>0 0 0.125</pose>`. **Never written by Physics** | **Inferred**: the URDF weld origin through the converter. §8.2 called it "measured"; that was wrong |
| `T_dummy_body` | none: `base_link` is a `<frame>` attached to `dummy_joint` at identity (fixed-joint reduction) | URDF `dummy_joint` | **Inferred**; the same physics body, so it cannot move at run time |
| `T_body_lidar` | sensor `lidar` in `dummy_link` at (0.051, −0.045, 0.141, yaw 1.5708); `lidar_link` is a lumped frame | URDF `lidar_joint` | **Inferred**; same body |
| Weld | joint `spiderx_fixed_base_weld`, fixed, parent `world` (runtime `ign model -j`) | DART weld | **Structure confirmed at run time** |

**Consequence.**
- The composition equals `X_WL · T_dummy_body`: the **physics** pose of the merged body in all
  6 DOF, as of its last change.
- The check of the `dummy_link` entry against `/robot_description` is a **conversion and
  association check**, not a measurement.
- The attachment check compares the physics body with the weld assumption. It is not circular,
  but under an intact weld it can only confirm that DART holds the body where the SDF put it.
- Runtime evidence that this path writes z in this stack: M2 (free base, same model, canonical
  link and bridge), where the model entry followed the body to its 0.0545 m rest height.

### 9.2 z, roll and pitch: observability with the actual entities

| Channel | z | roll and pitch | Notes |
|---|---|---|---|
| `spiderx` model entry (physics) | ✅ | ✅ | Written on any change > 1e-6 of the canonical body. The only channel |
| `/scan` (fit) | ❌ | Only as `x + 0.141·pitch`, `y − 0.141·roll` (lidar lever arm) | A horizontal scan of vertical walls cannot separate tilt from translation, and assuming the body did not translate would use the weld to validate the weld |
| `/scan` (geometry) | Only within (−0.14, 0.26) m | Only below about 0.08 rad (0.055 rad towards a corner) | Scan plane at 0.266 m. Every beam still hits its wall (3.0–4.2 m away) below the 0.5 m top and above the ground; beams on box_b (0.4 m top) bound z from above |
| `/joint_states` effort | — | — | NaN for all 12 joints: `gz_ros2_control` does not report effort here |
| Other sensors | — | — | None on the model: no IMU, no camera |

**U-L2 stays UNRESOLVED.** No independent sensor in this model observes z, roll or pitch at the
1 mm / 0.0033 rad level. What is resolved is *what* the reported values are: physics outputs of
the merged body, not values inferred from the URDF. Closing U-L2 independently would need an
added sensor, for example an IMU on the body or a world-fixed range sensor viewing it. That is a
model change outside this task.

**Independence of `/scan` is limited too.** The `gpu_lidar` is rendered from the same model and
link pose components that `pose/info` publishes. `/scan` is independent of the SceneBroadcaster →
bridge path and of our composition, but not of Gazebo's entity-component state.

### 9.3 Freshness and advancing simulation time before dispatch (correction)

**Finding (gz-sim 6.16.0).**
- `SimulationRunner::Step` publishes the clock every iteration (lines 799, 453) and runs every
  system's `PostUpdate`, paused or not.
- `SceneBroadcaster::PostUpdate` publishes `pose/info` at up to 60 Hz (lines 345, 658).

So a paused world keeps `/clock` and `/spiderx/sim/world_poses` arriving, each with an unchanged
time. Only `/joint_states` stops, because the controller manager updates on sim-time progress.

**Gap before the correction.**
- M6.1 readiness judged the body pose by receipt age only (≤ 1.0 s).
- Sim progress was implied only by the M6.0-D joint-state check: ≥ 2 messages with strictly
  increasing stamps in a 2 s window. That is a different stream.
- The observer's `sim_clock` check counted the first `/clock` message as an advance. A world paused
  before a 4 s preflight therefore showed `sim_clock` OK; it was NOT READY only through the
  joint-state check.

**Correction (smallest change).**
- **`m61a_fixed_base.check_sim_progress`.** Over the last `progress_window_s` = 1.0 s of wall time,
  the `/clock` time at receipt of the usable body-pose samples must rise by at least
  `min_sim_advance_s` = 0.1 s. That is a real-time factor of at least 0.1; Cloud runs at about
  0.7. A step back by more than 1 ms, an unknown time, or fewer than two samples also fail. The
  code is `body_pose_sim_time_not_advancing`.
- **Where it applies.** It is applied in M6.1 readiness as a new layer `with_sim_progress`, before
  the confirmation and again before the send, in the live and mock paths. It is also applied in the
  observer.
- **`ClockMonitor` fix.** Only an actual advance counts. "Messages but no advance" is
  `sim_time_not_advancing`.
- **Tests.**
  - Unit: running, at the floor, paused, too slow, outside the window, one sample, unknown time,
    and backwards.
  - Tracker history; config refusal.
  - A paused world that is NOT READY although every stream is fresh: the observer CLI, the mock
    `sim_paused` scenario, and the enabled live path with fakes, where nothing is sent and the only
    code is the new one.

**In flight (unchanged, still required).**
- `/joint_states` staleness: 0.5 s wall, which is what stops on a pause.
- G5 sim stall: 5.0 s.
- G6 sample gap: 0.25 s sim.
- Body-pose freshness: 1.0 s.
- G8.

**What remains (U-L1, content liveness).** On a weld the model entry is written once, so before
motion no stream can show that the *content* is current. The demonstration in §9.5 shows:
- the gz-side `pose/info` header time advancing with the clock;
- the stream regenerated at 60 Hz;
- the readiness correction catching a pause.

Content liveness still needs motion: plan E9 (leg-link entries against FK during the cycle).

### 9.4 Gate-mode tests: both modes pass, meaningfully

**Before.**
- The patch header listed five tests expected to fail on an enabling branch.
- Applying the patch in a temporary worktree (tests only, no simulator) and running the whole
  controller suite found **three more** gate-dependent assertions (`enabled_mode_full_pytest.txt`):
  - `test_success_outcome_fields`;
  - `test_mock_cli_writes_the_evidence_directory`;
  - `test_dry_run_report_is_never_overwritten`.

  Each asserted that the recorded gate was `False`.
- One of the original five, `test_live_cli_with_gate_false_imports_no_ros_client`, would have run
  the **real** `--live` path on domain 0 in an enabled build.

**Changes.** Assertions were made mode-explicit; none was weakened.

| Test | Now |
|---|---|
| `test_m61_gate_matches_the_single_test_expectation` (was `…is_false_and_pinned`) | The committed gate `is` its pin (`test/m61_gate.py`); `LIVE_STATE` says HARD-DISABLED exactly when the gate is off, and ENABLED exactly when it is on; the M6.0-D gate is unchanged |
| `test_m61_gate_is_a_single_literal_equal_to_the_pin` (renamed) | Unchanged logic: one assignment, in `m61_live_contract.py`, equal to the pin |
| `test_no_environment_or_flag_can_enable_m61_dispatch` | Same eight files and forbidden words; only the exact pinned gate line is exempt, and it must occur exactly once |
| `test_the_enabler_scan_has_teeth_in_both_build_modes` (new, False and True) | The scan flags: a flipped, missing or duplicated gate line; an indented or `c61.`-prefixed assignment; and every forbidden word injected into each file |
| `test_live_cli_refused_exit_3_before_anything`, `test_gate_false_blocks_the_live_wiring_too` | Fixture `disabled`: the gate is set False explicitly |
| `test_live_cli_with_gate_false_imports_no_ros_client` | The child process sets the gate False before `main`, so it can never reach a ROS graph in either build; it also checks `REFUSED` and `HARD-DISABLED` |
| Three recorded-gate tests | Assert the recorded value `is` the pin |
| `test_evidence_records_the_gate_as_it_is_in_both_modes` (new, False and True) | The dry-run report, `_gate_state()` and `as_dict()` record the gate as it is |

Enabled-path tests use `M61FakeTransport` and fake graph observations only. This now includes the
paused-world refusal.

**Results.**

| Build | Scope | Result |
|---|---|---|
| Disabled (committed) | `test_m61_gait_replay.py`, `test_m61a_fixed_base.py`, `test_m61a_observer.py` | 266 passed |
| Enabled (patch applied, temporary worktree) | The same three files | 266 passed |
| Enabled (patch applied, temporary worktree) | Whole `spiderx_controller` suite | **1457 passed, 0 failed** |
| Disabled | Clean build + full `colcon test` | see §9.7 |

The worktree was removed afterwards; no enabling branch exists. The patch header now states:
expected test delta **none**.


### 9.5 Live demonstration in the simulator (`run_05L`, no motion)

Evidence: [`evidence/m61a_cloud/liveness_b982d8d/`](evidence/m61a_cloud/liveness_b982d8d/README.md).
It was one launch at `b982d8d`, with the world paused and resumed through Gazebo's
`WorldControl` service.

| | A running | B paused | C resumed |
|---|---|---|---|
| gz `pose/info` header time (captures 1 s apart) | 5.282 → 6.239 s | **14.666 → 14.666 s** | 16.933 → 17.771 s |
| ROS `/clock` over 5 s | 490 Hz, +2.54 s | **651 Hz, +0.000 s** | 675 Hz, +3.61 s |
| ROS pose stream over 5 s (node clock at receipt) | 43 Hz, 8.445 → 10.894 s | **52 Hz, 14.666 → 14.666 s** | 57 Hz, 19.079 → 22.682 s |
| ROS `/joint_states` over 5 s | 52 Hz | **0** | 74 Hz |
| Observer `--preflight` (corrected) | READY: `body_pose_sim_progress` 0.659 s in 1.0 s (56 samples) | **NOT READY**: `body_pose_sim_time_not_advancing`, `sim_time_not_advancing`, `joint_states_missing`, while **`body_pose_fresh` passed** (age 0.002 s) | READY: 0.668 s (57 samples) |

In `run_04` the corrected preflight was READY with 0.603 s of sim advance in the last 1.0 s (55
samples).

**Before dispatch, the following are demonstrated:**
- stream freshness (receipt);
- advancing simulation time on the dispatching side's clock, while the pose stream is received;
- the gz-side regeneration of `pose/info` at the current sim time.

**During execution:** the unchanged monitors catch a pause within 0.5 s (`/joint_states`
staleness).

### 9.6 Criterion 6: decision

Proposal and derivation: [`M61A_CRITERION6_V2_PROPOSAL.md`](M61A_CRITERION6_V2_PROPOSAL.md)
(frozen at `f44c27c`).

**1. Original wording (version 1, criterion of record).**
- Text: "the `/scan` ranges to the world walls agree with the welded pose … to within the M0
  ±8 mm."
- Measurement: individual ranges, with M0's tolerance from spot checks of individual distances.
- Objective: the pose.

**2. Version-1 results.**
- Runs 1–3: retained, **FAIL**.
- `run_04`: **FAIL** — 53.1 % of 13,840 single-scan ranges and 85.5 % of 346 40-scan means within
  ±8 mm (max 18.8 mm).
- The best that **any** planar pose can reach is a maximum |residual| of 17.7 mm, against
  16.7–17.1 mm in runs 1–3.

**3. Why a revision is justified.** The reason is not that 6a passes:
- the gpu_lidar's noise (10 mm per scan);
- its systematic, launch-to-launch identical rendering error of about 5 mm RMS;
- together these make version 1 unsatisfiable by any pose. It measures the sensor, not the pose.

**4. Version 2** (derived from the G8 objective, the sensor model and the uncertainty; thresholds
and beam exclusions frozen before `run_04`):
- the fitted body planar pose compared with the `pose/info` body;
- τ 1.946 mm / 6.103 mrad;
- σ_max 0.641 mm / 2.369 mrad;
- prerequisites, failing which the result is INCONCLUSIVE.

**5. Validation on ONE fresh no-motion observation, `run_04`**
([`evidence/m61a_cloud/observation_b982d8d/`](evidence/m61a_cloud/observation_b982d8d/README.md)).

| Item | Result |
|---|---|
| Frozen harness | Tools byte-identical (checked against `frozen_harness.json`); `crit6_v2` files unchanged since `f44c27c` (`sha256sum -c`), applied once |
| Reference (`pose/info` body, 430 samples) | (0, 0, 0.125, 0, 0, 0), spread 0 |
| Fit − reported | dx 0.483 mm, dy 0.046 mm, dyaw 0.117 mrad: **\|dxy\| 0.49 mm ≤ 1.946**, **\|dyaw\| 0.117 mrad ≤ 6.103** |
| Uncertainty | σ_robust 0.267 / 0.276 mm, 0.134 mrad; σ_jackknife 0.578 / 0.356 mm, 0.260 mrad; **σ_used 0.578 mm ≤ 0.641**, 0.260 mrad ≤ 2.369 |
| Prerequisites | 40 scans; 346/346 beams; walls 70/75/70/82 beams; `pose/info` constant: all met |
| **Version 2** | **PASS** |
| Counterfactual (the measured error pattern on a displaced body) | (+3, 0) mm → \|dxy\| 3.49 mm FAIL; (0, +3) mm → 3.09 FAIL; ±0.01 rad → 10.11 / −9.99 mrad FAIL; (+1, 0) mm → 1.48 PASS; +0.002 rad → 2.12 mrad PASS. Recovery errors ≤ 0.11 mm / 0.10 mrad. The diagonal (−2.12, +2.12) mm case FAILs (2.72 mm); the tool lists it as "recovery only" because the frozen constant puts it 2e-11 m below δ. Recorded, not changed |

**6. Decision.**
- Version 1 is kept as the criterion of record, and **phase-1 criterion 6 is FAILED under it**
  (runs 1–3 and `run_04`).
- Version 2 is a **proposal validated once**: PASS with an xy resolving power at 90 % of its
  limit.
- Accepting version 2 for phase 1 is the owner's decision. Nothing is relabelled.
- The frozen harness's "GATED" labels for 6a/6b (`frozen_harness.json`) were a closeout proposal.
  Version 2 replaces them; it does not reuse their 8 mm bound.

**`run_04` and the other phase-1 criteria** (`observation_b982d8d/run_04_requirements.md`), all
**PASS**:
- 1, 1b, 2a–2e, 4, 7, 8 (clean, 5 s);
- 3: the pose receipt gap max is **0.340 s**, p99.9 0.058 s. That is below the 1.0 s limit but
  larger than runs 1–3 (≤ 0.076 s). During the 120 s observation this session read the preflight
  JSON and wrote one README (light I/O), recorded as a caveat. The joint-state sim gap max is
  0.02 s;
- M, Z, Z2, Z3;
- 5 (foot-level screenshot: feet clear of the floor, qualitative).

### 9.7 Tests, builds and commits

| What | Where | Result |
|---|---|---|
| M6.1 / M6.1-A test files (3), disabled | `b982d8d` working tree | 266 passed |
| Same, enabled (patch applied, temporary worktree) | `b982d8d` + patch | 266 passed |
| Whole `spiderx_controller` suite, enabled | `b982d8d` + patch | **1457 passed, 0 failed** (an earlier run: 3 failed, see §9.4) |
| Clean build + full `colcon test`, disabled | `b982d8d` | **1497 tests, 0 errors, 0 failures, 0 skipped** |
| `crit6_v2` synthetic self-test | `f44c27c` | 5/5 checks pass |
| Cloud simulation | `b982d8d` | `run_04` (frozen harness) and `run_05L`: no motion, clean shutdowns |

Gates: `M61_LIVE_DISPATCH_ENABLED = False`, `LIVE_DISPATCH_ENABLED = False`. The patch is
unapplied on every branch, and `git apply --check` passes.

### 9.8 What still blocks the one cycle

1. **Criterion 6.**
   - Under version 1, phase 1 is FAILED.
   - Under version 2, validated once, it passes. Only the owner can accept version 2.
2. **U-L2, z / roll / pitch.**
   - There is no independent confirmation; they are the physics state of the merged body.
   - The owner either accepts this or requires an added sensor (a model change).
3. **U-L1, content liveness.** It closes only in motion (plan E9). Freshness and advancing sim
   time before dispatch are demonstrated (§9.3, §9.5).
4. **Approvals:** the phase-2 criteria E1–E11 and the enabling patch (2 lines; no expected test
   failures).
5. **Owner PC.** Phase 1 has not run there.

Superseded by §10.7.

## 10. One-cycle prerequisites closed in Cloud (after `76a868b`)

**Commits** (each pushed before the run that uses it):
- `927acb2`: `m61a_link_check`, the E8 and E9 analysis, frozen. Its SHA-256 is
  `649b0c4fba9ad84cb4fca9157474ed990163dc759d8d78ef83011d4cf7059e31`;
- `1732c32`: the E5 check of the streams at the send, and the E11 restore-and-verify script;
- the documentation and evidence commit that follows.

**Launches.** Two Cloud launches, both no motion:
- `run_06`: the E8 provenance recording;
- `run_07`: the E11 validation, in which the script stopped the running launch itself.

No goal was sent. Both gates stayed `False`, except in one temporary, detached test worktree
(§10.5), which was removed.

### 10.1 Criterion 6: what 53.1 % and 85.5 % count, and a recommendation

Both figures come from `run_04` (`observation_b982d8d/run_04/criterion6_v2.json`). Each
compares `/scan` ranges with those ray-cast from the welded pose, at ±8 mm.

| Population | n | Aggregation | Within ±8 mm | Max \|error\| |
|---|---|---|---|---|
| Single-scan ranges | 13,840 = 40 scans × 346 beams | none | **53.1 %** (7,346) | 48.0 mm |
| Per-beam means | 346 beams | mean of each beam over the 40 scans | **85.5 %** (296) | 18.8 mm |
| Best planar body pose (minimax over x, y, yaw) | — | — | — | 17.7 mm |

**Why they differ.**
- **Single ranges.** Each one carries the sensor noise (σ 10 mm) plus a per-beam systematic
  error: rendering, about 4–5 mm RMS, and 4.0 mm robust σ after the fit. With a total σ of about
  11 mm, a Gaussian puts 53 % within ±8 mm, as observed.
- **Per-beam means.** Averaging 40 scans cuts the noise to 1.6 mm but leaves the systematic part.
  A few beams near edges carry up to 18.8 mm.
- **Neither is a pose result.** No pose satisfies version 1: the best still leaves 17.7 mm.
  Version 1 stays **FAIL** on record.

**Recommendation: adopt version 2** as a separately versioned planar-pose check. Version 2 tests
what version 1 meant: the scan-fitted body (x, y, yaw) against `pose/info`, with δ = G8, a 1 %
false-fail rate and a 5 % miss rate. **Its limitations:**
1. **Planar only.** z, roll and pitch are not tested; they rest on E8 (§10.2).
2. **One validation run** (`run_04`).
3. **Not independent of Gazebo's entity state.** `/scan` is rendered from it, so version 2 checks
   the reporting and composition chain, not the physics.
4. **Static only.** It needs 40 scans of a stationary body.
5. **Little margin.** σ_used was 0.578 mm against σ_max 0.641 mm (90 %). A noisier run can be
   INCONCLUSIVE.
6. **Small offsets pass by design.** Offsets well below 3 mm / 0.01 rad pass; the miss rate at δ
   is 5 %.

### 10.2 E8: the pose source, traced to DART and shown at run time

**Source** (installed versions, read for this):
- **Re-report rule.** gz-physics 5.3.2 dartsim, `SimulationFeatures::Write` (lines 82–118):
  - after each step it reports every link whose DART world pose differs from its **last reported**
    pose by more than 1e-6 in any position axis or quaternion component;
  - every link is reported on the first step;
  - so an unreported change never accumulates beyond that bound.
- **Model pose.** gz-sim 6.16.0, `Physics.cc` `UpdateModelPose` (lines 2567–2679):
  - X_WM = X_WL · X_ML⁻¹, with X_WL from DART;
  - it is cached in `modelWorldPoses` (line 2612) and written to the model's pose (line 2625);
  - no other code fills that cache.
- **Other model-pose write.** Lines 2111–2121 apply to static models on a pose command only.
- **Non-canonical links** (lines 2785–2802):
  - X_M,link = `modelWorldPoses`⁻¹ · X_W,link, from DART;
  - without a cached model pose the link is skipped, with `Internal error: parent model […] does
    not have a world pose available`;
  - the canonical link's own pose (X_ML) is never written.
- **Composition.**
  - The `spiderx` entry times the `dummy_link` entry equals X_WL(DART) exactly. The `dummy_link`
    entry is the same X_ML that Physics divided by.
  - Then T_dummy_body: `base_link` is a URDF frame at identity inside the same rigid body.
  - **Physics-derived:** the model entry.
  - **Fixed transforms:** X_ML from the SDF, which cancels, and T_dummy_body from the URDF.
  - The weld origin is used only as the specification compared against.

**Measurement** (`run_06`, `927acb2`; the analysis was frozen and pushed before the launch).
Evidence: [`evidence/m61a_cloud/linkcheck_927acb2/`](evidence/m61a_cloud/linkcheck_927acb2/README.md).
The 15 s rosbag2 recording held 678 pose messages (all 68 entries), 869 joint states and 10,181
clock messages.

| Check | Result |
|---|---|
| Body pose: composition against the weld origin (limits 1 mm, 3.33 mrad, \|dz\| 1 mm) | **PASS**, deviation 0 (exact) in 678 of 678 messages; the model entry stayed the identity |
| Provenance: 12 leg-link entries relative to `dummy_link`, against URDF FK at the measured joints and against FK(0) | **PASS** in 678 of 678. Residual against FK(q): ≤ 7.6e-8 m, 7.5e-7 rad (limits 1e-5). FK(q) and FK(0) differ by up to 1.07e-4 rad. Entries differ from FK(0) by ≥ 9.6e-5 rad |
| Server log | No `Internal error` and no `does not have a world pose` line |
| Joints | Held at 4.4e-5 to 9.7e-5 rad (gravity sag), constant to about 1e-19 rad: the values of `run_04` |

**What this shows.**
- The leg-link entries are DART's poses relative to the body at the measured joints. They are not
  the SDF initial values, so Physics wrote them in this launch.
- By the source, that needed `UpdateModelPose` to write the model entry from DART first. So the
  composed body pose is DART's pose of the merged body, within 1e-6 per axis and reported on
  every change.
- E8's comparison with the weld origin is therefore a measurement compared with a specification.
  The weld is not assumed.
- A separate sensor is not needed for this in simulation.

**Unproven.**
1. **DART is trusted** as the ground truth (WeldJoint rigidity, the integrator). That is the
   trust boundary.
2. **The canonical re-report after the first write** was exercised at run time only in the
   free-base M2 launches: the model entry followed the body to 0.0545 m. A welded body never
   moves.
3. **The first write is shown only indirectly,** through the leg entries and the source, because
   its value equals the SDF default.
4. **Motion is not covered.** That is E9 (§10.3).

**U-L2.** It is resolved *if* the owner accepts the simulator physics state as ground truth, which
is recommended.

### 10.3 E9 corrected, and its analysis frozen

**Correction.** The leg-link entries are expressed relative to the body. Matching them to
joint-state FK while they change shows **dynamic link/kinematic consistency**: the stream's content
is live and the URDF chain matches DART's. It is **not** an independent measurement of global body
height or tilt. A synthetic test shows the point: a body displaced by 5 cm and tilted 0.1 rad
passes E9, while E8 fails.

**Frozen** in `927acb2`, before any motion run, as module constants pinned by a test:
- **Alignment.** Each pose message gets the latest `/clock` received before it.
- **Match.** It must match FK(q(t')) for some |t' − t| ≤ 25 ms (1 ms grid) within 1 mm / 5 mrad
  on all 12 links.
- **Coverage.** At least 100 samples, and no gap over 1.0 s.
- **Motion.** Every foot link turns ≥ 0.02 rad relative to the body. Otherwise the verdict is
  `NO_MOTION`: E9 not satisfied.
- **Counterfactual.** The same stream frozen at its first sample must fail.

The earlier wording (5 mm / 0.05 rad) was too loose for a cycle that moves joints by at most
0.11 rad: a stale stream would have passed for much of the cycle.

**No-motion check.** On `run_06` the verdict is `NO_MOTION`, with 0 unexplained samples.

### 10.4 E5: the streams at the send

**The gap.**
- The M6.0-D check at the send (D3) bounds only the *age* of the readiness result: 10 s.
- Nothing spins during the confirmation prompt or the server wait (rclpy `wait_for_server` does
  not spin).
- The in-flight monitors start at acceptance.
- So nothing showed that the joint states, the body pose and the clock were current at the send.

**Change** (`1732c32`; the M6.0-D modules are unchanged).
- `M61Session._fresh_at_send` runs D3, then `streams_check`.
- `M61AFixedBaseTransport.streams_now()`:
  1. drains what was queued;
  2. spins for one progress window (1.0 s);
  3. returns what arrived after the drain.
- `m61a_fixed_base.streams_at_send` then applies the readiness thresholds:
  - joint states received in the window, ≤ 0.5 s old and complete;
  - a usable body pose received in the window, ≤ 1.0 s old;
  - sim time advancing ≥ 0.1 s over the window;
  - the plant and attachment checks.
- **On failure.** The code becomes the refusal reason and is recorded in
  `freshness_at_send.streams_now`. Nothing is sent and there is no retry.

**Found while testing (isolated domain).** The first version had no drain. After the publishers
stopped, the transport processed the queued messages (up to each subscription's depth) only at its
next spin. They were stamped with the processing time, so a joint state looked 0.25 s old when the
stream had stopped 1.45 s earlier. With the drain and the window, the same test reports
`joint_states_stale`, `body_pose_stale` and `body_pose_sim_time_not_advancing`.

The M6.0-D joint-state readiness window could in principle be satisfied by such a backlog after a
long prompt. The M6.1 check now covers the send regardless.

**Tests.**
- Unit tests of `streams_at_send`, the window included.
- Four mock scenarios:
  - `slow_server`: sent;
  - `joint_states_stop_before_send`, `body_pose_stops_before_send`, `sim_pauses_before_send`:
    refused at the send, with readiness READY twice and D3 permitting.
- The enabled live path with fakes.
- The transport on an isolated domain.
- The mock re-anchors its result time at the actual send, so a slow server does not shorten the
  goal.

### 10.5 E11: restore and verify, whatever happened

`scripts/m61_restore_and_verify_disabled.sh` (`1732c32`) runs these steps:
1. **Stop.** SIGINT to the launch group, SIGTERM to Xvfb, stop the CLI daemon. Then no simulation
   process may remain; the script's own process chain is excluded.
2. **Preserve, before any checkout or deletion.** The enabling commit's id, `show`,
   `format-patch` and the SHA-256 of its `src/` diff. Uncommitted `src/` changes are saved, and
   discarded only on request.
3. **Restore.** Check out the base, and require `src/` to equal it (no diff, no untracked file).
4. **Rebuild** cleanly.
5. **Verify, in a fresh `env -i` environment run from `/`:**
   - the imported `m61_live_contract` and `m6_live_contract` come from this workspace and are both
     `False`;
   - `ros2 pkg prefix` points to this workspace;
   - **only then**, `--live` exits 3 with HARD-DISABLED;
   - the operator's shell imports nothing enabled.
6. **Delete the enabling branch** only on request, and only after 1–5 pass.

**Found while testing.**
- **The current directory.** Python resolves a package from the current directory first. Run from
  `src/spiderx_controller`, the check imported the source tree instead of the install. The checks
  now run from `/`.
- **Self-matching.** The leftover check no longer matches its own invoking shell; earlier stop
  scripts did.

**Hermetic tests** (8: a fake workspace and a fake `ros2`, nothing real touched) cover:
- the disabled case;
- an enabled checkout, where the live path is never reached;
- a full restore, checking that preservation comes before restore and deletion comes after
  verification;
- a broken install, where the branch is kept;
- a shadowing calling environment;
- a leftover process with a pattern-bearing invoking shell;
- an interrupted apply;
- usage errors.

**Cloud validation** ([`evidence/m61a_cloud/restore_1732c32/`](evidence/m61a_cloud/restore_1732c32/README.md)):

| Case | Result |
|---|---|
| A. `run_07`: a running no-motion launch, then the script with the launch and Xvfb pid files | Launch group stopped by SIGINT, Xvfb stopped, no leftover; `src/` = base `1732c32`; clean rebuild (8 packages); imported gates `False` from this workspace; `--live` exit 3, HARD-DISABLED; **`DISABLED AND VERIFIED`, exit 0** |
| B. Temporary detached worktree at `1732c32` with the patch applied and built (tests only, no simulator) | `src/` differs from the base (2 files); imported `M61_LIVE_DISPATCH_ENABLED = True`; **`--live` NOT run**; **`NOT VERIFIED`, exit 1**. No `log/m61_run` directory exists in that worktree, so the live path never started. The worktree was removed afterwards |

### 10.6 Tests, builds and commits

| What | Where | Result |
|---|---|---|
| `m61a_link_check` tests (synthetic Physics-rule streams, rosbag2 round trip) | `927acb2` | 30 passed; 4 deliberate mutants each caught |
| M6.1 / M6.1-A focused files (fixed base, gait replay, observer, link check), disabled | `1732c32` | 318 passed |
| Restore-script tests | `1732c32` | 8 passed |
| Whole `spiderx_controller` suite, **enabled** (patch applied, temporary worktree, tests only) | `1732c32` + patch | **1517 passed, 0 failed** (7 min 49 s) |
| Clean build + full `colcon test`, disabled | `1732c32` | **1559 tests, 0 errors, 0 failures, 0 skipped** (+60 new tests, +2 new test files). A first attempt without `PYTEST_DISABLE_PLUGIN_AUTOLOAD=1` ran no test: every file stopped at pytest start-up on the `launch_testing` plugin incompatibility; recorded in `restore_1732c32/disabled_mode/` |
| M5.5 after merging these commits | M5.5 branch | Recorded there (`docs/M55_KEYBOARD_WALKING.md` §14) and in robovision2210/spiderx_ws#17 |
| Cloud simulation | `927acb2`, `1732c32` | `run_06`, `run_07`: no motion, clean shutdowns |

Gates: `M61_LIVE_DISPATCH_ENABLED = False` and `LIVE_DISPATCH_ENABLED = False` on every
branch. The patch is unapplied, and `git apply --check` passes.

### 10.7 What still blocks the one cycle

1. **Owner: criterion 6.** Adopt version 2 as a versioned planar check (recommended, with the
   limitations of §10.1). Version 1 stays FAIL.
2. **Owner: z, roll and pitch.** Accept the simulator physics state as ground truth (recommended;
   §10.2). The alternative is an added sensor, which is a model change.
3. **Approvals.**
   - The revised E1–E11 (E5, E8, E9 and E11 changed).
   - The enabling patch: 2 lines, header updated to point to the restore script.
   - One Cloud run, not on the owner PC.
4. **E9 needs the motion run itself.** It cannot be shown beforehand.
5. **Owner PC.** Phase 1 has not run there.
