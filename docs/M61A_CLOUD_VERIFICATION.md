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

**Outcome (Cloud simulation, `7f4c30f`): M6.1-A phase-1 launch-only observation PASSED in three
separate launches** against the §10 provisional criteria of
[`M61A_FIXED_BASE_IMPLEMENTATION.md`](M61A_FIXED_BASE_IMPLEMENTATION.md), with two recorded
qualifications: the visual criterion is qualitative (run 1 inconclusive, runs 2–3 legs clear), and
the controller-reference check was not measured in run 1. Two earlier launch attempts did not
become observation runs and are reported in §4.1. Owner-PC verification is still outstanding.

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
  1σ ≤ 0.31 mm), and every surface's median residual is within 2.4 mm: the scan agrees with the
  welded pose well inside ±8 mm. The raw all-beam result is preserved next to it.
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
- **Weld exactness.** A world weld cannot show attachment noise; the attachment tolerances still
  need the separately approved phase-2 cycle (motion) for evidence under load.
- **Not shown by any of this:** walking, contact, balance, odometry or hardware behaviour. No goal
  was sent; both gates stay `False`.

**Before a separately approved fixed-base cycle (M6.1 phase 2):** owner review of this Cloud
phase-1 evidence (or the same observation on the owner PC), approval of the provisional §10
criteria as used here, and the separate enabling change described in M6.1. None is done here.
