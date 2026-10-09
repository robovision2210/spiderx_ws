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
- **Criterion 6 (`/scan`):** passes as **pose agreement** (fitted pose within 0.4 mm). Under the
  strict per-beam reading of M0 it **fails**: 84–88 % of individual ranges are within ±8 mm,
  systematically, with a maximum of 18 mm (§8.3). This is an owner decision.
- **Pose-stream liveness:** UNRESOLVED (§8.2).
- **Run 1's supplementary controller-hold check:** UNMEASURED.

Two earlier launch attempts did not become observation runs (§4.1). Owner-PC verification is still
outstanding.

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
| `T_model_dummy` | Gazebo entry `dummy_link` | **Measured**: (0, 0, 0.125), zero rotation in every sample |
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
  which reading applies before phase 2.

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
