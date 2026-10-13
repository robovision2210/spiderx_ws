# M6.1-A local verification and launch-only observation: consolidated report

**Outcome: launch-only observation NOT PERFORMED and NOT PASSED.** One required test failed in the local test suite. Per the instruction "Stop before simulator observation if a required check fails. Preserve the failure evidence", no simulator was launched. `run_01/`, `run_02/` and `run_03/` are intentionally empty.

## 1. Revision

| Item | Value |
|---|---|
| Workspace | `/home/user/spiderx_ws` |
| Branch | `claude/stoic-shannon-ur2mes` (remote equal; fetched, nothing to fast-forward) |
| HEAD | `41fc10f038c52ebe54b1c388aa916ecd05e801a4` |
| Ancestry | `9d24b1e` (the Cloud-tested commit) is an ancestor. `9d24b1e..HEAD` touches only `docs/evidence/README.md`, `docs/evidence/m61a/README.md` and `docs/evidence/m61a/clearance.json`. |
| Tracked tree | Clean before and after (`git status --porcelain` shows only the untracked `spiderx_evidence/`) |
| Gates | `m6_live_contract.LIVE_DISPATCH_ENABLED = False` and `m61_live_contract.M61_LIVE_DISPATCH_ENABLED = False`. The M5.5 gate is not on this branch. Not changed. |
| PR | #17 is draft and unmerged. Not touched, not monitored. |

## 2. Environment and differences from the owner PC

- **Machine:** a Cloud Firecracker VM, not the owner's Ubuntu 22.04 PC. The container runs Ubuntu 24.04.4 with 4 CPUs, as root.
- **ROS and simulator:** ROS 2 Humble from RoboStack (`/opt/mm/activate.sh`), with Python 3.11.16 and numpy 1.26.4. libignition-gazebo6 6.16.0, sdformat 12.8.0, ros_gz 0.244.20.
- **Build:** setuptools 59.6 from a private site directory, used only for the build. The shared environment keeps setuptools 84.
- **Tests:** run with `PYTEST_DISABLE_PLUGIN_AUTOLOAD=1`.
- **Display:** none. The planned GUI launch would have used `Xvfb :99` with Mesa software rendering.
- **Planned observation environment:** identical in every shell, set by `tools/env.sh`:
  - `ROS_DOMAIN_ID=0` and `ROS_LOCALHOST_ONLY=1`;
  - the default RMW, with no discovery server.
- **Graph:** no external ROS graph was reachable or contacted.

## 3. Build and tests (run once)

| Check | Result |
|---|---|
| `colcon build` (clean `build/` and `install/`; `log/` preserved) | Exit 0. 8 packages in 31.1 s. `build_test/build.log` |
| `colcon test` + `colcon test-result` | **1465 tests, 0 errors, 2 failures, 0 skipped.** `colcon test` took 6 min 17 s (07:54:47–08:01:04 UTC). `build_test/test.log` |

The total of 1465 is 1427 pytest cases plus 38 CTest wrappers; it matches the Cloud count at `9d24b1e`.

**The 2 failures are one test counted twice:** the pytest case and its CTest wrapper.

- **Test:** `src/spiderx_controller/test/test_m6d_isolated_success.py::test_gate_enabled_main_succeeds_end_to_end_with_one_goal`.
- **What it does:** it is unchanged M6.0-D code. It enables the gate inside the test only (monkeypatch), against an in-process fake controller stack. That stack runs on the isolated, localhost-only domain 150 + pid % 50, which was 160 here.
- **Observed:**
  - `rc == 2`, outcome `('REFUSED', 'readiness_not_ready', [])`.
  - stdout shows that readiness #1 passed, the confirmation was read, and the run then ended in "final state REFUSED, reason readiness_not_ready, goals sent 0, cancels sent 0".
  - The re-observation before dispatch (`m6_live_readiness.dispatch_permitted`) returned a fresh, compatible/warning report with `ready = False`.
  - The log also contains `The following exception was never retrieved: cannot use Destroyable because destruction was requested`.
- **Fail-safe behaviour held:** no goal was sent, even to the fake stack.
- **Root cause: unconfirmed.** The outcome JSON named the failing readiness code. It was written to `/tmp/pytest-of-root/pytest-599/...` and deleted by pytest's keep-three temporary-directory rotation before it could be copied (now `pytest-602..604`). **This is an evidence gap.**
  - Candidate causes from code inspection, all timing-sensitive in `m6_live_preflight`, none verified:
    - `joint_states_no_messages` (fewer than the minimum messages in the 2 s window);
    - `joint_states_stale`;
    - `controller_manager_unavailable`.
  - **A known confounder:** while the suite ran, I was running CPU-heavy scan-fit self-tests in parallel on the same 4-CPU VM.
  - The same test passed in every earlier Cloud run this session (1330, 1448, 1610, 1465 and 1681 test totals).
- **No retry was made**, as instructed.

**Preserved:**
- `build_test/failure/`: the failing xunit file, CTest `Test.xml`, and the colcon test log with the per-package stdout/stderr;
- `build_test/all_xunit/`: all 38 xunit files.

## 4. Static and configuration checks (run once, offline, no simulator)

These were run after the test failure. They need no simulator and publish nothing. Every refusal path exits before `rclpy.init()`.

| Check | Exit | Result |
|---|---|---|
| `scripts/validate_fortress.sh` (static) | 0 | 60 PASS, 0 FAIL |
| `scripts/validate_m1_control.sh` (static) | 0 | 14 PASS, 0 FAIL |
| `scripts/validate_m2_posture.sh` (static) | 0 | 17 PASS, 0 FAIL |
| `scripts/validate_m3_kinematics.sh` (static) | 0 | 17 PASS, 0 FAIL (embedded pytest: 84 passed) |
| `scripts/validate_m4_all_leg_ik.sh` (static) | 0 | 18 PASS, 0 FAIL (embedded pytest: 136 passed) |
| `m61a_clearance --check-config` | 0 | `passive bound (recomputed): 0.10906 m below base_link`; `Config consistent with the geometry.` |
| `m6_gait_replay.py --dry-run --no-write` | 0 | trajectory `241760e7dfd5ef12`, content `94a492c43fcc0461`, fingerprint `9dba1a17…`; all match the docs |
| `m6_live_playback --dry-run --no-write` | 0 | trajectory `44f0a7ad52e5c330`, fingerprint `0d6ef417…`; both match the docs |
| `m61a_observe_fixed_base --interface-only` | 0 | `publishers: {}`, `action_clients: {}` |
| `m6_gait_replay.py --live` | 3 | `REFUSED` (gate False), nothing sent |
| `m6_live_playback --live` | 3 | `REFUSED` (gate False), nothing sent |

Logs are in `build_test/static/`; `summary.tsv` has the exit codes and UTC times. The command list is in `tools/static_checks.sh`.

## 5. Observation runs

| Run | Launch | Preflight | 120 s observation | Pose/attachment | Timing/RTF | /scan | Ownership | Shutdown | Verdict |
|---|---|---|---|---|---|---|---|---|---|
| run_01 | not started | — | — | — | — | — | — | — | **NOT PERFORMED** |
| run_02 | not started | — | — | — | — | — | — | — | **NOT PERFORMED** |
| run_03 | not started | — | — | — | — | — | — | — | **NOT PERFORMED** |

Because no simulator ran, none of the following has any evidence from this session:
- pose and frame interpretation;
- attachment and 0.125 m mount verification;
- timing, gaps and real-time factor (RTF);
- /scan agreement;
- controller initialisation and zero-command evidence;
- visual leg clearance;
- clean shutdown.

All M6.1-A §10 phase-1 criteria (1–8) remain **unresolved**. No Gazebo, Xvfb or launch process was started; `pgrep` found no `gz`, `ign`, `ros2` or `Xvfb` process afterwards.

### Prepared but unused tooling (in `tools/`)

These scripts are kept for a future authorised run. They are untracked evidence tooling, not repository source.

| Script | What it does |
|---|---|
| `stage_start.sh` | Foreground-equivalent GUI launch under Xvfb, with output captured to `RUN/launch.log` |
| `stage_wait_controllers.sh` | Waits until both controllers are `active`, then the settle time |
| `stage_observe.sh` | `--domain-id 0 --preflight`, then `--duration 120` (wall time) |
| `stage_capture.sh` | Read-only captures: controllers, hardware interfaces, graph, publisher and action-client info, `ign model` pose, joint and weld, one `/scan`, a 10 s capture, a root-window screenshot |
| `capture.py` | A read-only rclpy subscriber. It creates no publisher and records its own publisher list as proof. |
| `scan_check.py` | 2-D ray-cast of the lidar plane (z = 0.266 m at the weld) against `spiderx_fortress.sdf`, with a robust planar pose fit. A horizontal scan checks only x, y and yaw, not z, roll or pitch. Self-test in `tools/selftest/`: the true pose recovers 0.0/0.0 mm and 0.07 mrad; a +20/−10 mm, +10 mrad offset recovers +20.4/−9.6 mm and +9.92 mrad. |
| `run_analysis.py` | Builds the per-run criterion table |
| `stage_stop.sh` | Process-group SIGINT (the Ctrl+C equivalent), `ros2 daemon stop`, then a leftover-process check |

## 6. Remaining blockers and prerequisites

1. **The M6.0-D isolated end-to-end test failure must be resolved or explained** before any simulator observation is attempted under this procedure. The owner should decide:
   - **(a)** a supervised diagnostic re-run of the suite, or just this test, with no concurrent load; or
   - **(b)** a timing investigation of this test.
   
   Under either option, the run should keep its `live_outcome.json`, for example with `--basetemp` or by copying it out before rotation, so the readiness failure code is captured. That code would determine whether the cause is load-dependent timing in the test double or a real defect.
2. **The three launch-only observation runs** (M6.1-A §10 phase 1) then remain to be done and to pass. The owner's own Ubuntu 22.04 PC is still the reference environment for phase 1; this report is from a Cloud VM.
3. **A separately approved fixed-base cycle stays blocked** until both of the above have passed.

## 7. Boundaries kept

- No trajectory goal or motion command was published.
- No trot, crawl, keyboard walking or homing was run, and no gate was changed.
- No source or configuration edits, commits, pushes, merges or PR monitoring.
- `log/` is preserved: only colcon's own `build_2026-10-09_07-54-08`, `test_2026-10-09_07-54-47` and `test-result_2026-10-09_08-01-04` entries were added; see `build_test/log_dir_before.txt`.
- All evidence is under this directory, outside any build or install cleanup target. `SHA256SUMS` covers every file here.
