# S3 free-base level-0 crawl: the one authorized live attempt (2026-11-01)

**Verdict (committed scorer `scripts/s3/score_s3.py`, criteria unchanged): INCONCLUSIVE.** A1, A2 and A4–A8 PASS. A3 (tracking) is UNMEASURED: the final, scripted estop goal has no feedback, and the pre-specified N/A proof fails on a 0.5 ms timestamp ordering (below).

What the measurements show: Gazebo simulation, CAD masses, assumed contact.
- **Motion.** A bounded, scripted level-0 forward crawl of six cycles moved the body 0.1201 m.
  - Lateral drift ≤ 19 µm and yaw ≤ 8.7e-5 rad per cycle.
  - Height within 35 µm of standing; tilt ≤ 0.0026 rad.
- **Estop.** The estop held the standing robot for 10 s of sim.
- **Not shown.** Keyboard walking (S4), other levels, terrain, balance control, or anything about hardware.

## Baseline, commits and builds
- **Disabled run baseline:** `d91879989138543f43b6c6733217669b773c0337` (`claude/spiderx-s3-preparation`).
  - It is the shutdown-handler fix on top of `236238e`.
  - The shadow P4 evidence is committed as `a7917fc`.
- **Clean disabled build and full suite** (`disabled/`): toolchain-only environment, st59site for the build only, `PYTEST_DISABLE_PLUGIN_AUTOLOAD=1`.
  - 1946 tests: 1896 pytest cases and 50 CTest entries; 0 errors, failures or skips.
  - Fresh-collection check PASS: 50 of 50 registered targets, including `test_m55_rclpy_shutdown`.
  - Direct tests that colcon does not collect: `src/spiderx_scripts/test` 2 passed; `scripts/s3` 69 OK (42 driver, 27 scorer).
  - Gates False from this workspace. Library digest `62f8a100…` (54 goals); `library.json` `917fc4d7…`, the same as the P4 run and Phase C.
- **Enabling** (`enabled/`, `restore/enabling_*`):
  - Local branch `local/m55-s3-20261101T120227Z`, commit `85ac127`.
  - It applies `docs/patches/m55_enable_s2.UNAPPLIED.patch` (`d1dacac5…`; `git apply --check` passed first), changing the M55 gate and its pin, 2 lines. M61 and M6.0-D stay False.
  - Enabled build: 245 M5.5 tests passed. The digest and `library.json` are identical to the disabled build.
  - The branch was never pushed and was deleted only after the restore verified.

## Runtime checks before the first goal (`run/premotion/`)
- **Launch.** `fortress_m55_walking.launch.py` (free base) at 12:07:16Z. Source order: toolchain, r2 overlay `local_setup`, workspace.
- **Server and libraries.** Server pid 8186 is in the launch session. It maps only the r2 libraries, `3a8f2044…` and `06777e14…`, with matching inodes and provenance hashes; no stock or mixed library.
- **Configuration.** Gain 1.0 (one log line, and the parameter); `update_rate` 100; 0.01 s controller period against 0.001 s physics.
- **Ownership.**
  - Both controllers are active.
  - Node 8103 reports "up in DISPATCH mode (dispatch gate True)".
  - It is the action's only client (`/spiderx_locomotion`). `joint_trajectory` has 0 publishers, and there is no other teleop or driver.
- **Settling.** 10.09 s of sim (7.19–17.28), 96 fresh samples, no violation: z 0.054500 m, range 9.0 µm, tilt 1.5e-7 rad.
- **Screenshot 1** was taken at rest before the first goal.

## The run (`driver/`, `node/`, `run/`)
- **Driver.** `scripts/s3/s3_driver.py` in default (live) mode, unchanged since the P4 run (`0af5b90a…`). It ran 12:09:49–12:15:16Z and exited 0, `script complete`.
- **Script.** Arm, then run A (1 cycle), then run C (5 cycles), then the estop (`w`, then `x` with goal 55 in flight), then the 10 s hold, then `q`. Every step completed.
- **Goals and cancels.**
  - **Node:** 55 goals sent and accepted, 54 SUCCEEDED, goal 55 CANCELED; 1 cancel (server code 0).
  - **Controller log:** 55 "Received/Accepted new action goal", 54 "Goal reached", 1 cancel request.
  - **Faults:** none other than the scripted estop. No rate rejection, command rejection or tracking finding.
  - **Refusals:** two `disarm` requests after `q`, refused with `faulted_reset_required`, as expected.
- **RTF and freshness** (`analysis/run_analysis.json`):
  - RTF 0.418 over the script (10 s windows 0.194–0.630).
  - Joint-state age reported by the node: at most 0.140 s, a margin of 0.360 s to the 0.5 s limit.
  - Recorder `/joint_states` gap: at most 0.222 s, a margin of 0.278 s.
  - Pose age at most 0.083 s; graph age at most 1.025 s, against a 3.0 s limit.
- **Clocks.** Node events and the driver log use CLOCK_MONOTONIC. Bag receipt times are aligned to it through 663 status pairs, p05–p95 within −1.53 to +1.05 ms. Sim time comes from the joint-state stamps.

## A1–A8 (`score/summary.json`)
| # | Verdict | Measured |
|---|---|---|
| A1 | PASS | All 54 run goals SUCCEEDED; runs A and C each ended READY at boundary 0; no unplanned fault |
| A2 | PASS | 6 cycles; z_stand 0.054500 m; values in the table below |
| A3 | **UNMEASURED** | Goals 1–54: feedback on each, max \|error\| 0.00225 rad (abort 0.05), no finding over 0.02. Goal 55: no feedback, and its N/A proof fails (below) |
| A4 | PASS | No continuity refusal; max 4.5e-9 rad |
| A5 | PASS | 55 goals, 1 cancel, 0 foreign clients, 0 command publishers. `joint_trajectory` absent, with all 7 support checks true |
| A6 | PASS | FAULTED with 1 cancel, goal CANCELED. Over sim 149.32–159.32 s (2517 pose, 1001 joint samples): joint motion 1.2e-11 rad, tilt 3.1e-6 rad, min z 0.054495 m |
| A7 | PASS | DISABLED AND VERIFIED: gates False, both M6 `--live` exit 3, no leftover |
| A8 | PASS | `library.json` `917fc4d7…`, enabled = disabled |

**A2 per cycle** (band: forward 10–30 mm; \|lateral\| ≤ 10 mm; \|yaw\| ≤ 0.05 rad; height ±10 mm; tilt ≤ 0.10 rad):

| Cycle | Forward (mm) | Lateral (µm) | Yaw (rad) | Height vs standing (µm) | Max tilt (rad) |
|---|---|---|---|---|---|
| 1 | 20.021 | 3.7 | 1.06e-5 | −2.3 | 0.00256 |
| 2 | 20.029 | −8.8 | −6.29e-5 | −13.4 | 0.00256 |
| 3 | 20.023 | −2.8 | −8.65e-5 | −16.1 | 0.00256 |
| 4 | 20.021 | −9.1 | −4.80e-5 | +26.5 | 0.00257 |
| 5 | 20.026 | −9.4 | −6.60e-5 | −1.1 | 0.00256 |
| 6 | 20.015 | −19.2 | 1.36e-5 | −34.9 | 0.00258 |

**Why A3 is UNMEASURED.** Goal 55 was sent at 1753.4159 s (CLOCK_MONOTONIC). The node handled the estop and queued the cancel at 1753.4311, before the controller's acceptance at 1753.4661; the result was CANCELED at 1753.5173.
- **Passing checks.** Of the proof's 12 checks, these 9 pass: final goal, estop identity, accepted, the scripted `x` during the goal, exactly one cancel, server code 0, CANCELED, no refusal, no feedback.
- **Failing checks.** `estop_fault_after_key`, `early_cancel_request` and `no_unplanned_fault_before` fail. Each requires the fault and the cancel at or after the driver's logged `x` time, 1753.4316, which is 0.5 ms later.
- **Cause.** The driver stamps a key after its pty write returns.

Under "no criteria revision after seeing the result", A3 stays UNMEASURED.

**Recorded, not scored.** `score/slip.csv`: 220 rows, largest foot travel in a goal window 27.3 mm. S2's unloaded response-law fit is not applied on contact. The plant is r2, gain 1.0 (provenance above).

## Shutdown and restore
- **Shutdown.** SIGINT went to the launch group, which was empty after 5 s.
- **Simulator server** (strace): it got SIGINT and then `+++ exited with 0 +++`.
- **Launch processes.** All 8 "finished cleanly", including `m55_locomotion_node`. There was no Traceback, SIGTERM or SIGKILL.
  - Whether `ExternalShutdownException` occurred is not shown, so the new handler's runtime path is not demonstrated by this run.
- **Leftovers.** None. The watchdog never fired.
- **Restore** (`restore/restore_verify_20261101T121550Z.txt`) against `d918799`:
  - `src/` equals the base, after the clean rebuild with st59site for the build only;
  - all three gates False from this workspace, and both M6 `--live` tools exit 3;
  - **DISABLED AND VERIFIED**;
  - the enabling branch was deleted after that.

  The final state is HEAD `d918799` on `claude/spiderx-s3-preparation`.

## Harness and media
- **Harness.** `tools/` is the P4 shadow harness, adapted for the enabled build and the live driver (`provenance/harness_vs_shadow2.diff`, `harness_SHA256SUMS.txt`):
  - the driver runs without `--shadow-rehearsal`;
  - the pre-launch, pre-motion and settle checks expect M55 True and DISPATCH mode, with the node as the only action client;
  - the restore is the full clean restore;
  - the Phase C screenshot after the script is added.
- **Deviation.** The shadow-only settle bag was dropped: the committed scorer reads the first `ros2 bag record` command for its A5 recorder evidence. The "pre-shadow" step labels in `steps.log` are text inherited from the shadow harness.
- **Media** (`media/`): screenshot 1 at rest before the first goal (12:09:30Z) and screenshot 2 after the script (12:15:21Z).
  - Both are cropped to the 1200×900 Gazebo window; the removed strip was uniformly black. `media/uncropped_sha256.txt` has the originals' hashes.
  - Software rendering on Xvfb, not showcase quality. A still image does not show motion; the bag measurements do.
- **Bag.** 178 MB (`run/bag_hashes.txt`), kept outside the repository under `/root/spiderx_evidence/s3_free_base_20261101T115002Z/`. Scoring did not change it. `SHA256SUMS` covers every file here.
