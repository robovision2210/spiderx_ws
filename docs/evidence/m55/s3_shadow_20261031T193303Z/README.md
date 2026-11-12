# S3 prerequisite P4: shadow rehearsal 2 (retry), dispatch disabled (2026-10-31)

**Run.** One free-base launch of `fortress_m55_walking.launch.py` at 19:34:37Z, on baseline `236238e` (`claude/spiderx-s3-preparation`).
- **Configuration.** The r2 overlay was unchanged, and all three dispatch gates were `False`. The locomotion node ran in **SHADOW** mode.
- **Driver.** `scripts/s3/s3_driver.py --shadow-rehearsal`.
- **Process.** No relaunch, retry or code change during the run.

**No trajectory goal or cancel reached the controller. This is a shadow rehearsal, not crawl, walking, contact or gait-stability acceptance.**

Rehearsal 1 is preserved unchanged in `../s3_shadow_20261031T190613Z/` (commit `25eb0fe`). The `--shadow-rehearsal` fix is commit `236238e`.

| Item | Result |
|---|---|
| **P4** free-base settling and shadow rehearsal | **PASS** (shadow only) |
| Planned shadow goals / shadow cancels | **55 / 1** |
| Real-controller goals / cancels | **0 / 0** |
| Simulator server shutdown | exit 0 after SIGINT; no abort, SIGTERM or SIGKILL; no leftovers |
| Restore `--verify-only` against `236238e` | **DISABLED AND VERIFIED**, all three gates `False` |

## Before the script (`run/premotion/`)
- **Server and libraries.** Server pid 4620 is in this launch's session. It maps only the r2 libraries, and their hashes match the provenance:
  - `3a8f2044…`;
  - `06777e14…`.

  No stock library is mapped.
- **Configuration.**
  - The gain log line "set to: 1" appears exactly once, and the parameter reads 1.0.
  - `update_rate` is 100.
  - The plugin reports a 0.01 s controller period against 0.001 s physics.
- **Ownership.**
  - Both controllers are active.
  - The action has 0 clients.
  - `joint_trajectory` has 0 publishers.
  - Node 4536 runs in the launch's session, in SHADOW mode, with no action client.
- **Settling** (`settle_check.json`): 10.02 s of sim (sim 10.59–20.61 s), 93 fresh samples, no violation.
  - Standing height **0.054500 m**, range **9.0 µm**, tilt ≤ **1.5e-7 rad**.
  - Within ±2.5 mm of the standing height from sim 0.06 s after spawn (spawned at z 0.0750 m).

## Shadow script (`driver/`, `node/`, `analysis/shadow_analysis.json`)
- **Driver result.** Exit 0, `script complete (shadow rehearsal: dispatch disabled; not crawl acceptance)`.
  - Its own settle window (sim 24.33–34.34 s) gives the same height and range.
  - The node mode was verified (SHADOW, gate False) before the teleop started and in every status.
- **Script.** Run A (1 cycle), then run C (5 cycles), then the planned estop (`w`, then `x` with goal 55 in flight), then the 10 s hold, then `q`.
- **Node events.**
  - `goal_sent` 55 (`node/shadow_goals.json` 55). Results: 54 SUCCEEDED, plus goal 55 CANCELED with "shadow cancel".
  - The planned `estop` fault, then `cancel_sent` 1 and `cancel_response` code 0. This is the shadow transport's cancel, not a controller cancel.
  - No rate rejection, command rejection, tracking finding or unplanned fault.
  - After `q`, the node refused two `disarm` requests (`faulted_reset_required`), as expected after an estop.
- **Real controller: 0 goals and 0 cancels.**
  - The controller log has 0 "Received/Accepted new action goal" and 0 cancel lines.
  - The `controller_state` reference changed by 0.0 rad over 6416 messages.
  - `joint_trajectory` was absent from the bag and had 0 publishers before and after.
  - `_action/feedback` carried 0 messages.
  - The node had no action client before or after. After the run, the only action client listed is the bag recorder, through its feedback subscription.
- **Per-cycle displacement** (measured, recorded):

  | Cycle | 1 | 2 | 3 | 4 | 5 | 6 |
  |---|---|---|---|---|---|---|
  | Forward (m) | −3.08e-10 | −2.60e-10 | −2.86e-10 | −3.28e-10 | −2.75e-10 | −2.77e-10 |

  - Lateral was at most 6.2e-16 m and yaw at most 1.7e-14 rad.
  - Each record carries "Minimum forward-progress condition not applicable: dispatch disabled."
  - This is the standing body's creep, not crawl progress.
- **Hold observation.** Sim 148.76–158.84 s, 93 fresh samples. Joint motion was at most 5.9e-12 rad, tilt at most 1.5e-7 rad, and z at least 0.054495 m.
- **Whole script.**
  - Joints moved at most 5.9e-12 rad.
  - Body z stayed within 0.0544953–0.0545043 m, tilt at most 1.5e-7 rad.
  - xy travel was 2.2e-9 m.
  - The minimum URDF joint margin was 0.4363 rad.
- **Clocks.** Three clocks are kept separate:
  - *sim* is the node's joint-state stamps; the bridged pose headers carry no stamp;
  - *bag receipt* is the recorder's wall clock (UTC);
  - *node wall* is the node's monotonic clock (`events.jsonl`).

## Real-time factor and freshness
- **Real-time factor:** 0.217 over the script (631 s wall, 137.06 s sim); 10 s windows 0.189–0.238.
- **Joint-state age reported by the node:** at most 0.077 s, a margin of 0.423 s to the 0.5 s limit.
- **Largest `/joint_states` receive gap at the recorder during the script:** 0.230 s, a margin of 0.270 s.
- **Pose age:** at most 0.050 s. **Graph age:** at most 1.006 s, against a 3.0 s limit.
- **Before the node was up** (19:34:47–19:35:11, sim < 0.42 s): the simulation crawled while loading. Receive gaps reached 1.32 s, with 10 s RTF windows down to 0.009. The node came up at 19:35:38, and there was no gap over 0.3 s after that.

## Shutdown (`run/shutdown.txt`, `run/server_exit_strace.txt`, `run/launch.log.gz`)
- **Server.** SIGINT went to the launch group, which was empty after 3 s. strace shows server 4620 received SIGINT and then `+++ exited with 0 +++`.
- **Log.** No `terminate called`, `Aborted`, `Signal sent by`, SIGTERM, SIGKILL or "sending signal" line. Seven launch processes "finished cleanly".
- **Finding: the locomotion node exited with code 1** (`process has died`). At SIGINT, rclpy's signal handler shut the context down first, so `executor.spin()` raised `rclpy.executors.ExternalShutdownException`. `m55_rclpy.main` catches only `KeyboardInterrupt`.
  - This is a shutdown race: in rehearsal 1 the same node finished cleanly.
  - Its `finally` block still closed the node, and `final_status.json` and `events.jsonl` were written.
  - The node was not killed and left nothing behind. No code was changed.
- **Leftovers.** `cleanup.log` lists one entry, the operator's polling shell, whose command line contains the search patterns; it is not a run process. The restore found no simulation process.
- **Watchdog.** Armed at 3600 s and disarmed after the stop; it never fired.

## Harness and evidence
- **Harness.** The rehearsal-1 harness was reused; its files match `../s3_shadow_20261031T190613Z/provenance/harness_SHA256SUMS.txt`.
  - It changed only in the driver flag, the restore paths and baseline, and the analysis clock labels.
  - Settle-bag sim times now come only from joint-state stamps.
  - The changes are in `provenance/harness_vs_rehearsal1.diff`, with hashes in `provenance/*SHA256SUMS.txt`.
- **Input checks** are in `provenance/input_checks.txt`:
  - HEAD `236238e`, clean tree;
  - r2 libraries and patch hashes;
  - gates False from a fresh interpreter, with module paths;
  - library `62f8a100…`, 54 goals.

  `provenance/prelaunch_checks.txt` has verdict PASS.
- **Focused tests** at `236238e`: `provenance/focused_tests.txt`, 42 OK, flake8 clean.
- **Bags.** 301 MB and 48 MB, kept outside the repository under `/root/spiderx_evidence/s3_shadow_20261031T193303Z/`. Their hashes are in `run/bag_hashes.txt`. `SHA256SUMS` covers every file here.
- **Screenshot.** None taken; it was optional.
