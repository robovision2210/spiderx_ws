# S3 prerequisites P7 and P4: one free-base launch, dispatch disabled (2026-10-31)

Branch `claude/spiderx-s3-preparation`, HEAD `e0a952e`. There was one launch of `fortress_m55_walking.launch.py`, at 19:11:33Z, with all three dispatch gates `False`.

The locomotion node ran in **SHADOW** mode, so no trajectory goal or cancel reached the controller. There was no relaunch, retry or code change.

**This is not a crawl, walking, contact or gait-stability result.**

| Prerequisite | Verdict |
|---|---|
| **P7** runtime loading, configuration and shutdown (r2 overlay) | **PASS** |
| **P4** free-base settling and shadow rehearsal | **INCONCLUSIVE**: settling PASS; the shadow script stopped after run A (below) |

## P7: loading and configuration (before any key; `run/premotion/`)
- **Server process.** Exactly one process maps the plugin: pid 1580, `ign gazebo server`. It is in this launch's session; its parent chain is 1491 → 1489, where 1489 is the launch's `ign gazebo-1` process.
- **Libraries.** Both plugin libraries are mapped only from `/root/spiderx_gz_overlay_ws_r2`. Inodes match, and the mapped-object SHA-256 equals the r2 provenance:
  - `libgz_ros2_control-system.so`: `3a8f2044…`;
  - `libgz_hardware_plugins.so`: `06777e14…`.

  No stock copy and no mixed copy is mapped.
- **Configuration.** Each item was checked as stated:
  - the gain log line "has been set to: 1" appears exactly once;
  - `ros2 param get` returns `position_proportional_gain` 1.0;
  - `update_rate` is 100;
  - the plugin warning reports a 0.01 s controller period against a 0.001 s physics period;
  - there are no init or invalid-gain errors.
- **Ownership.**
  - Both controllers are active.
  - The action has 0 clients.
  - `joint_trajectory` has 0 publishers.
  - Node pid 1503 runs in the launch's session and has no action client. It logged "locomotion node up in SHADOW mode (dispatch gate False)".

## P7: shutdown (`run/shutdown.txt`, `run/server_exit_strace.txt`, `run/launch.log.gz`)
- **Signal.** The harness sent SIGINT to the launch process group, and the group was empty after 4 s.
- **Server exit.** strace (signals only, attached just before the stop) shows that server 1580 received SIGINT and then `+++ exited with 0 +++`.
- **Log.** The launch log has no `terminate called`, `what()`, `Aborted`, `Signal sent by`, `process has died`, SIGTERM, SIGKILL or "sending signal" line. All eight launch processes "finished cleanly".
- **Comparison.** The Phase C overlay (r1) aborted at this point.
- **Cleanup.** `cleanup.sh` found nothing left. The restore verify-only step found no simulation process.
- **No escalation.** No SIGTERM or SIGKILL was sent to the server by the launch, the harness or the restore script.
- **Shutdown log note.** The one "remaining" entry in `shutdown.txt` is the operator's own polling shell. Its command line contains the search patterns; it is not a run process.

## P4: settling on the gain-1.0 plant (`run/premotion/settle_check.json`, `analysis/settle_timing.json`)
- **Pre-shadow check.** 10.05 s of sim (sim 8.59–18.64 s), 94 fresh samples, no violations, SHADOW, gate False, DISARMED:
  - standing height **0.054500 m**;
  - z range **9.0 µm**;
  - tilt ≤ **1.5e-7 rad**.
- **Driver's own settle window.** 10.11 s of sim gives the same values.
- **Settling duration.** Spawned at z 0.0750 m; minimum 0.05398 m at sim 0.06 s. Within ±0.5 mm of the standing height from sim 0.09 s, and within ±10 µm from sim **0.59 s**.
- **Creep.** In the settle window the stationary body crept −1.46e-10 m along body +y.

## P4: shadow script (`scripts/s3/s3_driver.py`, unchanged)
- **Driver result.** Exit 2, `STOP CONDITION: cycle 1 moved -0.0 mm along body +y (allowed 0..40)`.
  - Run A completed one cycle: 9 shadow goals, then space, STOPPING, and READY at boundary 0.
  - The first cycle check then measured forward −3.28e-10 m. That is the stationary body's sub-nanometre creep: in shadow no trajectory runs. The approved "progress < 0" stop therefore fired.
  - The driver pressed `x` at rest. The node went READY → FAULTED (`estop`), and no goal was in flight.
- **Not rehearsed.** Run C (45 goals), the planned estop with its cancel, and the 10 s sim hold observation.
- **No faults besides the estop.** No rate rejection, no refusal, no command rejection, no tracking finding.
- **State sequence** (node events): DISARMED → READY (arm) → STARTING → WALKING → STOPPING (stop) → READY (stopped) → FAULTED (estop).
- **Shadow versus real.**

  | | Shadow (node) | Real controller |
  |---|---|---|
  | Goals | 9 (`goal_sent` 9, `node/shadow_goals.json` 9) | 0 |
  | Cancels | 0 | 0 |

  Evidence for the real side:
  - the controller log has 0 "Received/Accepted new action goal" and 0 cancel lines;
  - the `controller_state` reference changed by 0.0 rad over 1456 messages;
  - `joint_trajectory` was absent from the bag and had 0 publishers before and after;
  - `_action/feedback` was subscribed but carried 0 messages;
  - the node had no action client.

  After the run, `ros2 action info` lists one client, the bag recorder, through its feedback subscription.
- **Holding the standing pose** (shadow window):
  - joints moved at most 5.9e-12 rad (`rr_foot_joint`);
  - body z stayed within 0.0544953–0.0545043 m;
  - tilt was at most 1.54e-7 rad;
  - xy travel was 4.5e-10 m;
  - the minimum joint margin to the URDF limits was 0.4363 rad.
- **Free-base RTF** with the GUI at nice +19: 0.212 over the script (146 s wall, 31.1 s sim); 10 s windows 0.201–0.228.
- **Freshness.**
  - Largest joint-state age reported in the node status: 0.085 s during the script, a margin of 0.415 s to the 0.5 s limit.
  - Largest `/joint_states` receive gap at the bag recorder during the script: 0.134 s, a margin of 0.366 s.
  - Largest gap after the node was up: 0.347 s, at 19:12:53 during the pre-shadow CLI checks with the node DISARMED, a margin of 0.153 s.
  - Before the node was up (19:12:06–19:12:34), the simulation nearly stalled while loading: 10 s windows had RTF down to 0.009, with gaps up to 1.2 s.
  - Pose age was at most 0.066 s, and graph age 1.007 s against a 3.0 s limit.

## Remaining before S3 motion authorization
A completed P4 shadow rehearsal: run C, the planned estop with its shadow cancel, and the 10 s hold.

The unchanged driver cannot get there in shadow. Its per-cycle progress stop needs a forward displacement ≥ 0, and a stationary body meets that only by chance. This needs an owner decision on how that stop behaves in shadow mode; the live limits stay unchanged. After that, one more authorized shadow launch.

## Screenshot
`screenshot/s3_shadow_standing_rest.png`, taken at 19:14:32Z before the driver started.
- **Capture.** Taken with the pre-installed ImageMagick `import` of the Xvfb root window. It was cropped to the 1200×900 Gazebo window; the removed 400 px strip was uniformly black. Uncropped SHA-256: `020257e7…`.
- **Not published.** It is not referenced from any README yet.

*Free-base standing/shadow verification in simulation. Gait dispatch disabled; no crawl or walking demonstrated.*

## Harness and evidence
- **Harness.** The run-local harness is in `tools/`:
  - the Phase C components, adapted only for this launch, the r2 source order and the new checks;
  - `provenance/harness.diff` against the frozen sources;
  - hashes in `provenance/harness_SHA256SUMS.txt`.

  `stage_wait_controllers.sh` and `stage_stop.sh` are unchanged.
- **Adaptations.**
  - Inputs: the r2 overlay in `env.sh`; the walking launch with `evidence_dir`.
  - Run sequence: no separate node; the launch's node reniced to −5 as in Phase C; a settle bag from the spawn; a go/abort gate before any key.
  - Checks: all three gates expected `False`; `cleanup.sh` covers the S3 driver and stops the launch before killing a leftover node; the weld check replaced by `settle_check.py`.
  - Shutdown and restore: strace observes the server exit; the restore runs `--verify-only`.
- **Input checks** are in `provenance/input_checks.txt`:
  - branch, HEAD and clean tree;
  - the overlay source equals upstream `bfc5845` + 0001 + 0002;
  - the SpiderX install is a symlink install of this tree;
  - gates False, with module paths;
  - library `62f8a100…`, 54 goals.

  The focused tests in `../s3_preparation_20261031/` were reused because their inputs are unchanged.
- **Watchdog.** Armed at 3600 s and disarmed after the stop; it never fired.
- **CLI daemon.** The ROS 2 CLI daemon faulted once while the run waited for the controllers (19:11:56Z). `stage_wait_controllers.sh` restarted it (`run/daemon_events.txt`).
- **Restore.** `restore/restore_verify_20261031T191826Z.txt`: `--verify-only` against `e0a952e`. Result: **DISABLED AND VERIFIED**, all three gates `False`, `src/` equal to the base, no rebuild.
- **Bags.** 69 MB and 60 MB, kept outside the repository under `/root/spiderx_evidence/s3_shadow_20261031T190613Z/`. Their hashes are in `run/bag_hashes.txt`. `SHA256SUMS` covers every file here.
