# S4d-M shadow rehearsal, 2026-11-10 (dispatch disabled)

**Result: PASS (SHADOW ONLY: not S4d acceptance).** The committed `score_s4d.py --shadow` rated:
- M1–M10 and M12–M16 PASS;
- M11 tracking N/A in shadow.

This was the one authorized free-base SHADOW launch, with all three gates `False`. **Zero**
trajectory goals, cancels or feedback reached the real controller.

This is a rehearsal of the committed S4d-M script (`scripts/s4/s4d_driver.py --shadow-rehearsal`),
**not** S4d acceptance:
- In shadow the body does not move, so no 0.3 m leg, cycle band or drift is demonstrated.
- No live attempt, S4d-E or enabling was authorized or done.

This file is an evidence index; the evidence is uncommitted, for review.

## Baseline and inputs (`provenance/`)
- **Tested executable.** `8f99bcc4ab27294265da91a31ce03999423acc7f` on `claude/spiderx-s4d-preparation`. The worktree was clean and there was no `local/*` branch.
- **Run installation.** The fresh **disabled** build of `8f99bcc` in new bases. It is batch step 2, recorded in `../s4d_fresh_disabled_20261110T071506Z/`: 1983 collected tests, 0 failures, 0 skipped. The direct `scripts/s2` tests, run separately, have 3 pre-existing `S2_BAG` skips.
  - `/root/spiderx_fresh_disabled_20261110T071506Z/install` (`--symlink-install`) was sourced after the toolchain and the r2 overlay.
  - The existing workspace installation was not in the environment.
- **Input checks: PASS** (`input_checks.txt`).
  - The r2 overlay: 0.7.15 `bfc5845` plus 0001 `14238400…` and 0002 `520e73cc…`; libraries `3a8f2044…` and `06777e14…`; its source equals upstream plus both patches.
  - The run installation's 169 runtime symlinks equal their HEAD blobs.
  - All three gates are `False` on a fresh import from the run installation.
  - The library has 54 goals, `62f8a100…`.
  - Every module the S4d driver and scorer import equals its HEAD blob. The unchanged ones equal the S4c level-2 shadow run's tools.
- **Pre-launch checks: PASS** (`prelaunch_checks.txt`): ament, pluginlib and dlopen resolve the r2 overlay. The disabled `library_disabled.json` is `917fc4d7…`, identical to the frozen library.

## Harness (`tools/`; `provenance/harness_vs_s4c_l2_shadow.diff`, `harness_SHA256SUMS.txt`)
It is the committed S4c level-2 shadow harness. Its copies verify against that run's manifest (`historical_copies_check.txt`). Changes:
- **Driver.** `scripts/s4/s4d_driver.py --out RUN/driver --shadow-rehearsal`, with no other argument.
- **`env.sh`.** It sources the fresh disabled run installation instead of the workspace installation. The run script refuses to launch if:
  - the workspace installation is in the environment, or the run installation is missing;
  - the imported gates are not all `False`.
- **OD8 watchdog.** The default is 7200 s, armed before the launch. The run script refuses unless the watchdog is running with at least the 90-minute script cap + 15 min left. It checks before the launch and again before the first key.
- **OD16 disk.** The rule max(5 GiB, 2E + 0.1 GB) is checked before the launch and again just before recording. E = 2 × 30.8 MB/min × the watchdog time left, and the 2× rate is a **planning assumption**.
- **Owned-process cleanup.** `cleanup.sh` stops only processes reached through pids, process groups or sessions the run recorded, and only with the expected command line: the driver, its two teleop pty sessions, the recorder, strace, the launch session and Xvfb. Name patterns only report.
- **Restore.** `restore_cmd.sh` runs `--verify-only` against `8f99bcc`. Then `run_install_gates.sh` verifies the run installation in a fresh environment (OD12, M15).
- **Tested before the launch** (`provenance/harness_tests/`):
  - the watchdog fires, stands down, and defaults to 7200 s;
  - owned cleanup leaves an unowned decoy untouched;
  - the run script refuses with no watchdog, with a 6000 s watchdog, and with a failing disk rule;
  - with time shortened before `go.flag`, it aborts before the first key.
  The first cleanup test's setup was invalid and is kept, labelled.

Unchanged from S4c:
- source order toolchain, r2 overlay, then the workspace layer;
- `fortress_m55_walking.launch.py` starts the node itself;
- one bag outside the repository, without `_action/status` or `/clock`;
- the trap and the 90-minute driver cap.

## Run (`run/`)
**The one launch.**
- The journal recorded "launch used: YES" at 07:44:21Z, when the guarded run started.
- The watchdog was armed at 07:44:21Z: pid 16828, deadline 09:44:21Z.

**Launch and pre-key checks: PASS.**
- **Timeline.** Launch at 07:44:25Z (`launch_start_utc.txt`). Both controllers were active at 07:44:56.8Z. The node was up in SHADOW (gate `False`) at 07:45:26Z.
- **Libraries.** Server 17143 alone maps the two r2 libraries, with the provenance hashes and no stock copy.
- **Configuration.** Gain 1.0 (log and parameter), 100 Hz, physics step 0.001 s.
- **Ownership.** 0 action clients, 0 command publishers and no driver or teleop process. Node 17038 is in the launch session.
- **Settling.** 10.03 s of sim, 98 fresh samples: z 0.054500 m, range 9.0 µm, tilt 1.5e-7 rad. Maximum ages: joint states 0.070 s, pose 0.025 s, graph 1.003 s.
- **Budgets.** Before the first key, 7048 s of the watchdog were left. Before recording, 22.4 GB was free against 14.6 GB required.

**Driver.** It ran 07:46:59–08:13:36Z and exited 0 ("script complete (shadow rehearsal: dispatch disabled; not acceptance)").
- **Keys.** `+ a w + ' ' s ' ' w w s h w q`, then the probe's `+ + w q`. Requests before `q`: arm, stop, stop, home.
- **Plan.** Exactly the 111 planned shadow goals, all SUCCEEDED, with **0 cancels**, no fault, refusal or command rejection.
- **Speed change (OD13).**
  - `+` 1 came while DISARMED, before arm: `2/3`, 1.90 mm/s.
  - `+` 2 came while goal 2 (L1 1→2) was in flight: `3/3`, 2.34 mm/s.
  - Each confirmation was attributed to its own write.
  - `level_change` 0→1 occurred in READY before goal 1; 1→2 occurred **in WALKING** between goal 9 and goal 10. Goals 1–9 ran at L1 and 10–111 at L2.
- **Transitions** (24):
  - arm;
  - F: STARTING, WALKING, with no stop from goal 1 to goal 54; STOPPING `stop` at goal 54; READY `stopped`;
  - R: the same, ending at goal 99;
  - X: STOPPING `intent_stop` at goal 104, READY; goal 105, then the switch to goal 106 (6→5); STOPPING `intent_stop`, READY; home: 4 forward goals, then READY `homed` at boundary 0;
  - Q: goal 111, STOPPING `disarm`, then DISARMED `lease_expired` at boundary 1.
- **Cycle counts.** 1 after goal 9, 6 at goal 54, 1 at goal 99, 2 at `homed`.
- **Releases.** Detected 0.847 s (goal 104) and 0.825 s (goal 106) after the recorded release: at least 0.717 s and below the 0.983 s finding threshold. READY came in 3.12 s and 4.14 s of sim (bounds 4.2 s and 5.2 s). Home took 10.49 s of sim (bound 12.0 s).
- **Close while moving.**
  - The repeat stopped, then `q` was written 0.2 s later, while goal 111 was in flight.
  - The node logged disarm, stop, disarm, stop.
  - It reached DISARMED 1.89 s of sim after the last status before `q` (bound 2.9 s).
  - **Reason: `lease_expired`**, matching the node-clock evidence by rule (b). The last valid command came 8.13 s of node time before the transition. The command count was unchanged between the last in-flight status and the first DISARMED status (15248), and no command followed the status.
  - Teleop 18193 exited 0 by itself (wait status 0).
- **After the close.** DISARMED with no goal over 10 s of sim (347.97–357.97). From the bag: joint motion 5.9e-12 rad, tilt 1.5e-7 rad, z ≥ 0.054495 m.
- **Reconnect probe (OD11).**
  - A second teleop (21285) started; `+ +` gave `2/3` then `3/3`, each attributed.
  - `w` was held 5–6 s. Statuses showed intent forward at level 2, and `commands_accepted` rose from 15248 to 15317.
  - The release returned the intent to 0 within 5 s; `q` followed.
  - The node stayed DISARMED with no goal and logged no arm, goal, transition, refusal or rejection.
  - The close requests were accepted as no-ops: "already disarmed" and "not moving (DISARMED); held until a zero command", × 2 each. Teleop 21285 exited 0 by itself.

## Scorer verdicts (`score/summary.json`, committed `score_s4d.py --shadow`, unchanged; run after the bag was closed)
| | Verdict | Detail |
|---|---|---|
| M1 | PASS | 111 planned goals in order, each SUCCEEDED; 0 cancels, faults or refusals; keys and pre-`q` requests as scripted; SHADOW with the gate `False` in every status |
| M2 | PASS | `+` before arm and while goal 2 was in flight, each attributed; exactly two `level_change`, 1→2 in WALKING between goals 9 and 10; L1 for goals 1–9, L2 for 10–111; strides 40 and 60 mm |
| M3 | PASS | Cycle count 1 after goal 9. The shadow cycle is stationary (−3.5e-10 m); the signed band is not applied |
| M4 | PASS | Continuous walking from goal 1 to 54; space during goal 54; one STOPPING `stop`; READY at 0 within 2.2 s of sim; count 6. F leg −2.0e-9 m: N/A in shadow (recorded only) |
| M5 | PASS | As M4 for goals 55–99; READY within 2.9 s of sim; count 1. R leg N/A in shadow |
| M6 | PASS | Cumulative \|lateral\| at most 2.02e-15 m and \|heading\| at most 2.31e-13 rad at every boundary 0; tilt within 0.10 rad |
| M7 | PASS | Both releases: one STOPPING `intent_stop` after the hold window; READY within the in-flight goal + 1.0 s of sim; no extra goal |
| M8 | PASS | Switch while goal 105 was in flight, no release; goal 106 6→5 at boundary 6, with no stop between |
| M9 | PASS | 4 forward homing goals, no motion key; `homed` at 0 in 10.49 s of sim; count 2 |
| M10 | PASS | Close checks as above; reason `lease_expired` = expected from node-clock evidence; teleop exit 0 within 15 s; still over 10 s of sim |
| M11 | N/A | No trajectory runs in shadow |
| M12 | PASS | Maximum continuity error 2.2e-16 rad |
| M13 | PASS | 111 shadow goals, 0 cancels, 0 foreign clients, 0 publishers; controller log 0 received, 0 accepted, 0 cancel requests; no feedback in the bag; `joint_trajectory` absent, with every support check true |
| M14 | PASS | Driver complete; minimum joint margin 0.436 rad (`rr_foot_joint`); maximum node-observed ages: joint states 0.084 s, pose 0.032 s, graph 1.008 s |
| M15 | PASS | `--verify-only` restore DISABLED AND VERIFIED; `library.json` `917fc4d7…` = the disabled one; the run installation imports all three gates `False` |
| M16 | PASS | Every probe bound and every piece of evidence in §5.5 |

**Real controller: zero goals, zero cancels, zero feedback.**
- The JTC log has 0 goal-received, 0 goal-accepted and 0 cancel lines.
- The bag has 0 `_action/feedback` messages and no `joint_trajectory` message.
- The controller's reference changed by 0.0 rad over 16,124 `controller_state` samples.
- Action clients: 0 before; afterwards only `/rosbag2_recorder`.

**Supplementary** (`analysis/`, not used for the verdict):
- RTF 0.212 overall, with 10 s windows of 0.184–0.240;
- 3235 statuses, all SHADOW with the gate `False`;
- maximum node-observed ages over all statuses: joint states 0.084 s, pose 0.055 s, graph 1.008 s.

## Stop and verification
Each process's outcome is recorded separately; no exit code is inferred.
- **Simulator server (17143): exit 0, observed by strace.** strace recorded one SIGINT, sent by pid 17028: the `ign gazebo` launcher, its parent, which received the launch group's SIGINT. This is as in the S4c level-2 shadow.
- **Locomotion node (17038):** "process has finished cleanly", as the launch reports it.
- **Teleops:** 18193 (the run) and 21285 (the probe) each exited 0 by itself, observed by the driver. None was left in their sessions.
- **GUI (17146):** its own exit status was not observed. The launch log has no escalation line, and its launcher (`ign gazebo-1`, 17025) finished cleanly.
- **Launch.** Every launch process finished cleanly. The launch group was empty 5 s after SIGINT.
- **Cleanup and watchdog.** The trap's owned cleanup found nothing left to stop, and no relevant process remained. The watchdog stood down at 08:14:05Z without acting.
- **Restore.** `--verify-only` against `8f99bcc`: **DISABLED AND VERIFIED**.
  - Gates `False` on a fresh import of the workspace installation; both M6 `--live` tools refused (exit 3); `src/` equals the base; no simulation process.
  - The run installation imports all three gates `False` (`run_install/`, M15).
  - Details: `restore/`, `disabled_state_after.txt`.
- **Bag** (kept outside the repository): `/root/spiderx_evidence/s4d_shadow_20261110T072920Z/bag/bag_0.db3`.
  - sha256 `c285b2e6…`, 760,479,744 bytes, 151,711 messages.
  - Unchanged by scoring and by the supplementary analysis (`run/bag_hashes.txt`).
  - About 28.4 MB per wall minute over the bag's 1605.2 s (26.8 min), below the 30.8 MB/min figure that OD16's planning assumption doubles.
- **Findings.**
  - At startup, `gz_ros2_control` logged two `[ERROR]` lines, 0.50 s apart: "robot_state_publisher service not available, waiting again". It connected before the controllers were active. This is the startup wait S4b and the S4c level-2 shadow also recorded.
  - One ros2 CLI daemon restart happened while waiting for the controllers (`daemon_events.txt`). That is the observer side.
  - `provenance/run_started.txt` first named pid 16842. That was the exited `setsid` parent: the session shell has job control, so `setsid` forked. The running wrapper's pids are in `wrapper_processes.txt`.

## Limits of this rehearsal
- **No body motion in shadow.** The boundary poses, cycles, legs and drift are those of a stationary body, so they do not validate moving-body boundary extraction. S4c already recorded real level-2 single-cycle walking: one forward and one reverse cycle, each stopped at boundary 0 (`../s4c_l2_live_20261103T111733Z/`). The remaining gap is continuous multi-cycle level-2 boundary extraction. Walked-through boundaries have been checked only on the level-0 S3 bag, so this remains a live-validation risk.
- **Timing differs from live.** The close was decided `lease_expired` at shadow RTF ≈ 0.21. A live close may produce `disarm`, `lease_expired` or insufficient evidence (UNMEASURED), depending on node timing. The frozen scorer is not to be weakened to guarantee a PASS.
- **Not exercised here.** This rehearsal shows the script, the stops not firing, the probe, the scorer and the harness. It does not show tracking (M11), displacement bands or any live stop firing.

## Contents
`run/` (logs; `launch.log.gz`; `premotion/`; `captures/`), `driver/` (`driver.jsonl.gz`, summary,
output), `node/`, `media/` (two screenshots, both at rest: before the first key and after the
script), `provenance/`, `tools/`, `restore/`, `run_install/`, `score/`, `analysis/`,
`disabled_state_after.txt`, `SHA256SUMS`.
- **Added when the evidence was committed:**
  - `run/bag_retention.txt` records how to identify and read the raw bag, which is not in the repository;
  - `provenance/s4d_batch_journal_snapshot_20261110T113338Z.md` is a snapshot of the external run journal, reviewed for secrets.
- **Compression.** The launch log, the driver log and the full memory map are gzipped.
- **Secret scan.** No credential, key or personal address was found. The screenshots carry no text chunks.
