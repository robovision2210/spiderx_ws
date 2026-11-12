# S4d-M live attempt, 2026-11-11 (cloud simulation, dispatch enabled on a local branch)

**Result: PASS, for the approved S4d-M simulation scope only.** The unchanged committed
`scripts/s4/score_s4d.py`, run without `--shadow`, rated **M1–M16 PASS**.

This was the one authorized S4d-M live launch. It ran in Cloud Gazebo from a local enabling branch
that was never pushed. Restoration verified the disabled state of both installations.

It does **not** complete S4. These remain unvalidated:
- S4d-E (OD10) and S4e, human operation at a physical keyboard (OD9);
- hardware and navigation.

Full S4 acceptance is not complete.

## Baseline and inputs (`provenance/`, `disabled/`)
- **Exact disabled baseline.** `9cc1f6844de11abc630387d4c6c441a623463094` on `claude/spiderx-s4d-preparation`. It is a documentation-only commit on top of the tested executable `8f99bcc4ab27294265da91a31ce03999423acc7f`; `src/` and `scripts/` have not changed since `8f99bcc`.
- **Input checks: PASS** (`input_checks.txt`, 05:36Z).
  - The workspace installation equals HEAD.
  - Gates `False`; library `62f8a100…`; configuration `ab0dccd4…`.
  - Every tool module equals the shadow run's.
  - The patch's hash, scope and `git apply --check` pass.
  - The S4d shadow evidence manifest verifies.
  - OD16 disk: PASS.
- **Reused validation.** The fresh disabled build and the 1983-test result (`../s4d_fresh_disabled_20261110T071506Z/`, `fdcbb70`) were reused unchanged, because their inputs still matched.
- **Before enabling** (`disabled/install_check.txt`), from fresh imports:
  - all three gates `False`, imported from the workspace installation;
  - both M6 `--live` tools refused, exit 3;
  - `library.json` `917fc4d7…`.

## Harness (`tools/`; `provenance/harness_vs_s4c_l2_live.diff`, `harness_vs_s4d_shadow.diff`, `harness_SHA256SUMS.txt`)
New run-local files, derived from the committed S4c level-2 **live** harness and the S4d **shadow** harness. Both historical copies verify against their manifests (`historical_copies_check.txt`); neither was edited in place.
- **Driver.** `scripts/s4/s4d_driver.py --out RUN/driver`, live mode, with no `--shadow-rehearsal` flag.
- **Run installation.** `env.sh` sources the fresh **enabled** installation. The run script refuses to launch unless all of the following hold:
  - the gates are exactly M55 `True`, M61 `False`, M6.0-D `False`, imported from that installation;
  - the checked-out branch is the recorded `local/m55-s4d-m-*` branch, with no remote ref;
  - the branch differs from `9cc1f68` by exactly the two-line patch, and the worktree is clean.
- **OD8 and OD16.**
  - OD8: a 7200 s watchdog, with ≥ 5400 + 900 s left, checked before the launch and before the first key.
  - OD16: max(5 GiB, 2E + 0.1 GB), checked before the launch and before recording. E = 2 × 30.8 MB/min × the watchdog time left, and that rate is a **planning assumption**.
- **Owned-process cleanup.** As in the shadow run.
- **Restore and watchdog.** `restore_cmd.sh` and `watchdog.sh` were fixed before enabling (`provenance/restore_tool_fixed_before_enabling.txt`).
  - `restore_cmd.sh` runs the **full** restore (not `--verify-only`) against `9cc1f68`, with the recorded enabling ref, then returns to the preparation branch.
  - It then checks the run installation in a fresh environment (`run_install_gates.sh`).
- **Stub tests before enabling: PASS** (`provenance/harness_tests/`).
  - Refusals for installation, gate, branch, scope, watchdog, OD8 and OD16 (L1–L9).
  - Watchdog fire, stand-down and default (W1–W3).
  - Interruption before and after the driver started (I1, I2).
  - Owned cleanup with an unowned decoy that survives (C2).
- **Inherited comment.** Item 1 of `s4d_live_run.sh`'s header comment still reads "SHADOW: no action client", inherited verbatim from the S4c level-2 live script. It is a comment only. The live pre-key checks require DISPATCH and exactly one action client.

## Enabling and B1 build (`enabled/`)
- **Watchdog.** Armed at 05:45:29Z: pid 4069, deadline 07:45:29Z, so the launch had to start by 06:00:29Z.
- **Enabling commit.** `local/m55-s4d-m-20261111T054531Z` was created from `9cc1f68` at 05:45:31Z.
  - Commit `bd4a40e40d8ae5b0595d7c7a755ad40ab40c79cc` applies only `docs/patches/m55_enable_s2.UNAPPLIED.patch` (sha256 `d1dacac5…`): the M55 gate and its test pin, `1 1` each.
  - **Never pushed:** no upstream, no remote-tracking ref, and `ls-remote` shows nothing (`never_pushed_check.txt`). It is preserved in `enabled/enabling_commit.patch` and `restore/enabling_commit.patch`.
- **B1 build.** A fresh `--symlink-install` build in new bases `/root/spiderx_live_enabled_20261111T052817Z`. It ran from a toolchain-only `env -i` shell with the r2 overlay, with `st59site` used for the build only. 8 packages built in 28 s.
- **Enabled tests.** The seven S4c enabled-mode M5.5 files: **245 passed** in 112.2 s. The full historical suite was not rerun; it was not required.
- **Imports.** From the new installation: M55 `True`, M61 `False`, M6.0-D `False`, through its symlinks into `src/`.
  - Library `62f8a100…`.
  - The enabled `library.json` is `917fc4d7…`, identical to the disabled one.
- **Duration.** 239 s in total, recorded in `summary.txt`.
- **Pre-launch checks: PASS** at 05:50:02Z (`provenance/prelaunch_checks.txt`): ament, pluginlib and dlopen resolve the r2 overlay; the gates are as above.

## Run (`run/`, `driver/`, `node/`, `media/`)
**The one launch.**
- The journal recorded "live launch used: YES" at 05:50:08Z, immediately before the guarded run started.
- Launch at 05:50:09Z. Both controllers were active at 05:50:37.4Z, and the node was up in DISPATCH (gate `True`) at 05:51:05Z.
- At the launch, the watchdog had 6920 s left. OD16 then had 21.55 GB free against 14.31 GB required.

**Pre-key checks: PASS** (`run/premotion/`, `run/go.flag`, 05:52:40Z):
- **Libraries.** Server 6683 alone maps the two r2 libraries, with provenance hashes `3a8f2044…` and `06777e14…` and no stock copy.
- **Configuration.** Gain 1.0 (log and parameter), `update_rate` 100, physics step 0.001 s.
- **Ownership.** Both controllers active. Node 6565 is in DISPATCH with the gate `True`, in launch session 6480. It is the action's only client (`/spiderx_locomotion`). There were 0 command publishers and no driver or teleop process.
- **Settling.** 10.08 s of sim, 88 fresh samples: z 0.054500 m, range 9.0 µm, tilt 1.5e-7 rad. Maximum ages: joint states 0.053 s, pose 0.019 s, graph 1.002 s.
- **Screenshot 1** at rest.
- **Budgets.** Before the first key, 6768 s of the watchdog were left. Before recording, 21.55 GB was free against 13.99 GB required.

**Driver.** It ran 05:52:46–06:02:43Z and exited 0 with "script complete".
- **Keys.** `+ a w + ' ' s ' ' w w s h w q`, then the probe's `+ + w q`. Requests before `q`: arm, stop, stop, home.
- **Plan.** Exactly the 111 planned goals, all SUCCEEDED. **0 cancels**, and no fault, refusal or command rejection.
- **Speed change.**
  - `+` 1 came while DISARMED, before arm: `2/3`, 1.90 mm/s.
  - `+` 2 came while goal 2 (L1 1→2) was in flight: `3/3`, 2.34 mm/s. Its confirmation arrived before the write call returned. The scorer attributed it to that write.
  - `level_change` 0→1 occurred in READY before goal 1; 1→2 occurred **in WALKING** between goal 9 and goal 10. Strides were 40 and 60 mm.
- **Transitions** (24):
  - arm;
  - F: STARTING, WALKING, with no stop from goal 1 to goal 54; STOPPING `stop` at goal 54; READY `stopped`;
  - R: the same, from goal 55 to goal 99;
  - X: STOPPING `intent_stop` at goal 104, READY; goal 105, then the switch to goal 106 (6→5); STOPPING `intent_stop`, READY; home: goals 107–110, READY `homed` at boundary 0;
  - Q: goal 111; STOPPING `disarm`, then DISARMED `lease_expired` at boundary 1.
- **Distances.** These are body poses from the bag at the cycle boundaries, in B9's frame.
  - L1 cycle: +40.054 mm (band 30–50 mm).
  - **F leg** (goals 9→54): **+0.30006 m**, in the band [0.285, 0.315] m. This means approximately 0.3 m, never "at least 0.3 m". Cycles F1–F5 were +60.007 to +60.016 mm.
  - **R leg** (goals 54→99): **−0.29985 m**. Cycles R1–R5 were −59.962 to −59.974 mm.
  - X composite cycle (goals 99→110): +60.009 mm.
  - Net from B9 to B99: +0.21 mm, recorded only.
  - Arm-relative travel at the boundaries peaked at about 0.340 m, at goal 54. The 0.40 m stop did not fire.
- **Drift.**
  - Per cycle: \|lateral\| ≤ 0.36 mm, \|yaw\| ≤ 0.00143 rad, \|height\| ≤ 0.03 mm.
  - Cumulative at every boundary 0 (B9…B99): \|lateral\| ≤ 0.56 mm (limit 30 mm) and \|heading\| ≤ 0.0068 rad (limit 0.10 rad).
  - The all-sample maxima include the designed sway and are recorded only: lateral 13.1 mm, heading 0.0068 rad.
  - No per-cycle or cumulative drift stop fired.
- **Tracking.** All 111 goals have feedback, 22–148 samples each and 10,076 in total. The maximum \|error\| was **0.00225 rad**. No goal exceeded the 0.02 rad finding level.
- **Releases.** They were detected 0.822 s (goal 104) and 0.879 s (goal 106) after the recorded release. READY followed in 3.19 s and 4.13 s of sim (bounds 4.2 s and 5.2 s). Home took 10.72 s of sim (bound 12.0 s).
- **Close while moving.**
  - The repeat stopped, then `q` was written while goal 111 was in flight.
  - The node logged disarm, stop, disarm, stop.
  - It reached DISARMED at boundary 1, 1.98 s of sim after the last status before `q` (bound 2.9 s).
  - **Reason: `lease_expired`**, the reason the node-clock evidence predicts:
    - there was no command after the last status;
    - the command count was 5661 both at that status and at the transition;
    - the last command came 4.57 s of node time before DISARMED.
  - Teleop 7632 exited 0 by itself.
- **After the close.** DISARMED with no goal over 10 s of sim (361.74–371.74). Joint motion 2.5e-16 rad, tilt 5.8e-4 rad, z ≥ 0.054998 m.
- **Reconnect probe.**
  - A second teleop (8494) started; `+ +` gave `2/3` then `3/3`, each attributed.
  - While `w` was held, the intent showed forward at level 2, and `commands_accepted` rose from 5661 to 5731.
  - The release returned the intent to 0; `q` followed.
  - The node stayed DISARMED with no goal and logged no arm, goal, transition, refusal or rejection.
  - The close requests were accepted as no-ops: "already disarmed" and "not moving (DISARMED); held until a zero command", × 2 each. Teleop 8494 exited 0 by itself.

## Scorer verdicts (`score/summary.json`; committed `score_s4d.py`, no `--shadow`; run after the restore)
The arguments are in `score/score_provenance.txt`. Alignment uncertainty: 0.84 ms.

| | Verdict | Detail |
|---|---|---|
| M1 | PASS | 111 planned goals in order, all SUCCEEDED; 0 cancels, faults or refusals; keys and pre-`q` requests as scripted; DISPATCH with the gate `True` throughout |
| M2 | PASS | Both `+` presses as scripted and attributed; exactly two `level_change`, 1→2 in WALKING between goals 9 and 10; L1 for goals 1–9, L2 for 10–111 |
| M3 | PASS | L1 cycle +40.054 mm, in the 30–50 mm band; count 1 after goal 9 |
| M4 | PASS | Continuous from goal 1 to 54; F leg +0.30006 m and every F cycle in band; space during goal 54; one STOPPING `stop`; READY at 0; count 6 |
| M5 | PASS | As M4 for goals 55–99: R leg −0.29985 m and every R cycle in band; count 1 |
| M6 | PASS | Cumulative \|lateral\| ≤ 0.56 mm and \|heading\| ≤ 0.0068 rad at every boundary 0; tilt within 0.10 rad |
| M7 | PASS | Both releases: one STOPPING `intent_stop` after the hold window; READY within bound; no extra goal |
| M8 | PASS | Switch while goal 105 was in flight, without release; goal 106 6→5 at the boundary |
| M9 | PASS | 4 forward homing goals; `homed` at 0 in 10.72 s of sim; X cycle +60.009 mm; count 2 |
| M10 | PASS | Close checks as above; `lease_expired` matches the node-clock evidence; teleop exit 0 within 15 s; still over 10 s of sim |
| M11 | PASS | 111 goals tracked; maximum \|error\| 0.00225 rad; no finding |
| M12 | PASS | Maximum continuity error 4.5e-9 rad |
| M13 | PASS | 111 goals, 0 cancels, 0 foreign clients, 0 publishers; controller log 111 received, 111 accepted, 0 cancel requests; every support check true |
| M14 | PASS | Driver complete; minimum joint margin 0.261 rad (`lf_thigh_joint`); maximum node-observed ages: joint states 0.059 s, pose 0.031 s, graph 1.007 s |
| M15 | PASS | Full restore DISABLED AND VERIFIED; `library.json` `917fc4d7…` = the disabled one; the run installation imports all three gates `False` |
| M16 | PASS | Every probe bound |

**Real controller.**
- The JTC log has 111 goals received, 111 accepted and 0 cancel requests.
- The bag has 10,076 `_action/feedback` messages and no `joint_trajectory` message.
- Action clients were `/spiderx_locomotion` only before the run; afterwards `/spiderx_locomotion` and `/rosbag2_recorder`.

**Supplementary** (`analysis/`, not used for the verdict):
- RTF 0.588 overall, with 10 s windows of 0.223–0.714;
- 1234 statuses in the bag, all DISPATCH with the gate `True`;
- maximum node-observed ages over all statuses: joint states 0.062 s, pose 0.031 s, graph 1.007 s;
- CPU 2 min into the driver (`run/captures/top_during.txt`): GUI 180 % at nice 19, server 80 %, node 20 % at nice −5.

## Stop, cleanup and restoration
Each process's outcome is recorded separately; no exit code is inferred.
- **Simulator server (6683): killed by SIGKILL** (strace). strace recorded one SIGINT, from pid 6554: its parent, the `ign gazebo` launcher, which received the launch group's SIGINT. The launch log then shows "Escalating to SIGKILL on [Ignition Gazebo Server]", and strace "+++ killed by SIGKILL +++". This happened after the script completed, the bag was stopped and screenshot 2 was taken.
- **Locomotion node (6565):** "process has finished cleanly", as the launch reports it.
- **Teleops:** 7632 (the run) and 8494 (the probe) each exited 0 by itself, observed by the driver (wait status 0). None was left in their sessions.
- **GUI (6686):** its own exit status was not observed. Its launcher, `ign gazebo-1` (6551), reported "process has finished cleanly".
- **Launch.** The process group was empty 9 s after SIGINT. The ros2 daemon and Xvfb were stopped. The guarded run exited 0 at 06:03:02Z.
- **Cleanup and watchdog.** The trap's owned cleanup found nothing left to stop; no relevant process remained (`run/leftovers_after.txt`). The watchdog stood down at 06:03:21Z without acting.
- **Restore** (`restore/`, 06:03:21–06:03:25Z): the full restore against `9cc1f68` gave **DISABLED AND VERIFIED**.
  - The enabling commit and patch were preserved; `src/` equals the base; the incremental rebuild of the workspace installation succeeded.
  - In a fresh environment, all three gates were `False`, imported from the workspace.
  - Then both M6 `--live` tools refused, exit 3.
  - No simulation process remained.
  - Only after verification was the enabling branch deleted. It was never on the remote.
  - The checkout returned to `claude/spiderx-s4d-preparation` at `9cc1f68`, with a clean worktree.
- **Both installations** (`disabled_state_after.txt`, `run_install/`, `after/`), each from a fresh environment:
  - the workspace installation and the run installation import all three gates `False`, through symlinks into the restored `src/`;
  - each gives library `62f8a100…` with the M55 gate imported `False`, and `library.json` `917fc4d7…`.
- **Bag** (outside the repository; `run/bag_retention.txt`): `bag_0.db3`.
  - sha256 `9320eb43…`, 348,086,272 bytes, 101,644 messages.
  - Unchanged before the restore, after scoring and after the supplementary analysis (`run/bag_hashes.txt`).
  - The original was delivered as a conversation attachment. A durable off-container copy is **unconfirmed**.

## Findings
- **Startup.** `gz_ros2_control` logged six `[ERROR]` lines, 0.5 s apart over 2.5 s: "robot_state_publisher service not available, waiting again". It then logged "connected to service!!", and both controllers loaded and became active. This is not a pre-key criterion. Every listed pre-key check passed on this run's own data.
- **Shutdown.** The simulator server did not exit on SIGINT within the launcher's grace period, and was killed by SIGKILL. The S4d shadow and S4c level-2 live servers exited 0. It occurred after the attempt's last measured step. Its cause is not established.
- **Recording rate.** About 34.5 MB per wall minute: 348.1 MB over 605.1 s of recording. That is above the 30.8 MB/min observed figure, but within the doubled rate that OD16 assumes. Likely contributors, not analysed further: more sim time per wall minute (RTF 0.59, against 0.21 in shadow), and the feedback topic, which is empty in shadow.
- **Inherited stale comment** in `tools/s4d_live_run.sh` (see Harness).

## Limits
- **Cloud simulation only.** Ignition Fortress on the RoboStack toolchain, the r2 `gz_ros2_control` overlay (gain 1.0) and an idealized actuator model. Nothing here was run on the home PC.
- **Automated keyboard path.** The real `m55_teleop_keyboard` was driven from a script through a pseudo-terminal. That is not human operation (S4e).
- **One attempt.** One attempt at one RTF (about 0.59). The 0.3 m legs are "approximately 0.3 m" within ±0.015 m.
- **Not validated here.** S4d-E, S4e, physical keyboard, hardware, navigation and unrestricted walking.

## Contents
`run/` (logs; `launch.log.gz`; `premotion/` with `maps_full.txt.gz`; `captures/`; `bag_retention.txt`), `driver/`
(`driver.jsonl.gz`, `driver.out`, `driver_summary.json`), `node/`, `media/` (two screenshots, both at rest: before the
first key and after the script), `enabled/` (`build.log.gz`), `disabled/` (before enabling), `restore/`
(`rebuild_*.log.gz`), `run_install/`, `after/`, `score/`, `analysis/`, `tools/`, `provenance/` (input, harness, stub-test
and pre-launch records; `operator_steps/` with the post-run, scoring and monitoring commands and their outputs; the run
journal snapshot; `gzip_originals_SHA256.txt` with the hash and size of each original before gzip),
`disabled_state_after.txt`, `SHA256SUMS`.
- **Secret scan.** No credential, key or personal address was found in text files, gzip contents or image metadata.
