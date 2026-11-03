# S4c level-2 live attempt, 2026-11-03: first contact at internal level 2

**Scorer verdict (`score/summary.json`, committed `score_s4c.py --level 2`, criteria unchanged): PASS, C1–C11.** This was one authorized live attempt on the free-base, gain-1.0, r2 plant at internal level 2 (60 mm stride, teleop label 3/3): **18 goals and 0 cancels**, with no emergency response. There was no level-1 rerun, extra cycle, home, recovery, retry or second launch, and no S4d.

**Builds.** The enabled build and the restore rebuild used the established **incremental** `colcon build --symlink-install`, as the owner authorized for this attempt (`provenance/build_procedure.txt`, recorded before enabling). This is **not** a clean enabled build. `build/`, `install/` and `log/` were not deleted.

This file is an evidence index. The project README and STATUS were not changed. The evidence is uncommitted, for review.

## Baseline and checks before enabling
- **Disabled baseline.** `77d97fc` (pushed), the S4c level-2 shadow-evidence commit. Since `069385c` only `docs/`, README and STATUS changed.
- **Input checks: PASS** (`provenance/input_checks.txt`).
  - The r2 overlay: 0.7.15 `bfc5845` plus 0001 `14238400…` and 0002 `520e73cc…`; libraries `3a8f2044…` and `06777e14…`; every source file equals upstream plus both patches.
  - The installed runtime: 169 symlinks resolve to their HEAD blobs; no generated file changed after its build; no `src/` change since.
  - All three gates `False`. Library: 54 goals, `62f8a100…`.
  - The enabling patch: `d1dacac5…`, applies cleanly, exactly the gate and its pin.
  - The driver (`d0ff8acb…`), the scorer (`fe1b5c03…`) and their imports equal the level-2 shadow run's and the level-1 runs'.
- **Disabled validation reused, not rerun.** The restoration follow-up's fresh disabled build and full suite (`d5a6692`, at `0d84ebd`): 1972 tests, 0 failures. `src/`, `scripts/` and `third_party/` are unchanged since.
- **Gates, then refusals** (`disabled/`).
  - Fresh import: all three gates `False`, imported through the workspace installation into `src/`; the config resolves to `src/`.
  - Disabled `library.json`: `917fc4d7…`.
  - Only then were both M6 `--live` tools checked: both refused, exit 3, DISABLED AND VERIFIED (`disabled/m6_refusals/`).

## Enabling (local only)
- **Branch and commit.** `local/m55-s4c-l2-20261103T112215Z`, commit `78501f9`. It changes exactly two lines: the M55 gate and its test pin. M61 and M6.0-D stay `False`.
- **Never pushed.** No push was run for it.
  - At creation it had no upstream and no remote-tracking ref. That check was printed to the console and not saved.
  - After the restore, `git ls-remote` found no `local/*` ref on the remote (`disabled_state_after.txt`).
  - It was deleted only after the restore verified; its commit and diff are kept in `enabled/` and `restore/`.
- **Watchdog.** Armed at 11:22:15Z, before enabling (`provenance/watchdog_armed.txt`).
- **Enabled build** (`enabled/`): incremental, exit 0. Imports M55 `True`, M61 and M6D `False`, through the installation into `src/`. 245 M5.5 tests passed. The enabled `library.json` equals the disabled one (`917fc4d7…`).

## Harness (`tools/`; `provenance/harness_vs_s4c_l1_live.diff`, `harness_vs_s4c_l2_shadow.diff`, `harness_SHA256SUMS.txt`)
This is the S4c level-1 live harness with the level-2 shadow input checks. The historical copies match their committed hashes (`historical_copies_check.txt`). Changes:
- `scripts/s4/s4c_driver.py --level 2`, in its default LIVE mode;
- the full restore to `77d97fc`, with the same incremental rebuild and `--delete-enabling-branch`, fixed before enabling and unchanged after the launch (`restore/restore_tool_check.txt`);
- the input checks: baseline `77d97fc`, the reused build, the patch, and the level-2 shadow evidence;
- comments only elsewhere. The disabled-build tools are not used, and no deletion command remains.

Source order: toolchain, r2 overlay `local_setup`, then the workspace. `fortress_m55_walking.launch.py` starts the node itself. One bag, without `_action/status` or `/clock`.

## Run (`run/`)
- **Timeline.** Launch at 11:28:22Z; both controllers active at 11:28:56Z; node in DISPATCH mode (gate `True`) at 11:29:48Z.
- **Startup.** `gz_ros2_control` connected to `robot_state_publisher` on its first attempt: no wait line and no `[ERROR]` line (`run/premotion/startup_lines.txt`).
- **Pre-arm checks: PASS** (`run/premotion/`).
  - Server 2759 maps only the r2 libraries, with the provenance hashes and no stock copy.
  - Gain 1.0, 100 Hz, physics step 0.001 s. Both controllers active.
  - Node 2687 is the action's only client; 0 command publishers.
  - Settling: 10.03 s of sim, 126 fresh samples, z 0.054500 m, range 9.0 µm, tilt 1.5e-7 rad. Ages: joint states 0.085 s, pose 0.047 s, graph 1.039 s.
- **Driver** (default LIVE mode). It ran 11:31:34–11:35:31Z and exited 0 with "script complete".
  - Keys: exactly `+ + a w`, space, `s`, space, `q`; every write result 1.
  - **Selection.** Both `+` while DISARMED, before arm. Each confirmation was received after its own write returned and before the next key's write began:
    - `+` 1, written 1136.269826998–1136.269869422 s: `speed level 2/3: 1.90 mm/s`, received at 1136.270121670 s, 0.295 ms after the write began;
    - `+` 2, written 1136.320223803–1136.320252567 s: `speed level 3/3: 2.34 mm/s`, received at 1136.320755975 s, 0.532 ms after the write began;
    - the next key, `a`, began at 1136.370637886 s.
  - **Level.** The node armed at level 0 and made one `level_change`, 0 → 2, with goal 1. Every goal was at level 2.
  - **Goals.** 0→1 … 8→9, then 9→8 … 1→0: 18 goals, all SUCCEEDED, with 0 cancels and no fault.
  - **Cycles.** Each stopped at its boundary by space, with READY `stopped` at 0. Each was checked at a fresh sample before the next key.
- **Close.** `q` at READY at 0.
  - The teleop issued stop, then disarm, in its `q` handler, and stop, then disarm, again in its shutdown sequence. These are asynchronous calls to two separate services.
  - The node recorded disarm, stop, disarm, stop: the same two pairs, in a different order.
  - The frozen scorer compares the requests after `q` as counts (stop × 2, disarm × 2), not as an order (C5).
  - One transition, READY → DISARMED (`disarm`).
  - The teleop (pid 3846) exited 0 by itself. No teleop process remained.

## Scoring, C1–C11 (`score/`)

| | Verdict | Detail |
|---|---|---|
| C1 | PASS | The 18 planned goals in order, all SUCCEEDED; 0 cancels, faults or refusals; keys and pre-`q` requests (arm, stop, stop) as scripted; DISPATCH with the gate `True` in every status |
| C2 | PASS | Two `+` while DISARMED before arm; 2/3 at 1.90 mm/s, then 3/3 at 2.34 mm/s, each attributed by full-precision receipt to its own write; one `level_change` 0 → 2 before goal 1; every goal at level 2; node stride 0.06 m |
| C3 | PASS | **Forward +60.015 mm** (band [50, 70]); lateral −0.044 mm, yaw −1.39e-3 rad, height −0.05 µm, tilt ≤ 0.0069 rad. Space while 8→9 was in flight; READY in 1.18 s sim (≤ 2.2 s) |
| C4 | PASS | **Reverse −59.959 mm** (band [−70, −50]), measured from the end of the forward cycle; lateral 0.116 mm, yaw 1.34e-3 rad, height +4.9 µm, tilt ≤ 0.0070 rad. READY in 1.81 s sim (≤ 2.9 s) |
| C5 | PASS | stop × 2 and disarm × 2, one READY→DISARMED, teleop exit 0; no goal, key or other request after `q`. Still over 10 s of sim (85.11–95.11): joint motion 5.9e-12 rad, tilt 1.6e-5 rad, z ≥ 0.054495 m |
| C6 | PASS | Feedback for every goal, with no N/A: 26–183 samples each, 2145 messages, none unattributed. Maximum error 0.00225 rad; no finding above 0.02 rad |
| C7 | PASS | Maximum continuity error 4.5e-9 rad |
| C8 | PASS | 18 goals, 0 cancels, 0 foreign clients, 0 command publishers; the controller log shows 18 received, 18 accepted and 0 cancel requests |
| C9 | PASS | Minimum joint margin 0.261 rad (`lf_thigh_joint`). Maximum node-observed ages: joint states 0.103 s, pose 0.022 s, graph 1.009 s. Driver complete |
| C10 | PASS | Restore DISABLED AND VERIFIED: gates `False`, both M6 `--live` tools exit 3, no leftovers |
| C11 | PASS | `library.json` `917fc4d7…`: enabled equals disabled |

- **Net return error** (recorded only; an exact return is not assumed): +0.056 mm forward, −0.011 mm lateral, −4.4e-5 rad yaw.
- **Supplementary analysis** (`analysis/`, read-only; not used for the verdict):
  - RTF 0.306, with 10 s windows of 0.135–0.600;
  - maximum node-observed ages over all 496 statuses: joint states 0.103 s, pose 0.043 s, graph 1.009 s;
  - action clients: before, only `/spiderx_locomotion`; after, it and `/rosbag2_recorder`;
  - the launch log has no `[ERROR]` line and no escalation.

## Stop and process outcomes
Each process's outcome is recorded separately; no exit code is inferred.
- **Simulator server (2759): exit 0, observed by strace.** strace recorded one SIGINT, sent by pid 2675: the server's parent, the `ign gazebo` launcher, which received the launch group's SIGINT. That is the same pattern as the level-2 shadow run. No SIGTERM or SIGKILL.
- **Locomotion node (2687):** "process has finished cleanly", as the launch reports it.
- **Teleop (3846):** exit 0 by itself, observed by the driver.
- **GUI (2764):** its own exit status was not observed. The launch log has no escalation line, and its launcher process (`ign gazebo-1`, 2673) finished cleanly.
- **Launch.** Every launch process finished cleanly. The launch group was empty 5 s after SIGINT. No leftovers; the watchdog stood down without acting. No ros2 CLI daemon restart.

## Restore and final state
- **Full restore to `77d97fc`** (`restore/`). The enabling commit and diff were saved, `src/` was checked against the base, and the toolchain rebuild was incremental. Gates `False` on a fresh import; the package prefix is this workspace; both M6 `--live` tools refused (exit 3). The verdict is DISABLED AND VERIFIED, after which the enabling branch was deleted.
- **Final state** (`disabled_state_after.txt`): back on branch `claude/spiderx-s4c-preparation` at `77d97fc`, clean tree, all three gates `False` imported through the installation, no `local/*` branch locally or on the remote, no simulation, node, driver, bag or watchdog process.
- **Bag** (kept outside the repository): `/root/spiderx_evidence/s4c_l2_live_20261103T111733Z/bag/bag_0.db3`, sha256 `fe87666e…`, 120,213,504 bytes, unchanged by scoring (opened read-only).
- **Media.** Two screenshots, both at rest: before the first key, and after the script. They are not proof of motion.
- **Compression.** The enabled build and restore rebuild logs, the launch log, the driver log and the full memory map are gzipped. The secret scan found no credential. The only address is `noreply@anthropic.com`, in the local enabling commit's author line and trailer. The screenshots carry no text chunks.
