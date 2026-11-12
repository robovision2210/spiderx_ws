# S4c level-1 live attempt, 2026-11-02: first contact at internal level 1

**Scorer verdict (`score/summary.json`, committed `score_s4c.py --level 1`, criteria unchanged): PASS, C1–C11.** This was one authorized live attempt on the free-base, gain-1.0, r2 plant at internal level 1 (40 mm stride, teleop label 2/3): **18 goals and 0 cancels**. Level 2 and S4d were not run.

This file is an evidence index. The project README and STATUS were not changed. The evidence is uncommitted, for review.

## Baseline and checks before enabling
- **Disabled baseline.** `ea69d3f` (pushed), the S4c level-1 shadow-evidence commit. Its robotics source and tools equal `069385c`.
- **Input checks: PASS** (`provenance/input_checks.txt`).
  - The r2 overlay: 0.7.15 `bfc5845` plus 0001 `14238400…` and 0002 `520e73cc…`; libraries `3a8f2044…` and `06777e14…`.
  - All three gates `False`. Library: 54 goals, `62f8a100…`.
  - The enabling patch: `d1dacac5…`, applies cleanly, exactly the gate and its pin.
  - The driver, scorer and their imports equal the level-1 shadow run's.
- **Clean disabled build and full suite, run once** (`disabled/`):
  - 1972 tests (1922 xunit + 50 CTest), with 0 errors, failures or skips;
  - fresh collection: PASS;
  - direct tests: `spiderx_scripts` 2 passed, `scripts/s3` 86 OK (all three test files), `scripts/s4` 146 OK;
  - disabled `library.json`: `917fc4d7…`.
- **Gates, then refusals.** All three gates were `False` on a fresh import. Only then were both M6 `--live` tools checked: both refused, exit 3 (`disabled/m6_refusals/`).

## Enabling (local only)
- **Branch and commit.** `local/m55-s4c-l1-20261102T183944Z`, commit `2d499a8`. It changes exactly two lines: the M55 gate and its test pin. M61 and M6.0-D stay `False`.
- **Never pushed.** The branch had no upstream and no remote ref. It was deleted only after the restore verified.
- **Watchdog.** Armed at 18:39:46Z, before enabling (`provenance/watchdog_armed.txt`).
- **Enabled build** (`enabled/`): imports M55 `True`, M61 and M6D `False`; 245 M5.5 tests passed; enabled `library.json` equals disabled (`917fc4d7…`).
- **Deviation: incremental builds.** A built-in safety check stopped the clean-build deletion of `build/` and `install/` in this session. So:
  - the enabled build and the restore rebuild are incremental `colcon build --symlink-install` runs, with st59site confined to the build;
  - the enabling diff is two Python lines, installed as symlinks into `src/`;
  - `restore_cmd.sh` was changed before the launch (`provenance/harness_change_before_launch.txt`);
  - the clean deletion is left to the owner.

## Harness (`tools/`; `provenance/harness_vs_s4c_shadow.diff`, `harness_vs_s4b_live.diff`, `harness_SHA256SUMS.txt`)
This is the S4c level-1 shadow harness with the tested S4b live components:
- the enabled-gate pre-launch checks;
- the DISPATCH pre-arm checks: the node must be the action's sole client;
- the full restore with `--delete-enabling-branch`;
- the live input checks;
- `scripts/s4/s4c_driver.py --level 1` without `--shadow-rehearsal`.

The historical copies match their committed hashes. Source order: toolchain, r2 overlay `local_setup`, then the workspace. `fortress_m55_walking.launch.py` starts the node itself. One bag, without `_action/status` or `/clock`.

## Run (`run/`)
- **Timeline.** Launch at 18:46:03Z; both controllers active at 18:46:18Z; node in DISPATCH mode (gate `True`) at 18:47:11Z.
- **Pre-arm checks: PASS** (`run/premotion/`).
  - Server 12887 maps only the r2 libraries, with the provenance hashes and no stock copy.
  - Gain 1.0, 100 Hz, physics step 0.001 s. Both controllers active.
  - Node 12798 is the action's only client; 0 command publishers.
  - Settling: 10.01 s of sim, 102 samples, z 0.054500 m, range 9.0 µm, tilt 1.5e-7 rad. Ages: joint states 0.084 s, pose 0.022 s, graph 1.007 s.
- **Driver** (default LIVE mode). It ran 18:48:45–18:51:54Z and exited 0 with "script complete".
  - Keys: exactly `+ a w`, space, `s`, space, `q`; every write result 1.
  - **Selection.** The teleop confirmed `speed level 2/3: 1.90 mm/s`, received 1.66 ms after the `+` write began, after it returned; attributed to that one key.
  - **Level.** The node was at level 0 until goal 1. It made one `level_change`, 0 → 1, with goal 1, and every goal was at level 1.
  - **Goals.** 0→1 … 8→9, then 9→8 … 1→0: 18 goals, all SUCCEEDED, with 0 cancels and no fault.
  - **Cycles.** Each stopped at its boundary by space, with READY `stopped` at 0. Each was checked at a fresh sample before the next key.
- **Close.** `q` at READY at 0.
  - The node logged disarm, stop, disarm, stop: the teleop's two pairs.
  - One transition, READY → DISARMED (`disarm`).
  - The teleop (pid 13847) exited 0 by itself. No teleop process remained.
- **Process exits.**
  - The simulator server (12887) exited 0. strace recorded no signal to it.
  - `m55_locomotion_node` "finished cleanly", as did every launch process.
  - The launch group was empty 4 s after SIGINT. No leftovers; the watchdog stood down without acting.
  - **GUI.** The launch log has no SIGKILL escalation this time (the shadow run had one). The GUI's own exit status is not observed separately.

## Scoring, C1–C11 (`score/`)

| | Verdict | Detail |
|---|---|---|
| C1 | PASS | The 18 planned goals in order, all SUCCEEDED; 0 cancels, faults or refusals; keys and pre-`q` requests (arm, stop, stop) as scripted; DISPATCH with the gate `True` in every status |
| C2 | PASS | One `+` while DISARMED before arm; 2/3 at 1.90 mm/s, attributed by full-precision receipt; one `level_change` 0 → 1 before goal 1; every goal at level 1 |
| C3 | PASS | **Forward +40.052 mm** (band [30, 50]); lateral 0.372 mm, yaw −1.8e-4 rad, height −4 µm, tilt ≤ 0.0050 rad. Space while 8→9 was in flight; READY in 1.04 s sim (≤ 2.2 s) |
| C4 | PASS | **Reverse −39.967 mm** (band [−50, −30]), measured from the end of the forward cycle; lateral 0.297 mm, yaw −1.8e-4 rad, height 0.0 mm, tilt ≤ 0.0050 rad. READY in 1.66 s sim (≤ 2.8 s) |
| C5 | PASS | stop × 2 and disarm × 2, one READY→DISARMED, teleop exit 0; no goal, key or other request after `q`. Still over 10 s of sim (79.45–89.45): joint motion 5.9e-12 rad, tilt 5.5e-6 rad, z ≥ 0.054495 m |
| C6 | PASS | Feedback for every goal, with no N/A: 38–107 samples each, 1450 messages, none unattributed. Maximum error 0.00225 rad; no finding above 0.02 rad |
| C7 | PASS | Maximum continuity error 4.5e-9 rad |
| C8 | PASS | 18 goals, 0 cancels, 0 foreign clients, 0 command publishers; the controller log shows 18 received, 18 accepted and 0 cancel requests |
| C9 | PASS | Minimum joint margin 0.343 rad (`lf_thigh_joint`). Maximum node-observed ages: joint states 0.058 s, pose 0.018 s, graph 1.006 s. Driver complete |
| C10 | PASS | Restore DISABLED AND VERIFIED: gates `False`, both M6 `--live` tools exit 3, no leftovers |
| C11 | PASS | `library.json` `917fc4d7…`: enabled equals disabled |

- **Net return error** (recorded only; an exact return is not assumed): +0.085 mm forward, 0.661 mm lateral, −3.6e-4 rad yaw.
- **Supplementary analysis** (`analysis/`, read-only; not used for the verdict):
  - RTF 0.336, with 10 s windows of 0.170–0.662;
  - maximum node-observed ages over all 402 statuses: joint states 0.089 s, pose 0.036 s, graph 1.006 s;
  - action clients: before, only `/spiderx_locomotion`; after, it and `/rosbag2_recorder`;
  - the launch log has no `[ERROR]` line.

## Restore and final state
- **Full restore to `ea69d3f`** (`restore/`): an incremental toolchain rebuild (see the deviation above); gates `False` on a fresh import; both M6 `--live` tools refused (exit 3). The verdict is DISABLED AND VERIFIED, after which the enabling branch was deleted.
- **Final state** (`disabled_state_after.txt`): branch `claude/spiderx-s4c-preparation` at `ea69d3f`, clean tree, no `local/*` branch or remote ref, no simulation, node, driver, bag or watchdog process.
- **Bag** (kept outside the repository): `/root/spiderx_evidence/s4c_l1_live_20261102T182303Z/bag/bag_0.db3`, sha256 `ef760383…`, 100,065,280 bytes, unchanged by scoring (opened read-only).
- **Media.** Two screenshots, both at rest: before the first key, and after the script. They are not proof of motion.
- **Compression.** Build, rebuild and launch logs, the driver log and the full memory maps are gzipped. The secret scan found no credential; the only address is the `noreply` commit trailer of the enabling commit.
