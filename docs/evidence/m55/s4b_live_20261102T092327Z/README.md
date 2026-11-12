# S4b live attempt, 2026-11-02: release, reversal, home and lease loss at level 0

**Scorer verdict (`score/summary.json`, committed `score_s4b.py`, criteria unchanged): PASS, B1–B11.** This was one authorized live attempt on the free-base, gain-1.0, r2 plant: the exact committed 12-goal script, with **12 goals and 0 cancels**.

This file is an evidence index. The project README and STATUS were not changed. The evidence is uncommitted, for review.

## Baseline and checks before enabling
- **Disabled baseline.** `2b22e44` (pushed), the S4b shadow-evidence commit. Its robotics source equals `e406590`: no file outside the shadow evidence changed.
- **Input checks** (`provenance/input_checks.txt`):
  - the r2 overlay: 0.7.15 `bfc5845` plus 0001 `14238400…` and 0002 `520e73cc…`, with libraries `3a8f2044…` and `06777e14…`;
  - the gates, all `False`;
  - the library: 54 goals, `62f8a100…`;
  - the enabling patch: `d1dacac5…`, applies cleanly, and changes exactly the gate and its pin;
  - the driver and scorer equal the shadow run's (`s4b_driver.py` `f0e68c64…`, `score_s4b.py` `a9716d52…`).
- **Clean disabled build and full suite, run once** (`disabled/`):
  - 1959 tests (1909 xunit + 50 CTest), with 0 errors, failures or skips;
  - fresh collection: PASS;
  - direct tests: `spiderx_scripts` 2 passed, `scripts/s3` 69 OK, `scripts/s4` 90 OK;
  - disabled `library.json`: `917fc4d7…`.
- **Gates, then refusals.** All three gates were `False` on a fresh import. Only then were both M6 `--live` tools checked: both refused (`disabled/m6_refusals/`, `--verify-only`, DISABLED AND VERIFIED).

## Enabling (local only)
- **Branch and commit.** `local/m55-s4b-20261102T093832Z`, commit `0b30636`. It changes exactly two lines: the M55 gate and its test pin. M61 and M6.0-D stay `False`.
- **Never pushed.** The branch had no upstream and no remote ref. It was deleted only after the restore verified.
- **Watchdog.** Armed at 09:38:33Z, before enabling (`provenance/watchdog_armed.txt`).
- **Enabled build** (`enabled/`):
  - imports M55 `True`, M61 and M6D `False`;
  - 245 M5.5 tests passed;
  - the library digest and `library.json` (`917fc4d7…`) are the same as the disabled build's.

## Harness (`tools/`; `provenance/harness_vs_s4a_live.diff`, `harness_vs_s4b_shadow.diff`, `harness_SHA256SUMS.txt`)
This is the S4a live harness (the S4b shadow harness with the tested S4a live checks). Apart from comments and the run root, it changes only:
- `scripts/s4/s4b_driver.py` without `--shadow-rehearsal`;
- `s4b_driver` added to cleanup and to the premotion forbidden list;
- the full restore to `2b22e44` with `--delete-enabling-branch`.

Source order: toolchain, r2 overlay `local_setup`, workspace. The launch is `fortress_m55_walking.launch.py`, which starts the node itself. One bag, without `_action/status` or `/clock`.

`tools/analyze_s4b_live_supplementary.py` was added after the run, for the supplementary analysis only (`analysis/analyze_live_vs_shadow.diff`). It is not in `harness_SHA256SUMS.txt`.

## Run (`run/`)
- **Timeline.** Launch at 09:43:31Z. Both controllers active at 09:43:47Z. Node in DISPATCH mode (gate `True`) at 09:44:41Z.
- **Pre-arm checks: PASS** (`run/premotion/`).
  - Server 10909 maps only the r2 libraries, with the provenance hashes.
  - Gain 1.0, 100 Hz, physics step 0.001 s.
  - Both controllers active. The node is the action's only client; 0 command publishers.
  - Settling: 10.01 s of sim (10.12–20.13), 108 fresh samples, z 0.054500 m, range 9.0 µm, tilt 1.5e-7 rad. Maximum ages: joint states 0.060 s (limit 0.5, unchanged), pose 0.031 s, graph 1.012 s.
- **Driver** (default LIVE mode). It ran 09:46:13–09:48:17Z and exited 0 with "script complete".
  - Keys: exactly `a w w s h w`, every write result 1. No space, `x`, `q`, `r` or `z`. The only requests were arm and home.
  - The 12 planned goals, in order: 0→1 … 4→5; 5→6, then 6→5 (the same phase back); 5→6 … 8→9; 0→1. All SUCCEEDED, with no fault and no refusal.
  - The last exercise was the lease test: a teleop SIGKILL while goal 12 (0→1) was in flight. After it there was no key, request, goal, teleop restart or re-arm.
- **Transitions** (`node/events.jsonl`): DISARMED→READY `arm`; a release, STOPPING `intent_stop` then READY `stopped` at 5, twice; READY `homed` at 0; STOPPING `lease_expired` then DISARMED `lease_expired` at 1.
- **Process exits.**
  - The simulator server exited 0. strace recorded no signal to it.
  - `m55_locomotion_node` "finished cleanly". All launch processes finished cleanly; none died.
  - The launch group was empty 4 s after SIGINT, with no forced escalation. No leftovers. The watchdog stood down at 09:48:43Z without acting.

## Scoring, B1–B11 (`score/`)

| | Verdict | Detail |
|---|---|---|
| B1 | PASS | Exactly the 12 planned goals, level 0, all SUCCEEDED; 0 cancels, 0 faults; one goal per seq; keys and requests as scripted |
| B2 | PASS | Release 1 (goal 5, no space): detection 0.889 s after the release (≥ 0.717 s; documented ≤ 0.983 s, no finding). READY `stopped` at 5 in 2.82 s sim (≤ 3.8 s) |
| B3 | PASS | `s` written 0.200 s after the repeat stop, while 5→6 was in flight; 6→5 sent at that boundary with no stop between. Release 2 (goal 7): detection 0.818 s; READY at 5 in 2.50 s sim (≤ 3.5 s) |
| B4 | PASS | `h` at READY 5; one forward homing of 4 phases; READY `homed` at 0 in 8.23 s sim (≤ 12.0 s) |
| B5 | PASS | SIGKILL delivered (0.14 ms bracket) while 0→1 was in flight. STOPPING `lease_expired` 0.466 s later (≥ 0.35 s; documented ≤ 0.55 s, no finding). DISARMED at 1 in 1.79 s sim (≤ 2.7 s). Then 10 s of sim (62.38–72.38) DISARMED with no goal: joint motion 8.9e-15 rad, tilt 7.6e-4 rad (limit 0.10), z ≥ 0.054989 m |
| B6 | PASS | Composite cycle: +20.026 mm forward (band [10, 30]); lateral 14.6 µm; yaw −5.6e-5 rad; height −7.0 µm; tilt ≤ 0.0026 rad |
| B7 | PASS | Feedback for every goal (50–104 samples each; 923 messages, none unattributed). Maximum error 0.00224 rad; no finding above 0.02 rad |
| B8 | PASS | Maximum continuity error 4.6e-9 rad; no continuity refusal |
| B9 | PASS | 12 goals, 0 cancels, 0 foreign clients, 0 command publishers; the JTC log shows 12 received and 12 accepted, 0 cancel |
| B10 | PASS | Restore DISABLED AND VERIFIED: gates `False` imported, both M6 `--live` tools exit 3, no leftovers |
| B11 | PASS | `library.json` `917fc4d7…`: enabled equals disabled |

- **Recorded only.** Goal 12 (0→1, `homed` to DISARMED) moved the body 19.9 mm forward and 16.6 mm laterally. It is outside the cycle band by design.
- **Supplementary analysis** (`analysis/`, read-only; not used for the verdict):
  - RTF 0.395, with 10 s windows of 0.173–0.662;
  - maximum node-observed age over all 278 statuses: joint states 0.083 s (margin 0.417 s), pose 0.026 s, graph 1.003 s;
  - minimum joint margin 0.354 rad (`rf_foot_joint`);
  - action clients: before, only `/spiderx_locomotion`; after, it and `/rosbag2_recorder`.
- **Findings.**
  - The S4b shadow's pre-arm joint-state age of 0.434 s did not recur. The maximum was 0.083 s, including pre-arm.
  - One startup `[ERROR]` line at 09:43:36Z: `gz_ros2_control` waited for the `robot_state_publisher` service. It connected 0.23 s later, before the controllers were active. The line is absent from the S4b shadow and S4a live logs.

## Restore and final state
- **Full restore to `2b22e44`** (`restore/`): clean rebuild; gates `False` on a fresh import; both M6 `--live` tools refused (exit 3). The verdict is DISABLED AND VERIFIED, after which the enabling branch was deleted.
- **Final state** (`disabled_state_after.txt`): branch `claude/spiderx-s4b-preparation` at `2b22e44`, clean tree, no `local/*` branch or remote ref, no simulation, node, driver, bag or watchdog process.
- **Bag** (kept outside the repository): `/root/spiderx_evidence/s4b_live_20261102T092327Z/bag/bag_0.db3`, sha256 `af3aa4e5…`, 68,108,288 bytes, unchanged by scoring (opened read-only).
- **Media.** Two screenshots, both at rest: before the first key, and after the script. They are not proof of motion.
- **Compression.** Build, rebuild and launch logs, the driver log and the full memory maps are gzipped. The secret scan found no credential; the only address is the `noreply` commit trailer of the enabling commit.
