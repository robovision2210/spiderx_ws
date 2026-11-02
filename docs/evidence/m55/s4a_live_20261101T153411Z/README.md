# S4a live attempt, 2026-11-01: reverse first contact at level 0

**Scorer verdict (`score/summary.json`, committed `score_s4a.py`, criteria unchanged): PASS, A1–A8.** This was one authorized live attempt on the free-base, gain-1.0, r2 plant.

This file is an evidence index. The project README and STATUS were not changed. The evidence is uncommitted, for review.

## Baseline and checks before enabling
- **Disabled baseline.** `e5899f4` (pushed), the S4a shadow-evidence commit. Its robotics source equals `7c1ecbc`.
- **Input checks** (`provenance/input_checks.txt`):
  - the r2 overlay: 0.7.15 `bfc5845` plus 0001 `14238400…` and 0002 `520e73cc…`, with libraries `3a8f2044…` and `06777e14…`;
  - the gates, all `False`;
  - the library: 54 goals, `62f8a100…`;
  - the enabling patch: `d1dacac5…`, applies cleanly, and changes exactly the gate and its pin;
  - the S4a shadow evidence matches the current inputs.
- **Clean disabled build and full suite, run once** (`disabled/`):
  - 1952 tests (1902 xunit + 50 CTest), with 0 errors, failures or skips;
  - fresh collection: PASS (50/50 targets, none stale);
  - direct tests: `spiderx_scripts` 2 passed, `scripts/s3` 69 OK, `scripts/s4` 48 OK;
  - disabled `library.json`: `917fc4d7…`.

## Enabling (local only)
- **Branch and commit.** `local/m55-s4a-20261101T154813Z`, commit `5469069`. It changes only the M55 gate and its test pin; M61 and M6.0-D stay `False`.
- **Never pushed.** The branch had no upstream and no remote ref. It was deleted only after the restore verified.
- **Watchdog.** Armed before enabling (`provenance/watchdog_armed.txt`).
- **Enabled build** (`enabled/`):
  - imports M55 `True`, M61 and M6D `False`;
  - 245 M5.5 tests passed;
  - the library digest and `library.json` (`917fc4d7…`) are the same as the disabled build's.

## Harness (`tools/`; `provenance/harness_vs_s4a_shadow.diff`, `harness_SHA256SUMS.txt`)
The S4a shadow harness, with only the live changes:
- the S3 live DISPATCH and M55-`True` checks, verbatim;
- `s4a_driver.py` without `--shadow-rehearsal`;
- the full restore to `e5899f4` with `--delete-enabling-branch`.

Source order: toolchain, r2 overlay `local_setup`, workspace. The launch is `fortress_m55_walking.launch.py`, which starts the node itself. One bag, without `_action/status` or `/clock`.

## Run (`run/`)
- **Timeline.** Launch at 15:52:45Z. Controllers active at 15:53:01Z. Node up in DISPATCH mode (gate True) at 15:53:51Z.
- **Pre-arm checks: PASS** (`run/premotion/`).
  - Server 29834 maps only the r2 libraries, with the provenance hashes.
  - Gain 1.0, 100 Hz, physics step 0.001 s.
  - Both controllers active. The node is the action's only client; 0 command publishers.
  - Settling: 10.04 s of sim, z 0.054500 m, range 9.0 µm, tilt 1.5e-7 rad, fresh streams, sim advancing.
- **Driver** (default LIVE mode). Go at 15:55:27Z; the driver ran 15:55:33–15:58:30Z and exited 0 with "script complete".
  - Goals 1–9 were reverse (9→8 … 1→0) and goals 10–18 the forward return (0→1 … 8→9). Goal 19 (0→1) was cancelled by the planned estop.
  - Totals: 19 goals and 1 cancel. The last script step was `q` after the hold.
  - Each cycle was checked at a fresh sample before the next direction key.
- **Process exits.**
  - The simulator server exited 0. strace recorded no signal to it; the S3 and shadow runs each recorded one SIGINT.
  - `m55_locomotion_node` "finished cleanly". The launch group was empty after 4 s, with no forced escalation.
  - No leftovers. The watchdog stood down without acting.

## Scoring, A1–A8 (`score/`)

| | Verdict | Detail |
|---|---|---|
| A1 | PASS | All 18 normal goals SUCCEEDED, at level 0, reverse then forward; READY at boundary 0 after each run; no unplanned fault |
| A2 | PASS | Reverse −19.992 mm (band [−30, −10]); return +20.024 mm (band [10, 30]). Lateral +14.0 / −9.7 µm; yaw +3.6e-5 / −3.8e-5 rad; height +1.5 / −6.2 µm; tilt ≤ 0.0026 rad |
| A3 | PASS | 18 goals with feedback (≥ 51 samples each), maximum error 0.00225 rad, no finding. Goal 19 is N/A: the estop proof qualifies (17/17 checks). `x` was written 5214.275012–5214.275046 (result 1), and the node faulted 0.963 ms after the write start |
| A4 | PASS | Maximum continuity error 4.6e-9 rad |
| A5 | PASS | 19 goals, 1 cancel, 0 foreign clients, 0 command publishers |
| A6 | PASS | Measured hold, sim 76.97–86.97 s: joint motion 5.9e-12 rad, tilt 2.5e-6 rad, z ≥ 0.054495 m |
| A7 | PASS | Restore DISABLED AND VERIFIED |
| A8 | PASS | `library.json` `917fc4d7…`: enabled equals disabled |

- **Net displacement after the return** (recorded only): +0.031 mm forward, 3.6 µm lateral, −1.9e-6 rad yaw.
- **Supplementary analysis** (`analysis/`, read-only):
  - RTF 0.337, with 10 s windows of 0.195–0.638;
  - maximum node-observed age: joint states 0.075 s, pose 0.024 s;
  - minimum joint margin 0.354 rad;
  - the JTC log shows 19 goals received and accepted, and 1 cancel.

## Restore and final state
- **Full restore to `e5899f4`** (`restore/`): clean rebuild; gates `False` on a fresh import; both M6 `--live` tools refused (exit 3). The verdict is DISABLED AND VERIFIED, after which the enabling branch was deleted.
- **Final state** (`disabled_state_after.txt`): branch `claude/spiderx-s4a-preparation` at `e5899f4`, clean tree.
- **Bag** (kept outside the repository): `/root/spiderx_evidence/s4a_live_20261101T153411Z/bag/bag_0.db3`, sha256 `45a8fc78…`, 93,208,576 bytes.
- **Media.** Two screenshots, both at rest: before the first key, and after the script. They are not proof of motion.
