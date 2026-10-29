# S2 fixed-base crawl, one authorized attempt (Cloud, 2026-10-29): FAIL

The attempt stopped on the authorized tracking condition during goal 1.
- **Sent:** 1 phase goal and 1 cancel.
- **Plant:** the welded body (weld at 0.125 m, identity spawn).
- **Gates:** only the M5.5 gate was enabled, on a local branch, and it is restored. The M6.1 and
  M6.0-D gates stayed False throughout.
- **Claims:** none about walking or balance.

## What happened

| Item | Value |
|---|---|
| Base | `f4f00b2` on `claude/spiderx-m55-keyboard-walking`, `src/` = `ec7f122` |
| Enabling | `local/m55-s2-20261029T102100Z` = `5403dbf…` (local only; never pushed; deleted after verification). Patch SHA-256 `d1dacac5…20236`; exactly 2 lines (gate + pin). Identity preserved in `restore/` |
| Driver | `scripts/s2/s2_driver.py` at `95b00f0` (copy in `tools/`, identical) |
| Launch | controllers active; Gazebo GUI reniced +19; preflight READY; node up in DISPATCH mode after 77 s; RTF 0.70 |
| Script | arm (READY at boundary 0), hold `w`: goal 1 = level 0, forward, shift b0→b1, 1.7 s sim |
| Stop | at about 0.93 s sim into goal 1, status tracking error **0.02072 rad > 0.02** (driver stop condition). The driver pressed `x` 1.43 s wall after `w`: estop, node FAULTED (`estop`), **1 cancel**, goal 1 result CANCELED (5) |
| Last scripted step | goal 1 of the level 0 forward cycle (no cycle completed) |

## Acceptance (design §4)

| # | Verdict | Evidence |
|---|---|---|
| P1 runs complete, all SUCCEEDED | **FAIL**: 1 of 54 cycle goals sent, none completed | `summary.json`, `node/events.jsonl` |
| P2 tracking ≤ 0.02 rad | **FAIL**: max feedback \|error\| 0.02097 rad (`rr_thigh_joint`; the node aborts at 0.05) | `tracking.csv`, `goal1_tracking_profile.txt` |
| P3 continuity ≤ 0.005 rad | PASS (1 goal): 9.7e-5 rad | `tracking.csv` |
| P4 ownership and counts | **FAIL** on counts: 1 goal, not 59, and 1 cancel. Ownership clean: 0 foreign clients, 0 command-topic publishers, 0 `joint_trajectory` messages (the topic was never published) | `summary.json`, `run/after_node_info.txt` |
| P5 stop, lease, estop | **UNMEASURED** for stop and lease (not reached). Estop: FAULTED with exactly 1 cancel, goal CANCELED | `node/events.jsonl` |
| P6 welded (E8 rule) | PASS: 847 physics-state body poses, deviation 0 m / 0 rad | `summary.json` |
| P7 library identity | PASS: `library.json` SHA-256 identical, disabled vs enabled (`917fc4d7…`). The 54-fingerprint digest is identical in both builds (`62f8a100…e732`) | `node/library.json`, `disabled/` and `enabled/library_fingerprint.json` |
| P8 restore | PASS: DISABLED AND VERIFIED | `restore/restore_verify_20261029T102818Z.txt` |

## Finding: the 0.02 rad limit sits at the controller's lag × the crawl's peak speed

In goal 1 the error grew in proportion to the commanded joint speed: an apparent lag of
0.05–0.10 s, reaching 0.021 rad at 0.22 rad/s (`goal1_tracking_profile.txt`).
- 0.22 rad/s is the crawl's validated peak, under a 0.25 rad/s bound.
- M6.1 measured a similar physical lag of about 0.06–0.1 s.
- So a correct run is expected to cross 0.02 rad on its first fast phase, while staying well below
  the node's 0.05 rad abort. Nothing unsafe happened.
- No threshold, gain or gait was changed.

## Prerequisites run in this session

- **Disabled clean build, full suite once:** 1929 tests, 0 errors, 0 failures, 0 skipped.
  - The `spiderx_scripts` package's 2 tests were not run by colcon. With setuptools 59.6 scoped
    to the build only, colcon's test step reads `setup.py` with the environment's newer
    setuptools, ignores `tests_require`, and falls back to unittest ("Ran 0 tests").
  - Run directly with pytest: 2 passed, so 1931 tests in all.
- **Gates:** imported in a fresh environment, all three False, from `src/` of this workspace.
  `--live` exits 3 with HARD-DISABLED for both M6 tools.
- **Driver bench** (stdlib, in-memory): 8 of 8 pass.
- **Disabled real-graph estop-service check:**
  - the driver's `/spiderx/locomotion/estop` fallback found the service, and the call succeeded;
  - the shadow node went DISARMED → FAULTED;
  - 0 goals, 0 cancels, 0 action feedback, joints unmoved (1e-19 rad);
  - verify-only restore: DISABLED AND VERIFIED.
- **Enabled build:** imported gates M55 True, M61 False, M6D False. The M5.5 tests: 241 passed,
  0 failed.
- **Toolchain differences (disclosed):** Python 3.11.17 (3.11.16 before) and pytest 9.2.0 (9.1.1
  before). Gazebo 6.16.0 and every pinned ROS package matched.

**Resolved document conflicts:**
- The driver is `scripts/s2/s2_driver.py`.
- The restore base is the branch (`f4f00b2`), not `--base ec7f122`, which would detach HEAD.
  `src/` was verified equal to `ec7f122` separately.
- The restore rebuild uses the recorded `/opt/mm/st59site` build path.
- No relaunch was needed.

**Not in the repository:** the bag, under `~/spiderx_evidence/s2_run_20261029T100210Z/bag`
(`run/bag_metadata.yaml` is copied here).
