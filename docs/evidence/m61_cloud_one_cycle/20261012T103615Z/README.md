# M6.1 one fixed-base trot cycle in Cloud, third authorization: run `20261012T103615Z`

**Verdict: PASS. 1 goal, 0 cancels.** The run ended `SUCCEEDED`, exit 0, and E1–E11 all PASS
(§3).
- The goal was accepted. It ran the full 7.0 s cycle and ended with status 4 (SUCCEEDED), error 0.
  The controller's message was "Goal successfully reached!".
- Tracking:
  - the client's max in-flight |error| was 0.0085 rad against the 0.05 rad limit;
  - the independent tracking channel passed over 501 samples;
  - goal tolerances passed;
  - no gate tripped.
- There was no retry, no second goal and no return goal. Cleanup ended
  `DISABLED AND VERIFIED`, twice.
- **The action feedback was recorded this time.** All 214 messages carried
  `desired.time_from_start = 0`. The controller started the trajectory 0.037 s before the client
  processed the acceptance (§2). The `b0d8e75` reference therefore anchored at acceptance, used the
  recorded fallback, and tracked correctly.

What this run does **not** show:
- walking, balance or navigation;
- contact or ground interaction: the body was welded at 0.125 m and the feet never touched the
  ground;
- anything about hardware;
- any free-base or M5.5 behaviour.

It shows one thing: one approved joint trajectory was dispatched once, then tracked and supervised
on a welded simulated body, in Cloud.

The authorization is in `authorization.txt`. It is new; the `a1a7d60` (refused, 0 goals) and
`24782f6` (TRACKING_FAILED, 1 goal, 1 cancel) attempts are spent and their FAIL records stand. The
plan is [`docs/M61_CLOUD_ONE_CYCLE_PLAN.md`](../../../M61_CLOUD_ONE_CYCLE_PLAN.md) at `d748891`.
The report is [`docs/M61A_CLOUD_VERIFICATION.md`](../../../M61A_CLOUD_VERIFICATION.md) §15.

## 1. Sequence (UTC, 2026-10-12)

| Step | Result | Record |
|---|---|---|
| 0. Prior use | None. No `local/m61-one-cycle*` branch, locally or on the remote. No cycle evidence newer than `m61_tracking_fix/20261012T081545Z`. robovision2210/spiderx_ws#17 has 0 comments and 0 reviews, and is still draft | this README |
| A. Base and identity | `HEAD` = origin = `d74889128719a184c3967e61ba7ba591e7310788`, clean, no commit after it. `src/` equals `b0d8e757ab23ba8031619abdd8b69fee927d91f2` (0 diff lines, no untracked file). Patch SHA-256 `b6213f21dedf30e61e4959d22cd45a44525a8bc4432b641134406fe7f3277595` = the plan = the second cycle's record; `git apply --check` OK. Link check `649b0c4f…9e31` and restore script `e6fe9ee1…8257` as frozen. 6 config, launch and model hashes identical to the second cycle's record. Tracking probe `c2397497…5c4c` = its `SHA256SUMS`. Gates `False`. No simulation process. 14 frozen harness files byte-identical (`pre/A_harness_check.txt`) | `pre/A_checks.txt` |
| A. Gate-off tools | Clean rebuild. Dry run PASS: `241760e7dfd5ef12` / `94a492c43fcc046125d6bc71ee7f1d9a8a5d8466f1a1bdd9958a16044dd58e64` / `9dba1a173e212bfc172ffcd7da59124f87e98a321906e914e9d60e55595e5c3a`. Mocks: `success` SUCCEEDED; `attachment_drift` GATE_TRIPPED, 1 cancel; `late_action_server` SUCCEEDED; `feedback_tfs_zero` SUCCEEDED; `tracking_error_tfs_zero` TRACKING_FAILED, 1 cancel. `--live` exit 3 | `pre/A_tool_checks.txt`, `pre/identities.txt`, `pre/dryrun/`, `pre/mock/` |
| A2. Disabled build, 10:37–10:41 | Preflight READY. 3 rehearsals, each with bag and observer first: **READY 3 of 3**. Discovery 0.50, 0.25 and 1.50 s; 0 goals; 0 dispatch calls. Tracking-path probe: **PASS, 10 of 10 checks**. Live `time_from_start` 0 in 1,158 messages; anchor at acceptance; tracked from 3.0 s; the non-moving robot tripped at 3.63 s (predicted 3.624 s). Clean stop. `--verify-only`: **DISABLED AND VERIFIED** | `a2/`, `a2_restore/`, `a2_restore_verify.out` |
| B. Enabling | `local/m61-one-cycle-20261012T104206Z` = `87eac0c2f681ee894c430a356498e3ed90348689`: the 2-line patch, with no upstream, local only | `enabling_*` |
| C. Enabled build, no simulator | Gate `True`, pin `True`, M6.0-D `False`; imported from this workspace; identity unchanged. `test_m61_gait_replay.py` 166 and `test_m61_tracking.py` 49: **215 passed, 0 failed** | `C_enabled_*`, `enabled_tests.log` |
| D. Launch, 10:43:26 | Cold start, no `create` death, so no relaunch. Controllers active 10:43:38. Preflight **READY 12/12** | `run/launch.log`, `run/preflight.txt` |
| E. Bag, observer, then 10 s | The bag recorded `_action/feedback` with `--include-hidden-topics`; `_action/status` was never recorded. Observer at 10:44:04.9, tool at 10:44:14.9 | `run/e_start.out`, `run/bag_record.log`, `run/*_utc.txt` |
| E. Readiness #1 | Discovery found the server at the first poll. READY, `compatible`, age 4.11 s | `M/readiness_before.json` |
| E. Confirmation, 10:44:20.6 | The prompt was present with no refusal or error, so the word was written **once** | `run/confirm_decision.txt`, `run/live_at_decision.txt` |
| E. Readiness #2 and the send | READY, age 4.01 s. At the send: age 4.19 s ≤ 10 s. `streams_now` ok: joint states 0.007 s, pose 0.0003 s, sim advance 0.641 s in 1.0 s, plant and attachment ok | `M/readiness_after.json`, `M/live_outcome.json` |
| E. Goal | Sent at session t 9.69 s, accepted at 9.74 s (sim 36.687). The fallback event was logged once at 9.79 s. Result status 4, error 0, at 20.47 s. Independent tracking passed: `SUCCEEDED`, exit 0 at 10:44:38 | `run/live.txt`, `M/live_outcome.json` `events` |
| F. Records | The 90 s observer ended READY. Bag stopped. Captures: 0 action clients, 0 command publishers. 4 GUI views, taken after the run. Stop: process group empty after 3 s, nothing left. Frozen link check: E8 and E9 PASS. Feedback analysis (§2) | `run/captures/`, `run/shutdown.txt`, `run/link_check/`, `run/feedback_anchor_analysis.*` |
| G. Restore | Run 1: preserve, restore `d748891`, rebuild, verify: **DISABLED AND VERIFIED**. Run 2: verify, then delete the branch: **DISABLED AND VERIFIED** | `restore/`, `restore_run1.out`, `restore_run2.out` |

`M` = `run/m61_run/20261012T104415Z/`.

## 2. The recorded action feedback against the acceptance anchor

The analysis is `tools/feedback_anchor_analysis.py` → `run/feedback_anchor_analysis.{json,txt}`. It
is read-only and offline, and uses the tool's own `m61_trot_cycle.reference_positions`. It fits on
a 1 ms grid, ±0.5 s, over reference time 3.0–7.0 s.

| Item | Value |
|---|---|
| Action feedback messages (one goal id) | 214 |
| `desired.time_from_start` | **0 in 214 of 214**; none positive |
| Controller start, from the feedback's desired positions | **acceptance − 0.037 s** (rms 6e-16 rad: an exact fit; 119 samples) |
| Controller start, from `controller_state.reference` | acceptance − 0.038 s (rms 1.2e-4 rad; 170 samples) |
| Recorded `/joint_states` best fit | acceptance + 0.061 s, about 0.1 s behind the setpoint (rms 0.0012 rad; 400 samples). This is physical lag and is not compensated |
| Joints vs trajectory timed from acceptance | max 0.0085 rad over 501 samples, the same as the tool |
| Joints vs trajectory timed from the unfiltered M6.0-D estimate (acceptance + 3.42 s) | max **0.0835 rad**. The pre-fix rule would have cancelled again |
| The controller's own reported error | at most 0.0135 rad |

- **This closes the link inferred in Cloud report §14.3.** This controller (JTC 2.48.0) leaves the
  action feedback's `time_from_start` at 0, and the acceptance anchor is within 0.04 s of its real
  start.
- The offset was −0.015 s in the second cycle and −0.037 s here. Both are within the unchanged
  0.1 s limit.

## 3. E1–E11 (plan §6). Missing evidence is UNMEASURED, never PASS

`SUCCEEDED` alone is not scored as a pass. Each requirement is checked on its own record.

| # | Result | Basis | Evidence |
|---|---|---|---|
| E1 | **PASS** | Preflight READY 12/12, incl. `sim_clock` and `body_pose_sim_progress`. 0 command publishers, 0 action clients | `run/preflight.txt`, `run/observer/20261012T104346Z/` |
| E2 | **PASS** | Enabled build: 215 passed, 0 failed (`test_m61_gait_replay.py` 166, `test_m61_tracking.py` 49) | `enabled_tests.log`, `C_enabled_checks.txt` |
| E3 | **PASS** | Enabled dry run and live records: `241760e7dfd5ef12` / `94a492c4…8e64` / `9dba1a17…5c3a` | `C_enabled_checks.txt`, `M/trajectory.json`, `M/goal_fingerprint.txt` |
| E4 | **PASS** | `SUCCEEDED`, exit 0. `goals_sent` 1, `cancels_sent` 0, retries 0, automatic returns 0. Acceptance `accepted`; final goal status 4, error 0. Final evidence phase written | `run/live.txt`, `M/live_outcome.json` |
| E5 | **PASS** | Discovery `found` at the first poll, within the 10 s bound, and the observation started after it. READY before (age 4.11 s) and after (4.01 s) confirmation, with `with_sim_progress` ok. At the send: permitted, age 4.19 s ≤ 10 s, `streams_now` ok. Complete per-check reports recorded | `M/readiness_before.json`, `M/readiness_after.json`, `M/live_outcome.json` |
| E6 | **PASS** | Max in-flight \|error\| **0.0085 rad** < 0.05. Tracking channel `passed`: independent check over 501 samples, no uncovered interval, worst `lr_thigh_joint` at 6.37 s. Goal tolerances `passed`. No G6 gap. `tracking_reference`: anchor `acceptance`, t0 = acceptance (36.687 s). The `time_from_start = 0` fallback was used and recorded: 214 feedback messages, 0 informative, logged once, 697 in-flight checks on it. The limit is unchanged at 0.05 rad | `M/live_outcome.json` (`tracking`, `tracking_reference`, `channels`), `M/commanded_vs_observed.csv`, `run/feedback_anchor_analysis.json` |
| E7 | **PASS** | No gate tripped (`trips_in_order` empty). G1 z 0.125 m; G2 tilt 0; G3 worst 16.5 % of limit (`rf_foot_joint`); G5, G6, G8 and pose freshness not tripped. G7 report-only, no flag | `M/gates.json` |
| E8 | **PASS** | Measured from the physics state. Link check E8 body: deviation 0 m / 0 rad over 5,442 pose messages. Gates: 690 body samples, z 0.125 m, tilt 0, attachment 0. Observer attachment 0. Provenance PASS: 4,755 steady samples, 1,329 discriminating, residual vs FK(q) ≤ 1.0e-7 m / 1.3e-6 rad. **Margin note:** the cycle ended at neutral, so the steady samples sit near q = 0 (gravity sag about 1e-4 rad). The separation from FK(0) is therefore small: residual vs FK(0) ≥ 9.6e-5 rad, about 10× the frozen 1e-5 rad tolerance. In the second cycle the robot stopped mid-cycle, which gave 0.078 rad | `run/link_check/`, `M/gates.json`, `M/body_pose.csv`, `run/observer/20261012T104537Z/` |
| E9 | **PASS** | Frozen check (`649b0c4f…`): all 5,442 samples explained by FK(q(t)). The foot links turned 0.080–0.111 rad (≥ 0.02). No gap over the limit. The frozen-stream counterfactual scored 22.1 (> 1 fails) | `run/link_check/link_check.json` |
| E10 | **PASS** | Command-owner layer 0/0 in both readiness observations. Observers 0/0 at preflight and over the 90 s window. The recorder never subscribed to `/leg_trajectory_controller/joint_trajectory`, because no publisher appeared. After the run: 0 action clients, 0 command publishers | `M/readiness_*.json`, `run/observer/*/`, `run/bag_record.log`, `run/captures/` |
| E11 | **PASS** | Restore runs 1 and 2: DISABLED AND VERIFIED (imported gates `False` from this workspace, pkg prefix here, `--live` exit 3, shell clean). The enabling commit was preserved before deletion (src diff SHA-256 `8604d06b…fff3`). The branch was deleted only after run 2 and never pushed | `restore/`, `restore_run1.out`, `restore_run2.out`, `run/shutdown.txt` |

Machine-readable: `requirements.json`.

## 4. Preserved

- **Bag.** `run/bag.tar.xz`; unpack with `tar -xJf`. The originals are `57ef8f62…c19b`
  (`bag_0.db3`) and `ec8fa1c4…b9fe` (`metadata.yaml`), and the round trip was verified identical.
  Message counts:

  | Topic | Messages |
  |---|---|
  | `/clock` | 65,857 |
  | `/joint_states` | 6,603 |
  | `/spiderx/sim/world_poses` | 5,442 |
  | `controller_state` | 2,726 |
  | `_action/feedback` | 214 |
  | `/robot_description` | 1 |

  The command topic was never recorded: it had no publisher.
- **Images.** 4 GUI screenshots in `run/captures/`, taken after the run with the robot back at
  neutral. There is no video: the harness has no recorder.
- **Not copied:** the raw bag directory (archived above), the pytest basetemp of step C, and the
  confirmation FIFO. The A2 rehearsal bags were removed by their script; their metadata is kept.
- **Redactions.** The session scratchpad prefix is `<scratchpad>` in `restore_run1.out` and
  `restore/restore_verify_20261012T104804Z.txt`.
- **Paths.** Absolute paths `/root/spiderx_evidence/m61_one_cycle_20261012T103615Z/` refer to this
  folder. `M/git_state.txt` reads "not a git repository" because the tool ran from the run
  directory, as in the second cycle.
- **`tools/`** holds:
  - the 14 frozen harness files (byte-identical, plus `selftest/`);
  - this run's scripts: `a2_run.sh` (the second cycle's version plus the probe step),
    `a2_rehearse.sh` (unchanged), `e_start.sh` (the second cycle's version with the plan's bag
    change only), `e_confirm.sh` (unchanged) and `feedback_anchor_analysis.py`.
- **Manifest.** `SHA256SUMS` covers every file except itself.
