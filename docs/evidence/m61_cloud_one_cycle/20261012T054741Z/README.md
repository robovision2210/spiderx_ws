# M6.1 one fixed-base trot cycle in Cloud, second authorization: run `20261012T054741Z`

**Verdict: FAIL. 1 goal, 1 cancel.** The run ended `TRACKING_FAILED`, exit 1.
- The goal was accepted. It was cancelled once, about 6.2 s into the 7.0 s cycle, by the
  client's in-flight tracking check: the tool recorded |error| 0.0805 rad against its 0.05 rad
  limit. The final status was CANCELED.
- There was no retry, no second goal and no return goal. Cleanup ended `DISABLED AND VERIFIED`,
  twice.
- **The cancel came from a defect in the client's reference timing, not from measured tracking
  failure** (§2). Against the approved trajectory timed from goal acceptance, the joints tracked
  within 0.0108 rad. The controller's own reported error stayed at or below 0.0126 rad.
- The acceptance criteria are scored as written: E4 and E6 FAIL.

What this run does **not** show: walking, balance, contact, navigation or anything about hardware.
The body was welded.

The authorization is in `authorization.txt`. It is a new one; the `a1a7d60` attempt was spent and
was refused with 0 goals. The plan is [`docs/M61_CLOUD_ONE_CYCLE_PLAN.md`](../../../M61_CLOUD_ONE_CYCLE_PLAN.md)
at `dcac464`. The report is [`docs/M61A_CLOUD_VERIFICATION.md`](../../../M61A_CLOUD_VERIFICATION.md) §13.

## 1. Sequence (UTC, 2026-10-12)

| Step | Result | Record |
|---|---|---|
| 0. Prior use | None: no `local/*` branch, locally or on the remote; no cycle evidence newer than the rehearsal; no PR robovision2210/spiderx_ws#17 comments | this README |
| A. Base and identity | `HEAD` = origin = `dcac464c898f1b8b57b3e0659abcb2d84123615e`, clean. `src/` equals `02d284c1a64941d501e0b429d39fad0871365b4b`. Patch SHA-256 `b6213f21dedf30e61e4959d22cd45a44525a8bc4432b641134406fe7f3277595`, matching the plan, and `git apply --check` OK. Link check `649b0c4f…9e31`, restore script `e6fe9ee1…8257`, 6 config, launch and model hashes = plan. Gates `False`. 14 frozen harness files byte-identical (`pre/A_harness_check.txt`) | `pre/A_checks.txt` |
| A. Gate-off tools | Clean rebuild. Dry run PASS: `241760e7dfd5ef12`, `94a492c43fcc046125d6bc71ee7f1d9a8a5d8466f1a1bdd9958a16044dd58e64`, `9dba1a173e212bfc172ffcd7da59124f87e98a321906e914e9d60e55595e5c3a`. Mocks: `success` SUCCEEDED; `attachment_drift` GATE_TRIPPED, 1 cancel; `late_action_server` SUCCEEDED. `--live` exit 3 | `pre/A_tool_checks.txt`, `pre/dryrun/`, `pre/mock/` |
| A2. Rehearsal, disabled build, 05:48–05:52 | Preflight READY. 3 rehearsals, each with bag and observer first and then 10 s: **READY 3 of 3**. Discovery 2.01, 0.75 and 0.00 s; 0 goals; 0 dispatch calls. Clean stop. `--verify-only`: **DISABLED AND VERIFIED** | `a2/`, `a2_restore/`, `a2_restore_verify.out` |
| B. Enabling | `local/m61-one-cycle-20261012T055225Z` = `79ffdb2d0a64f1d21540945bedd170a14b58e9df`: the 2-line patch, local only | `enabling_*` |
| C. Enabled build, no simulator | Gate `True`, pin `True`, M6.0-D `False`; identity unchanged. `test_m61_gait_replay.py` and `test_m61_discovery.py`: **222 passed, 0 failed** | `C_enabled_*`, `enabled_tests.log` |
| D. Launch, 05:54:27 | Cold start: one world request, no `create` death, so no relaunch. Controllers active 05:54:38. Preflight **READY 12/12** | `run/launch.log`, `run/preflight.txt` |
| E. Bag, observer, then 10 s | Observer at 05:54:55.8, tool at 05:55:05.8 | `run/e_start.out`, `run/*_utc.txt` |
| E. Readiness #1 | Discovery found the server at the first poll (0.0 s). READY, `compatible`, age 4.03 s | `M/readiness_before.json` |
| E. Confirmation, 05:55:11.5 | The prompt was present with no refusal or error, so the word was written **once** | `run/confirm_decision.txt`, `run/live_at_decision.txt` |
| E. Readiness #2 and the send | READY, age 4.10 s. At the send: age 4.36 s ≤ 10 s; `streams_now` ok (joint states 0.002 s, pose 0.010 s, sim advance 0.535 s in 1.0 s, plant and attachment ok) | `M/readiness_after.json`, `M/live_outcome.json` |
| E. Goal | Sent at session t 10.11 s, accepted at 10.16 s (sim 29.814). **Cancel requested at 19.85 s** (`tracking_error`). Cancel response 0; result status 5 (CANCELED). `TRACKING_FAILED`, exit 1 at 05:55:27 | `run/live.txt`, `M/live_outcome.json` `events` |
| F. Records | Observer ended READY. Bag stopped. Captures and 4 GUI views (after the run). Stop: group empty after 4 s, nothing left. Frozen link check | `run/captures/`, `run/shutdown.txt`, `run/link_check/` |
| G. Restore | Run 1: preserve, restore `dcac464`, rebuild, verify: **DISABLED AND VERIFIED**. Run 2: verify, then delete the branch: **DISABLED AND VERIFIED** | `restore/`, `restore_run1.out`, `restore_run2.out` |

`M` = `run/m61_run/20261012T055506Z/`.

## 2. Why the tracking check cancelled: the client's reference timing

**The mechanism.** The tool tracks against the approved trajectory and needs the trajectory's start
time.
- `m6_live_playback.LiveSession._ref_time` (M6.0-D, reused by M6.1) starts from goal acceptance.
  It switches to a "feedback start estimate", the median of `stamp − desired.time_from_start` over
  the action feedback, whenever that estimate differs from acceptance by more than 0.1 s
  (`FEEDBACK_OFFSET_LIMIT_S`).
- This controller reports `desired.time_from_start = 0` in every message of the goal window
  (`run/tracking_reference_analysis.json`: 255 of 255 `controller_state` samples).
- So the "estimate" is just the median feedback stamp. It grows to about half the elapsed time:
  33.030 at the cancel, 3.216 s after acceptance at 29.814.
- The trajectory's tracked part starts at `time_from_start` 3.0 s, after the 3.0 s lead-in. With
  the shifted start, the client's reference time reached 3.0 only at sim 36.03, which is 6.2 s
  into the goal.
- On that first tracked sample, the reference was the neutral first waypoint while the legs were
  mid-cycle (thighs ±0.077 rad). The check cancelled.

**What the joints actually did.** `tools/tracking_reference_analysis.py`, read-only and offline.
It uses the tool's own `m61_trot_cycle.reference_positions`.

| Reference start | Samples tracked | Max \|error\| |
|---|---|---|
| Goal acceptance (sim 29.814) | 322 (trajectory time 3.006–6.216 s) | **0.0108 rad** |
| The tool's feedback estimate (sim 33.030) | 1 | 0.0768 rad, the cancel |

The controller's own reported error (`controller_state`) was at most **0.0126 rad**.

**Conclusion.** The cancel is a false positive of the client-side tracking reference against a
controller that leaves `desired.time_from_start` at 0. It does not show that the controller failed
to follow the trajectory. It also does not show success:
- the last 0.8 s of the cycle was cancelled;
- goal tolerances were never evaluated;
- E4 and E6 are failed as written.

The isolated test double used by the M6.0-D tests fills `time_from_start`, which is why no test
caught this.

## 3. E1–E11 (plan §6). Missing evidence is UNMEASURED, never PASS

`M` = `run/m61_run/20261012T055506Z/`.

| # | Result | Basis | Evidence |
|---|---|---|---|
| E1 | **PASS** | Preflight READY 12/12 (incl. `sim_clock`, `body_pose_sim_progress`); 0 command publishers, 0 action clients | `run/preflight.txt`, `run/observer/20261012T055446Z/` |
| E2 | **PASS** | Enabled build: 222 passed, 0 failed (`test_m61_gait_replay.py` 164, `test_m61_discovery.py` 58) | `enabled_tests.log`, `C_enabled_checks.txt` |
| E3 | **PASS** | Enabled dry run and live records: `241760e7dfd5ef12` / `94a492c4…8e64` / `9dba1a17…5c3a` | `C_enabled_checks.txt`, `M/trajectory.json`, `M/goal_fingerprint.txt` |
| E4 | **FAIL** | `TRACKING_FAILED`, exit 1. `goals_sent` 1, `cancels_sent` 1, retries 0, automatic returns 0. Accepted; final status 5 (CANCELED), not 4 | `run/live.txt`, `M/live_outcome.json` |
| E5 | **PASS** | Discovery `found` within the bound (0.0 s), and the observation started after it. READY before (age 4.03 s) and after (4.10 s) confirmation. At the send: permitted, age 4.36 s ≤ 10 s, `streams_now` ok. Complete per-check reports recorded | `M/readiness_before.json`, `M/readiness_after.json`, `M/live_outcome.json` |
| E6 | **FAIL** | As recorded: max in-flight \|error\| 0.0805 rad > 0.05; tracking channel `failed`; goal tolerances `unavailable` (cancelled). No G6 gap. The diagnosis in §2 (0.0108 rad from acceptance) explains the failure; it does not change the score | `M/live_outcome.json` `tracking`, `channels`; `M/commanded_vs_observed.csv`; `run/tracking_reference_analysis.json` |
| E7 | **PASS** (executed portion) | No gate tripped (`trips_in_order` empty) over the executed 6.2 s and the settle: G1 worst z 0.125 m; G2 tilt 0; G3 worst 16.5 % of limit; G8 attachment 0; pose freshness and G5/G6 not tripped. G7 report-only, no flag. The last 0.8 s of the cycle never ran | `M/gates.json` |
| E8 | **PASS** | Measured from the physics state during the goal. Link check E8 body: deviation 0 m / 0 rad over 5,487 pose messages. Provenance PASS: 4,710 of 4,710 samples discriminating, FK(q) vs FK(0) up to 0.078 rad, residual ≤ 1.3e-7 m / 1.3e-6 rad. Gates: 549 body samples, z 0.125 m, tilt 0, attachment 0. Observer: 5,061 samples, attachment 0 | `run/link_check/`, `M/gates.json`, `run/observer/20261012T055629Z/` |
| E9 | **PASS** | Frozen check (`649b0c4f…`): every sample explained by FK(q(t)); the entries changed; the frozen-stream counterfactual scored 22.1 (> 1 fails) | `run/link_check/link_check.json` |
| E10 | **PASS** | Command-owner layer 0/0 in both readiness observations. Observer 0 command publishers and 0 action clients before and after. The recorder never subscribed to `/leg_trajectory_controller/joint_trajectory`, because no publisher appeared. After the run: 0 publishers, 0 action clients | `M/readiness_*.json`, `run/observer/*/`, `run/bag_record.log`, `run/captures/` |
| E11 | **PASS** | Restore runs 1 and 2: DISABLED AND VERIFIED (imported gates `False` from this workspace, pkg prefix here, `--live` exit 3, shell clean). The enabling commit was preserved before deletion, and the branch deleted only after run 2 | `restore/`, `restore_run1.out`, `restore_run2.out`, `run/shutdown.txt` |

Machine-readable: `requirements.json`.

## 4. Preserved

- **Bag.** `run/bag.tar.xz` (`tar -xJf`; originals `62d69885…a9e` and `9bb7e280…dc`; round trip
  verified identical). Counts: `/clock` 65,943; `/joint_states` 6,410;
  `/spiderx/sim/world_poses` 5,487; `controller_state` 2,643; `/robot_description` 1. The command
  topic was never recorded: it had no publisher.
- **Images.** 4 GUI screenshots in `run/captures/`, taken after the run with the robot back at
  rest. There is no video: the harness has no recorder.
- **Not copied:** the pytest basetemp of step C, the confirmation FIFO, and the A2 rehearsal bags
  (their metadata is kept).
- **Redactions.** The session scratchpad prefix is `<scratchpad>` in `restore_run1.out` and
  `restore/restore_verify_20261012T055824Z.txt`.
- **Paths.** Absolute paths `/root/spiderx_evidence/m61_one_cycle_20261012T054741Z/` refer to this
  folder.
- **Harness.** `tools/` holds the 14 frozen harness files (byte-identical, plus `selftest/`) and
  the scripts of this run: `a2_run.sh`, `a2_rehearse.sh`, `e_start.sh`, `e_confirm.sh`,
  `tracking_reference_analysis.py`.
- **Manifest.** `SHA256SUMS` covers every file except itself.
