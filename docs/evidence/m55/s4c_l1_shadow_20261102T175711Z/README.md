# S4c level-1 shadow rehearsal, 2026-11-02 (dispatch disabled)

**Result: PASS (SHADOW ONLY).** The committed `score_s4c.py --level 1 --shadow` rated C1–C5 and C7–C11 PASS; C6 tracking is N/A in shadow. This was one free-base SHADOW launch at internal level 1 (40 mm stride, teleop label 2/3), with all three gates `False`. **Zero** trajectory goals or cancels reached the real controller.

This is a rehearsal of the committed S4c script, **not** live contact acceptance. Live level 1, level 2 and S4d are not authorized.

This file is an evidence index; the evidence is uncommitted, for review.

## Baseline and inputs (`provenance/`)
- **Correction first.** `069385c` changed only the level-confirmation evidence path:
  - each teleop-output chunk now carries a full-precision receipt time;
  - lines are rebuilt across chunks;
  - each confirmation is attributed to its one `+` write.
  It is the disabled rehearsal baseline: branch `claude/spiderx-s4c-preparation`, clean, no `local/*` branch.
- **Focused tests at HEAD** (`focused_tests_at_head.txt`): S4c driver 26 OK, S4c scorer 30 OK, all of `scripts/s4` 146 OK; flake8 clean. The historical suite was not rerun.
- **Input checks: PASS** (`input_checks.txt`).
  - The r2 overlay: 0.7.15 `bfc5845` plus 0001 `14238400…` and 0002 `520e73cc…`; libraries `3a8f2044…` and `06777e14…`; every source file equals upstream plus both patches.
  - The installed runtime is the S4b live restore's clean rebuild of `2b22e44`, reused. Its 169 runtime files resolve to their HEAD blobs; since that build only a test file has changed, and it is not installed.
  - All three gates `False` on a fresh import. Library: 54 goals, `62f8a100…`; `library_disabled.json` `917fc4d7…`.
  - The S4c driver and scorer are at their HEAD blobs; the tools they import are unchanged.
- **Pre-launch checks: PASS** (`prelaunch_checks.txt`): ament, pluginlib and dlopen resolve the r2 overlay.

## Harness (`tools/`; `provenance/harness_vs_s4b_shadow.diff`, `harness_SHA256SUMS.txt`)
This is the committed S4b shadow harness. The historical copies are verified against their committed hashes (`s4b_shadow_tools_SHA256SUMS.txt`). Changes:
- `scripts/s4/s4c_driver.py --level 1 --shadow-rehearsal`;
- `s4c_driver` added to cleanup, the premotion forbidden list and the input process check;
- the teleop processes left after the driver are recorded before the existing safety-net `pkill`;
- `--verify-only` restore against `069385c`;
- the S4c input checks;
- a supplementary analysis that is never used for the verdict.

As before: source order toolchain, r2 overlay `local_setup`, then the workspace; `fortress_m55_walking.launch.py` starts the node itself; one bag, with no `_action/status` or `/clock`; the watchdog (armed at 18:01:09Z) and the trap, before the launch.

## Run (`run/`)
**Launch and pre-key checks: PASS.**
- **Timeline.** Launch at 18:01:21Z; both controllers active at 18:02:07Z; node in SHADOW (gate `False`) at 18:02:34Z.
- **Libraries.** Server 1717 alone maps the two r2 libraries, with the provenance hashes and no stock copy.
- **Configuration.** Gain 1.0 (log and parameter), 100 Hz, physics step 0.001 s.
- **Ownership.** 0 action clients and 0 command publishers. Node 1653 in the launch session.
- **Settling.** 10.05 s of sim (7.45–17.50), 106 fresh samples: z 0.054500 m, range 9.0 µm, tilt 1.5e-7 rad. Maximum ages: joint states 0.065 s, pose 0.050 s, graph 1.002 s.

**Driver.** It ran 18:04:13–18:09:43Z and exited 0 ("script complete (shadow rehearsal: dispatch disabled; not acceptance)").
- **Selection.** One `+` while DISARMED. The teleop confirmed `speed level 2/3: 1.90 mm/s`:
  - the write ran 996.352454955–996.352475769 s;
  - the line was received at 996.353160779 s, 0.706 ms after the write began and after it returned;
  - it is attributed to that one `+` (C2).
- **Keys.** Exactly `+ a w`, space, `s`, space, `q`; every write result 1.
- **Plan.** Exactly 18 planned shadow goals at level 1, all completed, with **0 cancels**, no fault and no refusal: 0→1 … 8→9, then 9→8 … 1→0.
- **Level.** One `level_change`, 0 → 1, before goal 1.
- **Transitions.** DISARMED→READY `arm`. Each cycle: STARTING, WALKING, STOPPING `stop`, then READY `stopped` at 0. Finally READY→DISARMED `disarm`.
- **Cycles.** Each was checked at a fresh sample before the next key. Shadow displacement: forward −3.5e-10 m, reverse −3.0e-10 m. That is stationary creep; the signed bands are not applied in shadow.
- **Close.** `q` at READY, boundary 0.
  - The node logged disarm, stop, disarm, stop: the teleop's two pairs, with the node handling disarm first.
  - One transition, READY → DISARMED.
  - The teleop (pid 2677) exited 0 by itself. No teleop process remained after the driver.
- **After the close.** DISARMED with no goal over 10 s of sim (74.16–84.16). From the bag: joint motion 5.9e-12 rad, tilt 1.5e-7 rad, z ≥ 0.054495 m.

## Scorer verdicts (`score/summary.json`, committed `score_s4c.py --level 1 --shadow`)

| | Verdict | Detail |
|---|---|---|
| C1 | PASS | The 18 planned goals in order; 0 cancels, faults or refusals; keys and pre-`q` requests as scripted; SHADOW with the gate `False` in every status |
| C2 | PASS | One `+` while DISARMED before arm; its 2/3 confirmation attributed by receipt; one `level_change` 0 → 1; every goal at level 1 |
| C3 | PASS | Space while 8→9 was in flight; one STOPPING `stop`; READY at 0 in 1.16 s sim (≤ 2.2 s); checked before `s` |
| C4 | PASS | Space while 1→0 was in flight; READY at 0 in 1.80 s sim (≤ 2.8 s); checked before `q` |
| C5 | PASS | stop × 2 and disarm × 2, one READY→DISARMED (`disarm`), teleop exit 0; no goal, key or other request after `q`; still over 10 s of sim |
| C6 | N/A | No trajectory runs in shadow |
| C7 | PASS | Maximum continuity error 2.2e-16 rad |
| C8 | PASS | 18 shadow goals, 0 cancels, 0 foreign clients, 0 publishers. The controller log shows 0 received, 0 accepted and 0 cancel requests; the bag has no feedback |
| C9 | PASS | Minimum joint margin 0.436 rad (`rr_foot_joint`). Maximum node-observed ages: joint states 0.071 s, pose 0.030 s, graph 1.006 s. Driver complete |
| C10 | PASS | `--verify-only` restore: DISABLED AND VERIFIED |
| C11 | PASS | `library.json` `917fc4d7…`: the node's equals the disabled one |

- **Recorded only.** Net return −6.2e-10 m. The forward cycle's recorded "within the live stop band" is `False`: −3.1e-10 m is below 0 in a stationary shadow cycle. It is not applied in shadow.
- **Real controller: zero goals and zero cancels.** The bag has 0 `joint_trajectory` and 0 `_action/feedback` messages. The controller's reference changed by 0.0 rad, and the JTC log has 0 goal or cancel lines. Action clients: 0 before; afterwards only `/rosbag2_recorder`.
- **Supplementary analysis** (`analysis/`, not used for the verdict):
  - RTF 0.193, with 10 s windows of 0.169–0.207;
  - maximum node-observed ages over all 683 statuses: joint states 0.085 s, pose 0.030 s, graph 1.006 s.

## Stop and verification
- **Exits.**
  - The simulator server (1717) exited 0; strace recorded no signal to it.
  - `m55_locomotion_node` reported "finished cleanly", as did every launch process.
  - The launch group was empty 10 s after SIGINT. No leftovers; the watchdog stood down without acting.
- **Findings.**
  - During the stop, the Ignition launcher logged "Escalating to SIGKILL on [Ignition Gazebo GUI]". That is the non-essential GUI process, reniced +19, not the server. The line is absent from the S4b shadow log.
  - One ros2 CLI daemon restart happened while the run waited for the controllers (`daemon_events.txt`). That is the observer side, as in earlier runs.
- **Restore** `--verify-only` against `069385c`: **DISABLED AND VERIFIED**. Gates `False` on a fresh import, both M6 `--live` tools refused, no enabling branch (`restore/`, `disabled_state_after.txt`).
- **Bag** (kept outside the repository): `/root/spiderx_evidence/s4c_l1_shadow_20261102T175711Z/bag/bag_0.db3`, sha256 `a2f01471…`, 158,457,856 bytes, unchanged by scoring.
- **Media.** Two screenshots, both at rest: before the first key, and after the script.
- **Compression.** The launch log, the driver log and the full memory maps are gzipped. The secret scan found no credential or address.
