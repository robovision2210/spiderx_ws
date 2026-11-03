# S4c level-2 shadow rehearsal, 2026-11-03 (dispatch disabled)

**Result: PASS (SHADOW ONLY).** The committed `score_s4c.py --level 2 --shadow` rated C1–C5 and C7–C11 PASS; C6 tracking is N/A in shadow. This was one free-base SHADOW launch at internal level 2 (60 mm stride, teleop label 3/3), with all three gates `False`. **Zero** trajectory goals or cancels reached the real controller.

This is a rehearsal of the committed S4c script, **not** live level-2 acceptance. No 60 mm displacement is expected or claimed in shadow. Live level 2 and S4d are not authorized.

This file is an evidence index; the evidence is uncommitted, for review.

## Baseline and inputs (`provenance/`)
- **Baseline.** Branch `claude/spiderx-s4c-preparation` at `0cb2ccc`, clean, no `local/*` branch. Since `069385c` (the level-1 shadow baseline) only evidence and documentation changed.
- **Container.** The session's container restarted before this run. Its files persisted, and every input below was re-verified.
- **No rebuild, no test rerun.** The recorded fresh disabled build and suite are reused: restoration follow-up `d5a6692`, run at `0d84ebd`, 1972 tests with 0 failures, both installation checks PASS. `src/`, `scripts/` and `third_party/` are unchanged since that build.
- **Input checks: PASS** (`input_checks.txt`).
  - The r2 overlay: 0.7.15 `bfc5845` plus 0001 `14238400…` and 0002 `520e73cc…`; libraries `3a8f2044…` and `06777e14…`; every source file equals upstream plus both patches.
  - The installed runtime is the workspace installation: the level-1 live restore's incremental rebuild of `ea69d3f`, which the follow-up verified disabled. Its 169 runtime symlinks resolve to their HEAD blobs; no generated file changed after the build; no `src/` file changed since.
  - All three gates `False` on a fresh import. Library: 54 goals, `62f8a100…`; `library_disabled.json` `917fc4d7…`, identical to the level-1 shadow's.
  - The S4c driver (`d0ff8acb…`) and scorer (`fe1b5c03…`), and the tools they import, equal those of the level-1 shadow and live runs. The three committed level-1 evidence manifests verify.
- **Pre-launch checks: PASS** (`prelaunch_checks.txt`): ament, pluginlib and dlopen resolve the r2 overlay.

## Harness (`tools/`; `provenance/harness_vs_s4c_l1_shadow.diff`, `harness_SHA256SUMS.txt`)
This is the committed S4c level-1 shadow harness. Its historical copies verify against their committed hashes (`historical_copies_check.txt`). Changes:
- `scripts/s4/s4c_driver.py --level 2 --shadow-rehearsal`;
- `--verify-only` restore against `0cb2ccc`;
- the input checks: baseline `0cb2ccc`, the reused build's provenance, the level-1 tool identity. The overlay reference tree is extracted into a new scratch directory and kept: the earlier deletion of that tree was removed, and no deletion command remains;
- comments only in the watchdog, guarded wrapper, pre-launch checks and supplementary analysis.

As before: source order toolchain, r2 overlay `local_setup`, then the workspace; `fortress_m55_walking.launch.py` starts the node itself; one bag, with no `_action/status` or `/clock`; the watchdog (armed at 09:49:05Z) and the trap, before the launch.

## Run (`run/`)
**Launch and pre-key checks: PASS.**
- **Timeline.** Launch at 09:49:14Z; both controllers active at 09:49:53Z; node in SHADOW (gate `False`) at 09:50:33Z.
- **Libraries.** Server 1730 alone maps the two r2 libraries, with the provenance hashes and no stock copy.
- **Configuration.** Gain 1.0 (log and parameter), 100 Hz, physics step 0.001 s.
- **Ownership.** 0 action clients and 0 command publishers. Node 1651 in the launch session.
- **Settling.** 10.05 s of sim (8.36–18.41), 114 fresh samples: z 0.054500 m, range 9.0 µm, tilt 1.5e-7 rad. Maximum ages: joint states 0.072 s, pose 0.030 s, graph 1.004 s.

**Driver.** It ran 09:52:16–09:59:31Z and exited 0 ("script complete (shadow rehearsal: dispatch disabled; not acceptance)").
- **Selection.** Two `+` while DISARMED, before arm. Each teleop confirmation was received after its own write returned and before the next key's write began (C2):
  - `+` 1, written 1257.888402234–1257.888463246 s: `speed level 2/3: 1.90 mm/s`, received at 1257.888752080 s, 0.350 ms after the write began;
  - `+` 2, written 1257.938849688–1257.938879702 s: `speed level 3/3: 2.34 mm/s`, received at 1257.939257392 s, 0.408 ms after the write began;
  - the next key, `a`, began at 1257.989212811 s.
- **Keys.** Exactly `+ + a w`, space, `s`, space, `q`; every write result 1.
- **Plan.** Exactly 18 planned shadow goals at level 2, all completed, with **0 cancels**, no fault and no refusal: 0→1 … 8→9, then 9→8 … 1→0.
- **Level.** The node armed at level 0 and made one `level_change`, 0 → 2, before goal 1. Every goal was at level 2.
- **Transitions.** DISARMED→READY `arm`. Each cycle: STARTING, WALKING, STOPPING `stop`, then READY `stopped` at 0. Finally READY→DISARMED `disarm`.
- **Cycles.** Space while 8→9 was in flight; READY at 0 in 1.18 s sim (≤ 2.2 s). Space while 1→0 was in flight; READY at 0 in 1.89 s sim (≤ 2.9 s). Each was checked at a fresh sample before the next key. Shadow displacement: forward −3.7e-10 m, reverse −4.0e-10 m. That is stationary creep; the signed bands are not applied in shadow.
- **Close.** `q` at READY, boundary 0.
  - The node logged stop, disarm, stop, disarm: the teleop's two pairs, in order.
  - One transition, READY → DISARMED.
  - The teleop (pid 2863) exited 0 by itself (wait status 0). No teleop process remained after the driver.
- **After the close.** DISARMED with no goal over 10 s of sim (84.26–94.26). From the bag: joint motion 5.9e-12 rad, tilt 1.5e-7 rad, z ≥ 0.054495 m.

## Scorer verdicts (`score/summary.json`, committed `score_s4c.py --level 2 --shadow`)

| | Verdict | Detail |
|---|---|---|
| C1 | PASS | The 18 planned goals in order; 0 cancels, faults or refusals; keys and pre-`q` requests (arm, stop, stop) as scripted; SHADOW with the gate `False` in every status |
| C2 | PASS | Two `+` while DISARMED before arm; 2/3 then 3/3 at the node's speeds, each attributed by receipt to its own write; one `level_change` 0 → 2 before goal 1; every goal at level 2; node stride 0.06 m |
| C3 | PASS | Space while 8→9 was in flight; one STOPPING `stop`; READY at 0 in 1.18 s sim (≤ 2.2 s); checked before `s` |
| C4 | PASS | Space while 1→0 was in flight; READY at 0 in 1.89 s sim (≤ 2.9 s); checked before `q` |
| C5 | PASS | stop × 2 and disarm × 2, one READY→DISARMED (`disarm`), teleop exit 0; no goal, key or other request after `q`; still over 10 s of sim |
| C6 | N/A | No trajectory runs in shadow |
| C7 | PASS | Maximum continuity error 2.2e-16 rad |
| C8 | PASS | 18 shadow goals, 0 cancels, 0 foreign clients, 0 publishers. The controller log shows 0 received, 0 accepted and 0 cancel requests; the bag has no feedback |
| C9 | PASS | Minimum joint margin 0.436 rad (`rr_foot_joint`). Maximum node-observed ages: joint states 0.130 s, pose 0.059 s, graph 1.008 s. Driver complete |
| C10 | PASS | `--verify-only` restore: DISABLED AND VERIFIED |
| C11 | PASS | `library.json` `917fc4d7…`: the node's equals the disabled one |

- **Recorded only.** Net return −7.7e-10 m. The forward cycle's recorded "within the live stop band" is `False`: −3.7e-10 m is below 0 in a stationary shadow cycle. It is not applied in shadow.
- **Real controller: zero goals, zero cancels, zero feedback.** The bag has 0 `joint_trajectory` and 0 `_action/feedback` messages. The controller's reference changed by 0.0 rad, and the JTC log has 0 goal or cancel lines. Action clients: 0 before; afterwards only `/rosbag2_recorder`.
- **Supplementary analysis** (`analysis/`, not used for the verdict):
  - RTF 0.167, with 10 s windows of 0.116–0.189 (level-1 shadow: 0.193);
  - maximum node-observed ages over all 892 statuses: joint states 0.130 s, pose 0.059 s, graph 1.008 s.

## Stop and verification
Each process's outcome is recorded separately; no exit code is inferred.
- **Simulator server (1730): exit 0, observed by strace.** strace recorded one SIGINT, sent by pid 1636: the server's parent, the `ign gazebo` launcher, which received the launch group's SIGINT. The level-1 runs recorded no signal to the server. No SIGTERM or SIGKILL.
- **Locomotion node (1651):** "process has finished cleanly", as the launch reports it.
- **Teleop (2863):** exit 0 by itself, observed by the driver.
- **GUI (1734):** its own exit status was not observed. The launch log has no escalation line (the level-1 shadow's had one, for the GUI), and its launcher process (`ign gazebo-1`, 1634) finished cleanly.
- **Launch.** Every launch process finished cleanly. The launch group was empty 4 s after SIGINT. No leftovers; the watchdog stood down without acting.
- **Findings.**
  - At startup, `gz_ros2_control` logged two `[ERROR]` lines, 0.50 s apart: "robot_state_publisher service not available, waiting again". It connected 0.08 s after the second, before the controllers were active. That is the startup wait S4b also recorded. The level-1 shadow and live logs had no `[ERROR]` line.
  - One ros2 CLI daemon restart happened while the run waited for the controllers (`daemon_events.txt`). That is the observer side, as in earlier runs.
- **Restore** `--verify-only` against `0cb2ccc`: **DISABLED AND VERIFIED**. Gates `False` on a fresh import, both M6 `--live` tools refused (exit 3), no enabling branch (`restore/`, `disabled_state_after.txt`).
- **Bag** (kept outside the repository): `/root/spiderx_evidence/s4c_l2_shadow_20261103T094244Z/bag/bag_0.db3`, sha256 `295f48d1…`, 199,180,288 bytes, unchanged by scoring.
- **Media.** Two screenshots, both at rest: before the first key, and after the script.
- **Compression.** The launch log, the driver log and the full memory map are gzipped. The secret scan found no credential or address, and the screenshots carry no text chunks.
