# S4b shadow rehearsal, 2026-11-02 (dispatch disabled)

**Result: PASS (SHADOW ONLY).** This was one free-base SHADOW launch with all three gates `False`. **Zero** trajectory goals or cancels reached the real controller.

This is a rehearsal of the committed S4b script, **not** live S4b acceptance. Live S4b is not authorized.

This file is an evidence index; the evidence is uncommitted, for review.

## Inputs (`provenance/`)
- **Branch and HEAD.** `claude/spiderx-s4b-preparation` at `e406590`, clean, with no `local/*` branch.
- **r2 overlay.** 0.7.15 `bfc5845` plus 0001 `14238400…` and 0002 `520e73cc…`. Every present source file equals upstream plus both patches. Libraries `3a8f2044…` and `06777e14…`.
- **Installed runtime.** The S4a live restore's clean rebuild of `e5899f4` is reused. 169 runtime files resolve to their HEAD blobs; since that build only a test file has changed, and it is not installed.
- **Gates and library.** All three gates `False` on a fresh import. The library is 54 goals, `62f8a100…`, and the node's `library.json` is `917fc4d7…`.
- **Focused tests at HEAD** (`focused_tests_at_head.txt`): 90 OK and 31 passed, with flake8 clean.

## Harness (`tools/`; `harness_vs_s4a_shadow.diff`, `harness_SHA256SUMS.txt`)
The committed S4a shadow harness, with only these changes:
- `scripts/s4/s4b_driver.py --shadow-rehearsal`;
- `s4b_driver` added to cleanup and to the premotion forbidden list;
- `--verify-only` restore against `e406590`;
- an S4b analysis that runs the **committed `score_s4b.py` unchanged**.

Everything else is as before:
- source order: toolchain, r2 overlay `local_setup`, then the workspace;
- `fortress_m55_walking.launch.py`, which starts the node itself;
- one bag, with no `_action/status` or `/clock`;
- the watchdog and the trap, armed before the launch.

## Run (`run/`)
**Launch and pre-key checks: PASS.**
- **Timeline.** Launch at 09:03:46Z, both controllers active at 09:04:24Z, node in SHADOW (gate `False`) at 09:04:58Z.
- **Libraries.** Server 5825 alone maps the two r2 libraries, with the provenance hashes.
- **Configuration.** Gain 1.0, 100 Hz, physics step 0.001 s.
- **Ownership.** 0 action clients and 0 command publishers.
- **Settling.** 10.07 s of sim: z 0.054500 m, range 9.0 µm, tilt 1.5e-7 rad.

**Driver.** It ran 09:06:35–09:11:03Z and exited 0 ("script complete (shadow rehearsal: dispatch disabled; not acceptance)").

**Plan.** Exactly the 12 planned shadow goals, all completed, with **0 cancels**, no fault and no refusal:
- 0→1 … 4→5;
- 5→6, then 6→5 (the same phase played back);
- 5→6 … 8→9;
- 0→1.

The keys were exactly `a w w s h w`, every write result 1. There was no space, `x`, `q`, `r` or `z`, and the only requests were arm and home.

**Transitions.**
1. DISARMED→READY `arm`.
2. A release: STOPPING `intent_stop`, then READY `stopped` at 5. This happened twice.
3. Home: READY `homed` at 0.
4. Lease: STOPPING `lease_expired`, then DISARMED `lease_expired` at 1.

**Scorer verdicts** (committed `score_s4b.py`, unchanged; `analysis/s4b_shadow_analysis.json`): B1–B5 PASS, plus continuity (B8), restore (B10) and library (B11).

| Exercise | Measured | Bound or check |
|---|---|---|
| Release 1 (goal 5, no space) | detection 0.818 s after the release | ≥ 0.717 s (the hold window minus one repeat and one tick); documented ≤ 0.983 s, no finding |
| | READY in 2.77 s sim | ≤ 3.8 s |
| Release 2 (goal 7) | detection 0.880 s | same |
| | READY in 2.42 s sim | ≤ 3.5 s |
| Reversal | `s` written 0.200 s after the repeat stop, while 5→6 was in flight | 6→5 sent at that phase boundary, with no stop between |
| Home | 'h' at READY 5; one forward homing of 4 phases; `homed` at 0 | 7.94 s sim ≤ 12.0 s |
| Lease loss | SIGKILL delivered at 784.610 s (4.1 ms bracket); STOPPING `lease_expired` 0.497 s later | documented ≤ 0.55 s, no finding |
| | DISARMED at 1 in 1.71 s sim | ≤ 2.7 s |

**After the kill.**
- There was no key, request, goal, teleop restart or re-arm.
- The node stayed DISARMED with no goal for 10 s of sim (56.6–66.6).
- Stillness was measured from the bag (frozen S3 hold limits, PASS): joint motion 5.9e-12 rad, tilt 1.5e-7 rad, z ≥ 0.054495 m.

**Recorded only (shadow, never scored as live).** These describe live contact, and no trajectory runs in shadow:
- B6 cycle: progress −3.7e-10 m, stationary;
- B7 tracking: no feedback exists in shadow;
- B9: its controller-log term shows 0 received, which is the zero-goal evidence below.

**Real controller: zero goals and zero cancels.**
- The bag has 0 `joint_trajectory` and 0 `_action/feedback` messages.
- The controller's reference changed by 0.0 rad.
- The JTC log has 0 goal or cancel lines.
- Action clients: 0 before; afterwards only `/rosbag2_recorder`.

**Margins.**
- RTF 0.175, with 10 s windows of 0.153–0.189.
- Node-observed pose age at most 0.030 s (limit 1.0).
- Joint-state age at most 0.083 s while armed. A single pre-arm status, DISARMED at sim 20.51 s right after the teleop started, reached 0.434 s against the 0.5 s limit. That was a finding with no fault: 1 of 551 statuses was above 0.25 s.
- Minimum joint margin 0.436 rad. No tracking finding.

## Stop and verification
- **The teleop kill** was the scripted lease test only. After it, nothing was sent; the harness then stopped the launch at the process level.
- **Exits.**
  - The simulator server exited 0. strace recorded no signal to it, as in S4a live.
  - `m55_locomotion_node` reported "finished cleanly".
  - The launch group was empty after 5 s, with no forced escalation. No leftovers, and the watchdog stood down without acting.
  - A ros2 CLI daemon restart while the run waited for the controllers (`daemon_events.txt`) is the observer side, as in earlier runs.
- **Restore** `--verify-only` against `e406590`: **DISABLED AND VERIFIED**. Gates `False` on a fresh import, both M6 `--live` tools refused, and no enabling branch (`restore/`, `disabled_state_after.txt`).
- **Bag** (kept outside the repository): `/root/spiderx_evidence/s4b_shadow_20261102T085832Z/bag/bag_0.db3`, sha256 `9d3accbc…`, 126,050,304 bytes.
- **Media.** Two screenshots, both at rest: before the first key, and after the script.
