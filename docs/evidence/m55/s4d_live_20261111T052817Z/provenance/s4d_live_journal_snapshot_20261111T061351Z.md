# S4d-M LIVE batch journal (external; not in the repository)

Authorization (owner message, 2026-11-11): (1) correct stale evidence-status wording in the handoff and PR #24;
(2) run-local live harness preparation and stub tests; (3) ONE local enabling/build sequence; (4) ONE S4d-M live
simulation launch and attempt; (5) mandatory restoration, unchanged scoring, evidence preservation; (6) evidence and
factual documentation commits/pushes after restoration. No hardware, home-PC changes, S4d-E, S4e, extra cycle,
recovery motion, automatic retry, second launch, merge or push to main. Frozen: the 111-goal plan, 0 planned cancels,
approved limits, scorer. Emergency response overrides the budget and ends the attempt.
A restart or interruption NEVER authorizes another launch; if launch history is uncertain, STOP.

## Launch authorization
- live launch used: YES 2026-11-11T05:50:08Z (the ONE authorized S4d-M live launch: guarded run started; no retry, no second launch)

## Baseline
- run-local root: /root/spiderx_evidence/s4d_live_20261111T052817Z
- tested executable ancestor: 8f99bcc4ab27294265da91a31ce03999423acc7f (code unchanged since; only docs/ changed)
- docs-only stale-status fix: 9cc1f6844de11abc630387d4c6c441a623463094 (pushed) = EXACT DISABLED RESTORATION BASELINE
- PR #24 description: evidence-status wording corrected; draft, unmerged
- no prior s4d_live root or journal; no local/* branch locally or on origin; no relevant processes; display :77 free

## Steps
- 2026-11-11T05:28:17Z journal created (step A)
- 2026-11-11T05:36:04Z input checks PASS; disabled gate imports + both M6 --live refusals recorded (disabled/); INSTALLATION VERDICT: VERDICT: PASS
- 2026-11-11T05:41:25Z live harness written (tools/; diffs vs S4d shadow and S4c L2 live; hashes); stub tests PASS (L1-L9, W1-W3, I1-I2, C2); restore/cleanup/watchdog fixed (provenance/restore_tool_fixed_before_enabling.txt)
- 2026-11-11T05:45:31Z watchdog armed (7200 s): pid 4069, deadline 2026-11-11T07:45:29Z; OD8 launch-by (>= 6300 s left) 2026-11-11T06:00:29Z; before the first key the same 6300 s rule
- 2026-11-11T05:45:31Z enabling + B1 build started: enable_build.sh 20261111T054531Z (local/m55-s4d-m-20261111T054531Z; never pushed)
- 2026-11-11T05:49:40Z ENABLE AND BUILD: PASS (239 s): enabling commit bd4a40e40d8ae5b0595d7c7a755ad40ab40c79cc on local/m55-s4d-m-20261111T054531Z (never pushed); B1 build 28 s (8 packages); enabled M5.5 tests 245 passed (112.2 s); gates True/False/False from the run installation; library 62f8a100 / library.json 917fc4d7
- 2026-11-11T05:50:02Z prelaunch checks PASS (provenance/prelaunch_checks.txt)
- 2026-11-11T05:52:40Z launch 05:50:09Z; node up 05:51:05Z (DISPATCH, gate True); pre-key checks PASS (premotion, settle, startup lines, screenshot 1); go.flag written. Startup finding: 6 gz_ros2_control robot_state_publisher-service retry [ERROR] lines (2.5 s) before it connected
- 2026-11-11T06:03:21Z guarded run ended (guarded run exit 0); leftovers: none;; 2026-11-11T06:03:21Z watchdog 4069 stood down on stopped.flag without acting (no watchdog.txt); bag 9320eb43f999c87f... 348086272 bytes; restore starting
- 2026-11-11T06:03:25Z restore_cmd exit 0: DISABLED AND VERIFIED; restore exit 0; back on claude/spiderx-s4d-preparation at the baseline: yes; run installation gates exit 0
- 2026-11-11T06:07:10Z restore DISABLED AND VERIFIED (restore_verify_20261111T060321Z.txt); back on claude/spiderx-s4d-preparation at 9cc1f68; enabling branch deleted after verification (never pushed); both installations import all three gates False (disabled_state_after.txt)
- 2026-11-11T06:07:15Z scored (no --shadow): {"M1": "PASS", "M2": "PASS", "M3": "PASS", "M4": "PASS", "M5": "PASS", "M6": "PASS", "M7": "PASS", "M8": "PASS", "M9": "PASS", "M10": "PASS", "M11": "PASS", "M12": "PASS", "M13": "PASS", "M14": "PASS", "M15": "PASS", "M16": "PASS"} PASS; bag hashes: 3/3 equal
- 2026-11-11T06:10:34Z bag exported: original bag_0.db3 (348086272 B, sha256 9320eb43f999c87f...) + metadata.yaml delivered as conversation attachments (no compression needed); durable copy UNCONFIRMED
- 2026-11-11T06:13:51Z scoring result (committed score_s4d.py, no --shadow): M1-M16 PASS -> PASS (S4d-M simulation scope only). F leg +0.30006 m, R leg -0.29985 m; max tracking error 0.00225 rad; close lease_expired (= node-clock evidence); probe PASS. Findings: startup gz_ros2_control retry [ERROR] x6; server killed by SIGKILL (launcher escalation) at shutdown; bag 34.5 MB/wall-min (> 30.8 observed, < 2x planning); inherited stale header comment in s4d_live_run.sh
- 2026-11-11T06:13:51Z evidence assembled in docs/evidence/m55/s4d_live_20261111T052817Z (gzip copies verified lossless; README written); next: secret scan, manifest, evidence-only commit
