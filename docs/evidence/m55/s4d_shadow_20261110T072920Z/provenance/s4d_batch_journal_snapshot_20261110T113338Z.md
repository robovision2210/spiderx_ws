# S4d-M batch journal (external; not in the repository)

Authorization (owner message, 2026-11-10): A hardening, B one fresh DISABLED build + full suite,
C ONE S4d-M SHADOW rehearsal if prerequisites pass, D home-PC handoff document.
No live dispatch, enabling, S4d-E, hardware, main push or merge. A restart does NOT authorize
another launch; if launch history is uncertain, STOP.

## Launch authorization
- shadow launch used: YES - guarded run started 2026-11-10T07:44:21Z (the ONE authorized S4d-M shadow launch; no retry, no second launch)

## Step 0 checkpoint (2026-11-10T07:02:20Z)
- branch claude/spiderx-s4d-preparation, HEAD d4802ca1b5eaf7d029c23b2df39d559bd6da0169, clean, equals origin
- PR #24 draft open: head d4802ca, base claude/spiderx-s4d-plan 34396cfaae1841ad20e29b014beda454da408b0a
- no simulator/ROS/teleop/driver/recorder processes; no s4d evidence dirs; no previous journal
- gates in src: M61 False, M6 False, M55 False
- runtime: /opt/mm/activate.sh (RoboStack humble), /root/spiderx_gz_overlay_ws_r2/install, ws install
- disk: / 21G available

## Steps

### Step 1 (offline hardening) - findings
- 1A: bench library from the FROZEN r2 library.json (sha256 917fc4d796efe2344d0edcf038a1f7186d053622df250d9b389690a87d485d8e,
  read only; never rebuilt): goal durations 0.8-4.7 s, speeds 1.06/1.90/2.34 mm/s. Bench status cadence made realistic
  (2 Hz + on every state change, as m55_rclpy _after). Complete plan PASS on the bench at RTF 0.1, 0.17, 0.3, 0.6, 1.0;
  late space/release/switch/q at RTF 1.0 each produce an off-plan goal and stop; duplicate/stale/missing evidence covered.
  Scorer at frozen durations (live 0.17 and 1.0, shadow 0.17): event + synthetic-motion checks PASS; home 10.4-12.0 s sim.
- 1B: real continuous boundaries: S3 free-base bag (level 0) read only, sha256 52fdb5aea5a3426f... unchanged before/after.
  S4d rule finds all 4 walked-through boundaries (18,27,36,45) and both stopped ones at rest (0.07-0.12 mm within
  +-0.1 s sim); cycles match the committed S3 score within 0.09 um. Level 2 continuous boundaries: no retained recording
  (S4c runs stop at every boundary 0) -> remains a live-validation risk. Script + output:
  /root/spiderx_evidence/s4d_offline_checks/continuous_boundaries_s3.{py,json}; fixture committed for a regression test.
- 1C: close-reason inference unchanged. With the realistic cadence, every decided case equals the node's actual reason;
  default close decided 'lease_expired' at RTF 0.17/0.6/1.0. UNMEASURED remains for: no in-flight status with a lease age,
  commands after the transition (lease age non-empty), the 1 us band, and a last status exactly lease_s before the
  transition with a later command (bench 0.05 s clock coincidence at RTF 0.6/0.7, lease kept fresh). No loosening.
- 1D: target fs / (/dev/vda) avail 22,402,486,272 B. E = 2 x 30.8 MB/min (PLANNING ASSUMPTION) x 120 min (7200 s
  watchdog bound) = 7.392 GB; 2E = 14.784 GB; restore reserve 0.1 GB (measured restore evidence <= 11 KB; ws
  build+install 6.2 MB); previous fresh bases 11.0 MB. Requirement max(5.369, 14.884) = 14.884 GB -> PASS (margin 7.5 GB).
- step 1 committed and pushed: 8f99bcc4ab27294265da91a31ce03999423acc7f (tests + fixture only; driver/scorer
  code identical to d4802ca). CANDIDATE SHA for step 2: 8f99bcc4ab27294265da91a31ce03999423acc7f (2026-11-10T07:13:56Z)

## Step 2 (fresh disabled build + suite) - started 2026-11-10T07:15:06Z
- candidate HEAD 8f99bcc4ab27294265da91a31ce03999423acc7f; outputs /root/spiderx_evidence/s4d_fresh_disabled_20261110T071506Z; NEW bases /root/spiderx_fresh_disabled_20261110T071506Z (did not exist)
- tools copied from the committed S4c L1 follow-up tools; diff in /root/spiderx_evidence/s4d_fresh_disabled_20261110T071506Z/tools_diff.txt; hashes /root/spiderx_evidence/s4d_fresh_disabled_20261110T071506Z/tools_sha256.txt
- step 2 finished 2026-11-10T07:29Z: build exit 0; 1983 tests 0 failures 0 skipped; collection PASS; direct s2 48 OK
  (3 expected existing S2_BAG skips), s3 86 OK, s4 206 OK, spiderx_scripts 2 passed; installation A (fresh) PASS and
  B (existing) PASS, each with both M6 --live refusals after gates False; protected runtime sources unchanged since
  0cb2ccc; STEP 2 VERDICT: PASS (STEP2_SUMMARY.txt). Tested executable SHA 8f99bcc4ab27294265da91a31ce03999423acc7f.
- step 3 started 2026-11-10T07:29:20Z: run-local root /root/spiderx_evidence/s4d_shadow_20261110T072920Z (new; tools + provenance only, nothing launched)
- step 3 (prepare; nothing launched): harness from the committed S4c L2 shadow tools (manifest-verified copies in
  tools_s4c_l2_shadow/); diff provenance/harness_vs_s4c_l2_shadow.diff; hashes provenance/harness_SHA256SUMS.txt.
  Run installation = the fresh disabled step-2 install (env.sh); restore_cmd = --verify-only vs 8f99bcc + run-install
  gates JSON (M15). Input checks PASS; prelaunch PASS; library_disabled.json 917fc4d7... (= frozen library).
  Watchdog tests (7200 default, fire, stand-down) PASS; owned-process cleanup test PASS (decoy untouched); run-script
  refusals before launch: no watchdog / 6000 s watchdog / disk FAIL -> refused, nothing launched.
- step 4: LAUNCH COMMAND STARTED 2026-11-10T07:44:21Z (watchdog pid 16828 armed first, 7200 s); run /root/spiderx_evidence/s4d_shadow_20261110T072920Z/run, bag /root/spiderx_evidence/s4d_shadow_20261110T072920Z/bag
- step 4: pre-key checks PASS (premotion, settle, screenshot); go.flag written 2026-11-10T07:46:52Z
- step 5: run ended 08:13:51Z (driver exit 0, guarded run exit 0, server exit 0 by strace, launch group empty after 5 s, no leftovers); watchdog stood down 2026-11-10T08:14:05Z
- step 5 (2026-11-10T08:14-08:25Z): restore --verify-only vs 8f99bcc DISABLED AND VERIFIED; run installation gates False (M15 JSON);
  bag c285b2e6... 760,479,744 B unchanged before/after scoring and analysis; committed score_s4d.py --shadow:
  M1-M10, M12-M16 PASS, M11 N/A -> "PASS (SHADOW ONLY: not S4d acceptance)". Controller: 0 goals/cancels/feedback (JTC log 0,
  bag 0 feedback, no joint_trajectory, reference change 0.0 rad). Close reason lease_expired (rule b). RTF 0.212.
  Evidence UNCOMMITTED: docs/evidence/m55/s4d_shadow_20261110T072920Z/ and docs/evidence/m55/s4d_fresh_disabled_20261110T071506Z/
- step 6: docs/S4D_LOCAL_HANDOFF.md committed alone and pushed: c8d3bd26294185028905c3a0bba9f2d2449e774d (documentation only;
  parent 8f99bcc = tested executable SHA). PR #24 description updated with factual results. No new PR.
- batch end 2026-11-10: launch authorization USED once (07:44:21Z); no retry, no second launch; no live dispatch, enabling,
  S4d-E, hardware, main push or merge. Evidence dirs uncommitted for review.
- evidence preservation (owner message): review 2026-11-10T11:33:38Z: manifests OK (33/116 entries); copies identical to the run-local originals (32 + 115 files); secret scan clean (incl. gz and PNG chunks); bag re-hash c285b2e6... 760479744 B unchanged; disabled-state records consistent. Shadow README wording corrected (S4c already recorded real level-2 single cycles; the gap is continuous multi-cycle level-2 boundaries; live close outcomes disarm/lease_expired/UNMEASURED; collected-test wording). Added run/bag_retention.txt and this journal snapshot (provenance/s4d_batch_journal_snapshot_20261110T113338Z.md) before the evidence commits.
