# S2 Phase C: one attempt on the modified plant (welded body, gain 1.0), 2026-10-30: FAIL

**Verdict: FAIL** by the frozen scorer: P4 FAIL and P2 UNMEASURED; P1, P3 and P5–P8 PASS.
- **The script ran once, to its end:** 59 goals and 1 cancel (the scripted estop).
- **Offline plant check:** SUPPORTED (ĝ = 1.0).
- Neither non-pass item is an observed tracking or ownership violation (Notes). Still, no criterion is reinterpreted here: S2 is **not** PASS.
- **Claims:** none about walking, ground contact, balance or hardware. S3 is not authorized. The 2026-10-29 attempt stays FAIL.
- **Review status:** this run's evidence is uncommitted, for review. The bag is outside the repository.

## What happened

| Item | Value |
|---|---|
| Base | `claude/spiderx-sim-actuator-model` at `f32285a`, `src/` = `ca49db3` (`provenance/input_checks.txt`) |
| Enabling | `local/m55-s2-20261030T063644Z` = `af92e1e`, from the patch `d1dacac5…20236`: exactly 2 lines (gate + pin). Local only: no upstream and no remote ref (`restore/post_restore_state.txt`). Deleted after verification |
| Plant | Pinned overlay `bfc5845` + patch `14238400…`. Libraries `ef37a04f…` (`libgz_ros2_control-system.so`) and `c3db97f2…` (`libgz_hardware_plugins.so`). `position_proportional_gain` 1.0 |
| Launch | One launch of `fortress_m61a_fixed_base.launch.py` at 06:42:29Z. Controllers active after 20 s; preflight READY; GUI at nice 19 |
| Pre-motion checks (before the node existed) | **PASS** (`run/premotion/`):<br>- Simulator process: PID 8166, `ign gazebo server`, in this launch's session; the only process mapping the plugin.<br>- Libraries: overlay copies only; mapped-object hashes = provenance; no stock copy.<br>- Gain: one `set to: 1` log line; `ros2 param get` returns 1.0.<br>- Rate and timestep: update rate 100 Hz; physics period 0.001 s.<br>- Ownership: 0 action clients, 0 command publishers.<br>- Weld: z 0.125 m; attachment and spawn deviation 0 |
| Script | Node in DISPATCH mode after 66 s. Driver 06:44:49–06:48:42Z: 3 min 53 s wall, RTF 0.70.<br>- Levels 0, 1 and 2, forward and reverse: 54 goals, all SUCCEEDED.<br>- Stop + home, then lease + re-arm + home: 4 goals SUCCEEDED.<br>- Estop: goal 59 CANCELED; node FAULTED (`estop`) |
| Sent | **59 goals, 1 cancel** (`node/events.jsonl`). 2245 commands accepted, 0 rate rejections |
| Last scripted step | The final estop exercise: the script completed (`driver/driver_summary.json`: "script complete") |
| Stop | Harness SIGINT; process group empty after 6 s; no leftovers (`run/shutdown.txt`, `run/leftovers.txt`) |
| Shutdown | **Not graceful.** The Gazebo server aborted (SIGABRT): `std::terminate` with "Node needs to be associated with an executor", in `~GazeboSimROS2ControlPlugin()`. This came after the controllers and the hardware had shut down normally. No SIGKILL escalation (`run/launch.log.gz`, lines 466–520) |
| Restore | **DISABLED AND VERIFIED** (`restore/restore_verify_20261030T064957Z.txt`). Branch back at `f32285a`, clean; gates False on a fresh import; both M6 `--live` tools exit 3 |

## Offline plant check (`response_law/`, expected gain 1.0): SUPPORTED

- **Pooled:** ĝ = 0.9999999999996921 (SE 5.7e-12), over 8986 controller updates × 12 joints = 107,832 joint-updates. Largest residual 2.0e-12 rad (limit 1e-9).
- **Per joint:** all 12 IDENTIFIED, |ĝ − 1| ≤ 8.5e-12 (`rf_foot_joint`), at least 8986 updates each.
- **Segments:**
  - tracking: IDENTIFIED (95,076 joint-updates);
  - hold after goal: IDENTIFIED (10,068);
  - idle before the first goal: UNMEASURED (excitation 5.7e-6 rad, under the 1e-4 floor). This segment is not required.
- **Excluded:**
  - 322 `controller_state` samples ambiguous at rest;
  - 1 `controller_state` sample with no exact joint-state match;
  - 1 feedback sample without an identical joint state;
  - 1 update with its next update missing.
- **Recording:** 2 single-sample `/joint_states` gaps; 0 samples out of order.
- **Offline only.** This is an interpretation gate; it could not stop the run.

## Acceptance: FAIL

Scored by the frozen scorer `analyze_s2.py` (`e9c2efcb…`), with thresholds unchanged.

| # | Verdict | Key values |
|---|---|---|
| P1 | PASS | 54 cycle goals, all SUCCEEDED. All 6 runs end READY at boundary 0, in script order, with no unplanned fault |
| P2 | **UNMEASURED** | 58 of 59 goals have feedback: max \|error\| 0.00225 rad (limit 0.02), 0 samples above. Goal 59 (the scripted estop) has none (Notes) |
| P3 | PASS | Max continuity 9.7e-5 rad (limit 0.005) |
| P4 | **FAIL** | 59 goals, 1 cancel, 0 foreign clients, 0 command-topic publishers, exercise order OK. `joint_trajectory_msgs` is null (Notes) |
| P5 | PASS | Check 1.7: 1.77 s sim; check 1.9: 1.80 s sim (both bounds 2.7 s). Lease: DISARMED. Estop: one cancel, last goal CANCELED |
| P6 | PASS | 13,627 physics poses, deviation 0 m and 0 rad |
| P7 | PASS | `library.json` `917fc4d7…` identical, disabled vs enabled. Digest `62f8a100…` identical in both builds |
| P8 | PASS | DISABLED AND VERIFIED |

**Tracking cross-check.** The analyzer and the scorer agree on the tracking maximum: 0.0022469101426617205 rad (`rf_thigh_joint`, 155.04 s sim). Basis: 3947 feedback samples, 0 unusable, 0 error-identity mismatches. Per-run maxima are 0.00225 rad at every level (`tracking.csv`).

## Notes

- **P2 UNMEASURED: the estop goal has no feedback.** The driver pressed `x` 50 ms after `w`, as soon as it saw goal 59 in flight (node STARTING).
  - The node faulted and cancelled at once. The controller accepted the goal 41 ms later and returned CANCELED.
  - No action feedback could exist, and the frozen P2 needs feedback for every goal. The verdict is not overridden.
- **P4 FAIL: its only failing term is `joint_trajectory_msgs` = null.**
  - The scorer counts the topics present in the bag, and `ros2 bag record` never adds a topic that has no publisher.
  - Independent evidence shows zero publishers: `Publisher count: 0` in the captures before and after, and the node's graph monitor peaked at 0.
  - The frozen scorer does not treat absence as 0, so P4 stays FAIL. The 2026-10-29 record had the same null.
- **Both non-pass items are limits of the frozen criteria,** not observed tracking or ownership violations. Whether to amend P2 or P4 is an owner decision. No criterion, threshold or scorer was changed.
- **Two refusals after the script.** At teardown the driver quit the teleop (`q`), and the teleop sent stop + disarm. The node was FAULTED, so it refused `disarm` twice (`faulted_reset_required`), 3 s after the final result. No motion resulted, and this is not a scripted step.
- **Run-local harness** (`tools/`; diffs against the frozen scripts in `*.diff`):
  - **Pause:** after preflight the run waits for the pre-motion checks (`go.flag`).
  - **Driver exit code:** kept, to gate screenshot 2.
  - **Screenshot 2:** one capture after the script.
  - **Watchdog:** extended to run cleanup and then the restore. It never fired.
  - **Unchanged:** the stage scripts, `cleanup.sh`, the driver and the scorer are identical to the frozen copies (`tools/frozen_copies_sha256.txt`).
  - **Logging fix:** before the launch, a logging bug in the pre-launch check script was fixed: `tee /dev/stderr` had truncated the script's own log. The read-only check was then re-run.
- **Disabled P7 basis.** `disabled/library.json` was written offline with the node's own expression (`tools/library_json.py`). It is byte-identical to the 2026-10-29 node's file.
- **Screenshots** (`run/captures/`): two captures of the Gazebo window, on Xvfb with software rendering, one before the first goal and one after the script. They differ by 12 pixels. A static screenshot proves nothing about trajectories. The first is published in `media/s2_phase_c/20261030T062140Z/`.

## Prerequisites run in this session (disabled baseline `f32285a`)

- **Clean build and full suite, run once:** 1937 tests (1888 pytest cases + 49 CTest wrappers); 0 errors, failures or skips.
  - All 49 registered targets have fresh results (`disabled/collection_check.txt`). `test_sim_actuator_model` ran its 7 cases.
  - The count is the 1929 reference plus 7 new cases and 1 new wrapper.
- **`spiderx_scripts`:** colcon ran 0 tests (unittest fallback). Run directly: 2 passed.
- **Driver bench:** 8 of 8 pass.
- **Response-law tests:** 16 pass; the 3 original-bag tests pass with `S2_BAG` set.
- **Original-bag regression:** expected 0.1 gives SUPPORTED, expected 1.0 gives CONTRADICTED; the bag hash is unchanged.
- **Gates:** all False on a fresh import. Then both M6 `--live` tools exit 3 (verify-only restore: DISABLED AND VERIFIED).
- **Library:** digest `62f8a100…` (54 goals).
- **Enabled build:** gates M55 True, M61 False, M6.0-D False. M5.5 tests: 241 passed. Digest and `library.json` identical to the disabled build.
- **Overlay:** source = `bfc5845` + exactly the patch; library hashes = provenance; gtest 9 of 9, from the same build.

**Not in the repository:** the bag, `~/spiderx_evidence/s2_phase_c_20261030T062140Z/bag` (sha256 `9c77b9a2…ec2e`, 143.7 MB). See `run/bag_metadata.yaml` and `run/bag_hash.txt`.

`SHA256SUMS` covers every file here.
