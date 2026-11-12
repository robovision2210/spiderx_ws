# S4d-E shadow rehearsal, 2026-11-11 (dispatch disabled)

**Result: PASS (SHADOW ONLY: not S4d-E acceptance).** The committed `score_s4d_e.py --shadow` rated:
- E1–E7 and E9–E12 PASS;
- E8 tracking N/A, as the frozen shadow contract specifies.

This was the one authorized S4d-E SHADOW launch. All three gates were `False`, and the node ran in SHADOW
mode throughout.

| Controller traffic | Planned | Actual |
|---|---|---|
| Shadow goals (the node's model only) | 1 | 1 |
| Modeled shadow cancellations | 1 | 1 |
| Real-controller goals | 0 | **0** |
| Real-controller cancels | 0 | **0** |
| Real-controller feedback | 0 | **0** |

**This is not live cancel-and-hold validation.**
- In SHADOW, the node's shadow transport models the cancel (`cancel_response` 0, result CANCELED
  "shadow cancel"). No real goal ran, so this run cannot show that execution stopped after a cancel.
- The body did not move. The 10 s hold is the hold of a body at rest.
- The driver pressed `x` without waiting for feedback, because none can exist in shadow. The live design
  requires attributed real feedback before `x`.

The frozen contract is `docs/S4D_E_CONTRACT.md` (`f391f43`). A live S4d-E attempt needs separate review and
authorization.

## Baseline and inputs (`provenance/`)
- **Validated executable:** `87198b951506b35ee4d95f3c124c0327a9647ae6` on `claude/spiderx-s4d-e-preparation`.
  The worktree was clean, equal to `origin`, and there was no `local/*` branch.
- **Run installation:** the fresh disabled build of `87198b9`, `/root/spiderx_fresh_disabled_20261111T104338Z/install`.
  - Its record is `../s4d_e_fresh_disabled_20261111T104338Z/`: 1993 tests, 0 errors, 0 failures, 0 skipped;
    the direct tests passed; both installations PASS.
  - The workspace installation was not in the run's environment.
- **Input checks: PASS** (`input_checks.txt`):
  - the source gates are all `False`;
  - the run installation equals HEAD;
  - the r2 overlay libraries are `3a8f2044…` and `06777e14…`, and the config is `ab0dccd4…`;
  - no relevant process ran, display `:77` was free, and there was no run or bag directory yet;
  - the journal showed the E shadow launch unused;
  - OD16 disk passed.
- **Pre-launch checks: PASS** (`prelaunch_checks.txt`):
  - ament, pluginlib and dlopen resolve the r2 overlay;
  - the gates import `False` from the run installation.

## Harness (`tools/`; `provenance/harness_vs_s4d_shadow.diff`, `provenance/harness_SHA256SUMS.txt`)
The harness is derived from the committed S4d-M shadow harness (`../s4d_shadow_20261110T072920Z/tools/`).
- The copies it was derived from matched those committed files byte for byte, 16 of 16. Their manifest is
  `provenance/tools_s4d_shadow_SHA256SUMS.txt`; they are not duplicated here.
- Changes:
  - the run installation path;
  - the driver, `scripts/s4/s4d_e_driver.py --shadow-rehearsal`;
  - the OD8 budget: the E driver's 1800 s cap plus 900 s;
  - `cleanup.sh` and `premotion_checks.sh` recognize the E driver's name;
  - `restore_cmd.sh` runs `--verify-only` against `87198b9`, passing pid files only if they exist, then verifies
    the run installation's gates;
  - the supplementary analysis changed in its docstring only. That docstring still names `score_s4d.py`; the
    verdict here is `score_s4d_e.py --shadow`.
- **Tested before the launch** (`provenance/harness_tests/`, with stubs only):
  - refusals S1–S7: the run installation missing, the gates not all `False`, no watchdog, a 2000 s watchdog,
    the disk rule, OD8 before the first key, and OD16 before recording;
  - the watchdog (W1–W3): it fires, it stands down, and it defaults to 7200 s;
  - interruption during the pre-key wait (I1) and during the driver (I2): the trap ran `cleanup.sh`;
  - owned cleanup (C2): an unowned decoy with the same driver command line survived.

## Run (`run/`)
| UTC | Event |
|---|---|
| 10:58:30 | Watchdog armed: pid 29922, 7200 s, deadline 12:58:30Z |
| 10:58:32 | Journal: the E shadow launch is **used**. Guarded run started; OD8 and OD16 PASS. Launch. |
| 10:59:02 | Both controllers active |
| 10:59:30 | Node up in SHADOW (gate `False`); levels 1.06, 1.90 and 2.34 mm/s |
| 11:08:56–11:09:52 | Pre-key checks (`premotion/`): **PASS** |
| 11:10:03 | `go.flag`; OD8 before the first key PASS (6507 s left); captures before |
| 11:10:05 | OD16 before recording PASS; recorder started |
| 11:10:09–11:11:45 | Driver; exit 0 |
| 11:11:48 / 11:11:50 | Captures after; recorder stopped |
| 11:11:53 | Launch stop: SIGINT to the launch group |
| 11:12:00 | Simulator server exit 0 (strace); guarded run exit 0; cleanup found nothing |
| 11:12:23 | Watchdog stood down without acting |

**Pre-key checks** (`premotion/`):
- server 30216 alone maps the two r2 libraries, with no stock or mixed copy;
- gain 1.0 (log and parameter), `update_rate` 100, period 0.001 s;
- both controllers active;
- node 30117 in SHADOW, in the launch session;
- 0 action clients and 0 publishers towards `joint_trajectory`;
- the settle check passed over 10.02 s of sim: z range 9.0 µm, tilt 1.5e-7 rad, fresh ages.

**The pre-key wait lasted 630 s.** The operator's monitor watched for the wrong log wording; the run script
logs "pre-shadow checks: waiting". The checks therefore ran late. No key was pressed before `go.flag`, which
came within the script's 1500 s limit.

**Script** (`driver/` under `run/`, `node/`):
1. **`+ +` while DISARMED.** The teleop confirmed 2/3 (1.90 mm/s), then 3/3 (2.34 mm/s): level 2.
2. **`a`.** DISARMED → READY.
3. **`w`.** Goal seq 1 was dispatched in shadow: level 2, 0 → 1, the S1 `shift`, 1.9 s, 18 points,
   fingerprint `748bd1df…`.
   - The node went READY → STARTING → WALKING.
   - The driver saw goal 1 in flight 100 ms after the `w` write.
4. **`x`, 0.07 ms later.** The sim time elapsed since goal 1 was seen was 0.0 s (deadline 0.95 s).
   - In one handling at node time 3571.003376, the node logged:
     - the `estop` request;
     - the `estop` fault;
     - WALKING → FAULTED (why `estop`);
     - `cancel_sent` for seq 1.
   - Request to FAULTED: Δ = 0 s on the node clock. This holds by construction (contract §3); the limit is
     ≤ 0.050 s, inclusive.
   - Key write to request: 2.40 ms. It is recorded only: it runs from the driver's after-write time to the
     node's request time, both on the host's monotonic clock.
5. **Modeled cancellation.** `cancel_response` 0 and result CANCELED (5) "shadow cancel" came 49.8 ms later.
   No second goal was sent, and nothing was retried.
6. **Hold.** The node stayed FAULTED with no goal. Over the scorer's 10 s sim window [155.357, 165.357]:
   - joint motion 5.9e-12 rad;
   - tilt 1.5e-7 rad;
   - z ≥ 0.0545 m.
7. **`q`.**
   - `stop` was accepted twice: "not moving (FAULTED); held until a zero command".
   - `disarm` was refused twice with `faulted_reset_required`. These are the documented reset-required refusals,
     and there was no other refusal.
   - Teleop 32677 exited 0.
   - Final state: FAULTED; `goals_recorded` 1, `max_seq` 1; no reset, re-arm or home.

**Controller traffic, actual**:
- **Controller log.** `launch.log.gz` spans the controller's activation at 10:59 to its shutdown at 11:11:54.
  It shows 0 goals received, 0 accepted and 0 cancel requests.
- **Feedback.** The recorder subscribed to the feedback topic before the first key and recorded 0 messages.
  The driver also received 0 feedback samples.
- **Commands.** `joint_trajectory` had no publisher and is absent from the bag. The controller reference changed
  by 0.0 rad.
- **Node status.** Every status shows 0 foreign clients and 0 publishers.
- **`ros2 action info`.**
  - Before the run it showed 0 clients.
  - After the run it listed `/rosbag2_recorder` and the driver's node `/s2_driver_status`. Both only subscribe to
    the feedback topic, which `ros2 action info` reports as client membership.
  - The locomotion node is absent, as expected in SHADOW. In the S4d-M live run it was listed.

## Shutdown and processes
- The recorder stopped at 11:11:50, before the launch stop.
- **Simulator server 30216:** SIGINT came from the ign launcher, 30091. It exited 0 at 11:12:00 (strace).
  - There was no "Escalating to SIGKILL" line.
  - This shutdown was graceful, unlike the S4d-M live run's (`../s4d_live_20261111T052817Z_shutdown_postmortem_20261111T101720Z/`).
- **Other launch processes:** every one, the node 30117 included, logged "finished cleanly".
  - The GUI's (30219) exit status was not recorded individually. No process remained.
- **Owned cleanup** found nothing left (`cleanup.log`, `leftovers_after.txt`).
- **Finding:** one ros2 CLI daemon fault and restart at 10:58:55, during the controller wait (`daemon_events.txt`).
  The daemon serves the harness's own `ros2` CLI observers. It was stopped at shutdown.

## Restoration and disabled state (`restore/`, `run_install/`, `after/`, `disabled_state_after.txt`)
- **`--verify-only` against `87198b9`: DISABLED AND VERIFIED.**
  - All three gates are `False`, imported from the workspace.
  - Only after that, both M6 `--live` tools refused with exit 3.
  - No simulation process remained.
- **Run installation gates: PASS**, all three `False`.
- **Installations A (run) and B (workspace): PASS each**, after the run.
  - Each passes `install_check.sh` from the fresh-build record, verified against its manifest.
  - Library `62f8a100…` with 54 goals; `library.json` `917fc4d7…`, the same as the run node's `library.json`.
- **Repository state.** `claude/spiderx-s4d-e-preparation` at `87198b9`, worktree clean, no `local/*` branch.
- **No enabling at any point.**
- `restore/worktree_status_before_restore.txt` is the restore script's own `git status --short`, which is empty
  for a clean tree. It replaced an operator note of the same name.

## Score (`score/summary.json`, `score_provenance.txt`)
- **Scorer:** `scripts/s4/score_s4d_e.py`, blob `0981d911…` at `87198b9`. It ran unchanged with `--shadow` and
  exited 0.
- **E1 plan:** keys `+ + a w x q`; one goal; one cancel; requests before `q` exactly `arm` and `estop`.
- **E2 level:** 2/3, then 3/3, each attributed; goal 1 at level 2.
- **E3 x:** the `estop` came while goal 1 was active and within 0.95 s of sim. The feedback precondition is not
  applicable in shadow, and this is stated as such.
- **E4 fault:** one `estop` fault; FAULTED within 0.050 s on the node clock; one cancel in the same handling.
- **E5 cancel:** the modeled shadow cancel: response 0, CANCELED.
- **E6 hold:** measured, as above.
- **E7 no further:** no goal, request, reset, arm or home after `x`.
- **E8 tracking:** N/A (shadow).
- **E9 traffic:** zero, as above.
  - The `"ok": false` in its `controller_log` is the live-order flag, which expects one goal and one cancel. It is
    false by construction in shadow; the shadow term uses the counts.
- **E10 close:** exactly `stop` ×2 and `disarm` ×2. The disarms were refused `faulted_reset_required`, with no
  other refusal; teleop exit 0 within 15 s.
- **E11 margins:**
  - min joint margin 0.436 rad;
  - tilt 1.5e-7 rad;
  - travel 1.8e-10 m, against a 0.15 m limit;
  - node-observed ages within the monitor limits.
- **E12:** library identity; restore report; run installation gates.
  - Its field `enabled_library_sha256`, a name inherited from S4d-M, holds the run node's `library.json` here.
    Nothing was enabled.
- **Bag hash:** unchanged before the restore, after scoring and after the supplementary analysis
  (`run/bag_hashes.txt`, 3 of 3).
- **Supplementary analysis:** `analysis/run_analysis_supplementary.json`, read-only. It records RTF 0.22, the exits
  and the traffic. It carries no verdict.

## Files
- Bag: kept outside the repository; see `run/bag_retention.txt`. The original bag and `metadata.yaml` were delivered as conversation attachments; a durable off-container copy is unconfirmed.
- Gzipped here: `run/driver/driver.jsonl`, `run/premotion/maps_full.txt` and `run/launch.log`. Their original hashes
  are in `provenance/gzipped_originals_SHA256SUMS.txt`.
- The batch journal snapshot is in `provenance/`.
- `SHA256SUMS` covers every file here.

## What this does not show
- **No live S4d-E result.** No real goal was sent or canceled, so none of these is shown:
  - attributed feedback before `x`;
  - real tracking (E8);
  - stopping after a real cancel;
  - a post-cancel hold of a moving body.
- **No other claims.** Nothing here concerns the physical keyboard, hardware, navigation or unrestricted walking.
- **Cloud simulation only.** Ignition Fortress on the RoboStack toolchain, the r2 `gz_ros2_control` overlay
  (gain 1.0) and an idealized actuator model, at RTF 0.22. Nothing was run on the home PC.
