# S4d-E live attempt, 2026-11-11 (cloud simulation, dispatch enabled on a local branch)

**Result: PASS under the frozen S4d-E contract.** The unchanged committed `scripts/s4/score_s4d_e.py`, run without
`--shadow`, rated **E1–E12 PASS**.

This was the one authorized S4d-E live launch. It ran in Cloud Gazebo from a local enabling branch that was never
pushed. A full restoration then verified the disabled state of both installations.

**What it demonstrates.** The level-2 estop was used while a real goal was active:
- **The goal.** The real controller received and accepted ONE level-2 0→1 S1 shift goal. The driver received one real
  feedback sample attributed to that goal before `x`.
- **The estop.** `x` put the node in FAULTED (`estop`) on the same node-clock instant as the request. The node sent
  ONE cancel.
- **The cancel.** The controller logged the cancel and "Canceling active action goal". The cancel response was 0
  and the result was CANCELED.
- **The hold.** The controller's reference stayed unchanged, and the body held, over 10 s of sim. There was no
  reset, re-arm, home or recovery.
- **The close.** `q` gave exactly the two documented reset-required disarm refusals.

**What it does not demonstrate: stopping a moving body.** The cancel came ~0.04 s of sim after the goal started,
about 2% of its 1.9 s duration.
- **Motion before the cancel.** The controller reference and the joints had moved ≤ 1.2e-10 rad from the pre-goal
  pose (`analysis/goal_progress_supplementary.json`). The body was effectively at rest when it was stopped.
- **Tracking.** E8 rests on that single feedback sample, with error 0.0 rad.
- **Cause.** This follows the frozen design: `x` at the first moment the conditions held. The criteria were not
  changed after the result.

Not validated here:
- S4e, human operation at a physical keyboard;
- hardware, navigation and unrestricted walking.

Full S4 acceptance is not complete.

## Baseline and inputs (`provenance/`, `disabled/`)
- **Exact disabled baseline.** `16d939f5a0e06afa3478093f0b491e3e8ce60673` on `claude/spiderx-s4d-e-preparation`, equal to
  `origin`.
  - It is the validated executable `87198b9`, plus evidence-only commits.
  - `src/`, `scripts/` and `third_party/` are unchanged since `87198b9`.
- **Reused validation.** The fresh disabled build and the 1993-test result (`../s4d_e_fresh_disabled_20261111T104338Z/`)
  were reused, because their inputs still matched. Nothing was rerun.
- **Input checks: PASS** (`provenance/input_checks.txt`, 11:47Z):
  - the frozen contract is unchanged since `f391f43`;
  - the r2 overlay matches its provenance: libraries `3a8f2044…` and `06777e14…`, source = upstream + 0001 + 0002;
  - the workspace installation equals HEAD; gates `False`; library `62f8a100…`; config `ab0dccd4…`;
  - every module the E driver and scorer import equals its HEAD and `87198b9` blob;
  - the patch's hash, scope and `git apply --check` pass;
  - the E shadow evidence manifest verifies;
  - OD16 passes; the journal showed the E live launch unused; no relevant process.
- **Before enabling** (`disabled/install_check.txt`), from fresh imports:
  - all three gates `False`, from the workspace installation;
  - then both M6 `--live` tools refused, exit 3;
  - `library.json` `917fc4d7…`.

## Harness (`tools/`; `provenance/harness_vs_s4d_live.diff`, `harness_vs_s4d_e_shadow.diff`, `harness_origin.txt`, `harness_SHA256SUMS.txt`)
New run-local files, derived from the tested S4d-M live tools. `cleanup.sh` is the tested S4d-E shadow copy. Both
historical sets verify against their committed manifests (`historical_copies_check.txt`, 35 of 35); neither was edited.

**Changes:**
- **Run installation:** the fresh enabled E installation.
- **Baseline and branch:** baseline `16d939f`; enabling-branch pattern `local/m55-s4d-e-*`.
- **Driver:** `scripts/s4/s4d_e_driver.py --out RUN/driver`, live mode, with no `--shadow-rehearsal`.
- **OD8:** the E driver's 30-minute cap plus 15 min.
- **Enabling-to-launch bound:** at most 900 s from the start of `enable_build.sh`, as in the S4d-M live run.
- **Pre-key checkpoint:** a fixed "PRE-KEY CHECKPOINT" line, plus `premotion_wait.flag`.
- **`operator_prekey.sh` (new):** the S4d-M live operator's pre-key steps as a script.
  - It waits for either signal, then runs the unchanged pre-key checks, the settle check, the startup-line check and
    screenshot 1.
  - It writes `go.flag` only if every check passes.
  - This fixes the E shadow run's missed checkpoint.
- **Restore:** `restore_cmd.sh` runs the full restore against `16d939f`.
  - It passes the recorded enabling ref with `--delete-enabling-branch`.
  - It passes pid files only if they exist.
  - It returns to the E branch, then checks the run installation's gates.
  - Fixed before enabling: `provenance/restore_tool_fixed_before_enabling.txt`.

**Stub tests before enabling: PASS** (`provenance/harness_tests/`):
- **Refusals before the launch:**
  - L1: run installation missing;
  - L2: gates not True/False/False;
  - L3, L3m: no `local/m55-s4d-e-*` record, including an M-pattern branch;
  - L4: another branch checked out;
  - L4w: a wider diff;
  - L4d: a dirty worktree;
  - L5, L6: no watchdog, or a 2000 s one;
  - L7: the disk rule;
  - L10, L10n: the enabling-to-launch bound exceeded, or unrecorded.
- **Aborts with no key:**
  - L8: OD8 before the first key;
  - L9: OD16 before recording, with no recording.
- **Watchdog:** fire, stand-down, 7200 s default (W1–W3).
- **Interruption:** before the driver (I1) and while it ran (I2). The trap ran the owned cleanup.
- **Owned cleanup (C2):** with two unowned decoys, one with the E driver's command line. Both survived.
- **Restore arguments before pid files exist (R1–R4):**
  - before enabling: no enabling ref, no pid file;
  - after enabling: the enabling ref only;
  - a partial launch: `launch.pid` only;
  - all recorded.
- **Operator pre-key script:**
  - P1: go;
  - P2: abort on a wrong mode, with no driver or recorder;
  - P3: the run refused first, so no `go.flag`.

## Enabling and B1 build (`enabled/`)
- **Watchdog.** Armed at 11:50:13Z: pid 5343, deadline 13:50:13Z.
- **Enabling commit.** `local/m55-s4d-e-20261111T115013Z` was created from `16d939f` at 11:50:13Z.
  - Commit `01ed29ed70176656a6bcb383628bf0edc8f464c1` applies only `docs/patches/m55_enable_s2.UNAPPLIED.patch` (sha256
    `d1dacac5…`): the M55 gate and its test pin, `1 1` each.
  - **Never pushed:** no upstream, no remote-tracking ref, nothing in `ls-remote` (`never_pushed_check.txt`).
  - The commit is preserved in `enabled/enabling_commit.patch` and `restore/enabling_commit.patch`.
- **B1 build.** A fresh `--symlink-install` build in new bases `/root/spiderx_e_live_enabled_20261111T113831Z`.
  - It ran from a toolchain-only `env -i` shell with the r2 overlay, with `st59site` used for the build only.
  - 8 packages built in 13 s.
- **Enabled tests.** The seven enabled-mode M5.5 files: **245 passed** in 107.8 s. The full historical suite was not rerun.
- **Imports.** From the new installation: M55 `True`, M61 `False`, M6.0-D `False`, through its symlinks into `src/`.
  - Library `62f8a100…`.
  - The enabled `library.json` is `917fc4d7…`, identical to the disabled one.
- **Timing.** The sequence passed in 225 s (`summary.txt`).
- **Pre-launch checks: PASS** (`provenance/prelaunch_checks.txt`).

## Run (`run/`)
| UTC | Event |
|---|---|
| 11:54:20 | Journal: E live launch **used**. Enabling-to-launch 247 s (≤ 900). OD8 6953 s left; OD16 PASS. Launch |
| 11:54:34 | Both controllers active |
| 11:55:20 | Node 7121 up in DISPATCH (gate `True`); levels 1.06, 1.90 and 2.34 mm/s |
| 11:55:23 | PRE-KEY CHECKPOINT; the operator script saw it at 11:55:25, by both the flag and the label |
| 11:55:31–11:56:18 | Pre-key checks **PASS**; `go.flag` |
| 11:56:18–11:56:21 | OD8 before the first key PASS (6835 s left); captures before; OD16 PASS; recorder started |
| 11:56:24–11:58:03 | Driver; exit 0, "script complete" |
| 11:58:09 | Recorder stopped; screenshot 2 |
| 11:58:11 | Launch stop: SIGINT to the launch group |
| 11:58:17 | Simulator server exit 0 (strace); guarded run exit 0; cleanup found nothing |
| 11:58:45 | Watchdog stood down without acting; full restore started |

**Pre-key checks** (`run/premotion/`):
- **Libraries.** Server 7221 alone maps the two r2 libraries, with provenance hashes and no stock copy.
- **Configuration.** Gain 1.0 (log and parameter), `update_rate` 100, period 0.001 s.
- **Ownership.**
  - Both controllers active.
  - Node 7121 is in DISPATCH with the gate `True`, in launch session 7074.
  - It is the action's **only** client (`/spiderx_locomotion`).
  - 0 command publishers; no driver or teleop process.
- **Settling.** 10.08 s of sim, 91 fresh samples: z range 9.0 µm, tilt 1.5e-7 rad. Maximum ages: joint states
  0.065 s, pose 0.186 s, graph 1.005 s.
- **Startup.** 0 `[ERROR]`/`[FATAL]` lines; no process died.

**Script.** The driver's and node's times below are on the host's monotonic clock; sim time comes from joint-state stamps.
1. **Settle**, 10.1 s of sim. Then `+ +` while DISARMED: the teleop confirmed 2/3 (1.90 mm/s), then 3/3 (2.34 mm/s),
   each attributed.
2. **`a`.** DISARMED → READY.
3. **`w` (1371.258).**
   - The node logged `level_change` 0→2 in READY and `goal_sent` seq 1 (level 2, 0→1, the S1 `shift`, 1.9 s,
     fingerprint `748bd1df…`), then STARTING.
   - The controller logged "Received new action goal" and "Accepted new action goal".
   - `goal_response` accepted at 1371.420; WALKING.
4. **Feedback.** The driver received ONE real feedback sample at 1371.425: goal id `b4545785…`, max |error| 0.0 rad.
   It saw goal 1 in flight and accepted at 1371.4587 (sim 31.27).
5. **`x`.**
   - Written 1371.45877–1371.45913, 0.0 s of sim after goal 1 was seen (deadline 0.95 s).
   - At node time 1371.459638 the node logged, in one handling: the `estop` request, the `estop` fault,
     WALKING → FAULTED (why `estop`), and `cancel_sent` for seq 1.
   - **Request to FAULTED: Δ = 0 s on the node clock**, by construction (contract §3), against a limit of ≤ 0.050 s.
   - **Key write to request: 0.51 ms**, recorded only.
6. **Cancellation.**
   - The controller logged "Got request to cancel goal" and "Canceling active action goal because cancel callback
     received." (`launch.log` lines 239–240).
   - `cancel_response` 0 came at +12.5 ms. The result was **CANCELED (5)** at +60.9 ms.
   - No feedback came after the result. No second goal was sent, and nothing was retried.
7. **Hold.** FAULTED with no goal over the scorer's 10 s window [31.29, 41.29] s of sim:
   - joint motion 5.9e-12 rad, tilt 1.5e-7 rad, z ≥ 0.0545 m;
   - controller reference change 0.0 rad.
8. **`q` (1418.620).**
   - `stop` was accepted twice: "not moving (FAULTED); held until a zero command".
   - `disarm` was refused twice with `faulted_reset_required`. These are the only refusals in the run.
   - Teleop 8614 exited 0, 0.25 s after `q`.
   - FAULTED with no goal afterwards: `goals_recorded` 1, `max_seq` 1.

**Goal-1 progress before the cancel** (`analysis/goal_progress_supplementary.json`; supplementary, no verdict):
- The controller had goal 1 for ~88 ms of wall time, from "Accepted" to the cancel request: ~0.04 s of sim, from
  status sim 31.25 to 31.29.
- The reference and actual positions moved ≤ 1.2e-10 rad from the pre-goal pose.
- The feedback sample's desired position equalled the pre-goal reference within 5.8e-11 rad.

**Controller traffic, planned vs actual:**
- **Planned:** 1 goal and 1 cancel.
- **Actual:**
  - the controller log has 1 received, 1 accepted, 1 cancel request;
  - the bag has 1 feedback message, with one goal id, and no `joint_trajectory` message;
  - every status shows 0 foreign clients and 0 publishers.
- **Action clients.** `ros2 action info` listed only `/spiderx_locomotion` before the run. Afterwards it listed
  `/spiderx_locomotion` and `/rosbag2_recorder`; the driver had exited.

## Stop, processes and restoration
Each process's outcome is recorded separately; no exit code is inferred.
- **Simulator server (7221): exited 0** (strace). It received one SIGINT, from 7108, the `ign gazebo` launcher.
  - The launch log has no "Escalating to SIGKILL" line.
  - This shutdown was graceful, unlike the S4d-M live run's.
- **Locomotion node (7121):** "process has finished cleanly", as the launch reports it. Every other launch process
  reported the same.
- **Teleop 8614:** exited 0 by itself (wait status 0), observed by the driver. None was left in its session.
- **GUI (7224):** its own exit status was not observed. Its launcher, `ign gazebo-1` (7106), reported "finished cleanly".
- **Launch.** The process group was empty 4 s after SIGINT. The ros2 daemon and Xvfb were stopped.
- **Cleanup and watchdog.** The trap's owned cleanup found nothing left (`run/cleanup.log`, `run/leftovers_after.txt`).
  The watchdog stood down without acting.
- **Full restore** (`restore/`, 11:58:45Z) against `16d939f`: **DISABLED AND VERIFIED**.
  - The enabling commit and patch were preserved; `src/` equals the base; the incremental rebuild succeeded.
  - In a fresh environment, all three gates were `False`, imported from the workspace.
  - Only then did both M6 `--live` tools refuse, exit 3. No simulation process remained.
  - Only after verification was `local/m55-s4d-e-20261111T115013Z` deleted ("was 01ed29e"). It was never on the remote.
  - The checkout returned to `claude/spiderx-s4d-e-preparation` at `16d939f`, clean, with no `local/*` branch.
- **Run installation gates: PASS**, all three `False`.
- **Both installations, after** (`after/`, `disabled_state_after.txt`), each from a fresh environment:
  - **A** is the run installation, **B** the workspace installation;
  - each imports all three gates `False`, through symlinks into the restored `src/`;
  - for each, both M6 `--live` refusals ran after the gate check;
  - each gives library `62f8a100…` and `library.json` `917fc4d7…`;
  - both installation verdicts are PASS.

## Score (`score/summary.json`, `score_provenance.txt`)
The scorer is `scripts/s4/score_s4d_e.py`, blob `0981d911…`, the same at `16d939f` and `87198b9`. It ran unchanged,
without `--shadow`, and exited 0. Alignment uncertainty: 1.05 ms.

| | Verdict | Detail |
|---|---|---|
| E1 | PASS | Keys `+ + a w x q`; one goal (seq 1, level 2, 0→1 shift); one cancel; requests before `q` exactly `arm`, `estop`; one fault; DISPATCH with the gate `True` in every status; "script complete" |
| E2 | PASS | 2/3 then 3/3, each attributed, while DISARMED before arm; one `level_change` to 2 in READY before goal 1 |
| E3 | PASS | One attributed real feedback sample before the `x` write; `x` while goal 1 was active; within 0.95 s of sim |
| E4 | PASS | One `estop` fault; FAULTED Δ 0 s on the node clock; one cancel in the same handling; teleop `[estop] ok`; latency 0.51 ms, recorded only |
| E5 | PASS | Cancel response 0; result CANCELED; the controller log's one cancel request; no feedback after the result; reference change over the hold 0.0 rad |
| E6 | PASS | 10 s of sim from the result: joint motion 5.9e-12 rad, tilt 1.5e-7 rad, z ≥ 0.0545 m; FAULTED with no goal throughout |
| E7 | PASS | One goal in total; no request between `x` and `q`; no reset, arm or home; final state FAULTED |
| E8 | PASS | 1 attributed sample before the cancel; max \|error\| 0.0 rad, against a 0.05 rad limit and a 0.02 rad finding level. **A single sample.** |
| E9 | PASS | Controller log 1/1/1; one feedback goal id; 0 foreign clients and 0 publishers; captures 0; `joint_trajectory` absent with support |
| E10 | PASS | Exactly `stop` ×2 accepted with the FAULTED message, `disarm` ×2 refused `faulted_reset_required`, the only refusals; no transition after `q`; teleop exit 0 within 15 s |
| E11 | PASS | Minimum joint margin 0.436 rad; tilt 1.5e-7 rad; travel 2.2e-10 m (limit 0.15 m); ages within the monitor limits |
| E12 | PASS | Restore report DISABLED AND VERIFIED; library identity; the run installation imports all three gates `False` |

**Bag hash:** unchanged before the restore, after scoring and after the supplementary analyses (`run/bag_hashes.txt`).

## Findings
- **Goal-1 progress before the cancel.** The body was at rest when it was stopped (see above). It is recorded, not
  re-scored.
- **Real-time factor.** RTF 0.21 (10 s windows 0.19–0.23), compared with 0.59 in the S4d-M live run.
- **`run/captures/top_during.txt`** was taken at 11:58:25. That is after the run ended, because the harness's fixed
  120 s delay outlasted the 100 s driver. It is not informative.
- **Startup.** No `gz_ros2_control` startup `[ERROR]` retries this time; the S4d-M live run had six.

## Files
- **Bag:** kept outside the repository; see `run/bag_retention.txt`. The original bag and `metadata.yaml` were delivered
  as conversation attachments; a durable off-container copy is unconfirmed.
- **Gzipped here:** `run/driver/driver.jsonl` and `run/premotion/maps_full.txt`. Their original hashes and sizes are in
  `provenance/gzipped_originals_SHA256SUMS.txt`. Every other log is unchanged.
- **Operator steps:** `provenance/operator_steps/` holds the operator's commands and outputs: enabling, post-run, scoring
  and the supplementary goal-progress analysis.
- **Journal:** a snapshot of the run journal is in `provenance/`.
- **Manifest:** `SHA256SUMS` covers every file here.
- **Secret scan:** no credential, key or personal address was found in the text files, the gzip contents or the image
  metadata.

## Limits
- **Cloud simulation only.** Ignition Fortress on the RoboStack toolchain, the r2 `gz_ros2_control` overlay (gain 1.0)
  and an idealized actuator model. Nothing was run on the home PC.
- **Automated keyboard path.** The real `m55_teleop_keyboard` was driven through a pseudo-terminal. That is not human
  operation (S4e).
- **One attempt,** at one RTF. The cancel arrived before the shift produced measurable motion.
