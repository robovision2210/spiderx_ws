# M6.1 – ONE fixed-base trot cycle in Cloud simulation: source-verified plan (NOT executed)

**Status: PLAN ONLY.**
- Nothing here is enabled or executed; no goal has been sent.
- The enabling change exists only as an **unapplied** patch,
  [`patches/m61_enable_one_cycle.UNAPPLIED.patch`](patches/m61_enable_one_cycle.UNAPPLIED.patch)
  (SHA-256 `b6213f21dedf30e61e4959d22cd45a44525a8bc4432b641134406fe7f3277595`). Only its header
  changed since `06ce0f50…`: the revert instruction now names the restore script (§4 G). Its
  diff body is byte-identical.
- `git apply --check` confirmed it applies cleanly. On this branch it was never applied. It was
  applied only in a temporary worktree to run the tests in the enabled mode; no simulator was
  involved and the worktree was removed (Cloud report §9.4, §10.5).
- Running this plan needs the owner approvals in §1.

Every value below was read from the source at `1732c32`. That commit added the check of the
streams immediately before the send (§3) and the restore-and-verify script (§4 G). `b982d8d`
had added the sim-progress readiness check and mode-explicit gate tests. File and line
references are given. Labels:
**[V]** verified in source or by a command run on the gate-off build; **[P]** a proposed
procedure or threshold.

## 1. Preconditions (each is a blocker until done)

1. **Owner decision on criterion 6** ([`M61A_CRITERION6_V2_PROPOSAL.md`](M61A_CRITERION6_V2_PROPOSAL.md);
   Cloud report §9.6).
   - Version 1, "every range within ±8 mm", **failed** in runs 1–3 and again in `run_04`. No pose
     can satisfy it (minimax 16.7–17.7 mm).
   - The version-2 proposal (fitted planar pose, τ 1.946 mm / 6.103 mrad, frozen at `f44c27c`)
     gave **PASS** on its single validation run `run_04`: |dxy| 0.49 mm, |dyaw| 0.12 mrad, with
     σ 0.58 mm ≤ 0.641 mm.
   - **What the two version-1 figures count** (`run_04`, `criterion6_v2.json`). Both compare
     ranges with those ray-cast from the welded pose, at ±8 mm:
     - 53.1 %: the **13,840 single-scan ranges** (40 scans × 346 beams). Each carries the
       10 mm sensor noise plus a per-beam systematic error of about 4–5 mm RMS.
     - 85.5 % (296 of 346): the **346 per-beam means over the 40 scans**. Averaging cuts the
       noise to 1.6 mm but leaves the systematic part, up to 18.8 mm at a few beams.
     - Neither is a pose result. Even the best planar pose leaves 17.7 mm.
   - **Recommendation: adopt version 2** as a separately versioned planar-pose check, and keep
     version 1 on record as FAIL. Version 2 tests what version 1 meant to test, with stated power.
     Its limitations:
     - planar only (x, y, yaw). It says nothing about z, roll or pitch (item 2, E8);
     - one validation run;
     - `/scan` is rendered from Gazebo's own entity state, so it checks the reporting and
       composition chain, not the physics;
     - static only;
     - the σ budget was 90 % used, so a noisier run can be INCONCLUSIVE;
     - offsets well below G8 (3 mm / 0.01 rad) pass by design.
   - The owner accepts version 2 or keeps version 1. Under version 1, phase 1 stays failed and this
     plan stops here.
2. **Owner decision on z, roll and pitch** (Cloud report §9.2, §10.2, U-L2).
   - No separate sensor observes them, and in simulation none is needed if the value read *is*
     the physics state. `run_06` (§10.2) showed that in this launch:
     - the model entry is DART's pose of the merged body (the leg-link entries were written by
       Physics, which requires the model pose written from DART first);
     - the composition model · `dummy_link` · T_dummy_body cancels the SDF factor exactly;
     - the value is reported again whenever the body moves more than 1e-6 from its last
       report.
   - **Recommendation: accept the simulator physics state as ground truth for z, roll and pitch.**
     The trust boundary is DART itself. The canonical re-report after the first write was
     exercised only in the free-base M2 launches, because a welded body never moves.
   - The alternative is an added sensor, which is a model change.
3. **Owner approval of the phase-2 acceptance criteria** in §6. They extend M6.1-A §10 phase 2.
4. **Owner approval of the enabling patch** and of running it in Cloud, not on the owner PC. In
   Cloud, Gazebo runs on Xvfb with software rendering at a real-time factor of about 0.7.
5. **Pose-stream content liveness** (U-L1) closes only in motion (§6, E9).
   - Before dispatch, readiness now requires simulation time to advance while the body pose is
     received (§3). A paused world is refused although its streams are fresh (Cloud report §9.5).
   - Immediately before the send, the streams are checked again as they are *now* (§3, E5).
   - E9's analysis is frozen in `927acb2`: `spiderx_controller.m61a_link_check`, SHA-256
     `649b0c4f…7059e31`. If E9 cannot be shown, the leg-link stream is not considered live.
     E9 does not bear on body height or tilt; that is E8.

## 2. Exact identity [V]

| Item | Value | Where |
|---|---|---|
| Base branch | `claude/stoic-shannon-ur2mes` at the commit named in the approval (`1732c32` or later; `src/` must equal `1732c32`: `git diff 1732c32 HEAD -- src/` empty) | — |
| Enabling branch | `local/m61-one-cycle-<UTC>`: **local only, never pushed, never merged**, deleted after the run | §4 |
| Gate | `m61_live_contract.M61_LIVE_DISPATCH_ENABLED` (line 19), pinned by `test/m61_gate.py` (line 7) | patch |
| Other gates | M6.0-D `m6_live_contract.LIVE_DISPATCH_ENABLED` stays `False` (not read by M6.1); the M5.5 gate is not on this branch | `m6_gait_replay._gate_state()` |
| Tool | `ros2 run spiderx_controller m6_gait_replay.py --live --domain-id 0` (`--no-write` is refused with `--live`; the domain must be explicit, 0..232) | `m6_gait_replay._live_main` |
| Confirmation word | `SEND-ONE-TROT-CYCLE`, read only after readiness #1 passes | `m61_live_contract.CONFIRMATION_WORD`, `M61Session._run` step 3 |
| Trajectory | `trajectory_id 241760e7dfd5ef12`; content `94a492c43fcc046125d6bc71ee7f1d9a8a5d8466f1a1bdd9958a16044dd58e64`; goal fingerprint `9dba1a173e212bfc172ffcd7da59124f87e98a321906e914e9d60e55595e5c3a` (dry run on the gate-off build: `Dry run PASS: trajectory 241760e7dfd5ef12, content 94a492c43fcc0461, goal fingerprint 9dba1a17…; nothing sent`) | `M61_IMPLEMENTATION_NOTES.md` §1.1 |
| Motion | 9 points, 0.5 s segments, 7.0 s sim (3.0 s lead-in + 4.0 s cycle), neutral → pair A (LF+RR) swing → pair B (RF+LR) swing → neutral; hips 0; max \|q − neutral\| 0.1114 rad on the spline (cap 0.1223); peak 0.139 rad/s | same |
| Configs (SHA-256) | `config/m61_limits.yaml` `0ff8fc9e…70c7`; `config/m61_trot_cycle.yaml` `b91839fc…4f39`; `config/m61a_fixed_base.yaml` `1c7966fc…40b9` (mount 0.125 m, **provisional**; adds `progress_window_s` 1.0 and `min_sim_advance_s` 0.1); `config/spiderx_ros2_controllers.yaml` `c449f24f…6e01` (hash-pinned) | `sha256sum` |
| Launch / model | `fortress_m61a_fixed_base.launch.py` `f8d27e70…2504`; `spiderx_fixed_base.urdf.xacro` `b43b91da…469d` | `sha256sum` |
| Harness | The frozen phase-1 tools, `docs/evidence/m61a_cloud/observation_7f4c30f/tools/`, hashes in `docs/evidence/m61a_cloud/closeout/frozen_harness.json` | closeout |

## 3. Readiness and attachment checks the tool enforces [V]

All of these run **before the confirmation and again after it**. The second result must still be
fresh at the send (≤ 10 s from the start of its observation, `READINESS_MAX_AGE_S`).
Assembled in `m6_gait_replay._run_live`:

| Layer | Requires | Source |
|---|---|---|
| M6.0-D graph readiness | Exactly one action server at `/leg_trajectory_controller/follow_joint_trajectory` of the right type; both controllers `active` (`list_controllers`); one `/joint_states` publisher with ≥ 2 fresh, strictly increasing stamps; 12 named joints, finite; start pose within 0.05 rad of neutral; installed interface contract | `m6_live_preflight.evaluate` |
| Body pose (`with_body_pose`) | A ground-truth body pose ≤ 1.0 s old, z ≥ 0.045 m, tilt ≤ 0.26 rad | `m61_limits.yaml` |
| Fixed base (`with_fixed_base` → `m61a_fixed_base.assess_plant`) | A fixed-base `/robot_description` with the approved mount (0.125 m ≥ the clearance minimum); the Gazebo `dummy_link` entry consistent with it (0.1 mm / 1e-4 rad: a conversion and association check, since Gazebo never writes the canonical link's pose); model root at the identity spawn (3 mm / 0.01 rad); composed body (the physics pose of the merged body) at the weld pose, attachment ≤ 3 mm / 0.01 rad | `m61a_fixed_base.yaml` |
| Sim progress (`with_sim_progress` → `m61a_fixed_base.check_sim_progress`) | Over the last 1.0 s of wall time, the transport's `/clock` at receipt of the usable body-pose samples rose by ≥ 0.1 s (a real-time factor ≥ 0.1), with no step back. A paused world keeps `/clock` and `pose/info` arriving unchanged, so receipt freshness alone cannot show progress. Code `body_pose_sim_time_not_advancing` | `m61a_fixed_base.yaml` `freshness` |
| Command owner (`with_command_owner`) | 0 publishers on `/leg_trajectory_controller/joint_trajectory`; 0 FollowJointTrajectory clients other than its own (**counted as subscribers of `…/_action/status`**) | `m61a_fixed_base.command_owner_codes` |

**Immediately before the send [V] (`1732c32`).**
- **What the 10 s rule does not cover.** It bounds the age of the readiness *result*. It does not
  show that the streams are still current: nothing spins during the confirmation prompt or the
  server wait, and the in-flight monitors start only at acceptance.
- **The added check.** `M61Session._fresh_at_send` applies the M6.0-D rule, then
  `m61a_fixed_base.streams_at_send` on `M61AFixedBaseTransport.streams_now()`:
  1. drain what was queued (a backlog processed late gets the processing time as its receipt
     time and would look fresh);
  2. observe for one progress window (1.0 s);
  3. require, with the readiness thresholds:
     - joint states received in the window, ≤ 0.5 s old and complete;
     - a usable body pose received in the window, ≤ 1.0 s old;
     - sim time advancing ≥ 0.1 s over the window;
     - the plant checks, attachment included.
- **On failure.** The code is recorded under `freshness_at_send.streams_now`, nothing is sent and
  there is no retry. Mock scenarios: `joint_states_stop_before_send`,
  `body_pose_stops_before_send` and `sim_pauses_before_send` (refused); `slow_server` (sent).

**Consequence for evidence recording [V].** Nothing else may subscribe to
`/leg_trajectory_controller/follow_joint_trajectory/_action/status` while the tool runs. A bag or
capture of that topic counts as a competing client, and readiness would refuse. The phase-1
`capture.py` subscribes to it, so it is used only **after** the tool has exited.

**In flight:**
- G1 body height (z < 0.045 m, 2 samples);
- G2 tilt (> 0.26 rad, 2 samples);
- G3 joint above 80 % of its URDF limit (1 sample);
- G8 attachment (> 3 mm / 0.01 rad, 2 samples);
- body-pose freshness (1.0 s wall);
- G5 sim stall (5.0 s wall);
- G6 joint-state sample gap (0.25 s sim);
- `/joint_states` staleness (0.5 s wall);
- client tracking abort (\|error\| > 0.05 rad against the controller's cubic Hermite);
- controller presence (every 1.0 s);
- G7 drift (report only).

Sources: `m61_limits.yaml`, `m61a_fixed_base.yaml`, `m6_live_contract.py:25–49`.

## 4. Procedure (Cloud), exact commands [P]

Everything runs in `~/spiderx_ws` with the frozen harness copied to a fresh run directory:
`R=$HOME/spiderx_evidence/m61_one_cycle_$(date -u +%Y%m%dT%H%M%SZ); mkdir -p $R; cp -a docs/evidence/m61a_cloud/observation_7f4c30f/tools $R/tools`.
Check the copied tools against `frozen_harness.json` before use. Every shell sources
`$R/tools/env.sh` (domain 0, localhost only, `DISPLAY=:77`).

**A. Gate-off checks on the base commit** (record all output in `$R/pre/`):
```bash
git status --short && git rev-parse HEAD && git diff 1732c32 HEAD -- src/ | wc -l     # clean; 0
pgrep -af "ign gazebo|gz sim|ros2|parameter_bridge|controller_manager|Xvfb" || echo none
rm -rf build install && colcon build --symlink-install && source install/setup.bash
ros2 run spiderx_controller m6_gait_replay.py --dry-run --no-write          # 241760e7dfd5ef12 / 9dba1a17…
echo SEND-ONE-TROT-CYCLE | ros2 run spiderx_controller m6_gait_replay.py --mock --scenario success
echo SEND-ONE-TROT-CYCLE | ros2 run spiderx_controller m6_gait_replay.py --mock --scenario attachment_drift
ros2 run spiderx_controller m6_gait_replay.py --live --domain-id 0 < /dev/null   # REFUSED, exit 3
```

**B. Enabling branch (local only).**
```bash
B=local/m61-one-cycle-$(date -u +%Y%m%dT%H%M%SZ)
git switch -c $B
git apply docs/patches/m61_enable_one_cycle.UNAPPLIED.patch
git diff --stat           # exactly 2 files, 2 lines
git commit -am "LOCAL ONLY: enable ONE M6.1 trot cycle (never push, never merge)"
git rev-parse HEAD > $R/enabling_commit.txt
```

**C. Enabled-build checks, with NO simulation running.**
```bash
colcon build --symlink-install && source install/setup.bash
grep -n "^M61_LIVE_DISPATCH_ENABLED\|^EXPECTED_M61" \
  src/spiderx_controller/spiderx_controller/m61_live_contract.py \
  src/spiderx_controller/test/m61_gate.py                         # both True
grep -n "^LIVE_DISPATCH_ENABLED" src/spiderx_controller/spiderx_controller/m6_live_contract.py   # False
ros2 run spiderx_controller m6_gait_replay.py --dry-run --no-write          # identity UNCHANGED (E3)
cd src/spiderx_controller && python3 -m pytest test/test_m61_gait_replay.py -p no:cacheprovider \
  --basetemp=$R/enabled_tests -rA > $R/enabled_tests.log; cd -
```
Expected: **every test passes**; any failure stops the procedure.
- The gate tests are mode-explicit: the pin, the disabled path with the gate set False, the
  enabled path with fakes only.
- The `--live` subprocess test sets the gate False inside the child, so no test can reach a ROS
  graph.
- The step still runs with no simulation up, as a precaution.
- Verified in Cloud with the patch applied in a temporary worktree: 1457 passed, 0 failed
  (Cloud report §9.4); again on `1732c32`: 1517 passed, 0 failed (§10.6).

**D. Launch and preflight** (frozen harness):
```bash
bash $R/tools/stage_start.sh $R/run && bash $R/tools/stage_wait_controllers.sh $R/run
bash $R/tools/stage_observe.sh $R/run preflight          # READY, exit 0 (stop if not)
```
If `create` dies, that is the Gazebo GUI→server startup race. Record it as a failed launch, stop
cleanly, and relaunch **once** after recording it (no observation or goal ran).

**E. Recording and the one goal.**
```bash
source $R/tools/env.sh
ros2 bag record -s sqlite3 -o $R/run/bag /spiderx/sim/world_poses /joint_states /clock \
  /robot_description /leg_trajectory_controller/controller_state \
  /leg_trajectory_controller/joint_trajectory &
BAG=$!                                                   # NOT the _action/status topic (§3)
ros2 run spiderx_controller m61a_observe_fixed_base --domain-id 0 --duration 90 \
  --out $R/run/observer > $R/run/observe_during.txt 2>&1 &
OBS=$!
mkfifo $R/run/confirm
ros2 run spiderx_controller m6_gait_replay.py --live --domain-id 0 --out $R/run/m61_run \
  < $R/run/confirm > $R/run/live.txt 2>&1 &
TOOL=$!
exec 3> $R/run/confirm                                   # the operator's keyboard
# wait until live.txt shows "Readiness passed (including a fresh body pose and the M6.1-A
# fixed-base checks). Type SEND-ONE-TROT-CYCLE ..."; inspect it; then, and only then:
printf 'SEND-ONE-TROT-CYCLE\n' >&3
wait $TOOL; echo "exit=$?" >> $R/run/live.txt; exec 3>&-
```
- To refuse at the prompt, close the pipe (`exec 3>&-`): EOF refuses and nothing is sent.
- To stop during flight, send SIGINT to `$TOOL`: exactly one cancel follows.
- **Never** rerun the tool in this launch. A second goal is impossible by construction
  (`APPROVED_GOAL_COUNT = 1`, the single-use transport, fingerprint verification before any ROS
  call), and procedurally forbidden.

**F. After the tool exits.** Wait for the observer to finish. Then:
- `kill -INT $BAG`;
- `bash $R/tools/stage_capture.sh $R/run`, read-only and now safe to subscribe to the status
  topic: `ros2 action info` must show 0 clients;
- `bash $R/tools/stage_views.sh $R/run`;
- `bash $R/tools/stage_stop.sh $R/run`, which ends with no leftover process;
- E8 and E9, offline, with the analysis frozen in `927acb2` (SHA-256 `649b0c4f…7059e31`, checked
  first):
  ```bash
  sha256sum src/spiderx_controller/spiderx_controller/m61a_link_check.py
  python3 -m spiderx_controller.m61a_link_check --bag $R/run/bag --out $R/run/link_check \
    --server-log $R/run/launch.log
  ```

**G. Re-disable and prove it (always: success, failure or interruption).** Switching branches
is not proof. `scripts/m61_restore_and_verify_disabled.sh` (`1732c32`) does the following:
- stops what is left: the launch group, the virtual display and the CLI daemon;
- checks that no simulation process remains;
- **preserves the enabling commit before anything is deleted**: its id, `git show`,
  `format-patch` and the SHA-256 of its `src/` diff;
- checks out the base and requires `src/` to equal it;
- rebuilds cleanly;
- then verifies, in a fresh environment run from `/`:
  - the imported gate modules come from this workspace and are both `False`;
  - `ros2 pkg prefix` points to this workspace;
  - **only then**, `--live` exits 3 with HARD-DISABLED;
  - the operator's own shell imports nothing enabled.

The enabling branch is deleted only on request, and only after every check has passed. Run it
from the workspace root:
```bash
scripts/m61_restore_and_verify_disabled.sh --base claude/stoic-shannon-ur2mes --evidence $R/restore \
  --enabling-ref $B --launch-pid-file $R/run/launch.pid --xvfb-pid-file $R/run/xvfb.pid \
  --build-cmd '<the workspace build; in this Cloud image with the setuptools 59 path first on PYTHONPATH>'
# exit 0 and "VERDICT: DISABLED AND VERIFIED" required. Exit 1: stop; fix; re-run.
# Then, and only then, optionally re-run with --delete-enabling-branch (identity already in $R/restore).
git status --short && git branch --list 'local/m61-one-cycle-*'
```
- **Validated in Cloud** (Cloud report §10.5, `docs/evidence/m61a_cloud/restore_1732c32/`):
  - against a running no-motion launch: stopped, rebuilt, `DISABLED AND VERIFIED`;
  - against a temporary worktree with the patch applied and built: `NOT VERIFIED`, and
    `--live` was never run.
- **If the container is lost mid-run**, nothing enabled survives anywhere: the enabling branch was
  local only, and every pushed branch has both gates `False`. The evidence in `$R` is lost too.
  The run is not repeated without a new authorization.

Copy `$R` to `docs/evidence/m61_cloud_one_cycle/<UTC>/` with a README and `SHA256SUMS`, without
the enabling branch, which was never pushed. Then commit the evidence on the base branch.

## 5. One goal, no retry, no return goal [V]

- The approved goal itself starts and ends at neutral: the return is waypoint 8 of the same goal,
  not a second goal.
- `APPROVED_GOAL_COUNT = 1`. The transport is single-use (`SecondGoalForbidden`) and verifies the
  fingerprint before any ROS call, refusing every other goal, including a return-to-neutral, a
  second cycle and the M6.0-D crouch.
- There is no retry path. The outcome records `goals_sent`, `cancels_sent`, `retries` and
  `automatic_return_goals`.
- At most one cancel is sent, for the first reason; later trips are recorded in
  `gates.trips_in_order`.

## 6. Acceptance and evidence [P] (owner to approve)

| # | Requirement | Evidence |
|---|---|---|
| E1 | Phase-1 checks hold in this launch: preflight READY (including `sim_clock` advancing and `body_pose_sim_progress`), ownership 0/0 | `run/observer/*/observation.json` (preflight) |
| E2 | Enabled build: every test of `test_m61_gait_replay.py` passes (no expected failures; any failure stops) | `enabled_tests.log` |
| E3 | Identity unchanged: `241760e7dfd5ef12` / `94a492c4…` / `9dba1a17…` on the enabled build | dry run; `trajectory.json`, `goal_fingerprint.txt` |
| E4 | Final state `SUCCEEDED`, exit 0; `goals_sent 1`, `cancels_sent 0`, `retries 0`, `automatic_return_goals 0`; acceptance `accepted`, final goal status 4 | `m61_run/<UTC>/live_outcome.json` (phases reserved → pre_send → final) |
| E5 | Readiness READY before and after confirmation. The readiness composition includes `with_sim_progress`, so READY means sim time advanced ≥ 0.1 s within the last 1.0 s while the body pose arrived (a failure is listed as `body_pose_sim_time_not_advancing`). **And at the send, both of:** (a) the readiness result ≤ 10 s old (M6.0-D); (b) the streams *now* (§3): joint states, body pose and sim progress observed in a 1.0 s window after draining the backlog, plant and attachment included, `streams_now.ok` true. (a) does not replace (b): a ≤ 10 s snapshot says nothing about whether the streams are current at the send | `readiness_before.json`, `readiness_after.json` (`ready`, `failure_codes`; the per-layer report is not recorded there), `live_outcome.json` (`freshness_at_send`, incl. `streams_now`) |
| E6 | Tracking: max in-flight \|error\| < 0.05 rad; goal tolerances passed; no G6 sample gap | `live_outcome.json` (`tracking`, `channels`), `commanded_vs_observed.csv` |
| E7 | No gate tripped (G1, G2, G3, G5, G6, G8, pose freshness); G7 drift reported only | `gates.json` |
| E8 | Body stayed welded during the cycle, **measured from the physics state**. The body is the `spiderx` model entry (DART's pose of the merged body) · `dummy_link` entry (the SDF X_ML that Physics divided by, so it cancels) · T_dummy_body (URDF, identity, same rigid body). It must stay within 1 mm / 0.0033 rad of the description's weld origin, z within 1 mm of 0.125 m (the phase-1 rule). **Provenance in this launch** (`m61a_link_check`, frozen): the leg-link entries equal URDF FK at the measured joints and differ from FK(0). In gz-sim 6.16 that requires the model entry to have been written from DART first. The SDF weld value is never used as the measurement | `gates.json` attachment max, `body_pose.csv`, observer statistics; `run/link_check/link_check.json` (`e8_body_pose`, `e8_provenance` PASS, no contradicting server-log line) |
| E9 | **Dynamic link/kinematic consistency of the pose stream** (content liveness, U-L1). The 12 leg-link entries, relative to the `dummy_link` entry, change during the goal: every foot link turns ≥ 0.02 rad. In every sample they match URDF FK at `/joint_states` interpolated to the sample's `/clock` time (±25 ms) within 1 mm / 5 mrad; there are ≥ 100 samples and no gap > 1.0 s. The frozen stale-stream counterfactual must fail. **This is not a measurement of global body height or tilt**: the entries are relative to the body (E8 covers the body). Analysis frozen before any motion run: `927acb2`, `m61a_link_check.py` SHA-256 `649b0c4f…7059e31` | `run/bag`; `run/link_check/link_check.json` (`e9_link_consistency` PASS) |
| E10 | Zero foreign commanders before and after; no command-topic message from anything | readiness reports; bag (`joint_trajectory` must be empty) |
| E11 | After success, failure or interruption: `scripts/m61_restore_and_verify_disabled.sh` ends `DISABLED AND VERIFIED`. That covers:<br>- no simulation process left;<br>- the enabling commit preserved (id, show, patch, `src/` diff SHA-256) **before** any deletion;<br>- `src/` equal to the base, and a clean rebuild;<br>- in a fresh environment, the **imported** gate modules from this workspace and `False`, `ros2 pkg prefix` here, `--live` exit 3 with HARD-DISABLED;<br>- the operator's shell clean.<br>The local enabling branch is deleted only afterwards. A branch switch alone is not accepted as proof | `$R/restore/restore_verify_<UTC>.txt`, `enabling_commit*.{txt,patch}`, `enabling_src_vs_base.diff`, `rebuild_<UTC>.log`, `shutdown.txt` |

**A failure is reported, not retried.** A cancel path (`GATE_TRIPPED`, `TRACKING_FAILED`,
`READINESS_LOST`) with exactly one cancel is a *safe* outcome but **fails** acceptance. Any
unexplained failure stops the work for review.

**Not shown even by a full pass:** walking, balance, contact or ground interaction (none on a
welded body), and anything about hardware. It would show that one approved trajectory is
dispatched once, tracked and supervised on a welded simulated body, in Cloud.

## 7. Known Cloud-specific risks

- Real-time factor ≈ 0.7: the 7.0 s sim goal takes ≈ 10 s wall. The wall-time monitors are pose
  stale 1.0 s, `/joint_states` stale 0.5 s and sim stall 5.0 s.
  - They had large margins in runs 1–3: pose gaps < 0.08 s, joint-state wall gaps < 0.075 s.
  - `run_04` had one 0.34 s pose gap (p99.9 0.058 s). That is still below 1.0 s, but a reminder
    to keep the machine quiet during E.
- The Gazebo GUI→server startup race (1 of 5 launches): see §4 D.
- `ros2` CLI daemon fault: the frozen wait helper restarts it.
- No competing CPU load during E. The M6.0-D diagnosis (finding 2) showed that a starved
  in-process publisher can trip a sample-gap check. Here the publisher is Gazebo, not Python, but
  the rule stands.
