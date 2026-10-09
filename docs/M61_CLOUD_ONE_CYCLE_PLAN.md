# M6.1 – ONE fixed-base trot cycle in Cloud simulation: source-verified plan (NOT executed)

**Status: PLAN ONLY.**
- Nothing here is enabled or executed; no goal has been sent.
- The enabling change exists only as an **unapplied** patch,
  [`patches/m61_enable_one_cycle.UNAPPLIED.patch`](patches/m61_enable_one_cycle.UNAPPLIED.patch)
  (SHA-256 `4bd59225f333eab661b4c9d0715bf429bd2c1439510064217dfb359f8f6f2484`).
- `git apply --check` confirmed it applies cleanly; it was never applied.
- Running this plan needs the owner approvals in §1.

Every value below was read from the source at `b910df1` (the `src/` tree is identical to the
tested `7f4c30f`), and file and line references are given. Labels: **[V]** verified in source or
by a command run on the gate-off build; **[P]** a proposed procedure or threshold.

## 1. Preconditions (each is a blocker until done)

1. **Owner review of the Cloud phase-1 closeout**
   ([`M61A_CLOUD_VERIFICATION.md`](M61A_CLOUD_VERIFICATION.md) §8). It includes a decision on
   criterion 6: the aggregate `/scan` pose agreement passes, but under the strict per-beam
   reading of M0 ("every range within ±8 mm") it fails (84–88 % of beams within, systematic, up to
   18 mm).
2. **Owner approval of the phase-2 acceptance criteria** in §6. They extend M6.1-A §10 phase 2.
3. **Owner approval of the enabling patch** and of running it in Cloud, not on the owner PC. In
   Cloud, Gazebo runs on Xvfb with software rendering at a real-time factor of about 0.7.
4. **The pose stream's liveness is unresolved after phase 1** (M61A Cloud report §8.2). This plan
   closes it during the cycle (§6, E9). If it cannot be shown, the body-pose gates are not
   considered verified.

## 2. Exact identity [V]

| Item | Value | Where |
|---|---|---|
| Base branch | `claude/stoic-shannon-ur2mes` at the commit named in the approval (closeout head or later; `src/` must equal `7f4c30f`: `git diff 7f4c30f HEAD -- src/` empty) | — |
| Enabling branch | `local/m61-one-cycle-<UTC>`: **local only, never pushed, never merged**, deleted after the run | §4 |
| Gate | `m61_live_contract.M61_LIVE_DISPATCH_ENABLED` (line 19), pinned by `test/m61_gate.py` (line 7) | patch |
| Other gates | M6.0-D `m6_live_contract.LIVE_DISPATCH_ENABLED` stays `False` (not read by M6.1); the M5.5 gate is not on this branch | `m6_gait_replay._gate_state()` |
| Tool | `ros2 run spiderx_controller m6_gait_replay.py --live --domain-id 0` (`--no-write` is refused with `--live`; the domain must be explicit, 0..232) | `m6_gait_replay.py:571–633` |
| Confirmation word | `SEND-ONE-TROT-CYCLE`, read only after readiness #1 passes | `m61_live_contract.py:26`, `m6_gait_replay.py:609–615` |
| Trajectory | `trajectory_id 241760e7dfd5ef12`; content `94a492c43fcc046125d6bc71ee7f1d9a8a5d8466f1a1bdd9958a16044dd58e64`; goal fingerprint `9dba1a173e212bfc172ffcd7da59124f87e98a321906e914e9d60e55595e5c3a` (dry run on the gate-off build: `Dry run PASS: trajectory 241760e7dfd5ef12, content 94a492c43fcc0461, goal fingerprint 9dba1a17…; nothing sent`) | `M61_IMPLEMENTATION_NOTES.md` §1.1 |
| Motion | 9 points, 0.5 s segments, 7.0 s sim (3.0 s lead-in + 4.0 s cycle), neutral → pair A (LF+RR) swing → pair B (RF+LR) swing → neutral; hips 0; max \|q − neutral\| 0.1114 rad on the spline (cap 0.1223); peak 0.139 rad/s | same |
| Configs (SHA-256) | `config/m61_limits.yaml` `0ff8fc9e…70c7`; `config/m61_trot_cycle.yaml` `b91839fc…4f39`; `config/m61a_fixed_base.yaml` `eb04c612…6ddb` (mount 0.125 m, **provisional**); `config/spiderx_ros2_controllers.yaml` `c449f24f…6e01` (hash-pinned) | `sha256sum` |
| Launch / model | `fortress_m61a_fixed_base.launch.py` `f8d27e70…2504`; `spiderx_fixed_base.urdf.xacro` `b43b91da…469d` | `sha256sum` |
| Harness | The frozen phase-1 tools, `docs/evidence/m61a_cloud/observation_7f4c30f/tools/`, hashes in `docs/evidence/m61a_cloud/closeout/frozen_harness.json` | closeout |

## 3. Readiness and attachment checks the tool enforces [V]

All of these run **before the confirmation and again after it**. The second result must still be
fresh at the send (≤ 10 s from the start of its observation, `READINESS_MAX_AGE_S`).
Assembled in `m6_gait_replay._run_live` (lines 556–563):

| Layer | Requires | Source |
|---|---|---|
| M6.0-D graph readiness | Exactly one action server at `/leg_trajectory_controller/follow_joint_trajectory` of the right type; both controllers `active` (`list_controllers`); one `/joint_states` publisher with ≥ 2 fresh, strictly increasing stamps; 12 named joints, finite; start pose within 0.05 rad of neutral; installed interface contract | `m6_live_preflight.evaluate` |
| Body pose (`with_body_pose`) | A ground-truth body pose ≤ 1.0 s old, z ≥ 0.045 m, tilt ≤ 0.26 rad | `m61_limits.yaml` |
| Fixed base (`with_fixed_base` → `m61a_fixed_base.assess_plant`) | A fixed-base `/robot_description` with the approved mount (0.125 m ≥ the clearance minimum); the Gazebo `dummy_link` entry consistent with it (0.1 mm / 1e-4 rad); model root at the identity spawn (3 mm / 0.01 rad); composed body at the weld pose, attachment ≤ 3 mm / 0.01 rad | `m61a_fixed_base.yaml` |
| Command owner (`with_command_owner`) | 0 publishers on `/leg_trajectory_controller/joint_trajectory`; 0 FollowJointTrajectory clients other than its own (**counted as subscribers of `…/_action/status`**) | `m61a_fixed_base.command_owner_codes` |

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
git status --short && git rev-parse HEAD && git diff 7f4c30f HEAD -- src/ | wc -l     # clean; 0
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
Expected: the five gate-state tests listed in the patch header fail, and every other test passes.
Any other failure stops the procedure. One of the five,
`test_live_cli_with_gate_false_imports_no_ros_client`, runs `main --live --domain-id 0` in a
subprocess with stdin closed. That is why this step runs with no simulation up: with nothing on
domain 0 it can only end NOT READY / REFUSED, and EOF refuses the confirmation anyway.

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
  /leg_trajectory_controller/controller_state /leg_trajectory_controller/joint_trajectory &
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
- `bash $R/tools/stage_stop.sh $R/run`, which ends with no leftover process.

**G. Re-disable (always, whatever the outcome).**
```bash
git switch claude/stoic-shannon-ur2mes && git branch -D $B
rm -rf build install && colcon build --symlink-install && source install/setup.bash
grep -n "^M61_LIVE_DISPATCH_ENABLED" src/spiderx_controller/spiderx_controller/m61_live_contract.py   # False
ros2 run spiderx_controller m6_gait_replay.py --live --domain-id 0 < /dev/null   # REFUSED, exit 3
git status --short && git branch --list 'local/m61-one-cycle-*'                  # clean; none
```
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
| E1 | Phase-1 checks hold in this launch: preflight READY, ownership 0/0 | `run/observer/*/observation.json` (preflight) |
| E2 | Enabled build: exactly the five expected gate-state test failures, no others | `enabled_tests.log` |
| E3 | Identity unchanged: `241760e7dfd5ef12` / `94a492c4…` / `9dba1a17…` on the enabled build | dry run; `trajectory.json`, `goal_fingerprint.txt` |
| E4 | Final state `SUCCEEDED`, exit 0; `goals_sent 1`, `cancels_sent 0`, `retries 0`, `automatic_return_goals 0`; acceptance `accepted`, final goal status 4 | `m61_run/<UTC>/live_outcome.json` (phases reserved → pre_send → final) |
| E5 | Readiness READY before and after confirmation; freshness at the send ≤ 10 s | `readiness_before.json`, `readiness_after.json`, `live_outcome.json` |
| E6 | Tracking: max in-flight \|error\| < 0.05 rad; goal tolerances passed; no G6 sample gap | `live_outcome.json` (`tracking`, `channels`), `commanded_vs_observed.csv` |
| E7 | No gate tripped (G1, G2, G3, G5, G6, G8, pose freshness); G7 drift reported only | `gates.json` |
| E8 | Body stayed welded during the cycle: attachment ≤ 1 mm / 0.0033 rad and z within 1 mm of 0.125 m (the phase-1 rule) | `gates.json` attachment max, `body_pose.csv`, the observer's statistics |
| E9 | **Pose-stream liveness (closes the phase-1 open item):** the Gazebo leg-link entries (e.g. `lf_foot_1` relative to the model) change during the goal and match `leg_kinematics.forward_all` of the time-aligned `/joint_states` within 5 mm / 0.05 rad. The analysis script is written and frozen before the run | `run/bag` + an offline comparison |
| E10 | Zero foreign commanders before and after; no command-topic message from anything | readiness reports; bag (`joint_trajectory` must be empty) |
| E11 | Clean shutdown; gate re-disabled; `--live` exit 3 on the base build; enabling branch deleted | `shutdown.txt`, the §4 G output |

**A failure is reported, not retried.** A cancel path (`GATE_TRIPPED`, `TRACKING_FAILED`,
`READINESS_LOST`) with exactly one cancel is a *safe* outcome but **fails** acceptance. Any
unexplained failure stops the work for review.

**Not shown even by a full pass:** walking, balance, contact or ground interaction (none on a
welded body), and anything about hardware. It would show that one approved trajectory is
dispatched once, tracked and supervised on a welded simulated body, in Cloud.

## 7. Known Cloud-specific risks

- Real-time factor ≈ 0.7: the 7.0 s sim goal takes ≈ 10 s wall. The wall-time monitors
  (pose stale 1.0 s, `/joint_states` stale 0.5 s, sim stall 5.0 s) had large margins in phase 1
  (pose gaps < 0.08 s, joint-state wall gaps < 0.075 s).
- The Gazebo GUI→server startup race (1 of 5 launches): see §4 D.
- `ros2` CLI daemon fault: the frozen wait helper restarts it.
- No competing CPU load during E. The M6.0-D diagnosis (finding 2) showed that a starved
  in-process publisher can trip a sample-gap check. Here the publisher is Gazebo, not Python, but
  the rule stands.
