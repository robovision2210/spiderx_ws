# M6.1: the one authorized fixed-base trot cycle in Cloud, attempt `20261012T025531Z`

**Verdict: FAIL. The tool refused before dispatch: 0 goals, 0 cancels.**
- The refusal came at readiness #1, before the confirmation prompt. Its code was
  `action_server_missing`, so the classification was `incompatible` and the reason
  `stack_incompatible`. The tool exited 2.
- The prompt was never printed, so the confirmation word was never written. Nothing was sent to
  the controller.
- The authorization allowed one attempt. It was not retried and is now spent.
- Cleanup ended `DISABLED AND VERIFIED`, twice.

What this run does **not** show: anything about tracking, the gates under motion, E9, walking,
contact or hardware.

The authorization is in `authorization.txt`. The plan is
[`docs/M61_CLOUD_ONE_CYCLE_PLAN.md`](../../../M61_CLOUD_ONE_CYCLE_PLAN.md) at `8025668`, and the
report is in [`docs/M61A_CLOUD_VERIFICATION.md`](../../../M61A_CLOUD_VERIFICATION.md) §11.

## 1. Sequence (UTC, 2026-10-12)

| Time | Step | Record |
|---|---|---|
| 02:55:31 | Attempt opened; no earlier attempt found (local and remote branches, PR robovision2210/spiderx_ws#17) | `authorization.txt` |
| A | Base `8025668` = origin, no diff. Patch SHA-256 `b6213f21…7595` and `git apply --check` OK. Link check `649b0c4f…9e31`, restore script `e6fe9ee1…8257`, 6 config, launch and model hashes = plan. Gate-off checks: dry run PASS (full identities); mock `success` SUCCEEDED; mock `attachment_drift` GATE_TRIPPED with 1 cancel; `--live` exit 3 | `pre/` |
| B | Local branch `local/m61-one-cycle-20261012T025531Z`, never pushed or merged. Commit `9804c35`: the 2-line patch | `enabling_*.txt`, `enabling_applied.diff` |
| C | Enabled build (no simulator). Gates `True`, pin `True`, M6.0-D `False`. Identity unchanged. **156 passed, 0 failed** | `C_enabled_*`, `enabled_tests.log` |
| 02:58:22 | Launch `fortress_m61a_fixed_base.launch.py`, `ROS_LOCALHOST_ONLY=1`, domain 0 | `run/launch.log`, `run/env_*.txt` |
| 02:58:31–48 | `create` asked for the world 4 times (cold start), then created the entity. It did not die, so this is **not** the documented race: no relaunch | `run/launch.log` lines 43–54 |
| 02:58:43 | `ros2` CLI daemon fault; the frozen wait helper restarted it | `run/daemon_events.txt` |
| 02:58:51–53 | `gz_ros2_control` waited 5 × 0.5 s for the `robot_state_publisher` parameter service, then connected | `run/launch.log` lines 56–63 |
| 02:59:09.7 | Controllers active | `run/controllers_active*.txt` |
| 02:59:26 | Preflight observer **READY**, 12 of 12 checks | `run/preflight.txt`, `run/observer/20261012T025926Z/` |
| 03:00:21 | Bag recording started | `run/bag_record.log` |
| 03:00:23.5 | 90 s observer and live tool started together (`tools/run_one_goal_start.sh`) | `run/tool_start_utc.txt` |
| +0.16 s | `PREFLIGHTED`: trajectory `241760e7dfd5ef12`, fingerprint `9dba1a17…` | `live_outcome.json` `events` |
| +4.94 s | Readiness #1 (collected for 4.78 s): **NOT READY**, `action_server_missing`. `REFUSED`, before the prompt | `readiness_before.json` |
| 03:00:29.9 | Tool exit 2 | `run/live.txt`, `run/tool_end_utc.txt` |
| 03:01:56 | Observer ends READY: 0 command publishers, 0 action clients | `run/observer/20261012T030156Z/` |
| 03:01:59 | Bag stopped: 83,469 messages | `run/bag.tar.xz` |
| 03:02:27–03:04:22 | Captures, a read-only discovery replay (§2, 03:03:42), 4 GUI views | `run/captures/` |
| 03:04:27 | Stop: SIGINT to the launch group, empty after 5 s, daemon stopped, nothing left | `run/shutdown.txt` |
| 03:05:35 | Frozen link check on the bag | `run/link_check/` |
| 03:05:48 | Restore run 1 (rebuild): **DISABLED AND VERIFIED**; branch kept | `restore_run1.out`, `restore/restore_verify_20261012T030548Z.txt` |
| 03:06:10 | Restore run 2 (`--verify-only --delete-enabling-branch`): **DISABLED AND VERIFIED**; branch then deleted | `restore_run2.out`, `restore/restore_verify_20261012T030610Z.txt` |

## 2. The refusal: what failed, and why it is most likely a discovery false negative

**Only one check failed.** `readiness_before.json` lists exactly one failure code,
`action_server_missing`.
- Every readiness layer adds its own codes **unconditionally**. That covers the M6.0-B checks
  (`m6_live_preflight.evaluate`) and the four M6.1 wrappers in `m6_gait_replay.py`:
  `with_body_pose`, `with_fixed_base`, `with_sim_progress` and `with_command_owner`.
- So the controllers, the joint-state source, freshness, names and start pose, the body pose,
  the fixed-base plant, sim progress and the command owners all passed in that same observation.
- The per-layer report itself is not stored (plan E5 already notes this).

**The action server existed.**
- After the run, `run/captures/action_info.txt` shows 1 server, `/leg_trajectory_controller`,
  and 0 clients.
- The controllers had been active for 74 s when the tool started.
- The bag holds 2,880 `controller_state` messages.

**The mechanism.**
- `GraphProbe.collect` spins a fixed `discovery_s` = 2.0 s on the tool's new node
  (`m6_live_preflight.py` 418). It then queries the action servers **once**, by node
  (`action_servers`, lines 374–381), with no wait.
- The next check, `list_controllers`, waits up to 10 s for its service. The action check has no
  such wait.

**A read-only replay of that query** (`tools/probe_action_discovery.py`): three fresh nodes,
after the attempt, launch still up and the graph quiet.
- The server was first seen at **0.254, 0.505 and 1.518 s**.
- In trial 0, only 7 of the 9 nodes were known at 2.0 s; the ninth appeared at 3.27 s.
- During the attempt, the recorder, the observer and the tool all started within about 3 s,
  while Gazebo ran at a real-time factor of about 0.7. That adds discovery traffic and CPU load.

**Conclusion.** A discovery-latency false negative in a single fixed-time snapshot is
consistent with every record. It is **not proven**: no per-query timing was recorded. Nothing
points to a fault in the controller or the launch.

## 3. E1–E11 (plan §6). Missing evidence is UNMEASURED, never PASS

All paths are relative to this folder. `M` = `run/m61_run/20261012T030023Z/`. The same table, in
machine-readable form, is in `requirements.json`.

| # | Result | Basis | Evidence |
|---|---|---|---|
| E1 | **PASS** | Preflight READY, 12 of 12, including `sim_clock` and `body_pose_sim_progress`. Ownership 0 / 0 at start and end of the window | `run/preflight.txt`, `run/observer/20261012T025926Z/observation.json` |
| E2 | **PASS** | Enabled build: `test_m61_gait_replay.py` 156 passed, 0 failed | `enabled_tests.log`, `C_enabled_checks.txt`, `C_enabled_build.log` |
| E3 | **PASS** | Enabled dry run and the live records carry `241760e7dfd5ef12`, `94a492c4…8e64` and `9dba1a17…5c3a`, matching the approved full hashes | `C_enabled_checks.txt`; `M/trajectory.json`, `M/goal_fingerprint.txt`, `M/live_outcome.json` |
| E4 | **FAIL** | `REFUSED` (`stack_incompatible`), exit 2. `goals_sent` 0, `cancels_sent` 0, `retries` 0, `automatic_return_goals` 0; acceptance `not_attempted` | `run/live.txt`, `M/live_outcome.json` |
| E5 | **FAIL** | Readiness before confirmation was NOT READY (`incompatible`, `action_server_missing`). Readiness after confirmation and the check at the send never ran: `readiness_after.json` is `null` and `freshness_at_send` is `null` | `M/readiness_before.json`, `M/readiness_after.json`, `M/live_outcome.json` |
| E6 | UNMEASURED | No goal: 0 tracking samples; channels `unavailable`. `commanded_vs_observed.csv` has its header only | `M/live_outcome.json`, `M/commanded_vs_observed.csv` |
| E7 | UNMEASURED | Every gate shows `tripped: false` but observed nothing (0 body samples, `worst_observed` null). Never supervising is not passing | `M/gates.json` |
| E8 | UNMEASURED (cycle) | No cycle ran. See the at-rest supplement below | `run/link_check/`, `run/observer/20261012T030156Z/` |
| E9 | UNMEASURED | `NO_MOTION`: link poses consistent with the joints, but no foot link turned 0.02 rad, and E9 needs motion | `run/link_check/link_check.json` |
| E10 | **PASS** (attempt window) | See the E10 basis below | `run/observer/*/observation.json`, `run/captures/{command_topic_info,action_info}.txt`, `run/bag_record.log`, `run/bag.tar.xz`, `M/readiness_before.json` |
| E11 | **PASS** | Both restore runs `DISABLED AND VERIFIED` (§4) | `restore/`, `restore_run1.out`, `restore_run2.out`, `run/shutdown.txt` |

**E8 supplement, at rest in this launch.** It does not replace the cycle measurement.
- Link check E8 body: PASS, deviation 0 m and 0 rad over 5,529 pose messages.
- E8 provenance: PASS. 5,408 of 5,408 samples are discriminating; residual against FK(q) is at
  most 7.6e-8 m and 7.5e-7 rad; residual against FK(0) is at least 9.6e-5 rad.
- Observer: 5,179 body samples, z 0.125 m, tilt 0, attachment 0.

**E10 basis.**
- **Before.** The observer saw 0 command publishers and 0 action clients at 02:59:26.
- **During the window.** The observer's start snapshot at about 03:00:26 shows 1 action client.
  That is the tool's own `FollowJointTrajectory` client: `m6_live_adapter.py` 102 creates it at
  open, and the tool ran 03:00:23–03:00:30.
- **After.** The observer saw 0 / 0 at 03:01:56. Captured after the run: command topic
  0 publishers, action 0 clients.
- **No command message.** The recorder never subscribed to
  `/leg_trajectory_controller/joint_trajectory`, because no publisher ever appeared on it.
- **Readiness #1** also reported no command-owner code. Under the discovery hypothesis of §2,
  that snapshot is weaker evidence than the observer's.
- `readiness_after` was never taken.

## 4. Cleanup (E11)

**Run 1** (`--enabling-ref`, launch and Xvfb pid files, `--build-cmd`).
1. **Stop.** The launch group and Xvfb were already gone, and no simulation process was found.
2. **Preserve.** Recorded before anything was deleted: `local/m61-one-cycle-20261012T025531Z` =
   `9804c3522f785f0bbd11bc9045acadf318bafa08`. The diff of `src/` against the base has SHA-256
   `8604d06b…fff3`, the same as the patch applied in the earlier test worktree. Saved files:
   `restore/enabling_commit.{txt,patch}`, `enabling_commit_show.txt` and
   `enabling_src_vs_base.diff`.
3. **Restore.** `HEAD` = `8025668` (`claude/stoic-shannon-ur2mes`), and `src/` equals the base.
4. **Rebuild.** Clean rebuild (`rm -rf build install`); log in `restore/rebuild_20261012T030548Z.log`.
5. **Verify, in a fresh environment.**
   - a. `M61_LIVE_DISPATCH_ENABLED = False` and `LIVE_DISPATCH_ENABLED = False`, both imported
     from this workspace;
   - b. `ros2 pkg prefix` resolves here;
   - c. `--live` refused with HARD-DISABLED, **exit 3**;
   - d. the calling shell cannot import the package.

   Verdict: `DISABLED AND VERIFIED`.

**Run 2** (`--verify-only --delete-enabling-branch`). It repeated steps 1–3 and 5 with the same
results. Only after that did it delete the local branch. Verdict: `DISABLED AND VERIFIED`.

**Afterwards.**
- No `local/*` branch exists, locally or on the remote. `git worktree list` shows only the main
  checkout.
- The enabling commit was never pushed.

## 5. What was preserved, and how

- **The bag** is stored as `run/bag.tar.xz`. Extract it with `tar -xJf run/bag.tar.xz`, which
  gives `bag/metadata.yaml` and `bag/bag_0.db3`.
  - The originals have SHA-256 `464abfbf…242e` and `c736dca0…6f32`. The round trip was checked
    to be byte-identical.
  - Message counts: `/clock` 68,083; `/joint_states` 6,976; `/spiderx/sim/world_poses` 5,529;
    `controller_state` 2,880; `/robot_description` 1. `joint_trajectory` has no messages.
- **No video recording.** The frozen harness has no screen recorder. Four GUI screenshots of the
  static welded robot were taken after the refusal; they show no motion.
- **Not copied:**
  - the pytest `--basetemp` directory of step C, which is temporary test output;
  - the confirmation FIFO `run/confirm`, a named pipe that never carried the word.
- **Redactions.** The session scratchpad prefix (the setuptools-59 path in the rebuild command)
  is replaced by `<scratchpad>` in `restore_run1.out` and
  `restore/restore_verify_20261012T030548Z.txt`. The originals are kept outside the repository.
- **Absolute paths.** Some records keep the run directory's absolute path,
  `/root/spiderx_evidence/m61_one_cycle_20261012T025531Z/`. It is this folder.
- **`git_state.txt`.** It says `unavailable`: the tool runs with its working directory outside
  the repository. Steps A and C and the restore reports record the commits.
- **`tools/`.**
  - The 14 frozen harness files and `selftest/` are byte-identical to
    `../../m61a_cloud/observation_7f4c30f/tools/`. They were checked against
    `../../m61a_cloud/closeout/frozen_harness.json`.
  - `run_one_goal_start.sh` (`750ad569…71a6`) starts the bag, the observer, the confirmation FIFO
    and the tool.
  - `probe_action_discovery.py` (`4b9acb85…a3f4`) is the read-only replay of §2.
- **Manifest.** `SHA256SUMS` covers every file in this folder except itself.
