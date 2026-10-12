# M6.1 tracking-reference fix: evidence `20261012T081545Z` (no goal sent)

**Scope.** This folder explains and fixes the client's in-flight tracking-reference defect, and
rehearses the fixed path without a goal.
- The defect cancelled the second authorized Cloud cycle,
  [`../../m61_cloud_one_cycle/20261012T054741Z/`](../../m61_cloud_one_cycle/20261012T054741Z/README.md).
  **That record stays FAIL: 1 goal, 1 cancel, E4 and E6 FAIL.** It is not rescored, and nothing
  in its folder was changed.
- Fix commit: `b0d8e75`, M6.1 side only. Report:
  [Cloud report §14](../../../M61A_CLOUD_VERIFICATION.md).
- Both live gates are `False` throughout. **No goal was sent, and no motion was attempted.**

## 1. The two questions

**a. Why did the M6.0-D crouch not trigger this with the same tracking code?**
- It ran on the owner's PC, with `joint_trajectory_controller` 2.54.0. That version fills
  `desired.time_from_start` in the action feedback; upstream added the field in 2.49.0.
- With the field filled, the M6.0-D estimate is the true start, within 0.1 s of acceptance, so
  acceptance is used. Result: SUCCEEDED, 0.00603 rad in flight
  (`606d229:docs/M6D_FIRST_GOAL_RESULTS.md`).
- This Cloud image has 2.48.0, which leaves the field at 0.
- `tools/analysis_1a_m6d_crouch_tfs_zero.py` → `analysis/1a_m6d_crouch_tfs_zero.txt` is a model,
  not a measurement. It runs the unchanged rule on the crouch with perfect tracking:
  - field filled: error 0, no abort;
  - field 0: abort at 6.05 s with 0.1222 rad.

**b. What is the real offset between goal acceptance and the controller's start?**
- `tools/analysis_1b_controller_start_fit.py` → `analysis/1b_controller_start_fit.txt` fits the
  bag of that cycle.
- The controller's own setpoint (`controller_state.reference`) started at **acceptance − 0.015 s**
  (rms 1.08e-4 rad, 130 samples).
- The joints lag that setpoint by about 0.083 s. That lag is physical and is not compensated.
- Timed from acceptance, the joints are within 0.0105 rad (reference time 3.0–6.2 s), and within
  0.0108 rad over all 322 tracked samples.
- **Acceptance is the anchor the data supports.**

## 2. Root cause: proven, with one inferred link

- **The tool's own estimate proves the mechanism.** The estimate was acceptance + 3.216 s, about
  the median feedback stamp. An informative `time_from_start` would have given about
  acceptance − 0.015 s.
- **The replay proves the effect.** The recorded joints through the unchanged rule cancel;
  through the fixed rule they do not.
- **Inferred, not recorded: the raw feedback field.** Hidden action topics were not in the bag.
  `controller_state` is no direct witness:
  - its deprecated `desired` point is empty;
  - its `reference` point also carries `time_from_start = 0`.

  The next cycle's plan records `_action/feedback`.

## 3. Files

| Path | What |
|---|---|
| `tools/make_replay_fixture.py` | Builds `src/spiderx_controller/test/data/m61_cycle_20261012T054741Z_tracking_replay.json` from the cycle evidence and bag: 624 joint samples, a 255-message `controller_state` feedback proxy, and source SHA-256 values |
| `tools/run_mutants.py`, `mutants/mutants.txt` | 11 deliberate mutants of the fix: **11 caught**; sources restored byte for byte |
| `tools/analysis_1a_*.py`, `tools/analysis_1b_*.py`, `analysis/` | Questions a and b (§1) |
| `tools/tracking_path_probe.py` | The read-only live probe (§4) |
| `tools/*.sh`, `tools/*.py` (others), `tools/selftest/` | The frozen phase-1 harness, copied unchanged. `run/harness_check.txt`: 14 of 14 files match `frozen_harness.json` |
| `pre/` | `--verify-only` before the launch: `DISABLED AND VERIFIED` (`src/` = `b0d8e75`) |
| `run/` | The first launch (`launch.log`, `controllers_active.txt`, `shutdown.txt`), the probe and the readiness rehearsal |
| `post/` | `--verify-only` after the stop: `DISABLED AND VERIFIED` |
| `run2/`, `post2/` | The second launch: the feedback-bag check (§6), then `--verify-only`: `DISABLED AND VERIFIED` |
| `tests/` | Test results (§5) |

## 4. Read-only rehearsal on the disabled build

The build was clean at `b0d8e75`; the launch was `fortress_m61a_fixed_base.launch.py` via the
frozen harness; both controllers were active after 13 s.

**The probe** (`run/probe/tracking_path_probe.json`, `probe.txt`; 40 s): **PASS, 10 of 10
checks.**
- **Read-only by construction.** The node only subscribes: no action client, no publisher of its
  own. The session's transport is an in-memory stub.
- **Inputs, through the production adapter:** the live `/joint_states`, and each live
  `controller_state` copied into a FollowJointTrajectory feedback message.
- **Session:** fed into `M61Session` from a pseudo acceptance 2 s into the window, and the same
  events through the unchanged M6.0-D rule as a control.

| Check | Result |
|---|---|
| No goal | 0 action-status messages; 0 command messages; the probe had no action client. The bag (counts only, `bag_metadata.yaml`) has no `joint_trajectory` topic |
| Robot held neutral | max \|q − neutral\| 9.7e-5 rad |
| Live `time_from_start` | 0 in all 1206 converted messages; JTC 2.48.0 |
| Anchor | at the pseudo acceptance (17.96 s); the fallback recorded and logged once; 361 checks on it |
| Tracking start | the first tracked check at reference time 3.0 s |
| A genuinely bad track (no motion) | tripped at reference 3.63 s, 0.0506 rad. Predicted independently: 3.624 s |
| The M6.0-D rule | its estimate drifted 13.78 s; it started checking 3 s late |

**The other runs:**
- **The readiness rehearsal** (`run/readiness_rehearsal/`): READY; 0 goals, 0 dispatch calls.
- **The feedback-bag check** (`run2/feedback_bag_check/`, a second launch): see §6.
- **The stop:** clean, no process left. Then `post/`: `DISABLED AND VERIFIED`.

**Not shown (needs a goal):**
- the controller's action feedback during a goal;
- its start in this launch;
- a complete cycle;
- goal tolerances;
- any motion.

## 5. Tests

| Run | Result |
|---|---|
| `test_m61_tracking.py` | 49 passed |
| Mutants | 11 of 11 caught |
| Disabled: clean build + full `colcon test` (`tests/disabled_full_colcon_test.txt`) | **1678 tests, 0 errors, 0 failures, 0 skipped** |
| Enabled: patch `b6213f21…` in a temporary worktree at `b0d8e75`, whole `spiderx_controller` suite, fakes only (`tests/enabled_pytest_summary.txt`) | **1634 passed, 0 failed** |

A first disabled `colcon test` launch omitted `PYTEST_DISABLE_PLUGIN_AUTOLOAD=1`. Every test
file then failed at collection: the `launch_testing` pytest plugin was incompatible, and no test
body ran. It was re-run with the documented environment; the record is in
`tests/disabled_attempt1_no_plugin_env.txt`.

## 6. Feedback-bag check (plan §4 E change)

The plan's step E will also record `/leg_trajectory_controller/follow_joint_trajectory/_action/feedback`
with `--include-hidden-topics`; never `_action/status`, whose subscribers count as competing
action clients. This records the controller's real feedback during the next goal.
`tools/feedback_bag_check.sh` checked that change read-only, in a second launch on the disabled
build (`run2/`):
- the bag subscribed to `_action/feedback`: 1 publisher (`leg_trajectory_controller`) and 1
  subscription (`rosbag2_recorder`);
- `_action/status` had 0 subscribers;
- the production readiness rehearsal was **READY**: `foreign_action_clients` 0 and
  `command_publishers` 0 in both observations; 0 goals; 0 dispatch calls;
- the stop was clean, then `post2/`: `DISABLED AND VERIFIED`.

There is no feedback without a goal, so only the subscription and readiness were shown.
