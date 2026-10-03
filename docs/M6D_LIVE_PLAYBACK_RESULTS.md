# M6.0-D Live Playback – Implementation Results (offline, mock and isolated-domain only)

> **Status: M6.0-D implementation cloud + local verified in offline/mock/isolated-domain testing; live dispatch remains hard-disabled and local live playback remains pending separate approval.**
> Cloud evidence: §1–§10. Local verification passed on the owner's Ubuntu PC: [§12](#12-local-verification-passed-owners-ubuntu-pc-local-result).
>
> M6.0-D implementation is verified only by offline, mock, and isolated-domain tests. Live
> controller dispatch is hard-disabled. No trajectory goal was sent to a live controller; no Gazebo
> playback, movement, contact, balance, walking, navigation, or hardware operation occurred. A
> separate owner approval and enabling change are required before any local live goal.

- **Plan:** [M6D_LIVE_PLAYBACK_PLAN.md](M6D_LIVE_PLAYBACK_PLAN.md). The owner decisions D1–D17 in
  §16 govern.
- **Guide:** [SPIDERX_M6D_LIVE_PLAYBACK_GUIDE.md](SPIDERX_M6D_LIVE_PLAYBACK_GUIDE.md).
- **Tags used below:**
  - **[MEASURED]**: observed in this cloud session;
  - **[FACT]**: read from code or config;
  - **[NOT DONE]**: deliberately not executed.

## 1. Provenance [FACT]

| Item | Value |
|---|---|
| Branch | `claude/spiderx-m6d-live-playback`, from `origin/main` @ `9885057` (M6.0-B merged) |
| Plan adoption | `311a849` (D1–D17) |
| Batch A: contract, fingerprint, classification, confirmation | `28b72a4` |
| Batch B: single-goal live state machine, mock-tested | `e87c81a` |
| Batch C: rclpy transport and isolated-domain action test double | `d7fb6a2` |
| Batch D: readiness provider and CLI (`--dry-run` / `--mock`; `--live` hard-disabled) | `7fd0152` |
| Environment | Cloud container with ROS 2 Humble (RoboStack), Python 3.11.16, `rclpy` 3.3.16, `control_msgs` 4.8.0, `joint_trajectory_controller` 2.48.0. Build with setuptools 59.6.0, restored to 84.0.0 afterwards |
| Approved trajectory | `trajectory_id 44f0a7ad52e5c330` (unchanged from M6.0) |
| Approved goal fingerprint | `0d6ef4171f2d338a01e76b934be94d1a4386e7c0fc68fe74262fb3c65005ff83` (SHA-256 of the canonical goal spec, bound to trajectory ID, inputs hash, mode `single`, goal count 1) |

## 2. What was implemented [FACT]

All new code is in `src/spiderx_controller`. Nothing existing was edited except
`CMakeLists.txt`, where lines were only appended: 1 script install and 4 test registrations.

| Module | Role |
|---|---|
| `m6_live_contract.py` | Every M6.0-D limit as a constant. The single gate is `LIVE_DISPATCH_ENABLED = False`. Also holds the confirmation word `SEND-ONE-CROUCH-GOAL` and the exact-match parser (D10, no bypass) |
| `m6_goal_fingerprint.py` | Approved goal spec and SHA-256 fingerprint. `build_live_goal` composes D16 (goal velocity tolerance 0.05 rad/s) on top of the **unchanged** M6.0 `build_goal`. `verify_goal` refuses any other goal |
| `m6_live_readiness.py` | `compatible` / `warning` / `incompatible` classification (D4). The readiness result expires after 10 s (D3); `dispatch_permitted`; same-process `make_readiness_provider` |
| `m6_live_playback.py` | `LiveSession`, a poll-based single-goal state machine (§4), plus `InterruptLatch` (D2), `StreamMonitor` (D6), the controller-loss check (D7) and the CLI |
| `m6_live_mock.py` | Deterministic in-memory `FakeTransport` and named scenarios for `--mock` and the tests. It uses no ROS |
| `m6_live_adapter.py` | `RclpyLiveTransport`: own Context, `SignalHandlerOptions.NO`, a single-use `send_goal` that verifies the fingerprint **before** any ROS call, a single-use `cancel_goal`, and no publisher |
| `scripts/m6_live_playback` | CLI wrapper |

**Implementation note on D16.** Plan §16 expected `m6_action_client.py` and its tests to change.
Instead, the explicit 0.05 rad/s goal velocity tolerance is applied by composition in
`m6_goal_fingerprint.build_live_goal`. The M6.0 builder and tests are byte-unchanged, and a test
pins this (`test_m60_mock_goal_builder_is_unchanged`). `path_tolerance` velocity stays
unspecified (0.0).

**Live dispatch is hard-disabled [FACT, MEASURED].**
- `--live` prints `REFUSED: Live M6.0-D dispatch is HARD-DISABLED …` and exits **3**.
- It does this before it imports `rclpy`, reads input or builds a goal. The tests check that
  `rclpy` is not imported and stdin is not read.
- `_run_live` raises `PermissionError` while the flag is `False`.
- Static tests require:
  - exactly one assignment of the gate literal;
  - no environment-variable override;
  - no `--yes` or `--force` option;
  - `m6_live_adapter.py` as the only module that imports `rclpy`.

## 3. Build and full test suite [MEASURED]

| Run | Result |
|---|---|
| `rm -rf build install && colcon build --symlink-install` | 8 packages finished |
| `colcon test --return-code-on-test-failure`, then `colcon test-result --all` | **1116 tests, 0 errors, 0 failures, 0 skipped** |
| Full suite after Batches A / B / C / D | 1016 / 1075 / 1086 / 1116, each with 0 failures and 0 skipped |
| flake8 (`--max-line-length 100`) on every new module, test and the script | clean |
| `git diff --check` | clean |

## 4. M6.0-D focused suites [MEASURED]

| Suite | Tests | Result |
|---|---|---|
| `test_m6d_contract.py`: limits, goal fields, classification, freshness, confirmation, SHA-256 pins of protected files | 83 | passed |
| `test_m6d_session.py`: gates, results, interrupts, stream rules, mutations | 58 | passed |
| `test_m6d_cli_readiness.py`: readiness provider, CLI, determinism, static scans | 29 | passed |
| `test_m6d_adapter_isolated.py`: rclpy transport against the isolated-domain action test double | 10 | passed (run twice more on its own: 10/10 both times) |
| **All M6.0-D tests together** | **180** | **passed** |
| Fingerprint mutation suite (16 altered goal fields, 4 altered bindings, NaN/±inf, a non-approved trajectory and a refused trajectory) | 25 | passed |
| State-machine mutation suite (freshness gate removed, incompatible gate removed, confirmation bypassed, changed fingerprint, retry, automatic return, duplicate cancel, second dispatch without guard, plus an unmutated control) | 8 | passed |

Each mutation is a deliberately broken copy of the code. The suites pass only because each broken
copy is **caught**: every mutant either fails a gate or is detected as an extra command.

## 5. Isolated-domain action test double (D12) [MEASURED]

The test double meets all six owner conditions.

1. **Explicit non-default domain.** It uses `150 + pid % 50`. A test asserts that this differs
   from `ROS_DOMAIN_ID` (unset in the cloud, so domain 0) and that the transport refuses
   construction without an explicit domain.
2. **No other processes.** It runs in the same pytest process: one `rclpy` Context for the test
   double and one for the client. There is no Gazebo, launch, controller manager, controller,
   bridge or external ROS process.
3. **Automated tests only.** `test/m6d_isolated_action_server.py` lives under `test/` and is not
   installed as a script.
4. **Shut down after each test.** Every test closes its transport and stops its server in
   `finally`. A test proves that `close()` leaves the context not `ok()`.
5. **Domain isolation is proven.** With `ROS_LOCALHOST_ONLY=1` and after 1.5 s of discovery:
   - the only visible node is the test client;
   - no node serves any action;
   - `/joint_states` has 0 publishers;
   - `server_ready(0.5)` is false.
6. **No real endpoint can be contacted.** The test double is not a controller and publishes no
   topic. Before every send, the client's own fingerprint check refuses any non-approved goal; a
   test sends a return-to-neutral goal and the server records nothing.

Over the wire, all goal fields survived real serialization and re-verified to the same
fingerprint:
- header stamp 0;
- goal tolerance 0.05 rad and 0.05 rad/s;
- path tolerance 0.05 rad, with velocity unspecified;
- goal time tolerance 1 s.

The second `send_goal` and the second `cancel_goal` on a transport are both refused. A real
`SIGINT` raised during an accepted goal is latched once, the context stays alive, and exactly one
cancel reaches the server.

## 6. Dry-run and mock reports, run twice [MEASURED]

Each run used a fresh `--out` root under the session scratchpad: one `--dry-run` and the six
`--mock` scenarios. Both runs were **byte-identical** (SHA-256):

| Report | SHA-256 | Exit | Outcome |
|---|---|---|---|
| `dry_run/44f0a7ad52e5c330/dry_run_report.json` | `b3174cd56b9e10537f92e0b57cdc89d22ae45abec7ab28d78554a81b5e6befe4` | 0 | PASS; `goals_sent 0`; `live_dispatch_enabled false` |
| `mock/success/mock_report.json` | `b7d4689884f4c02c648cec8919cdd66b0726d5124d5fa51c374df8a53baee56d` | 0 | SUCCEEDED; mock transport: 1 goal, 0 cancels |
| `mock/interrupt/mock_report.json` | `3150d20640b70893c4269065a34a0ea7e2eb258583c6d8399466482cdd5d626e` | 1 | CANCEL_CONFIRMED, `operator_interrupt`; 1 cancel |
| `mock/tracking_error/mock_report.json` | `285d8516cbb022347fbacf543abc517428c8502bd66390f665e8e4ad84a8b3e9` | 1 | TRACKING_FAILED; 1 cancel |
| `mock/stale_joint_states/mock_report.json` | `fc43c5c63a6bd62c330573fa5c1feb1cf175d81492c99688331a3063ab76e802` | 1 | READINESS_LOST, `joint_states_stale`; 1 cancel |
| `mock/controller_lost/mock_report.json` | `688bad68c14e2623103f643137107c0bd7fd775e6cf5662193bf16b5cd8c27a2` | 1 | HELD_ERROR, `controller_lost`; 1 cancel |
| `mock/rejected/mock_report.json` | `722bfdfc82a4089de636aa4e52cc8b6d789a526fa2d445e7cbc74c1207d84c81` | 1 | REJECTED; 0 cancels |

- **Mock outcomes.** Every mock report shows `retries 0` and `automatic_return_goals 0`.
- **What "1 goal" means here.** It counts the in-memory `FakeTransport` and never a ROS endpoint.
- **Mock readiness.** It is classified `warning` from the owner-PC versions recorded by M6.0-B
  (version differences only, no failure codes).
- **Confirmation input.** A wrong-case confirmation word gave `REFUSED, confirmation_refused`, exit
  2, with 0 goals at the mock.
- **`--live`.** It gave exit 3 with the REFUSED message.

> **Later change (PR #16 review corrections).** The hashes above are the PR #15 build. The
> corrective batches added fields to the mock report (`dispatch`, `errors`, `freshness_at_send`,
> readiness `collection_s`), so the six mock hashes changed. The dry-run hash, trajectory ID and goal
> fingerprint did not. The new values are in the
> [live-enabling design §8](M6D_LIVE_ENABLING_DESIGN.md#8-corrective-changes-from-the-pr-16-read-only-review).

## 7. Static M1–M4 and Fortress validators (no arguments, so no `--runtime`) [MEASURED]

| Validator | Result |
|---|---|
| `validate_fortress.sh` | 58 PASS, 0 FAIL, `All checks passed.` |
| `validate_m1_control.sh` | 14 PASS, 0 FAIL, `All M1 checks passed.` |
| `validate_m2_posture.sh` | 17 PASS, 0 FAIL, `All M2 checks passed.` |
| `validate_m3_kinematics.sh` | 17 PASS, 0 FAIL, `All M3 checks passed.` |
| `validate_m4_all_leg_ik.sh` | 18 PASS, 0 FAIL, `All M4 checks passed.` |

These are static checks only. Their launch-file checks use `ros2 launch … --show-args`, which
parses arguments and starts nothing. Afterwards, `pgrep` found no Gazebo/`ign`, bridge,
`robot_state_publisher`, spawner, `ros2 launch`, controller manager or RViz process.

## 8. Scope audit vs `origin/main` [MEASURED]

**Changed:**
- `git diff origin/main..HEAD` before this docs commit touched only the adopted plan, the six new
  `m6_*` modules, the script, five new test files, and the appended `CMakeLists.txt` lines.
- This docs commit adds this file and the guide. It updates the plan status, `STATUS.md`, the
  roadmap and the testing guide.

**Unchanged:**
- `trajectory_client.py`;
- every M6.0 module (`m6_trajectory`, `m6_action_client`, `m6_envelope`, `m6_live_preflight`, …);
- controller YAML, controller launch, `package.xml`, `setup.py`;
- `spiderx_description` (URDF/xacro, meshes), `spiderx_bringup` (worlds, launch), and the
  top-level `scripts/` validators.

SHA-256 pins in `test_m6d_contract.py` also cover `trajectory_client.py`, the M1–M4 tools, the
M6.0 modules, the controller YAML and the controller launch file.

## 9. Runtime boundary [NOT DONE]

During the whole of M6.0-D (Batches A–E), none of the following happened:
- Gazebo, Fortress, `ros2 launch` (other than `--show-args` parsing), RViz, a bridge, a controller
  manager, a controller spawner or a GUI was started;
- graph-mode preflight was run;
- a live controller or action server was queried;
- a live `FollowJointTrajectory` goal was sent, accepted, cancelled or executed;
- a trajectory, joint command, `/cmd_vel`, controller command or parameter change was published;
- the default ROS domain or an existing ROS graph was used;
- M6.0-D was run on the owner's PC or in the cloud against a live stack.

The only `rclpy` use was the isolated-domain test double of §5. M6.1, protected replay, contact,
fixed/free base, gait playback, walking, navigation, odometry, SLAM, Nav2 and hardware work were
not started.

## 10. Non-claims

This work does **not** show any of the following:
- valid live dispatch;
- controller goal acceptance;
- real joint tracking;
- Gazebo movement;
- contact or body support;
- balance;
- gait replay or locomotion;
- walking or navigation;
- any hardware capability.

The mock and test-double results show only that the code follows its specified state machine,
gates, limits and refusal paths against simulated responses.

## 11. What remains before a live goal

1. **Owner review of this branch.** It is a draft PR, not merged.
2. **A separate owner approval and an enabling code change** that flips `LIVE_DISPATCH_ENABLED`
   after a final audit (plan §16). The tests that pin the disabled state would change with it.
3. **The D15 local stack re-check** of the owner's installed 2.54.x controller and action headers.
4. **The owner-approved manual M6.0-D runtime checklist** on the owner's Ubuntu PC, in GUI mode,
   with two terminals (D17, plan §13).

## 12. Local verification passed (owner's Ubuntu PC) [LOCAL RESULT]

**Status: M6.0-D implementation cloud + local verified in offline/mock/isolated-domain testing; live dispatch remains hard-disabled and local live playback remains pending separate approval.**

The owner ran this verification locally, in offline, mock and isolated-domain modes only. The
facts below are as reported by the owner.

### 12.1 State

- Branch `claude/spiderx-m6d-live-playback` at `412eb45`, clean and up to date with `origin`.
- `log/` is git-ignored.
- No tracked file was modified locally. No local commit, push, PR update or config change was
  made.

### 12.2 Build and tests

- 8 packages built in 15.8 s, with no warnings or errors.
- Full suite: **1116 tests, 0 errors, 0 failures, 0 skipped**.
- 180 M6.0-D tests passed:

| Suite | Tests |
|---|---|
| `test_m6d_contract` | 83 |
| `test_m6d_session` | 58 |
| `test_m6d_cli_readiness` | 29 |
| `test_m6d_adapter_isolated` | 10 |

**What the suites cover:**
- **Contract tests:** deterministic fingerprint, classification, fresh readiness, the confirmation
  parser, 25 fingerprint mutations, and SHA-256 pins of protected files.
- **Session tests:** gates, action outcomes, timeout and stream behaviour, single-use logic,
  cancel-only handling, and all mutation cases.
- **Mutants caught:**
  - removed freshness gate;
  - removed incompatible gate;
  - confirmation bypass;
  - changed fingerprint;
  - retry;
  - automatic return;
  - duplicate cancel;
  - second dispatch.
- **Isolated-domain suite:** passed both in full and in a focused rerun.

### 12.3 Contract (matches the cloud)

| Item | Value |
|---|---|
| Trajectory ID | `44f0a7ad52e5c330` |
| Goal fingerprint | `0d6ef4171f2d338a01e76b934be94d1a4386e7c0fc68fe74262fb3c65005ff83` |
| Points | neutral at 3 s → `crouch_10mm` at 6 s → neutral at 9 s; explicit point velocities 0.0 |
| Maximum displacement | 0.12229413600889982 rad, at `rr_foot_joint` |
| Cap | 0.1223 rad + 1e-9 epsilon |
| Path and goal position tolerance | 0.05 rad, for all 12 joints |
| Goal velocity tolerance | 0.05 rad/s |
| Path velocity tolerance | unspecified (0.0) |
| Goal-time tolerance | 1.0 s |
| Header stamp / goal count / mode | 0 / 1 / `single` |
| Mock reports | `retries 0`; `automatic_return_goals 0` |

### 12.4 Dry-run and mock

`--dry-run` and the six mock scenarios each ran twice, into separate ignored log roots. All seven
reports were byte-identical across the local runs, and **exactly matched the cloud SHA-256 values
in §6**.

| Run | Exit | Final state |
|---|---|---|
| `--dry-run` | 0 | PASS |
| `success` | 0 | `SUCCEEDED` |
| `interrupt` | 1 | `CANCEL_CONFIRMED` |
| `tracking_error` | 1 | `TRACKING_FAILED` |
| `stale_joint_states` | 1 | `READINESS_LOST` |
| `controller_lost` | 1 | `HELD_ERROR` |
| `rejected` | 1 | `REJECTED` |
| Wrong-case confirmation word | 2 | `REFUSED`, `confirmation_refused`; 0 goals, 0 cancels |

The mock reports state explicitly that they come from mock, in-memory operation and that nothing
is sent to a ROS graph.

**Import clarification [LOCAL RESULT].** Dry-run and mock modes do import ROS message-definition
packages: `control_msgs`, `action_msgs`, `trajectory_msgs` and `builtin_interfaces`. In fresh
processes they do **not** import `rclpy` or `rclpy.action`. They therefore build offline message
objects for validation and fingerprinting, but create no ROS node or action client and interact
with no graph.

### 12.5 Hard-disabled live mode

- `ros2 run spiderx_controller m6_live_playback --live` was run once, with stdin `/dev/null`. It
  printed `REFUSED` and exited **3**.
- The refusal happened before any of the following:
  - `load_sources()`;
  - goal construction;
  - the stdin confirmation;
  - `rclpy` or action-client initialization;
  - any graph query or node creation;
  - report generation.
- `LIVE_DISPATCH_ENABLED` remains hard-coded `False`.
- No `--yes`, `--force` or environment-variable override exists.
- `_run_live` has a second `PermissionError` gate.

### 12.6 Isolated domain

- The domain is `150 + pid % 50`, within [150, 200), and explicitly different from the default
  domain 0.
- The tests set `ROS_LOCALHOST_ONLY=1`.
- After discovery:
  - only the test client is visible;
  - there is no action server, no external node and no `/joint_states` publisher;
  - `server_ready` is false.
- The test server publishes no topic.
- Every transport, server and context closes in `finally`. Neither the focused rerun nor the full
  suite left any process, daemon or server running.

### 12.7 Static validators (no arguments)

| Validator | Result |
|---|---|
| Fortress | `All checks passed.` (58 PASS) |
| M1 | `All M1 checks passed.` (14) |
| M2 | `All M2 checks passed.` (17) |
| M3 | `All M3 checks passed.` (17) |
| M4 | `All M4 checks passed.` (18) |

### 12.8 Final safety

- Generated outputs exist only under the ignored `log/`.
- None of the following occurred:
  - Gazebo or launch;
  - a live graph or controller query;
  - an action goal or cancel;
  - playback or movement;
  - M6.1, M7–M10, navigation or hardware activity.
- `LIVE_DISPATCH_ENABLED` was never changed.
- No relevant process was left running.

This local verification does **not** show live dispatch, controller goal acceptance, joint
tracking, Gazebo movement, contact, balance, walking, navigation or hardware capability. §11 still
applies in full: a separate owner approval and an enabling change are required before any local
live goal.
