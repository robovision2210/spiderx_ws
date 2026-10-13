# SpiderX M6.0-D Live Playback Guide

```text
Live dispatch is HARD-DISABLED in this build (LIVE_DISPATCH_ENABLED = False).
`--live` refuses with exit 3 and sends nothing. --dry-run and --mock never touch a ROS graph.
Nothing in this guide shows motion, tracking, contact, balance, walking or hardware capability.
```

**Status:** implementation cloud + local verified in offline/mock/isolated-domain testing; the
PR #16 corrective batches (evidence, exception safety, send-time freshness, interruptible
confirmation, isolated success path) are cloud-verified in the same modes. Live dispatch remains
hard-disabled and local live playback remains pending separate owner approval.

| Document | What it holds |
|---|---|
| [M6D_LIVE_PLAYBACK_PLAN.md](M6D_LIVE_PLAYBACK_PLAN.md) | Design. Owner decisions D1–D17 are in §16 |
| [M6D_LIVE_PLAYBACK_RESULTS.md](M6D_LIVE_PLAYBACK_RESULTS.md) | Evidence |
| [SPIDERX_M6_PLAYBACK_GUIDE.md](SPIDERX_M6_PLAYBACK_GUIDE.md) | M6.0 layers it builds on: offline preflight, mock client, read-only graph preflight |

## 1. What M6.0-D is

The tool is built to send **exactly one** pre-checked `FollowJointTrajectory` goal to the
simulated `leg_trajectory_controller`:
- the motion is neutral → `crouch_10mm` → neutral, with 3 points at 3, 6 and 9 s and all
  velocities 0;
- it observes the goal independently;
- it never sends a second goal.

The live part is disabled in this build. A future, owner-approved run would show at most that this
one bounded joint trajectory executed in Gazebo, with joint-space tracking within 0.05 rad.

## 2. Layers

```text
m6_trajectory (M6.0) ─► m6_goal_fingerprint ─► approved spec + SHA-256 fingerprint
                         build_live_goal = unchanged M6.0 build_goal + goal velocity tol 0.05 rad/s (D16)

readiness provider ──► m6_live_readiness ──► compatible / warning / incompatible, fresh <= 10 s (D3, D4)

m6_live_playback.LiveSession (single-goal state machine)
   PREFLIGHTED ─► readiness #1 ─► typed confirmation ─► CONFIRMED ─► readiness #2 ─► rebuild + fingerprint
   ─► server ready ─► DISPATCHING ─► pre-send record ─► latch + freshness (at the send)
   ─► one send_goal ─► GOAL_PENDING ─► ACTIVE ─► terminal state
                                           │
LiveTransport ─┬─ m6_live_mock.FakeTransport     (in-memory; --mock and tests)
               └─ m6_live_adapter.RclpyLiveTransport (rclpy; tests on an isolated domain only)
```

| Module | ROS graph? |
|---|---|
| `m6_live_contract.py`: every limit, the single `LIVE_DISPATCH_ENABLED` gate, the confirmation word | No |
| `m6_goal_fingerprint.py` | No |
| `m6_live_readiness.py` | Only through an injected collector, which is never live in this build |
| `m6_live_playback.py` (state machine and CLI) | No. It talks to a transport |
| `m6_live_mock.py` | No |
| `m6_live_adapter.py` | Yes. It is the only module that imports `rclpy`; in this build it runs only in isolated-domain tests |

## 3. Commands

Build and source as usual (see the [testing guide](SPIDERX_TESTING_GUIDE.md)), then:

```bash
ros2 run spiderx_controller m6_live_playback --help
ros2 run spiderx_controller m6_live_playback --dry-run            # offline; writes log/m6d_playback/dry_run/<trajectory_id>/
echo SEND-ONE-CROUCH-GOAL | ros2 run spiderx_controller m6_live_playback --mock --scenario success
ros2 run spiderx_controller m6_live_playback --live --domain-id 0  # REFUSED, exit 3, nothing sent
```

**Future only (not runnable in this build).** After a separate owner approval and the enabling
commit, the live form is always given an explicit domain and writes its evidence under `--out`:

```bash
# FUTURE ONLY - refused with exit 3 while LIVE_DISPATCH_ENABLED is False
ros2 run spiderx_controller m6_live_playback --live --domain-id 0 --out log/m6d_playback
```

`--domain-id` is the domain of the running simulation, typed by the operator (0 if `ROS_DOMAIN_ID`
is unset there); it is never read from the environment.

| Option | Meaning |
|---|---|
| `--dry-run` | Builds and fingerprints the one approved goal offline and sends nothing. Prints `Dry run PASS: trajectory 44f0a7ad52e5c330, goal fingerprint 0d6ef417…` |
| `--mock` | Runs the full state machine against the in-memory mock. Reads the confirmation word from stdin. Scenarios: `success`, `interrupt`, `tracking_error`, `stale_joint_states`, `controller_lost`, `rejected` |
| `--live` | Hard-disabled. Refuses before importing `rclpy`, reading input or building a goal |
| `--domain-id N` | `--live` only (0..232). Refused with any other mode |
| `--out DIR` | Report root (default `log/m6d_playback`, git-ignored). Reports are **never overwritten**: an existing report means REFUSED, so choose a new `--out` |
| `--no-write` | Print only (`--dry-run`, `--mock`). **Refused with `--live`**: a live run always records evidence |

**Exit codes:**
- `0`: dry-run PASS or mock SUCCEEDED;
- `1`: terminal failure (rejected, aborted, timed out, tracking failure, readiness lost, cancel
  unconfirmed, held error), or a live run whose final evidence could not be saved;
- `2`: refused before dispatch (preflight, readiness, confirmation, fingerprint, server, interrupt,
  existing report, evidence that cannot be reserved or saved before the send);
- `3`: live dispatch disabled.

There is no `--yes` or `--force` option, and no environment variable bypass.

## 4. Gates before the one goal

Each gate refuses with a named code. A refusal reaches **no** transport `send_goal`.

1. **Preflight and approved content.** The M6.0 preflight must pass, and the trajectory must be
   exactly the approved neutral → `crouch_10mm` → neutral, 3 points, ≤ 0.1223 rad + 1e-9
   (`preflight_refused`, `not_approved_content`).
2. **Readiness before confirmation.** The check needs:
   - a result 0–10 s old (`readiness_missing`, `readiness_stale`);
   - a ready report (`readiness_not_ready`);
   - a known class (`classification_unknown`);
   - a class that is not `incompatible` (`stack_incompatible`). `warning` (version differences
     only) is allowed and recorded.
3. **Typed confirmation.** The operator must type exactly `SEND-ONE-CROUCH-GOAL`. Any other case,
   leading or trailing whitespace (other than a single newline), EOF or Ctrl+C refuses
   (`confirmation_refused`, `operator_interrupt`). The prompt answers Ctrl+C within about 0.1 s
   without waiting for Enter, and an interrupt wins over anything typed.
4. **Readiness after confirmation.** It is re-collected in the same process, with the same rules.
   A long pause at the prompt therefore never matters: only this fresh observation counts.
5. **Goal rebuild and fingerprint.** The goal is rebuilt, and its fingerprint must equal the
   preflighted one (`goal_fingerprint_mismatch`).
6. **Action server.** The action server must be ready within 10 s (`action_server_unavailable`).
7. **Pre-send evidence.** A `pre_send` record must be saved (`evidence_persistence_failed`).
8. **Immediately before the send.** No interrupt (`operator_interrupt`), and readiness #2 must
   still be fresh *at this instant* (`readiness_stale`). Freshness is counted from the **start**
   of the observation, so the collection time and the server wait both age the evidence. A stale
   result refuses once - no new observation, no retry.
9. **Transport check.** The transport verifies the fingerprint **again** before any ROS call.

## 5. In flight

| Event | Action |
|---|---|
| No goal response within 10 s | `TIMED_OUT` |
| Goal rejected | `REJECTED` (no cancel) |
| Result watchdog of 120 s | One cancel → `TIMED_OUT` |
| Observed tracking error > 0.05 rad | One cancel → `TRACKING_FAILED` |
| `/joint_states` stale > 0.5 s, sim time stalled > 5 s, or an extra publisher | One cancel → `READINESS_LOST` |
| Sample gap > 0.25 s | One cancel → `TRACKING_FAILED` |
| Controller lost (checked every 1 s) | One cancel → `HELD_ERROR` |
| Goal aborted by the controller | `ABORTED` (no cancel) |
| First Ctrl+C or SIGTERM | One cancel request, then wait 5 s for the cancel response and 5 s for the final status. The result is `CANCEL_CONFIRMED`, or `CANCEL_UNCONFIRMED` if no confirmation comes |
| Second Ctrl+C | `CANCEL_UNCONFIRMED` immediately; no second cancel. Two Ctrl+C within one 50 ms poll still send the one cancel first |
| Transport failure after the send (graph check, polling, an event, simulation time, the cancel call itself) | Recorded; the one cancel is requested through the same guard (never repeated); then `HELD_ERROR` with the final goal status marked unknown if it was not observed |
| Send call fails | `HELD_ERROR`; acceptance **unknown** (the goal may have reached the server); no retry |

For every cancel reason other than an operator interrupt, the terminal state stays the one
listed, and the report records whether the cancel was confirmed. The tool never retries, never
preempts, never sends a return-to-neutral goal, and never sends a second goal.

**Intended controller behaviour vs what is tested.**
- *Intended (plan §7.1; re-checked on the owner's stack by D15 before any live goal):* after a
  cancel, an abort or a tolerance failure, the joint trajectory controller holds position.
- *Tested (mock and isolated test doubles only):* the client side - at most one goal, at most one
  cancel request, no further command afterwards, and the outcome recorded truthfully. No test
  shows a real controller or robot holding.
- *Closing the tool does not stop a goal.* Closing the client releases local ROS resources only.
  If the final goal status is unknown (no response, an unconfirmed cancel, a transport failure),
  the controller may still be executing the goal; the tool says so and records
  `goal_may_still_be_executing: true`.

**One goal per session and process - not a durable prohibition.** The single-goal guarantee is
enforced inside one `LiveSession`, one transport and one process. Nothing prevents a second,
separate invocation from sending another goal; that is excluded only by the manual checklist
("never re-run after a goal was sent") and a new owner approval. No durable cross-process
dispatch marker exists.

## 6. Reports

`--dry-run` writes `dry_run_report.json` (schema `spiderx.m6d.dry_run/1`). It contains:
- the envelope, limits and goal spec;
- the fingerprint;
- `goals_sent: 0` and `live_dispatch_enabled: false`.

`--mock` writes `mock/<scenario>/mock_report.json`. It contains:
- the state, reason and readiness results;
- the action/tracking/graph channels and events;
- the cancel and result data, `retries` and `automatic_return_goals`;
- the non-claims.

Both reports are deterministic: two runs are byte-identical (results §6). The PR #16 corrective
batches added fields to the mock report (`dispatch`, `errors`, `freshness_at_send`, readiness
`collection_s`), so mock report hashes differ from those recorded for PR #15; the trajectory ID
and goal fingerprint are unchanged.

**Live evidence (future runs).** `--live` writes `live/<UTC>/live_outcome.json`, created
exclusively before any ROS initialization and then rewritten only through the descriptor that run
created. Its `evidence.phase` says how far the run got:
- `reserved`: written before ROS started; if it is the last record, nothing was sent;
- `pre_send`: every gate passed except the final latch and freshness checks; if it is the last
  record, a send may have happened and the goal status is **unknown**;
- `final`: the complete outcome.

The `dispatch` block distinguishes: dispatch not attempted; attempted with acceptance unknown;
accepted (with the goal ID); rejected; cancel attempted; cancel response known; final result
known; final goal status unknown; and whether the controller may still be executing the goal. If
the final record cannot be written, the outcome is printed to stderr, the exit code is 1, no
report is claimed and nothing is retried.

## 7. Tests

```bash
cd src/spiderx_controller
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 PYTHONPATH=$PWD:$PWD/test:$PYTHONPATH python3 -m pytest -q \
  test/test_m6d_contract.py test/test_m6d_session.py test/test_m6d_cli_readiness.py \
  test/test_m6d_live_enabling.py test/test_m6d_evidence_safety.py \
  test/test_m6d_freshness_confirmation.py \
  test/test_m6d_adapter_isolated.py test/test_m6d_isolated_success.py
```

Source `install/setup.bash` first; the tests load the installed URDF.

| File | What it covers | ROS |
|---|---|---|
| `test_m6d_contract.py` | Limits, goal fields, fingerprint mutations, classification, confirmation parser, protected-file pins | No |
| `test_m6d_session.py` | Session gates, outcomes, interrupts, stream rules, mutants | No (mock) |
| `test_m6d_cli_readiness.py` | Readiness provider, CLI, determinism, static scans | No |
| `test_m6d_live_enabling.py` | The gate, the gated wiring, identity pins under both gate states, protected baseline | No (mock) |
| `test_m6d_evidence_safety.py` | Evidence reservation and persistence, failure injection at every boundary | No (mock) |
| `test_m6d_freshness_confirmation.py` | Truthful readiness timing, send-time freshness, interruptible confirmation | No (mock; one real SIGINT to the test process) |
| `test_m6d_adapter_isolated.py` | `RclpyLiveTransport` against an in-process action test double; partial-open cleanup | Isolated domain |
| `test_m6d_isolated_success.py` | `main()` with the gate set True **in the test only**, end to end against an in-process fake controller stack; a failure prints every readiness report in full. Also the fake stack itself: stamp rounding, no overlapping ticks / in-order stamps, no orphaned executor callbacks at stop ([Cloud diagnosis](M61A_CLOUD_VERIFICATION.md) §1–2) | Isolated domain |

The isolated tests use domain `150 + pid % 50` with `ROS_LOCALHOST_ONLY=1`. The end-to-end test is timing-dependent: run it without competing CPU-heavy work (under 8 CPU burners it can end `TRACKING_FAILED`/`sample_gap`, the tool failing safe on a starved fake publisher). Use `--basetemp` to keep its `live_outcome.json` outside pytest's rotation. They first prove that
no other node, service, publisher or action server is visible (the success-path test skips, with
the reason, if isolation cannot be established). **Never** point them at a running stack or the
default domain. The fake controller stack proves nothing about a real controller.

## 8. Before any live goal

The gated `--live` wiring (it needs `--domain-id`; the confirmation prompt appears only after
readiness #1 passes), the exact two-line enabling commit and the future manual checklist are in
the [live-enabling design](M6D_LIVE_ENABLING_DESIGN.md). The gate is still `False`.

The following are all required, and none of them exists yet:
1. owner review of the draft PR;
2. a **separate owner approval**;
3. an **enabling code change** that sets `LIVE_DISPATCH_ENABLED = True` after a final audit (plan
   §16);
4. the D15 re-check of the owner's installed 2.54.x stack;
5. the owner-approved manual runtime checklist (plan §13): owner's Ubuntu PC only, GUI mode, two
   terminals (D17), never in the cloud.

Until then, `--live` refuses.
