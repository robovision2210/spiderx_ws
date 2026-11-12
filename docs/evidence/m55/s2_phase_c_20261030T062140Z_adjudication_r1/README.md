# Retrospective assessment under criteria revision r1: S2 Phase C attempt of 2026-10-30

**This is not the original frozen result.** The original result stands unchanged:
- **Frozen verdict: FAIL**, from `analyze_s2.py` (sha256 `e9c2efcb…`): P2 UNMEASURED, P4 FAIL; P1, P3 and P5–P8 PASS.
- **Records:** in `../s2_fixed_base_20261029T100210Z/tools/` and the committed evidence `../s2_phase_c_20261030T062140Z/` (commit `47b066d`). They are byte-identical, and nothing there was modified.
- **The 2026-10-29 attempt stays FAIL** permanently.

**Status:** an offline adjudication of the same run. Nothing was built, launched or moved for it. The owner accepted the r1 policy on 2026-10-30; the r1 result stays retrospective until the implementation (`scripts/s2/adjudicate_s2.py`) has been reviewed.

## Criteria revision r1 (owner-approved amendments, 2026-10-30)

| | Original (design §4) | r1 |
|---|---|---|
| P2 | Max feedback \|error\| ≤ 0.02 rad on **every** goal. A goal without feedback leaves P2 UNMEASURED | Unchanged (≤ 0.02 rad) for every normally executed goal. Only the **final scripted estop goal** may be "tracking N/A — planned cancellation before feedback" (**P2 proof** below). Missing feedback on any other goal stays UNMEASURED. No tracking value is invented for an N/A goal |
| P4 | 0 `joint_trajectory` messages, with the topic required in the bag | Zero recorded messages are distinguished from unavailable instrumentation. An **absent** topic counts as zero only with independent support (**P4 support** below); otherwise the term is UNMEASURED. A **present** topic with any message is FAIL |

**P2 proof** (all required):
- goal identity, and the goal response reporting the goal accepted;
- the scripted estop key during the goal;
- the cancel requested no later than the goal response was polled;
- the cancel processed by the server (cancel response code 0);
- exactly one cancel;
- result CANCELED;
- no refusal or unplanned fault before it;
- no feedback for it.

**P4 support** (all required):
- the topic was requested from the recorder;
- recording covered the goal interval, and discovery was active;
- the recorder log shows no failure;
- the node-monitored streams stayed within the node's own staleness limits;
- the owner monitor and node events show no foreign source;
- the captures show zero publishers;
- the goal and cancel counts and the exercise order match the script.

**Unchanged:** every other criterion, the 0.02 rad tracking limit, and all plant-identification thresholds.

**Method.**
- **Tool:** `scripts/s2/adjudicate_s2.py`. Its tests are in `scripts/s2/test_adjudicate_s2.py`: 20 pass, covering both amendments and the alignment.
- **Bag:** read-only (`mode=ro&immutable=1`); sha256 `9c77b9a2…ec2e` was the same before and after (`bag_hash.txt`, `assessment.json` → `bag`).
- **Clock alignment:** node events carry no action goal id, so feedback is attributed to goals by time.
  - Bag receipt times are mapped to the node's CLOCK_MONOTONIC with the 506 status messages that both the driver and the recorder received; all 506 paired uniquely.
  - Median offset; p5–p95 within ±1.5 ms; worst case 31.7 ms, which is used as the window tolerance.
- **Timing terms.** Every node time is the client's CLOCK_MONOTONIC (`m55_rclpy.py`, `m55_locomotion.py`):
  - `goal_sent` is stamped when the node calls `send_goal_async`.
  - `cancel_sent` is stamped when the node **requests** the cancel. Without a goal handle yet, the transport queues it and sends it from the goal-response callback; the log does not say which case applied.
  - `goal_response`, `cancel_response` and `result` are stamped at the 20 Hz session tick that **polled** them, so each callback happened in (previous poll, stamp].
  - Server-side acceptance and cancel processing are not recorded, and this assessment does not infer them.

## Results under r1

**P2 (r1): PASS.**
- **Normally executed goals:** all 58 feedback goal ids are attributed to exactly one goal window each, in order, goals 1–58. Max \|error\| 0.0022469 rad (limit 0.02). The per-goal maxima are identical to the frozen `tracking.csv`.
- **Goal 59:** "tracking N/A — planned cancellation before feedback". Every proof check holds (`assessment.json` → `P2.goals[58].estop_proof`):

| Check | Evidence (CLOCK_MONOTONIC s) |
|---|---|
| Identity | The final goal, 59 of 59: shift b0→b1, direction +1, level 0, equal to the scripted estop exercise |
| Goal sent | `goal_sent` at 1903.7059 (the node's `send_goal_async` call) |
| Scripted estop | Driver key `x` ("estop exercise: x") at 1903.7159. Node fault `estop` at 1903.7172 |
| Cancel requested | The run's only cancel request, for goal 59, at 1903.7172, 11.3 ms after sending. If the goal handle had not arrived yet, it was queued until the goal response; which applied is not logged |
| Goal response | Accepted, polled at 1903.7585: the client callback happened between sending and that poll. The server's acceptance time is not recorded |
| Cancel processed | Cancel response code 0, polled at the same tick, 1903.7585 |
| Result | CANCELED (5), polled at 1903.8044 |
| Server-side activity | Bounded above by 52.6 ms (sent → cancel response polled), about one feedback period. Shorter intervals cannot be resolved from these logs |
| Feedback | None recorded for goal 59, even with ±31.7 ms tolerance. The recorder captured 3947 feedback messages, all attributed to goals 1–58, at a median 50 ms period. The last one before goal 59 ended 0.158 s before it was sent |
| Before it | No refusal and no fault. The only fault is the scripted estop. The 2 refusals came later: `disarm` in FAULTED, at teleop quit |

**Context only, not a P2 measurement.**
- **`controller_state`:** 4 samples fall inside goal 59's window (gaps of at most 34.8 ms), each with \|reference − feedback\| ≤ 1.3e-12 rad.
- **Joint states:** 37 samples from 0.1 s before to 0.3 s after, with joint motion ≤ 1.3e-12 rad.
- **Not observable:**
  - action feedback for goal 59 at all;
  - the controller between `controller_state` samples and within the alignment tolerance;
  - the server-side acceptance and cancel-processing times;
  - whether the trajectory controller began sampling goal 59 before processing the cancel. Joint motion was ≤ 1.3e-12 rad either way.

**P4 (r1): PASS.** The command topic is absent, and every support check holds:
- **Requested:** in the recorder command of `tools/phase_c_run.sh`.
- **Recording covered the goal interval:** recording ran from 1672.93 to 1915.49, and the goals ran from 1680.46 to 1903.80 (223.3 s).
- **Discovery was active after start:** `/spiderx/sim/world_poses`, `/spiderx/locomotion/status`, and `/cmd_vel` at +4.8 s.
- **Recorder log:** no WARN, ERROR or FATAL (`inputs/bag_record.log`).
- **Continuity:** `/joint_states` max gap 57 ms (limit 0.5 s, `joint_states_stale_s`); `/spiderx/sim/world_poses` 47 ms (limit 1.0 s, `pose_stale_s`).
- **Owner monitor:** 486 status samples over the interval, each carrying a graph sample (taken at 1 Hz). Command-topic publishers 0 and foreign action clients 0 throughout; graph age ≤ 1.02 s (limit 3.0 s, `graph_stale_s`).
- **Node events and captures:** no ownership refusal; `Publisher count: 0` before and after the script.
- **Script:** 59 goals, 1 cancel, exercise order as scripted.

**Unchanged from the frozen scorer:** P1, P3 and P5–P8 PASS. The plant check is SUPPORTED (ĝ = 1.0, `../s2_phase_c_20261030T062140Z/response_law/`).

**r1 retrospective result: P1–P8 all PASS, plant SUPPORTED**, pending implementation review. The original frozen verdict for this run remains **FAIL**.

## Remaining uncertainty

- **Revised after the results.** r1 was written after this run's data were seen. Its rules are narrow and evidence-gated, but they have not yet been applied prospectively to an independent run.
- **Alignment is statistical:** the median of 506 pairs, with a worst case of 31.7 ms. The 158 ms gap between the last feedback and goal 59's send exceeds that.
- **Tick granularity.** Goal responses, cancel responses and results are known only to within one 20 Hz poll. For goal 59, the "no feedback" finding rests on the recording itself, not on a sub-period timing argument.
- **Sampling limits:**
  - the graph monitor samples once per second;
  - the recorder records only the publishers it discovers;
  - the command topic was captured only before and after the script.
  - A publisher that appeared and vanished between samples, without being discovered, would be invisible to all three.
- **The bag is outside the repository** (`~/spiderx_evidence/s2_phase_c_20261030T062140Z/bag`), and so is the recorder log, which is copied here.

## Files

- `assessment.json`, `summary.txt`: the tool's output, labelled as this retrospective assessment.
- `bag_hash.txt`: bag sha256 before and after.
- `inputs/bag_record.log`: copy of the recorder log.
- `shutdown_sigabrt_review.md`: the read-only review of the Gazebo server abort at shutdown.
- `SHA256SUMS`.
