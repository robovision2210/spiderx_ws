# A3 and foot-travel review of the first free-base S3 crawl (offline, 2026-11-01)

The original evidence is `../s3_free_base_20261101T115002Z/` (commit `a73ea95`). **Its verdict is unchanged: INCONCLUSIVE.** A1, A2 and A4–A8 PASS; A3 is UNMEASURED; the frozen scorer and its output are untouched.

This review is offline and read-only:
- no launch, build, goal or cancel;
- the bag was opened read-only, and its SHA-256 `52fdb5ae…` was the same before and after (`bag_sha256.txt`);
- the review code is `scripts/s3/review_a3.py` with its tests in `scripts/s3/test_review_a3.py`, separate from `score_s3.py`, which it imports unchanged.

## Timestamp semantics, from the code as run
All times below are CLOCK_MONOTONIC (the driver and the node read the same host clock), unless marked otherwise.

| Quantity | Recorded? | Value (s) | Source and meaning |
|---|---|---|---|
| Goal 55 sent | yes | 1753.415895 | node, when `send_goal_async` was called |
| Driver's last record before the key (`reached: estop exercise: goal in flight`) | yes | 1753.4301 (±0.05 ms rounding) | `log()` entry in the driver's command thread, before `key('x')` was called: an **inferred lower bound** on the write start |
| `x` write start | **no** | — | not recorded |
| `x` write completion | **no** | — | not recorded |
| `x` key record | yes | 1753.4316 (±0.05 ms) | `log()` entry **after `os.write` returned**: an **inferred upper bound** on the completion, not the press time |
| **Inferred write window** | **inferred, not measured** | after 1753.43015, no later than 1753.4316 | bounds on when the write happened; the write's actual time and duration are unknown |
| Teleop read of `x` and its estop service call | **no** | — | not recorded |
| Node estop request, fault, FAULTED and cancel **decision** | yes | 1753.431099 | node service callback (after its graph snapshot). There was no goal handle yet, so the cancel was queued |
| Cancel transmission to the controller | **no** | — | sent from the goal-response callback, at a time the node does not record |
| Controller: goal 55 received / accepted | its own log, ROS system time | ≈1753.42352 / 1753.42362 | mapped with the bag alignment offset (p05–p95 −1.53…+1.05 ms) |
| Controller: cancel request received | its own log | ≈1753.44158 | same mapping |
| Node `goal_response` (accepted) and `cancel_response` (code 0) | yes, poll time | 1753.466092 | 20 Hz session tick that polled the callbacks, not the callback time |
| Node `result` CANCELED | yes, poll time | 1753.517274 | same |
| Teleop prints `[estop] ok: <node message>` | yes, driver pump receipt | 1753.9097 | when the driver read the teleop's output |

A clock cross-check of the mapping is not possible after the fact. The container restarted after the run (uptime 222 s at review), so the current realtime−monotonic offset no longer applies.

**Why the frozen proof's three checks failed.** It uses the key record's 1753.4316 as the time `x` was sent, and requires the reaction at or after it.
1. **`estop_fault_after_key`.** The estop fault (1753.431099) is earlier than 1753.4316, so it is not counted as planned.
2. **`early_cancel_request`.** The cancel decision (1753.431099, the same callback) is also earlier than 1753.4316. Its other condition, being earlier than the polled goal response, holds.
3. **`no_unplanned_fault_before`.** Because of (1), the estop fault is counted as unplanned.

The review recomputes the frozen proof and reproduces exactly these three failures.

## A. Is the failed ordering check based on incorrect timestamp semantics?
**Yes.** The frozen check treats a post-write log time as the earliest possible send.

The write lies in an **inferred** window: after 1753.43015 and no later than 1753.4316. This is a bound, not a measured write time or duration. The node handled the estop at 1753.431099, inside that window: 0.95 ms after the earliest possible write start and 0.5 ms before the driver logged the key.

**Why the lower bound holds** (`scripts/s2/s2_driver.py`, `scripts/s3/s3_driver.py`):
- **Command-thread sequence.** Only `Driver.wait()` writes `reached` records, and it runs in the driver's command thread. `estop_exercise()` calls `wait('estop exercise: goal in flight', …)` and then `key('x', …)` in that same thread. `log()` takes its time on entry, so this record's time precedes the write.
- **Not file adjacency.** The executor thread writes `status` records and the pump thread writes `teleop_out` records into the same file. The key-repeat thread writes keys without logging. The review skips other record types and stops at any earlier key record.
- **Rounding.** The record's time is rounded to 0.1 ms. The bound uses the rounding interval's upper end (+0.05 ms), which only makes the check stricter.

A receiver can legitimately react before the sender's post-write log. The driver logs only after `os.write` returns and its thread resumes. No tolerance is needed to reach this conclusion; the only margin applied is the logger's own 0.05 ms rounding, and it makes the check stricter.

## B. Can the existing evidence support the planned-cancel exception?
**Yes, under a revised check; not under the frozen one.** Timing alone does not prove causality, so the exception also rests on this recorded chain. The estop's source is **inferred** from this evidence; the node does not record which client called its estop service.
- **One estop key.** The driver wrote exactly one estop-capable key, the scripted `x` (no Esc, no second `x`), and made no estop-service fallback call.
- **Teleop mapping.** The teleop code maps only `x` and Esc to the estop service.
- **One teleop call.** The teleop printed exactly one estop response, `[estop] ok: emergency stop: goal cancelled (if any), holding; reset and arm to continue`. It is the node's message word for word.
- **One node request.** The node recorded exactly one estop request in the whole run, at the fault. It was the only fault.
- **Graph at pre-arm.** Only the expected nodes were present, and no other teleop or driver process.
- **Goal 55.** It is the estop goal, (0→1, forward, level 0), the last of 55, and the controller accepted it.
- **One cancel.** There was exactly one cancel: decided at the fault and received once by the controller after its last acceptance. That order comes from the controller's own log. Mapped onto the node's clock with the bag alignment offset, the receipt was about 10.5 ms after the decision, within a few ms. It was answered with code 0 and ended in CANCELED.
- **Nothing else before the result.** No refusal before the result, and no feedback attributed to goal 55.

**Retrospective assessment under criteria revision S3-r1 — proposed, not adopted.** The frozen exception's point time `t_x` is replaced by the write bracket (`t_pre`, `t_post`]. The fault and the cancel decision must come after `t_pre` plus its rounding. These provenance checks are added: one estop key, one node estop request at the fault, one matching teleop response, no fallback, and the controller's log order. All 17 checks hold (`review.json` → `S3_r1_proof`).
- Under S3-r1, A3 would be PASS. Goals 1–54 have feedback on each goal, max |error| 0.00225 rad (abort 0.05), with no finding above 0.02. Goal 55 would be N/A.
- With A1, A2 and A4–A8 already PASS, the overall result would be PASS **only if** S3-r1 is adopted.

Adopting S3-r1 is the owner's decision. This review does not change the recorded INCONCLUSIVE.

## C. What remains unmeasured or ambiguous
- **Goal 55's tracking is unmeasured.** There was no feedback: the controller accepted the goal and received the cancel about 18 ms later. N/A is not a value. A6 (joint motion 1.2e-11 rad, tilt 3.1e-6 rad and z ≥ 0.054495 m over 10 s of sim after the cancel) shows that the hold held. It does not replace trajectory tracking.
- **Unrecorded times.** The `x` write's start and completion, the teleop's read and service call, and the node's cancel transmission are not recorded, only bounded. The controller times depend on the bag alignment mapping, which is within a few ms.
- **The estop caller is inferred.** The node does not record which client called the estop service. The inference rests on the single teleop response, the single node request and the absence of any other source in the recorded data. A ROS client outside the recorded graph cannot be excluded by the recorded data alone.

## Foot travel (`foot_travel.csv`, recorded only; no new criterion)
- **The 27.3 mm maximum** is the horizontal travel of the `rf_foot_1` **link origin** (the knee/foot-joint axis), in a body-shift phase (phase 4, no swing leg; seq 14). The same value appears in every cycle, and `rr_foot_1` has the same pattern.
  - The link origin is the `rf_foot_joint` axis, the knee: origin xyz (0.012, −0.0778, −0.0778) in the thigh frame. The contact surface is at the far end of the CAD foot mesh: its farthest vertex is 0.108 m from the joint axis.
  - During a shift the leg rotates about the planted tip, so the knee travels with the body.
- **CAD-tip material point** (a proxy, not the contact point). The CAD mesh vertex lowest in the world at each window start is carried rigidly with the link. On stance feet it moved **at most 0.11 mm horizontally** (seq 5, `rf_foot_1`), and its height stayed within −0.12 to +1.12 mm.
  - The rise of up to 1.1 mm is consistent with the foot rotating or rolling about its contact.
  - A swing foot's tip moved up to 20.03 mm (the stride) and rose up to 12.5 mm.
- **Frame composition** is the same as the scorer's: world = model pose × the link's pose relative to the model.
- **Defect in the frozen `slip_rows`.** Its swing exclusion uses the keys `lf/rf/lr/rr`, but the events name legs `front_left`, `front_right`, `rear_left` and `rear_right`. No swing foot was excluded: the 24 swing-foot rows, about 20 mm each, are in `slip.csv` as if they were stance. They are not the source of the 27.3 mm maximum.
- **Conclusion.** The recorded metric does not support a physical slipping claim.
  - The data has no contact positions or forces, so contact-point slip cannot be measured directly, and rolling cannot be separated from sliding below about 0.1 mm.
  - The CAD-tip material point moved at most 0.11 mm horizontally on stance feet. This is not a direct measurement of contact-point slip and does not establish zero slipping.

## Artifacts
- **`review.json`:** everything computed:
  - the key bracket, teleop responses, node estop requests and controller log order;
  - the frozen proof recomputed, with its inputs;
  - the S3-r1 proof and A3 rows;
  - the foot-travel summary.
- **`foot_travel.csv`:** per goal and foot: role, link-origin travel, tip travel, tip height range.
- **`focused_tests.txt`:** 17 synthetic tests, OK.
- **`bag_sha256.txt`:** bag hash before and after each run of the review.
- **`disabled_state_readonly.txt`:** read-only check (imports and git only): gates False, HEAD = remote, no enabling branch, no processes.
- **`review_stdout.txt`**, and `SHA256SUMS` over this directory.
