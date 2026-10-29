# Offline diagnosis of the S2 FAIL (goal 1), 2026-10-29

**Scope.** This analysis is offline only.
- It read the original bag and logs, plus the controller configuration and the installed packages.
- Nothing was launched or moved. No gate, source file or configuration changed.
- Outside this directory, the evidence folder is unchanged: `README.md`, `SHA256SUMS`, `REVIEW_NOTES.md`, `SHA256SUMS.review` and the raw files.

**Verdict:** **FAIL, unchanged.** 1 goal sent, 1 cancel, no completed cycle. The authorization is spent and S3 stays blocked.

## Inputs (read-only)

- **Bag:** `~/spiderx_evidence/s2_run_20261029T100210Z/bag/bag_0.db3`.
  - Opened through SQLite with `mode=ro&immutable=1`.
  - SHA-256 `53d093da…6256d816`, identical before and after reading.
- **Node and driver logs:** byte-identical to the committed `node/` and `driver/` files (hashes in `results.json`).
- **Controller log:** the relevant lines are in `launch_log_excerpt.txt`, with the source file's SHA-256.
- **Goal 1** was rebuilt from the repository's crawl code (`tools/rebuild_goal1.py`). The rebuild's fingerprint `c6212306…` equals the one the node dispatched (`goal1_rebuilt.json`).

**Clocks.** Three clocks are kept apart throughout:
- **sim:** message header stamps, plus the derived per-update series;
- **mono:** CLOCK_MONOTONIC of the node and the driver;
- **realtime:** the bag recorder's receipt times and the controller's log stamps.

## 1. Fields, joint order, timestamps (hypotheses C and E)

| Check | Result |
|---|---|
| Joint-name order | Goal, library, action feedback and `controller_state` all use the same controller order. `/joint_states` uses the broadcaster's own order, the same in all 1003 messages. It is mapped by name here, as the node does |
| error = desired − actual, same feedback sample | max \|difference\| **0.0**: bitwise, 26 samples × 12 joints |
| feedback `actual` vs `/joint_states` at the same stamp | **0.0** for positions and velocities. All 26 feedback stamps are update times |
| `/joint_states` coverage | Every 10 ms update from 67.73 to 77.75 s sim; none missing |
| `controller_state` error, and output vs reference | error = reference − feedback exactly. **`output` = `reference` exactly** in all 412 messages |
| `controller_state` header stamps | **Not the update time.** Each message's feedback equals exactly one `/joint_states` sample: 375 unique matches, 37 ambiguous rest matches, 0 unmatched. The header stamp trails that update by 0–8 ms (241 at 0 ms, 93 at −1 ms, the rest down to −8 ms) |
| Reference vs the dispatched goal | Rebuilt with the controller's spline rules, started at the **73.08 s** update from the measured state. It equals all 66 observed references bitwise (26 feedback, 40 `controller_state`), positions and velocities, through 74.07 s. Starting at 73.07 or 73.09 is off by 2.2e-3 rad |

**Cross-check method.** No interpolation was needed or used.
- Feedback is matched to `/joint_states` by identical stamp.
- `controller_state` is matched by exact value equality, because its stamps are unreliable.

**What this shows.** The scored error is exactly the desired-minus-measured difference within one controller update. The 2026-10-29 record shows no recording, reference or mapping mismatch.

The `controller_state` stamp offset is a separate finding about the recording. P2 is scored from the feedback, so it does not affect P2. It would, however, bias any analysis that aligned `controller_state` with `/joint_states` by header stamp, by up to 8 ms × the joint speed.

## 2. Timeline of goal 1 (`timeline.csv`)

**How wall-clock events are placed in sim time:**
1. Monotonic time is converted to realtime with an offset of 1793267635.4230 s. This is the median, over 13 statuses received by both the driver and the recorder, of (recorder receipt − driver receipt); the spread is 1.6 ms.
2. Realtime is then bracketed by the recorder's receipts of the 10 ms `/joint_states` messages.

| Sim time (s) | Event | Recorded as |
|---|---|---|
| [73.07, 73.08) | node sends goal 1; controller "Received / Accepted new action goal" | mono 2020.2246; realtime 1793269655.6535 |
| **73.08** exact | trajectory start: first update sampling goal 1 (lead-in from the measured state) | derived, bitwise |
| [73.10, 73.11) | node processes the goal response (accepted) | mono 2020.2745 (node tick) |
| **74.00** exact | **first \|error\| > 0.02 rad**, at the same update in the per-update series, the feedback and `controller_state`: `rf_thigh_joint` −0.020137 (`rr_thigh_joint` identical within 3e-11 rad) | derived and recorded |
| **74.04** exact | the feedback sample the stop status reported: `rr_thigh_joint` −0.020721, exact float equality with 0.02072062 | recorded |
| [74.06, 74.07) | the node publishes that status. Its own `sim_time_s` is 74.05 (newest joint state it had processed) | mono 2021.5286 |
| **74.07** exact | last feedback sample = **maximum recorded error**: `rr_thigh_joint` −0.020973. The recorder received it at about mono 2021.5358, before the stop request | recorded |
| (74.07, 74.08) | **driver stop request** (`x`) | mono 2021.5384 |
| (74.07, 74.08) | node processes disarm, then stop (teleop shutdown sequence after the driver's SIGTERM), then **estop** → FAULTED, and **sends the cancel** | mono 2021.5422, 2021.5432, **2021.5436** |
| (74.07, 74.08) | controller **cancel callback** ("Got request to cancel goal"). Its hold position equals `/joint_states` at **74.07** exactly | realtime 1793269656.9676 |
| **74.08** exact | **hold in effect**: first update with the hold reference | derived |
| [74.10, 74.11) | node processes the **cancel response** (code 0) | mono 2021.5745 (tick) |
| [74.11, 74.12) | node processes the **result CANCELED** (5) | mono 2021.6248 (tick) |

**Placing the stop chain.** The whole chain falls between the 74.07 and 74.08 updates, from the driver's `x` to the controller's cancel callback (6.2 ms wall). Two independent facts show this:
- the 74.07 joint state reached the recorder 9.2 ms before `x`;
- the callback's hold position is the 74.07 state, and the hold applied at 74.08.

**What cannot be reconstructed:**
- **Exact sim instants of wall-clock events.** This covers the stop request, disarm, stop, estop, cancel request and cancel callback. Sim time is not sampled by those processes, so these events are known only to within one 10 ms update interval.
- **Teleop send order.** The order in which the teleop sent estop, stop and disarm was not logged. The node processed them within 1.4 ms.
- **Node receipt times.** The node does not log when it received individual feedback or joint-state messages. At the stop status its newest joint state was 74.05, processed about 28 ms after the recorder received it; the recorder already had 74.06.
- **Node callback times.** The node's callback times for the goal response, cancel response and result were not logged. The listed times are its processing ticks, so they are upper bounds.
- **Result send time.** When the controller sent the result was not logged.
- **Sub-2 ms wall intervals across processes.** For example, the estop-to-callback interval is about 1.0 ms. The offset spread is 1.6 ms, so intervals this short cannot be resolved.

## 3. Tracking before the stop vs the hold (`tracking_per_update.csv`)

**Before the stop (73.08–74.07, 100 updates).** All 26 feedback samples fall in this window.
- The worst error, \|error\| 0.020973 rad at 74.07, is a tie: `rr_thigh_joint` and `rf_thigh_joint` agree within 2.5e-11 rad at every update.
- Both first exceed 0.02 rad at 74.00.
- The reference speed of these joints peaked at 0.2186 rad/s, around 74.00–74.01.
- Per-joint maxima scale with each joint's peak reference speed, at about 0.09–0.10 s × speed:

| Joints (identical pairs) | thigh rf/rr | hip rf/rr | hip lf/lr | thigh lf/lr | foot lf/lr | foot rf/rr |
|---|---|---|---|---|---|---|
| **Max \|error\| (rad)** | 0.02097 | 0.01623 | 0.01565 | 0.01178 | 0.01135 | 0.00649 |
| **Max \|desired velocity\| (rad/s)** | 0.2186 | 0.1704 | 0.1635 | 0.1227 | 0.1269 | 0.0683 |

**From the stop request to the hold.** No update occurred in this interval, so there was no tracking after the stop request and before the hold.

**Hold (from 74.08).**
- The reference is the 74.07 position.
- During the 74.07–74.08 update the joint kept moving under the last trajectory command. It therefore overshot the hold by 0.0021 rad (`rr`/`rf_thigh_joint`).
- It then converged, with the error shrinking by a factor of 0.9 per update. The maximum hold \|error\| was 0.0021 rad.
- There are no feedback samples in the hold, because the goal was cancelled.

## 4. Plant law and lag estimate (hypotheses A, B, D)

**Data.** `/joint_states` at every update (sim time), and the command c[k]. Since output = reference, c[k] is the reconstructed reference, which matches the record bitwise, or the hold position after 74.08.

| Test (pre-stop window 73.08–74.08: 1200 joint-steps) | Result |
|---|---|
| **x[k+1] − x[k] = g (c[k] − x[k])**, least squares over all joints | **g = 0.0999999999712**; each of the 12 joints 0.100000000. Residual: RMS 1.1e-12, max 1.9e-12 rad |
| x[k+1] − x[k] = 0.01 s · v[k+1], with v from `/joint_states` | max \|difference\| 4e-15 rad: the joint moves at one constant velocity per update |
| v[k+1] = K (c[k] − x[k]) | K = 10.000 s⁻¹; residual max 1.9e-10 rad/s |
| Same law in the hold (74.08–74.68: 720 joint-steps) | g = 0.1000000003; residual max 2.1e-12 rad |
| Ordering variants, 73.09–74.07 (B) | Command of the same update: max residual 1.9e-12 rad. Command one update older: 3.5e-5. Reference one update ahead: 3.2e-5 |

**Model comparison** (`model_fits.csv`, `plant_law.png`).
- **Method:** a one-parameter least-squares fit of free-run position predictions from the measured state at 73.08.
- **Window:** 73.08–74.07 (100 updates).
- **Continuous model:** integrates the reconstructed spline exactly on a 0.25 ms grid.

| Model | Parameter | Residual, 12 joints: RMS / max (rad) | Residual, `rr_thigh_joint`: RMS / max (rad) |
|---|---|---|---|
| Discrete first-order, 10 ms updates | g = 0.1000000000 | 9.9e-12 / 1.9e-11 | 5.2e-12 / 7.6e-12 |
| Continuous first-order | τ = 0.1013 s | 4.3e-5 / 1.4e-4 | 6.2e-5 / 1.4e-4 |
| Pure delay | d = 0.0902 s | 3.9e-4 / 1.3e-3 | 5.7e-4 / 1.3e-3 |

**Lag.** The simulated joint is a discrete first-order follower. It closes 10 % of the command error per 10 ms update, with an exponential time constant of −0.01/ln(0.9) = 0.0949 s. At a constant reference speed v, its error tends to (0.01 s / g) · v = 0.100 s · v.

It is not a pure delay:
- **Delay fit:** a pure delay fits 10⁷ times worse than the per-update law.
- **Error/speed ratio:** for `rr_thigh_joint` this ratio rises from 0.051 s (73.52) to 0.097 s (74.07), as the first-order response approaches that asymptote. The README's "apparent lag 0.05–0.10 s" is this ratio, not a delay.

**Limits:**
- one interrupted phase: the first 1.00 s of a 1.7 s goal, 12 joints, speeds ≤ 0.2186 rad/s;
- welded body with guaranteed clearance, so no foot contact; effort never limiting;
- RTF ≈ 0.76: the law is in sim time and the wall timing of ROS events varies with RTF;
- no prediction is made for the rest of goal 1 or for the other 53 cycle goals.

## 5. Hypotheses

| | Hypothesis | Verdict |
|---|---|---|
| **A** | Real simulated-joint lag | **Supported, as the mechanism.** The physics positions trail the command by the per-update law above. The lag comes from the commanded velocity, not from inertia, effort limits or contact: each update's velocity command is met to 2e-10 rad/s |
| **B** | Sampling or update ordering | **Ruled out as an artifact.** Each reported error compares the reference and the position of the same update. The only ordering in the law is the update cycle itself: read, command, then 10 physics steps. Shifted variants fit 10⁷ worse |
| **C** | Timestamp or reference mismatch | **Ruled out for the scored error.** Matches are bitwise: error = desired − actual, actual = `/joint_states`, reference = dispatched goal. Recording side-finding: `controller_state` stamps trail their update by 0–8 ms |
| **D** | Lead-in, interpolation or trajectory start | **Ruled out.** Start at 73.08, the first update after acceptance; a 0.2 s cubic lead-in from the measured state; splines reproduced bitwise. The error grows only in the fast part of the phase |
| **E** | Joint mapping or inconsistent fields | **Ruled out.** Names are consistent and `/joint_states` is mapped by name; the cross-topic matches are bitwise |

## 6. Root cause

- **Proven:**
  - The 0.02 rad stop and the P2 failure follow, to 1e-11 rad, from two things: the dispatched reference, and the plant law x[k+1] = x[k] + 0.1 (c[k] − x[k]) of the simulated position joints.
  - Goal 1's reference speed reached 0.2186 rad/s. Under the law, the error approaches 0.1 s × speed, about 0.022 rad at that speed, and it reached 0.02097 rad by 74.07.
  - Nothing was mis-measured, mis-timed or mis-mapped.
- **Supported, not source-verified:** that the law is gz_ros2_control's position-to-velocity conversion, v_cmd = position_proportional_gain × update_rate × (c − x), with the gain at its default of 0.1. The evidence is the runtime log line ("position_proportional_gain has been set to: 0.1") and the identity g = 0.1 × 100 Hz × 0.01 s. The plugin's source is not installed, so the code itself was not read.
- **Unresolved:**
  - whether a 0.1 s first-order lag is the intended simulated actuator (an owner decision);
  - how the law behaves under foot contact and load (S3);
  - what the other 53 goals would do (not assessed).

## 7. Control path (Part 3)

| Item | Value | Basis |
|---|---|---|
| Command interface | position, on all 12 joints | **Runtime log** (`gz_ros2_control` joint loading; JTC line 225); `spiderx_fortress_control.xacro` |
| State interfaces | position, velocity | Runtime log; xacro |
| Controller update | 100 Hz | `spiderx_ros2_controllers.yaml`; **runtime** warning: "Desired controller update period (0.01 s)…"; `/joint_states` every 10 ms |
| Physics step | 0.001 s; real-time-factor target 1.0 | `worlds/spiderx_fortress.sdf`; runtime warning (step 0.001 s); achieved RTF ≈ 0.76 over goal 1 |
| How position commands reach the joint | A joint velocity command on every physics step, from the command and the position read at the last update; held for the 10 steps | **Measured**: g = 0.1 per update, K = 10 s⁻¹. Code: upstream gz_ros2_control 0.7.15 (conda recipe tag `release/humble/gz_ros2_control/0.7.15-1`); only binaries and headers are installed |
| `position_proportional_gain` | 0.1 | **Runtime log**, line 120. Not set in the xacro or YAML, so it is the plugin default |
| JTC output | = reference, bitwise, all 412 messages | **Measured** |
| JTC PID gains | none configured; not acting on the command | YAML has no `gains`; measured output = reference |
| Interpolation | `splines`: cubic on positions + velocities, lead-in from the measured state | **Runtime log**, line 226; reproduced bitwise |
| State publish rate / action monitor rate | 50 Hz / 20 Hz | Runtime log, lines 227–228 (set in the YAML) |
| `allow_nonzero_velocity_at_trajectory_end` | true | Runtime log, line 221 (default) |
| `open_loop_control`, `cmd_timeout`, constraints | false, 0, defaults | **Installed defaults only** (`joint_trajectory_controller_parameters.hpp`). Not in the YAML; not recorded at runtime |
| Goal tolerances | path and goal 0.05 rad per joint; goal time tolerance 1.0 s | Node config (`node/library.json`) and the goal message builder |
| Joint limits | effort 100, velocity 100 (placeholders) | URDF; commanded speeds ≤ 0.22 rad/s |
| `controllers.yaml` (description package) | legacy Gazebo Classic file; not used | Its header says so |

**Not recorded at runtime:** the full JTC parameter set and any `ros2 param` dump. Values marked as installed defaults are defaults, not observations.

## 8. Next engineering action (Part 4)

**Ranking:**
1. **Correct a proven simulation/control-path defect.** Applicable: the cause is proven and lies in the plant configuration. The documentation assumes idealised, near-rigid placeholder actuators (`M2_SIMULATION_LIMITATIONS.md` row 1, `M3_SIMULATION_LIMITATIONS.md` row 2). The unset default gain instead gives a 0.1 s first-order lag for moving references.
2. **Re-time the gait.** This would bend validated gait timing to fit a simulator default that nobody chose.
3. **Revise acceptance.** This evidence contains no independent task-level justification for doing so.

**Correct a proven measurement or reference defect:** not applicable, because none exists.

**Recommendation (one action):** make the simulated actuator response an explicit, owner-decided setting. Concretely, set gz_ros2_control's `position_proportional_gain` in `spiderx_fortress_control.xacro` to the value that represents the actuator the project intends to simulate, instead of the unset default.

**Why:**
- It addresses the proven cause.
- It leaves the gait, the 0.02 rad criterion and the node untouched.
- It turns an implicit simulator default into a documented model choice.

**How to choose the value.** It must come from that actuator decision, never from the 0.02 rad pass mark. By the measured law, the quasi-steady error is (0.01 s / gain) × reference speed.

**How to verify.** The same law fit (`tools/s2_offline_analysis.py`) can confirm the new gain from the first bag of any future authorized run.

**Not done here:** no implementation, threshold edit or motion authorization is proposed.

## Files

| File | Contents |
|---|---|
| `ANALYSIS.md` | This document |
| `results.json` | All computed values: input hashes, checks, reconstruction, law fits, tracking, clocks, timeline |
| `timeline.csv` | Goal 1 events, with mono, realtime and sim columns kept separate |
| `tracking_per_update.csv` | Every update from 73.03 to 74.68 s, all joints: desired, actual, error, velocities, recorded errors |
| `model_fits.csv` | The three model fits |
| `worst_joint_tracking.png` | `rr_thigh_joint`: desired and actual, error, velocities against sim time |
| `plant_law.png` | Per-update law across all joints |
| `goal1_rebuilt.json` | Goal 1 rebuilt from the repository code; fingerprint verified |
| `launch_log_excerpt.txt` | The controller-log lines cited here |
| `tools/` | `s2_offline_analysis.py` and `rebuild_goal1.py` |
| `SHA256SUMS.analysis` | Manifest of this directory |
