# M5.5 S0b verification at `e4bd880` (Cloud, 2026-10-13)

S0b fixes the two findings of the Cloud shadow session. Owner-approved; no simulation was
launched, and no goal or command was sent. All gates stay `False`.

## Receive-rate rule (`m55_command.CommandLease`, `config/m55_locomotion.yaml`)

- **Rule.** Every receipt is counted, accepted or rejected. A message is rejected
  (`command_rate_exceeded`: intent STOP, lease not renewed) if either holds:
  - **sustained:** more than 20 receipts in (t − 1.0 s, t], i.e. `max_rate_hz` 20 ×
    `rate_window_s` 1.0;
  - **burst:** more than 4 receipts in (t − 0.1 s, t], i.e. `max_burst` 4 in
    `burst_window_s` 0.1.
- **Why these values.** The shadow data: the teleop published at 10 Hz only. Delivery jitter at
  RTF 0.20 gave gaps of 141 / 118 / 197 / 49 ms, plus one quit zero 0.3 ms after a heartbeat.
  - A 20 Hz cap is 2× the heartbeat, stricter than the old 50 Hz for any sustained flood.
  - The burst window is one heartbeat period. It tolerates a bunch of 4: a stall of about
    0.3 s, 1.5× the worst observed, and below the 0.5 s lease.
  - A 30 Hz stuck-key flood is rejected at its 21st message (< 0.7 s). A flood of ≥ 50 Hz is
    rejected by the burst cap within 40 ms.
- **Loader.** It refuses `max_burst` < 2 or non-integer, `burst_window_s` > `rate_window_s`,
  `max_burst` > `max_rate_hz` × `rate_window_s`, and (as before) `max_rate_hz` ≤ heartbeat.
- **Tests.** `test_m55_rate_rule.py` has 27 tests, all with fakes. They cover:
  - the recorded gap pattern, bunches of 2–4 and the quit zero, which pass;
  - a 30 Hz stuck-key flood, a 100 Hz flood and a burst at a low average, which are rejected;
  - draining after a flood; no lease renewal by rejected messages;
  - the boundaries of both caps and both window edges; six config refusals;
  - the session in both gate modes: shadow transport with the gate `False`, the dispatching
    fake with the gate `True`. Bunched heartbeats keep WALKING; a stuck-key flood and a burst
    each reach STOPPING.
- **Mutants.** `tools/run_mutants.py`, output in `mutants_output.txt`: **11 of 11 caught**, and
  the unmutated control passes.
  - Mutants: both caps `>` → `>=`; both window edges made inclusive; rejected receipts not
    counted; burst check removed; sustained check removed; no STOP on a rate rejection; the
    loader's `max_burst ≥ 2` removed; the session ignoring `max_burst`; the old per-pair rule
    restored.
  - The session-ignores-`max_burst` mutant survived the first pass. That led to the added
    session burst test.

## Check 1.7 (M5.5 §10)

- **Restated in sim time.** READY within the in-flight goal's `duration_s` + 1.0 s of sim time,
  measured with `monitors.sim_time_s` from the last status before the space to the first READY.
  That is ≤ 3.8 s at level 0 and ≤ 5.7 s at 60 mm.
- **Wall backstop:** 60 s, which covers RTF ≥ 0.1.
- No other threshold changed.
- **Re-scored offline** on the S0 bag (`tools/rescore_1_7.py`, `check_1_7_rescore.txt`):
  2.55 s sim ≤ 3.8 s, and 11.71 s wall ≤ 60 s: **PASS**. In S0 the stop had already begun
  through the spurious rejection.

## Build and suite

See `build_and_test_summary.txt` and `colcon_test_result.txt`.
- Clean build (`rm -rf build install && colcon build --symlink-install`), exit 0.
- `colcon test` with `PYTEST_DISABLE_PLUGIN_AUTOLOAD=1`, `ROS_LOCALHOST_ONLY=1`: **1922 tests,
  0 errors, 0 failures, 0 skipped**. That is the 1894 baseline + 27 `test_m55_rate_rule` + 1
  CTest entry; `test_m55_command` stays at 39.
- **Gates:** all three are `False`, in source and as imported in a fresh environment from this
  workspace.
- **`--live`:** exit 3 for both `m6_gait_replay.py` and `m6_live_playback`.
- No leftover processes.
