# M5.5 phase-1 shadow session in Cloud (`20261013T050401Z`, branch head `a8aa56f`)

No goal, no command: `M55_LOCOMOTION_DISPATCH_ENABLED`, `M61_LIVE_DISPATCH_ENABLED` and
`LIVE_DISPATCH_ENABLED` were `False`. The launch was `fortress_m55_walking.launch.py`, the
free-base bench, with RTF 0.199. The teleop ran in a pseudo-terminal driven by
`tools/teleop_driver.py`; the run script is `tools/shadow_run.sh`. Details: `summary.json`.

**Nothing was sent.**
- `/leg_trajectory_controller/joint_trajectory`: 0 publishers and 0 messages.
- `_action/feedback`: 0 messages.
- The locomotion node has no action client.
- The joints moved 1.3e-11 rad; the body tilted at most 1.5e-7 rad.
- The 6 "goals" are SHADOW records (`node_evidence/shadow_goals.json`).
- After the run, `ros2 action info` lists one client: the bag recorder, through its
  `_action/feedback` subscription.

**Phase-1 checks (M55_KEYBOARD_WALKING.md §10), as written:** 10 PASS, 1 FAIL, 1 N/A, 1 PARTIAL.
- **1.7 FAIL:** READY came 11.7 s after the space, against a 5 s wall-time bound for a 2.8 s
  sim-time phase at RTF 0.2. A spurious rate rejection had already started the stop.
- **1.11 N/A:** there is no physical keyboard in Cloud.
- **1.13 PARTIAL:** the teleop-crash lease path was not exercised.

The bag itself (51 MB) is not kept; `bag_metadata.yaml` holds the counts. `SHA256SUMS` covers
every file.
