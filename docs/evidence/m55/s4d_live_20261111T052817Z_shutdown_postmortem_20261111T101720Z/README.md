# S4d-M live attempt: shutdown postmortem (existing records only)

A read-only review, written 2026-11-11, of the shutdown after the S4d-M live attempt
(`../s4d_live_20261111T052817Z/`). It uses only records kept from that run, and makes no reproduction, patch or rerun. The exact lines
are in `excerpts.txt`. ros2 launch's own timestamped log is copied as `ros_launch_log_2026-11-11-05-50-12-vm-6480.log`.

## Finding
**The simulator shutdown was not graceful.**
- After the run's SIGINT, the server did not exit within the `ign gazebo` launcher's fixed 5 s grace period, and the launcher killed it with SIGKILL.
- The harness and the run's owned cleanup sent only SIGINT and killed nothing.
- The recorder had already closed the bag, 3.6 s before the server received its SIGINT.
- No stated cleanup criterion was violated.
- Why the server did not exit within 5 s is not established.

## Sequence (UTC, 2026-11-11)
| Time | Event | Source |
|---|---|---|
| 06:02:48.550 | Recorder: cache written, "Recording stopped"; `bag_0.db3` and `metadata.yaml` last written at 06:02:48.54 | `run/bag_record.log`; bag file times |
| 06:02:49–51 | Harness: "bag stopped", screenshot 2, strace attached to server 6683, "stop launch" | `run/steps.log` 19–23 |
| 06:02:51.521 | `stage_stop.sh`: SIGINT to process group 6480 (the launch) | `run/stop_utc.txt`, `run/shutdown.txt` |
| 06:02:51.65–51.74 | Bridges, `robot_state_publisher` and the locomotion node finished cleanly | ros2 launch log 14–17 |
| ≤ 06:02:52.136 | Launcher 6554, in group 6480, runs its INT trap. It sends SIGINT to the GUI's group 6686; the log has no GUI escalation line. It then sends SIGINT to the server's group 6683 | `cmdgazebo6.rb` 507–509, 182–183; strace `si_pid=6554` |
| 06:02:52.136 | The server's rclcpp signal handler runs (signum 2) | `run/launch.log` 637 |
| 06:02:57.1–57.7 | Launcher: "Escalating to SIGKILL on [Ignition Gazebo Server]", then SIGKILL to group 6683; strace: `+++ killed by SIGKILL +++` | `run/launch.log` 638; `cmdgazebo6.rb` 203–205; `run/server_exit_strace.txt` |
| 06:02:57.715 | `ign gazebo-1` (pid 6551) "finished cleanly"; ros2 launch shuts down | ros2 launch log 18–19 |
| ≈ 06:03:00.5 | Launch group empty, 9 s after SIGINT; daemon and Xvfb stopped; nothing remaining | `run/shutdown.txt` |
| 06:03:02–03 | Owned cleanup: everything already stopped, nothing signalled, no relevant process left | `run/cleanup.log` |

**Process groups.** The launcher forks the server and the GUI into their own process groups (`Process.setpgid(0, 0)`, `cmdgazebo6.rb` 485 and 501; `run/premotion/launch_group.txt`). The harness's group SIGINT therefore reached the launcher, not the server. strace recorded exactly one SIGINT to the server, sent by the launcher, pid 6554.

## Who sent the SIGKILL
- **The launcher, pid 6554.**
  - Its `killProcess` waits 5.0 s, polling every 1 ms, then prints the escalation line and immediately calls `Process.kill("-KILL", pid)` (`cmdgazebo6.rb` 182–206).
  - ros2 launch reported `ign gazebo-1` (the shell that ran the launcher) finished 5.58 s after the server's signal handler ran. That fits the 5 s window.
- **Not the harness.**
  - `stage_stop.sh` sends SIGINT only.
  - `cleanup.sh` logged no stop or kill: every owned process was already gone. Its SIGKILL paths cover only the teleop sessions, the recorder group and launch-session leftovers.
- **Not ros2 launch.** Its timestamped log has no SIGTERM or SIGKILL escalation. It signals its own children only; the server is a grandchild, in another process group.
- strace cannot name a SIGKILL's sender. The attribution rests on the launcher's message, the timing and the absence of any other kill.

## Recording integrity
- The recorder flushed and closed the bag at 06:02:48.55: 3.0 s before the launch SIGINT, and at least 8.5 s before the SIGKILL.
- The bag's sha256 was the same before the restore, after scoring and after the supplementary analysis (`run/bag_hashes.txt`).
- The SIGKILL did not affect the recording.

## Stated criteria
- **The S4 plan's cleanup rule: met.** The rule (`docs/S4_KEYBOARD_TELEOP_PLAN.md` §4) is: SIGINT, the server's exit observed, then the full restore, which must be DISABLED AND VERIFIED with all three gates `False`. The exit was observed by strace, and the restore was DISABLED AND VERIFIED.
- **The harness's own checks: met.** The launch group was empty, nothing remained, and no relevant process was left.
- **The scorer.** No S4d-M check (M1–M16) concerns the simulator's exit. The scorer's PASS is **not** evidence of a graceful shutdown.

## Lost record
The server's own ROS log file stops at exactly 32,768 bytes, mid-line, at 06:01:20.69: its last buffered output was lost when it was killed. The file is `~/.ros/log/ign gazebo server_6683_1794376223187.log`, not in the repository. The same messages, to the end, are in the launch's captured output, `run/launch.log`. The scorer used that output; its controller-log result was 111 received, 111 accepted, 0 cancels.

## Not established
- What the server was doing after SIGINT, and why it did not exit within 5 s.
- Whether it would have exited given more time.
- The exact time of the SIGKILL. It is only bounded, between 06:02:57.1 and 06:02:57.7.
- The GUI's exit status. It was not observed, and there is no GUI escalation line.
- Whether live dispatch, host load or the overlay contributed.

**Earlier runs.** In the launch logs kept for 16 runs, the server escalation also occurred in the actuator phase-B run (2026-10-30), and a GUI escalation in the S4c level-1 shadow. The other 13 have none, including the S4d-M shadow and S4c level-2 live (`excerpts.txt`, end).

## Impact and decision
- **Acceptance evidence.** No effect: the script had completed and the bag was closed before the stop began.
- **Harness.** No run-local cleanup defect is proven. The harness and the owned cleanup behaved as designed, so no correction is made and no timeout is changed.
- **Later launches.** Stop the recorder before the launch; observe the server's exit by strace; report a SIGKILL as such, never as a graceful exit. On this evidence, cleanup and ownership, and recording integrity, are not unsafe.
