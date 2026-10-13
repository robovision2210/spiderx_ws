# run_05L: pose-stream freshness, advancing simulation time and a paused world (Cloud, no motion)

**What this run is.**
- One supervised launch of `fortress_m61a_fixed_base.launch.py` at `b982d8d`. It is **not** a
  criterion-6 run.
- Start, wait and stop used the frozen phase-1 stage scripts (`tools/`, byte-identical to
  `../observation_7f4c30f/tools/`).
- Nothing was commanded and no goal was sent; the gates stayed `False`.
- The only intervention was a simulator control: the Gazebo world was paused and resumed through
  its `WorldControl` service (`B_pause_reply.txt`, `C_resume_reply.txt`: `data: true`).

## Tools

| File | Content |
|---|---|
| `tools/run_liveness.sh` | Start and wait, then three phases, then stop: **A** running, **B** paused (`pause: true`), **C** resumed (`pause: false`). Each phase records: gz-side `pose/info` (3 messages, then 3 more 1 s later), `/stats` and the world clock (`ign topic -e`); the ROS stream probe; and the observer `--preflight` |
| `tools/probe_streams.py` | A read-only rclpy node with subscriptions only. Over 5 s it records `/clock`, `/spiderx/sim/world_poses` and `/joint_states`: rates, the sim advance, and the node's `/clock` at the first and last pose receipt |

## Results (`run_05L/liveness/`)

| | A running | B paused | C resumed |
|---|---|---|---|
| gz `pose/info` header time, first message of each capture (1 s apart) | 5.282 → 6.239 s | **14.666 → 14.666 s** | 16.933 → 17.771 s |
| `pose/info` entries per message | 68 | 68 | 68 |
| `/stats` | RTF 0.80 | **`paused: true`** | RTF 0.93 |
| ROS `/clock` over 5 s | 490 Hz, +2.54 s sim | **651 Hz, +0.000 s** | 675 Hz, +3.61 s |
| ROS pose stream over 5 s | 43 Hz; node clock at receipt 8.445 → 10.894 s | **52 Hz; 14.666 → 14.666 s** | 57 Hz; 19.079 → 22.682 s |
| ROS `/joint_states` over 5 s | 52 Hz | **0 messages** | 74 Hz |
| Observer `--preflight` | READY (sim advance 0.659 s in the last 1.0 s, 56 samples) | **NOT READY**: `body_pose_sim_time_not_advancing`, `sim_time_not_advancing`, `joint_states_missing`. Meanwhile `body_pose_fresh` **passed** (age 0.002 s) | READY (0.668 s, 57 samples) |

**What this shows.**
- While paused, the clock and the pose stream keep arriving at full rate with a frozen time.
  Receipt freshness therefore cannot show progress.
- The corrected readiness refuses the paused world, by two separate codes, while the pose is
  "fresh".
- The gz-side header time of `pose/info` advances with the simulation when it runs: each message
  is regenerated from the current state at the current sim time.
- The model and `dummy_link` entries held one value throughout: `spiderx` (0, 0, 0) and
  `dummy_link` (0, 0, 0.125), as expected for a weld (§9.1 of the Cloud report). The content
  therefore cannot show liveness without motion (U-L1, plan E9).

## Integrity and redaction

- **Redacted lines.** The frozen stop script's leftover check (`pgrep -af …`) matched the shell
  that invoked `run_liveness.sh`, because that command line contains the pattern words.
  `run_05L/shutdown.txt` and `run_liveness.out` list it. That line is **redacted** here because
  it exposed internal session paths; the unredacted originals are kept outside the repository.
  It was not a simulator process: `ps` right after the run showed no simulator, bridge, Xvfb or
  daemon process.
- **Addresses.** The gz publisher address in `gz_pose_info_publishers.txt` is the container's
  private address (192.0.2.0/24, a documentation range).
- **Manifest.** `SHA256SUMS` covers every file in this folder.
