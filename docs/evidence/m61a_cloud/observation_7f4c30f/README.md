# M6.1-A phase-1 launch-only observation in Cloud simulation (`7f4c30f`)

**Result:** three separate launches of `fortress_m61a_fixed_base.launch.py`. Each had a READY
preflight, a 120 s wall-time observation that was READY at its end, and a clean shutdown, with
every §10 criterion passing. The qualifications (run 1's visual and holding checks) and two
attempts that did not become runs are described in
[`docs/M61A_CLOUD_VERIFICATION.md`](../../../M61A_CLOUD_VERIFICATION.md) §4. This was Gazebo
Fortress GUI on Xvfb with Mesa software rendering in the Cloud VM, **not the owner PC**. No goal
or command was sent; both gates were `False`.

| Path | What it is |
|---|---|
| `revision_env.txt` | Commit, kernel, CPU, memory, Gazebo and RMW versions |
| `tools/` | The scripts used: `env.sh` (shared ROS/discovery settings), `run_one.sh` (one run), the `stage_*.sh` steps, `capture.py` (read-only subscriber), `scan_check.py` / `scan_outliers.py` (the `/scan` comparison), `run_analysis.py` (the per-run criteria) and `summarize_runs.py` |
| `run_01/`, `run_02/`, `run_03/` | Per run, see below |
| `summary_runs.json` | The cross-run table |
| `attempt_aborted_helper_defect/` | 09:02 launch. The launch was fine, but the wait helper was defective (`WHY_ABORTED.txt`) |
| `attempt_failed_gazebo_startup_race_0905/` | 09:05 launch. The Gazebo GUI→server startup race stopped it (`WHY_FAILED.txt`, GUI log) |

Each run directory contains:
- **Launch and steps:** `launch.log`, `steps.log`, `run_one.out`.
- **Shell settings:** `env_A.txt` (launch shell) and `env_B.txt` (observer and CLI shell).
- **Controllers:** `controllers_active.txt`, plus its UTC timestamp.
- **Observer:** `preflight.txt` and `observe.txt`, with the observer's own `observer/<UTC>/observation.json` files.
- **Captures (`captures/`):**
  - controllers, hardware interfaces, nodes and topics;
  - command-topic information and action information;
  - one `/scan`;
  - the `ign model` pose, link and weld queries;
  - `capture.json`: 40 scans, joint states, poses, the clock, command/status/controller-state subscriptions and the weld;
  - the `scan_check*.json`/`.txt` and `scan_outliers.json` outputs;
  - the screenshots (`*.png`, real GUI captures).
- **Shutdown:** `shutdown.txt`, `stop_utc.txt`, `launch_tail.txt`.
- **Analysis:** `analysis.json` and `analysis.txt`, the per-criterion table.
- **Notes:** `NOTES.txt` in runs 1 and 2 describes the post-run corrections and the confounder.

`SHA256SUMS` covers every file.
