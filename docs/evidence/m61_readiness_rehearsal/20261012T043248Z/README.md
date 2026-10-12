# M6.1 readiness fix: no-motion Cloud rehearsal at `61f3ffc` (disabled build)

**What ran.** The fix for the refused one-cycle attempt (`a1a7d60`; Cloud report §11) was rehearsed
against Cloud Gazebo on the **disabled** build, `HEAD` = origin = `61f3ffc`, gates `False`
(`revision.txt`).
- One launch of `fortress_m61a_fixed_base.launch.py`, localhost only, domain 0. Preflight
  observer READY.
- 16 read-only trials, then captures, a clean stop and the restore script in verify-only mode.
- Nothing could dispatch, and nothing did: 0 goals, 0 cancels, 0 dispatch calls in every trial.
  `--live` was never enabled; the gate was never touched.

**Result.**
- **The fixed production path was READY in 8 of 8 trials.** Its bounded discovery found the
  action server after 0.0–3.02 s, and both readiness observations and the pre-send streams check
  then passed.
- **The pre-fix path refused once in 8 replays, with the attempt's exact signature.** In trial 13
  its single action-server query, at node age 2.08 s, saw no server: `action_server_missing`,
  `stack_incompatible`, every other check passed, collection 4.53 s (the attempt: 4.78 s). The
  server was running the whole time; the same node saw it at its next query, 2.5 s later.
- Cleanup: `DISABLED AND VERIFIED`, `--live` exit 3 (`restore/`).

## 1. Trials

Each trial (`tools/trial.sh`) starts the bag recorder, waits 3 s, starts the 30 s read-only
observer, waits `d` s, and only then starts the probe: **observation always starts before the
probe**. `d` = 0 reproduces the attempt's start-up (bag, observer and tool together); `d` = 10 is
a quiet start.
- **new**: `ros2 run spiderx_controller m61_readiness_rehearsal --domain-id 0`, the production
  readiness path of `61f3ffc` (bounded discovery, readiness #1, readiness #2, pre-send streams
  check) on `RehearsalTransport`, which cannot send or cancel.
- **old**: `tools/old_path_probe.py`, the readiness composition of `a1a7d60` (no discovery wait)
  replayed on the same `RehearsalTransport`, with each action-server query timed. Diagnostic only.

| # | Path | d | Result | Action server |
|---|---|---|---|---|
| 1, 3, 5, 7, 9, 11 | old | 0 | permitted | seen by the single query at node age 2.09–2.12 s |
| 2 | new | 0 | **READY** | found after **3.02 s** (node, status publisher and per-node server all first seen at 3.02 s) |
| 4, 6, 8, 10, 12 | new | 0 | **READY** | found after 0.25, 0.00, 1.00, 0.00, 1.50 s |
| **13** | **old** | **10** | **refused: `action_server_missing` → `stack_incompatible`** | **single query at node age 2.08 s saw none**; every other check passed |
| 15 | old | 10 | permitted | seen at node age 2.10 s |
| 14, 16 | new | 10 | **READY** | found after 1.01, 0.00 s |

Per trial (`run/trials_summary.txt`, `.json`): readiness #2 age at the send check was 5.1–5.2 s,
under the 10 s expiry. The streams check passed every time. The observer saw 0 command
publishers in every trial; its only action client was the probe's own, while the probe ran. No
publisher ever appeared on `/leg_trajectory_controller/joint_trajectory`, so the recorder never
subscribed to it (`bag_metadata.yaml`).

In trial 8 the server's status publisher and this client's match appeared at 0.75 s, but the
node and the per-node attribution only at 1.00 s. Endpoint discovery and node attribution can lag
each other; the per-node query, which readiness uses, waits for the slower one.

## 2. What this shows about the attempt's refusal

- **The mechanism is reproduced, with the pre-fix code against a running server.** A single
  per-node query about 2.1 s after the node is created can miss a running server. Trial 13
  reproduced the refusal with every field matching the attempt's record. Trial 2 shows the same
  latency through the new path: 3.02 s to first sight, beyond the old query time.
- **The attempt's own instance is attributed, not observed.** Its record has no per-query timing,
  so this is the best-supported explanation of the 2026-10-12 refusal, but not a direct
  observation of it. Concurrent start-up is **not** required: the reproduced miss was in a quiet
  trial.
- **The fix addresses that mechanism without relaxing anything.** It waits, bounded by the existing
  10 s server wait, for the same query to see the server. It then observes everything afresh. The
  worst latency seen here, 3.02 s, leaves a margin of about 3.3× under the bound. A server that
  never appears is still refused; that is covered by fake-clock and isolated-domain tests
  (`test_m61_discovery.py`), not exercised here.

## 3. Files

| Path | Content |
|---|---|
| `revision.txt` | `HEAD` = origin = `61f3ffc`, clean; both gates `False` |
| `tools/` | Frozen stage scripts copied from the attempt's harness (`env.sh`, `stage_start.sh`, `stage_wait_controllers.sh`, `stage_observe.sh`, `stage_stop.sh`); `trial.sh`, `run_rehearsal.sh` (the driver), `old_path_probe.py`, `summarize_trials.py` |
| `run/` | Launch log and timing, preflight (`observer/`, `preflight.txt`), `steps.log`, `trials.log`, `trials_summary.{txt,json}`, captures after the trials (`ros2 action info`: 1 server, 0 clients; command topic: 0 publishers), `shutdown.txt` (process group empty after 4 s, nothing left) |
| `run/trials/NN_<path>_d<d>/` | Per trial: observer record, bag metadata and recorder log, the probe's output and its complete record (`rehearsal/<UTC>/rehearsal.json` with every readiness check, monotonic timestamps, the discovery polls, domain and action name; or `old_path.json` with the timed query) |
| `run/trials/13_old_d10/bag.tar.xz`, `run/trials/02_new_d0/bag.tar.xz` | The bags of the reproduced refusal and of the slowest discovery (`tar -xJf`; round trip verified byte-identical). The other 14 bags (about 20 MB each) are summarized by their metadata only |
| `tests/` | Enabled-mode suite (patch applied in a temporary worktree, tests only): `enabled_mode_summary.txt`, 1583 passed; the superseded first run and its causes: `enabled_mode_first_run_superseded.txt`. Disabled clean build and full `colcon test`: `disabled_full_colcon_test.txt` |
| `restore/`, `restore_verify.out` | `scripts/m61_restore_and_verify_disabled.sh --verify-only` after the stop: no simulation process, `src/` = `61f3ffc`, imported gates `False` from this workspace, `ros2 pkg prefix` here, `--live` exit 3, shell clean: **DISABLED AND VERIFIED** |

The run directory's absolute path, `/root/spiderx_evidence/m61_rehearsal_20261012T043248Z/`,
appears in some records; it is this folder. `SHA256SUMS` covers every file except itself.

**Not shown:** anything about a goal, tracking, gates in motion, walking, contact or hardware.
