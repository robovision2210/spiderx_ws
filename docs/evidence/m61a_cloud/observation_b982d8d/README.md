# run_04: the single criterion-6 version-2 validation observation (Cloud, no motion)

**What this run is.**
- One fresh launch of `fortress_m61a_fixed_base.launch.py` at `b982d8d`.
- It used the frozen phase-1 harness unchanged: `tools/` here is byte-identical to
  `../observation_7f4c30f/tools/`, verified against `../closeout/frozen_harness.json` before the
  run. The procedure was `tools/run_one.sh run_04`.
- It came after the version-2 proposal was frozen and pushed (`f44c27c`).
- No motion: no goal, no command. Gates `False`.

| Path | Content |
|---|---|
| `run_04/` | Everything `run_one.sh` wrote: `steps.log`, `launch.log`, `preflight.txt`, `observe.txt`, `observer/<UTC>/observation.json` (preflight and 120 s), `captures/` (capture.json with 40 scans, `ign` and `ros2` captures, screenshots, the frozen supplementary scan checks), `analysis.*`, `shutdown.txt` |
| `run_04/criterion6_v2.json`, `.txt` | `../criterion6_v2/crit6_v2.py` applied once, unmodified, to `run_04/captures/capture.json`: version 1 (reported), version 2 (verdict) and the counterfactual |
| `run_04_requirements.md`, `.json` | Requirement table, from `run04_tables.py` |
| `revision.txt` | Commit and working-tree state at the run (documentation-only differences) |
| `tools/` | The frozen harness, byte-identical |

**How to read the "6" rows of `analysis.txt`.** The frozen `run_analysis.py` prints two
supplementary `/scan` rows labelled "6". They are the closeout's 6a/6b metrics, **not**
criterion 6. Criterion 6 version 1, the criterion of record, and the version-2 proposal are
reported separately (`criterion6_v2.json`, `run_04_requirements.md`).

**Caveats.**
- Pose receipt max gap 0.340 s (p99.9 0.058 s): below the 1.0 s limit, but larger than in runs
  1–3 (≤ 0.076 s). During the 120 s observation the operating session read the preflight JSON
  and wrote this README (light I/O).
- The CLI daemon fault seen while waiting for the controllers was restarted by the frozen wait
  helper (`daemon_events.txt`), as designed.

`SHA256SUMS` covers every file in this folder.
