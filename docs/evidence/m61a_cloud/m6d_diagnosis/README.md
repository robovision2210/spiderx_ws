# M6.0-D isolated-test failure: diagnosis evidence (Cloud, isolated domains only)

Diagnosis of the failure preserved in [`../incident_20261009T075335Z/`](../incident_20261009T075335Z/README.md).
Every run here uses in-process test doubles on private, localhost-only ROS domains (150–199).
None of them involves a simulator, a real controller or hardware. The write-up is in
[`docs/M61A_CLOUD_VERIFICATION.md`](../../../M61A_CLOUD_VERIFICATION.md) §1–2.

| Path | What it is |
|---|---|
| `diag_plugin.py` | pytest observer plugin. It runs the repository test unmodified and records both readiness reports in full, the isolation snapshots, tick overlaps and publish order in `runs/<label>/diag.json`. `DIAG_STACK=original` loads `original_stack/` |
| `original_stack/m6d_isolated_stack.py` | The test double as of `41fc10f` (SHA-256 `34817e38…912a`) |
| `run_diag.sh`, `campaign.sh`, `summarize.py` | One run, N runs, and a table. Each run has its own `--basetemp`, so its `live_outcome.json` is kept |
| `runs/smoke_orig_noload`, `runs/orig_noload_01..30` | Original double, no competing load: 31 of 31 passed |
| `runs/orig_load8_1..5` | Original double with 8 CPU burners: 4 passed; 1 `TRACKING_FAILED` / `sample_gap` |
| `summary_runs.jsonl` | One line per run, plus a totals line |
| `stress_order.py`, `stress/` | 60 s tick stress: overlapping ticks and stamp order |
| `reorder_demo.py`, `reorder/` | A 0–25 ms delay between stamping and publishing. Original: 84 backward stamps. Fixed: 0 |
| `stress_teardown.py`, `teardown/` | 150 start/stop cycles. Original: 12 orphaned `InvalidHandle` Tasks (`original_150_traced.log`). Fixed: 0 (`fixed_150_traced.log`). `repo_drain_only_attempt_*` is the first, insufficient fix |
| `ab_tests/` | The three new regression tests: they fail on the original double (`original.log`) and pass on the fixed one (`fixed.log`) |
| `campaign_orig_noload.log` | Campaign console output |

The scripts reference the workspace at `/home/user/spiderx_ws` (the Cloud VM). Adjust `WS` in
`run_diag.sh` and the paths in the `stress_*`/`reorder_*` scripts to rerun them elsewhere.
`SHA256SUMS` covers every file.
