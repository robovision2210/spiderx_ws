# Verification of the corrected revision `7f4c30f` (Cloud: offline and isolated domain)

The run had no competing CPU load (load average 0.1–0.7). It ran on the Cloud VM, not the owner PC.
The write-up is in [`docs/M61A_CLOUD_VERIFICATION.md`](../../../M61A_CLOUD_VERIFICATION.md) §3.

| Path | What it is |
|---|---|
| `revision.txt` | Branch, HEAD, tracked changes (0) and the competing-process check |
| `build.log` | Clean build (`rm -rf build install`; `log/` kept), 8 packages, exit 0 |
| `focused.log`, `focused.xunit.xml` | The 8 M6.0-D test files: 276 passed |
| `focused_outcomes/` | The `live_outcome.json` files from those tests. The pytest temp tree itself is not kept |
| `test.log` | `colcon test` + `colcon test-result --verbose`: **1468 tests, 0 errors, 0 failures, 0 skipped** |
| `all_xunit/`, `colcon_test_log/` | Per-file xunit results and the colcon test log (no "never retrieved" line) |
| `static/`, `static_checks.sh` | Static validators, `--check-config`, dry runs, `--interface-only`, `--live` refusals (`summary.tsv`) |
| `log_dir_before.txt` | `log/` listing before the build |

`SHA256SUMS` covers every file.
