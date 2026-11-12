# Review notes on the S2 attempt of 2026-10-29

Added at review, 2026-10-29. `README.md`, the raw evidence and `SHA256SUMS` (47 files) are unchanged.
This note and the original manifest are covered by `SHA256SUMS.review`.

## Verdict (unchanged)

**FAIL.**
- **Sent:** 1 phase goal and 1 cancel; no cycle completed.
- **Stop:** the authorized tracking condition, during goal 1 (level 0, forward, shift b0→b1).
- **Authorization:** spent. S3 stays blocked.

## Two different tracking numbers

| Value | What it is | Source |
|---|---|---|
| **0.02072 rad** | The **stop observation**: `goal.max_tracking_error_rad` in the node status received by the driver at monotonic 2021.5301 s. The node stamped that status at sim 74.05 s. The value is the node's running maximum of \|feedback.error.positions\| received so far. The driver's stop condition fired on it, and `x` followed 8.3 ms later (monotonic 2021.5384 s) | `driver/driver.jsonl.gz`, `driver/driver_summary.json` |
| **0.02097 rad** | The **maximum recorded feedback error**: the largest \|error\| over all 26 action-feedback samples of goal 1 in the bag (`rr_thigh_joint`). It is the last feedback sample recorded. Its timing relative to the stop is analysed separately and not stated here | `tracking.csv`, `summary.json` (P2), `goal1_tracking_profile.txt` |

P2 is scored on the recorded maximum (0.02097 rad). The stop was triggered by the status value
(0.02072 rad). Both exceed 0.02 rad.

## Test-suite accounting

- The full suite was run once through `colcon test`. `colcon test-result` reported **1929 tests,
  0 errors, 0 failures, 0 skipped**.
- The 2 tests of `spiderx_scripts` were **not executed by colcon**: its test step fell back to
  unittest and ran 0 tests.
- Those 2 tests were **executed directly with pytest** afterwards: 2 passed
  (`disabled/spiderx_scripts_pytest.txt`).
- The record is therefore 1929 through colcon plus 2 directly executed pytest cases. It is not
  "1931 collected by colcon". The README's "1931 tests in all" means that sum.
