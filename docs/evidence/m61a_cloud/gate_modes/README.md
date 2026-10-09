# M6.1 gate-mode test evidence (offline; no simulator)

These are the controller test suite results with the enabling patch
(`docs/patches/m61_enable_one_cycle.UNAPPLIED.patch`) applied in a **temporary detached
worktree** under the session scratchpad. The worktree had no branch and was removed afterwards.
It was tests only: no simulator, no launch, no ROS graph beyond the isolated-domain test doubles
on domains 150–199 with `ROS_LOCALHOST_ONLY=1`.

| File | Content |
|---|---|
| `tree_under_test.diff` | The `src/` changes under test on top of `f44c27c`. It is byte-identical to `git diff f44c27c b982d8d -- src/`, so the tree under test was `b982d8d` plus the patch |
| `enabled_mode_before_test_fix.txt` | First enabled-mode run of the whole `spiderx_controller` suite: **3 failed, 1452 passed**. The three failures (`test_success_outcome_fields`, `test_mock_cli_writes_the_evidence_directory`, `test_dry_run_report_is_never_overwritten`) asserted a recorded gate of `False`. The earlier patch header had not listed them |
| `enabled_mode_after_test_fix.txt` | After those three were made to assert the pin: **1457 passed, 0 failed** |
| `disabled_mode_build_and_test_summary.txt`, `disabled_mode_colcon_test_result.txt` | The disabled (committed) mode, at `b982d8d` (`src/` identical; the working tree's only differences were documentation): clean build plus the full `colcon test`, **1497 tests, 0 errors, 0 failures, 0 skipped** (`colcon test-result --all`) |

`SHA256SUMS` covers the files in this folder.
