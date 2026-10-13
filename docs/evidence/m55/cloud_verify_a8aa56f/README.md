# M5.5 after merging origin/main (`a8aa56f`; main = `f22ac9f`, PR #17)

**Offline (Cloud VM, RoboStack ROS 2 Humble).** No goal was sent. All gates are `False`: M5.5,
M6.1 and M6.0-D.

| File | Content |
|---|---|
| `build_and_test_summary.txt` | Clean build (8 packages, exit 0) and `colcon test` with `PYTEST_DISABLE_PLUGIN_AUTOLOAD=1` (exit 0) on `a8aa56f`; imported gates `False` from this workspace; `--live` exit 3 for both `m6_gait_replay.py` and `m6_live_playback`; 0 leftover processes |
| `colcon_test_result.txt` | `colcon test-result --all`: **1894 tests, 0 errors, 0 failures, 0 skipped** |

- **The merge** brought in `108561b`, the third M6.1 cycle record. It is docs and evidence only,
  with no `src/` change, so the count equals the previous M5.5 suite (1894).
- **Conflict:** `README.md`. The resolution keeps the updated M6.1 row and the M5.5 rows.

`SHA256SUMS` covers the files in this folder.
