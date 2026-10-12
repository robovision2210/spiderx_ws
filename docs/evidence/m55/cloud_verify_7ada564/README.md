# M5.5 after merging the M6.1 second-cycle record and tracking-reference fix (`7ada564`)

**Offline (Cloud VM, RoboStack ROS 2 Humble).** No simulator was used and no goal was sent. All
gates are `False`: M5.5, M6.1 and M6.0-D.

| File | Content |
|---|---|
| `build_and_test_summary.txt` | Clean build (`colcon build --symlink-install`, 8 packages, in a fresh worktree with `build/` and `install/` removed first) and `colcon test` with `PYTEST_DISABLE_PLUGIN_AUTOLOAD=1`. Both exit 0, on `7ada564`. The imported gate modules are `False` |
| `colcon_test_result.txt` | `colcon test-result --all`: **1894 tests, 0 errors, 0 failures, 0 skipped** |

- **What the merge brought in.** It came from `claude/stoic-shannon-ur2mes` at `d748891`:
  - `24782f6`: the second authorized M6.1 cycle record. It stays FAIL: 1 goal, 1 cancel, E4 and
    E6 FAIL.
  - `b0d8e75`: the M6.1 in-flight tracking reference, timed from goal acceptance
    (`m61_tracking`). A `time_from_start = 0` fallback is recorded, no threshold changed, and
    there are new tests in `test_m61_tracking.py` and two mock scenarios.
  - `d748891`: Cloud report §14, the no-goal rehearsal evidence, and the plan for a new
    authorization.
- **Conflicts.** Both were additive:
  - `README.md`: the updated M6.1 row, with the M5.5 rows kept;
  - the testing guide: the M5.5 numbering kept; the tracking-path probe is row 38 here.

  The CMake test list merged automatically.
- **Count.** The previous M5.5 full suite, at `00e9584`, was 1842. The +52 equals the base-branch
  delta from 1626 to 1678: 49 `test_m61_tracking` cases, 2 mock scenarios and 1 CTest entry.

`SHA256SUMS` covers the files in this folder.
