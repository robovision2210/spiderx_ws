# M5.5 after merging the M6.1 readiness fix (`00e9584`)

**Offline (Cloud VM, RoboStack ROS 2 Humble).** No simulator was used and no goal was sent. All
gates are `False`: M5.5, M6.1 and M6.0-D.

| File | Content |
|---|---|
| `build_and_test_summary.txt` | Clean build (`colcon build --symlink-install`, 8 packages, in a fresh worktree with no previous `build/`, `install/` or `log/`) and `colcon test` with `PYTEST_DISABLE_PLUGIN_AUTOLOAD=1`. Both exit 0, on `00e9584`. The imported gate modules are `False` |
| `colcon_test_result.txt` | `colcon test-result --all`: **1842 tests, 0 errors, 0 failures, 0 skipped** |

- **What the merge brought in.** It came from `claude/stoic-shannon-ur2mes`:
  - `a1a7d60`: the refused one-cycle attempt (0 goals), evidence and docs;
  - `61f3ffc`: bounded action-server discovery before readiness, the complete readiness record,
    and the read-only rehearsal;
  - `02d284c`: a mode-explicit gate test;
  - `60d217d`, `dcac464`: Cloud report §12 and the no-motion rehearsal evidence.
- **Conflicts.** All additive, all resolved by keeping both sides:
  - `CMakeLists.txt`: scripts and tests;
  - `README.md` and `STATUS.md`: the updated M6.1 rows plus the M5.5 rows;
  - the testing guide: the rehearsal is row 37 here.
- **Count.** The previous M5.5 full suite at `4c2d2e0` was 1775. The +67 equals the base branch
  delta, from 1559 to 1626.

`SHA256SUMS` covers the files in this folder.
