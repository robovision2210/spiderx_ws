# M5.5 after merging the M6.1-A one-cycle prerequisites (`4c2d2e0`)

**Offline (Cloud VM, RoboStack ROS 2 Humble).** No simulator was used and no goal was sent. All
gates are `False`: M5.5, M6.1 and M6.0-D.

| File | Content |
|---|---|
| `build_and_test_summary.txt` | Clean build (`colcon build --symlink-install`, 8 packages, in a fresh worktree with no previous `build/` or `install/`) and `colcon test` with `PYTEST_DISABLE_PLUGIN_AUTOLOAD=1` (the `launch_testing` plugin of this image is incompatible with its pytest). Both exit 0, on `4c2d2e0` |
| `colcon_test_result.txt` | `colcon test-result --all`: **1775 tests, 0 errors, 0 failures, 0 skipped** |

- **What the merge brought in.** It came from `claude/stoic-shannon-ur2mes`:
  - `927acb2`: the frozen link-pose check (E8 provenance and E9);
  - `1732c32`: the streams checked at the send (E5) and the restore-and-verify script (E11);
  - `8025668`: docs and evidence.
- **Conflicts.** Two, both additive: test registrations in `CMakeLists.txt` and rows of the
  testing guide. Both sides were kept. The two M6.1-A guide rows are numbered 35 and 36 here.
- **Count.** The previous M5.5 full suite at `c04bba2` was 1713. The +62 equals the M6.1-A delta,
  from 1497 to 1559: 60 tests and 2 new test files.

`SHA256SUMS` covers the files in this folder.
