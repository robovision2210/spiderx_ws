# S4d-M batch step 2: one fresh disabled build and full suite, 2026-11-10 (cloud)

**Result: PASS.** This is the disabled validation that preceded the S4d-M shadow rehearsal
(`../s4d_shadow_20261110T072920Z/`). It is offline: no launch, no goal, no enabling. This file is an
evidence index; the evidence is uncommitted, for review.

- **Tested executable SHA:** `8f99bcc4ab27294265da91a31ce03999423acc7f`, branch `claude/spiderx-s4d-preparation`, clean.
- **New colcon bases**, created for this run and previously nonexistent:
  `/root/spiderx_fresh_disabled_20261110T071506Z/{build,install,log}`. Nothing was deleted or overwritten.
- **Procedure** (`tools/fresh_build_test.sh`, from the committed S4c level-1 follow-up tool; `tools_diff.txt`):
  - fresh `env -i` shells: the toolchain, then the r2 overlay `local_setup`;
  - `/opt/mm/st59site` on `PYTHONPATH` for the build only;
  - `colcon build --symlink-install`;
  - `colcon test` with `PYTEST_DISABLE_PLUGIN_AUTOLOAD=1`, and `test-result`, both with explicit `--build-base`,
    `--install-base` and `--test-result-base` on the new bases.

## Results
| Item | Result |
|---|---|
| Build | exit 0, 8 packages (`fresh/build.log`) |
| Discovery | `spiderx_controller`: 50 registered CTest targets = 50 entries, 1933 xunit cases; no missing, not-passed or stale result (`fresh/collection_check.txt`) |
| Suite | **1983 tests, 0 errors, 0 failures, 0 skipped**. The previous follow-up had 1972; the +11 are the restore-script tests of `6a780ec` |
| Direct tests | `src/spiderx_scripts/test` 2 passed; `scripts/s2` 48 OK (3 skipped); `scripts/s3` 86 OK; `scripts/s4` 206 OK (`fresh/direct_tests.txt`) |
| Installation A (fresh) | PASS (`install_A/`) |
| Installation B (existing `/home/user/spiderx_ws/install`) | PASS (`install_B/`) |
| Protected runtime sources | no non-test `src/` or `third_party/` change since `0cb2ccc`; 169 runtime symlinks = their HEAD blobs (`runtime_source_identity.txt`) |

- **Skips.** The colcon suite has none. The 3 `scripts/s2` skips are `OriginalBagRegression`
  (`skipUnless S2_BAG`, unchanged since `e1ed37a`). They are expected and existing, not new.
  `scripts/s2` was not run in the previous follow-up.
- **Installation checks.** Each check covers:
  - module paths: imported from that installation;
  - the three gates: all `False`;
  - the library: 54 goals, `62f8a100…`;
  - `library.json`: equal to the disabled baseline, `917fc4d7…`;
  - the config: equal to `src/`.
  Only after the gates were `False`: both M6 `--live` tools refused, exit 3, `HARD-DISABLED`.
- **Footprint.** The fresh bases take 11.2 MB.
