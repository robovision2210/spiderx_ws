# S4d-E preparation: fresh disabled build and full suite, 2026-11-11

**Result: PASS.** This is batch step 6: one fresh **disabled** build and full suite, run because the S4d-E
preparation changed executable inputs. All three gates stayed `False`. No enabling patch or branch existed.

This file is an evidence index. The S4d-E shadow rehearsal that used this build is recorded separately
(`../s4d_e_shadow_20261111T104429Z/`).

## Validated executable
- `87198b951506b35ee4d95f3c124c0327a9647ae6` on `claude/spiderx-s4d-e-preparation`.
  - The worktree was clean and equal to `origin`.
  - There was no `local/*` branch.
- Changes under `src/` and `scripts/` since the S4d-M tested executable `8f99bcc` (other commits changed
  documentation and evidence only):
  - the new S4d-E driver, scorer and their tests (`scripts/s4/`, run directly; not collected by colcon);
  - `local/m55-s4d-e-*` support in `scripts/m61_restore_and_verify_disabled.sh`, with 10 new collected tests
    in `test_m61_restore_script.py`.
- No runtime node, teleop, gait, plant, gain, historical driver or scorer, or the S4d-M scorer changed.

## Procedure (`tools/`, `README_run.txt`)
- The tools are copies of `../s4d_fresh_disabled_20261110T071506Z/tools/`, verified against that record's
  manifest and unchanged (`tools_SHA256SUMS.txt`).
- New bases `/root/spiderx_fresh_disabled_20261111T104338Z/{build,install,log}`. They did not exist before.
  Nothing was deleted and no package was installed.
- The established clean shell (`env -i`): the toolchain `/opt/mm/activate.sh`, then the verified r2 overlay
  `local_setup.bash`; `PYTEST_DISABLE_PLUGIN_AUTOLOAD=1`; explicit result paths.

## Results (`fresh/`)
- **Build:** exit 0; 8 packages finished (`build.log`).
- **Full colcon suite:** exit 0. **1993 tests, 0 errors, 0 failures, 0 skipped**
  (`build_and_test_summary.txt`, `colcon_test_result.txt`).
  - That is 1943 xunit test cases plus 50 CTest entries.
  - The 10 more than the S4d-M baseline's 1983 are the new restore tests.
- **Collection check: PASS** (`collection_check.txt`). `spiderx_controller` has 50 registered CTest targets
  and 50 results, with none missing, not passed or stale.
- **Direct tests**, run once (`direct_tests.txt`):
  - `src/spiderx_scripts/test`: 2 passed;
  - `scripts/s2`: 48 OK, with the 3 pre-existing `S2_BAG` skips;
  - `scripts/s3`: 86 OK;
  - `scripts/s4`: 252 OK. The baseline had 206; the 46 new tests are the 18 E driver and 28 E scorer tests.
- No full-suite repeat was run.

## Installations (`install_A*`, `install_B*`)
Each was checked in its own fresh environment, with the other installation absent from it:
- **A**, the fresh run installation `/root/spiderx_fresh_disabled_20261111T104338Z/install`: **PASS**.
- **B**, the workspace installation `/home/user/spiderx_ws/install`: **PASS**.

For each:
- the package prefix is that installation;
- every checked module imports from that installation, through symlinks into this workspace's `src/`;
- all three gates are `False`;
- only after the gates were confirmed `False`, both M6 `--live` tools refused with exit 3 (HARD-DISABLED);
- the library has 54 goals, fingerprints `62f8a100…`, and `library.json` equals the disabled baseline
  `917fc4d7…`;
- the loaded config equals `src/`.

## Limits
- Cloud container only. This is a software validation of a disabled build. It shows no motion, hardware
  readiness or S4d-E acceptance.
