# S4c level-1 live run: fresh-build restoration follow-up, 2026-11-02 (offline)

**Result: PASS.** A fresh DISABLED build and the full suite, in new colcon bases, pass. Both the new installation and the normal workspace installation import all three gates `False` from their own paths, carry the disabled-baseline library, and refuse both M6 `--live` tools.

**What this is and is not.**
- **It verifies the disabled state after the run.** The workspace is at `0d84ebd`; its robotics source equals the run's disabled baseline `ea69d3f` and `069385c`.
- **It does not retroactively make the enabled runtime build clean.** The live run ([evidence](../s4c_l1_live_20261102T182303Z/README.md), committed as `0d84ebd`) keeps its documented deviation: its enabled build and its restore rebuild were incremental.
- **Offline only.** There was no simulator, launch, graph preflight, goal, enabling patch, level 2 or S4d. Nothing was deleted, and the existing `build/`, `install/` and `log/` were not modified (`state_after.txt`).

## Fresh disabled build and full suite (`fresh/`, `tools/fresh_build_test.sh`)
- **Bases.** New, uniquely named and previously nonexistent, outside the tracked tree. They are kept, not deleted, and the normal installation was not replaced: `/root/spiderx_fresh_disabled_20261102T185957Z/{build,install,log}`, 17 MB.
- **Fresh shells** (`env -i`): the documented toolchain (`/opt/mm/activate.sh`), then the verified r2 overlay `local_setup`.
  - The existing SpiderX installation was never sourced. Its path is absent from `env_build.txt` and `env_test.txt`; the tool refuses otherwise.
  - st59site was on `PYTHONPATH` for the build command only.
  - Tests ran with `PYTEST_DISABLE_PLUGIN_AUTOLOAD=1`, using the explicit build and install bases and `--test-result-base`.
- **Build:** exit 0. **Suite:** 1972 tests (1922 xunit + 50 CTest), 0 errors, failures or skips.
- **Fresh collection: PASS.** It was checked against the new build base: 50 registered CTest targets, all fresh, none stale or missing.
- **Direct tests**, which colcon does not collect, run with the fresh installation sourced: `spiderx_scripts` 2 passed, `scripts/s3` 86 OK, `scripts/s4` 146 OK.

## Both installations (`install_A/`, `install_B/`, `tools/install_check.sh`)
Each was checked in its own fresh shell: toolchain, r2 overlay, then only that installation, run from `/`.

| | A: fresh | B: normal (after the run's restore) |
|---|---|---|
| Prefix | `/root/spiderx_fresh_disabled_20261102T185957Z/install` | `/home/user/spiderx_ws/install` |
| `ros2 pkg prefix spiderx_controller` | `…/fresh…/install/spiderx_controller` | `/home/user/spiderx_ws/install/spiderx_controller` |
| Modules (6 checked) | imported from A's `site-packages`; symlinks into `src/` | imported from B's `site-packages`; symlinks into `src/` |
| Other installation in the environment | no | no |
| Gates (fresh imports) | all three `False` | all three `False` |
| Library | 54 goals, `62f8a100…` | 54 goals, `62f8a100…` |
| `library.json` | `917fc4d7…` = the run's disabled baseline | `917fc4d7…` = the run's disabled baseline |
| Loaded config | A's share → `src` config, identical | B's share → `src` config, identical |
| M6 `--live` (after the gates) | both refused, exit 3, HARD-DISABLED | both refused, exit 3, HARD-DISABLED |

The refusals use the restore script's invocation: unused domain 231, localhost only.

**Both are symlink installs that share one `src/`.** Their Python modules, scripts and config are symlinks into the same `src/` tree of this workspace at `0d84ebd`. They are **not** independent, immutable source snapshots. The checks show that each installation's own prefix, layout and generated files resolve correctly. The source they serve is the same disabled source.

**Install-tree comparison** (`compare/`, read-only, informational).
- 359 files each, none missing on either side.
- 355 are identical once each installation's own base paths are replaced by placeholders.
- The 4 that differ are the top-level `setup.{bash,sh,zsh,ps1}`. A's also chain the r2 overlay's `local_setup`, because the overlay was sourced for this build (`setup_bash_A_vs_B.diff`). B was built with the toolchain only, and the run harness sources the overlay explicitly before the workspace.

## Remaining deviation
The live run's enabled build and its restore rebuild remain incremental, as recorded in its evidence. This follow-up shows that the normal installation left by that rebuild matches a fresh build in layout and generated files, apart from the setup chain, and serves the disabled source. It does not re-run or re-qualify the enabled runtime. The fresh bases and the normal installation are both left intact.

## Tooling incident (did not execute)
While the installation-check tools were being written, one shell command I composed ended with an erroneous trailing deletion that targeted the system null device.
- The auto-mode safety classifier blocked the whole command before execution. Nothing in it ran, and nothing was deleted.
- The tool files were then written with the file-editing tool instead.
- The command is not reproduced here and was not retried.
