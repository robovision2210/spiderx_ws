# E11 validation: `scripts/m61_restore_and_verify_disabled.sh` at `1732c32` (Cloud)

**What this validates.** The one-cycle cleanup (plan §4 G, E11) must prove the disabled state
after success, failure or interruption. Switching branches is not proof. The script is run here
in two cases:
- **A**, against a running simulation (no motion);
- **B**, against a deliberately enabled build (tests only, no simulator).

Nothing was commanded and no goal was sent. On every branch the gates stayed `False`.

| Path | Content |
|---|---|
| `tools/run_restore_positive.sh` | Case A: start and wait with the frozen stage scripts, then run the restore script **while the launch is still running** |
| `tools/` (other files) | The frozen phase-1 harness. The 14 files are byte-identical to `../observation_7f4c30f/tools/` (checked against `../closeout/frozen_harness.json`) |
| `revision.txt` | `HEAD` = `claude/stoic-shannon-ur2mes` = `1732c32`; untracked documentation folders only |
| `run_07/`, `run_07.out` | Case A: launch, steps and controller state |
| `positive/` | Case A: the script's report, its rebuild log and the work-tree state |
| `negative/` | Case B: the script's report and stdout; the uncommitted `src/` diff it saved; `enabled_worktree_src_vs_base.diff` (the patch as applied, 2 files, SHA-256 `8604d06b…f3`); the tail of the worktree build |
| `enabled_mode/` | The whole `spiderx_controller` suite in the enabled worktree (tests only) |
| `disabled_mode/` | The committed (disabled) tree at `1732c32`: clean build and full `colcon test`, **1559 tests, 0 errors, 0 failures, 0 skipped**. Also a first attempt without `PYTEST_DISABLE_PLUGIN_AUTOLOAD=1`, in which no test ran (plugin incompatibility, not a test result) |

## Case A: a running launch (`run_07`)

The command was `run_restore_positive.sh run_07 positive '<build cmd>'`.

| Step | Result |
|---|---|
| 1. Stop | `SIGINT sent to process group 8699` (the launch), `SIGTERM sent to Xvfb 8694`, then **no simulation process** |
| 2. Preserve | No enabling ref in this case; no uncommitted `src/` |
| 3. Restore | `HEAD 1732c32 (claude/stoic-shannon-ur2mes)`; `src/` equals the base: no diff, no untracked file |
| 4. Rebuild | `rm -rf build install && colcon build --symlink-install`, 8 packages |
| 5a | `M61_LIVE_DISPATCH_ENABLED = False` and `LIVE_DISPATCH_ENABLED = False`, imported from `/home/user/spiderx_ws/src/...` (inside the workspace) |
| 5b | `ros2 pkg prefix spiderx_controller` = `/home/user/spiderx_ws/install/spiderx_controller` |
| 5c | `--live`: `REFUSED: … HARD-DISABLED …`, **exit 3** |
| 5d | The calling shell cannot import `spiderx_controller` (not sourced) |
| Verdict | **`DISABLED AND VERIFIED`, exit 0** |

`ps` afterwards showed no simulator, bridge, Xvfb or daemon process.

## Case B: an enabled build

**Setup.**
- A temporary detached worktree at `1732c32`.
- `git apply` of `docs/patches/m61_enable_one_cycle.UNAPPLIED.patch`: 2 files, 2 lines.
- `colcon build` in that worktree.

**Run.** The script was run with `--ws <worktree> --base 1732c32 --verify-only`.

| Step | Result |
|---|---|
| 2 | Uncommitted `src/` changes saved (`uncommitted_src_….diff`); FAIL, because they are discarded only on request |
| 3 | FAIL: `src/` differs from the base in the 2 patched files |
| 5a | **`M61_LIVE_DISPATCH_ENABLED = True`** imported: FAIL |
| 5c | **`--live NOT run`**: the gate was not verified disabled |
| Verdict | **`NOT VERIFIED`, exit 1** |

There is no `log/m61_run` directory in the worktree, so the live path never started. After the
tests the worktree was removed (`git worktree remove`), and `git worktree list` shows only the
main checkout.

**Enabled-mode suite in that worktree** (tests only; isolated domains 150–199, localhost):
**1517 passed, 0 failed** (7 min 49 s; `enabled_mode/enabled_mode_full_pytest_summary.txt`). The +60 against the earlier 1457 are the new tests of this round: 30 link check, 8 restore script, 22 in the fixed-base, gait-replay and observer files.

## Redaction and integrity

- **Redaction.** The temporary worktree and the setuptools-59 build path live in the
  session-specific scratchpad. That prefix is replaced by `<scratchpad>` in `run_07.out`,
  `positive/restore_verify_*.txt` and both `negative/` reports. The unredacted originals are kept
  outside the repository.
- **Shutdown record.** `run_07` has no `shutdown.txt`: the restore script, not the frozen stop
  script, stopped the launch, and its report records that step.
- **Manifest.** `SHA256SUMS` covers every file in this folder.
