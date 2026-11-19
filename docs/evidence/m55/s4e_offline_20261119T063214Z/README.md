# S4e offline build-out, 2026-11-19 (Cloud): evidence index

**Offline preparation only.** Nothing was launched. There was no enabling, goal, motion, hardware
or home-PC work. No person has operated SpiderX through these tools, and **human keyboard
operation is not validated**. Home-PC validation is pending.

The contract is [`docs/S4E_CONTRACT.md`](../../../S4E_CONTRACT.md); the operator steps and the
draft authorizations are in [`docs/S4E_OPERATOR_PACKAGE.md`](../../../S4E_OPERATOR_PACKAGE.md).

## Baseline
- `main` = `e7ffd207dbff6a871c6c522713ced1bddf4df0b7`, unchanged (the nine-PR stack, merged
  2026-11-12).
- The post-merge disabled validation of that commit had final results for every item: no item
  is UNVERIFIED. They are in [`phase0_inspection.md`](phase0_inspection.md).
- **Work clone:** `/root/spiderx_s4e_20261119T063214Z/spiderx_ws`, branch
  `spiderx/s4e-human-keyboard` from `e7ffd20`. The existing checkout `/home/user/spiderx_ws` was
  not modified.

## Order (the contract came first)
1. **The contract was frozen** at 06:48:39Z, before any S4e tool, test or harness file existed:
   sha256 `07339a7e…`, git blob `cb443954…`
   ([`freeze/contract_freeze.txt`](freeze/contract_freeze.txt)).
   - **The commit was not made.** Both a commit-and-push and a local-only commit were denied by
     this session's permission classifier. Nothing was committed or pushed in this batch.
2. **The terminal preflight, the observer, the scorer, the restore support and the harness
   template** were written after that, each with its tests.
3. **The scorer** was recorded before its first test (sha256 `df9a30a3…`). Its first run exposed
   two defects, fixed with no criterion changed:
   - H7 compared the result with a string, where the node logs GoalStatus codes;
   - H8 needed the whole plan to parse before finding `q`.

   The validated scorer is sha256 `653fd8e0…`
   ([`freeze/scorer_freeze.txt`](freeze/scorer_freeze.txt),
   [`freeze/scorer_defect_fixes.diff`](freeze/scorer_defect_fixes.diff)).

## Validation: one fresh disabled build and suite (`validation/`)
- **Validated content.** `e7ffd20` plus the working-tree files listed with their blob and sha256
  hashes in [`validation/validated_content_manifest.txt`](validation/validated_content_manifest.txt).
  - All 19 were byte-identical at the end of the run.
  - The docs written during the run (the walkthrough, exercises, operator package, README,
    STATUS and this folder) contain no executable input.
- **Toolchain:** setup script `57cc33f8…`, gcc 15.3.0, Python 3.11.17, ROS 2 Humble. The r2
  overlay patches and library hashes match, and the config is `ab0dccd4…`: **PASS**
  ([`validation/toolchain_checks.txt`](validation/toolchain_checks.txt)).
- **Build:** new bases `/root/spiderx_s4e_20261119T063214Z/bases/{build,install,log}`, which did
  not exist before; no old installation in the environment; exit 0, 8 packages.
- **Collected suite: 2005 tests** (1955 xunit + 50 CTest), 0 errors, 0 failures, 0 skips.
  - That is 1993 at `e7ffd20` plus the 12 new restore tests.
  - Collection check PASS. Discovery: 50 of 50 controller test files registered, sources in the
    clone, results fresh. PASS.
- **Direct tests, run once, with `SPIDERX_REPO` set to the work clone:**

  | Suite | Result |
  |---|---|
  | `spiderx_scripts` | 2 passed |
  | `scripts/s2` | 48 OK; the 3 known `S2_BAG` skips |
  | `scripts/s3` | 86 OK |
  | `scripts/s4` | 367 OK: the 252 earlier tests, plus 115 new (preflight 27, observer 31, scorer 40, harness template 17) |

  No other skip and no failure.
- **Installed imports, gates, refusals, identity:**
  - The package prefix is the fresh installation; all six checked modules resolve into the
    clone's `src`.
  - All three gates are `False`. Only after that check did both M6 `--live` tools refuse
    (exit 3, HARD-DISABLED).
  - Library: 54 goals, fingerprints `62f8a100…`, `library.json` `917fc4d7…` = the disabled
    baseline. The config matches `src` (`ab0dccd4…`).
  - Verdict: PASS ([`validation/install_check/`](validation/install_check/)).
- **flake8** (7.4.1, `--max-line-length 99`) on the seven new Python files: clean. The appended
  lines of the restore tests: clean; their file's line 92 is a pre-existing 100-column line.
  `bash -n` on the template scripts: clean ([`validation/flake8.txt`](validation/flake8.txt)).
- **Relative links:** [`validation/link_check.txt`](validation/link_check.txt).
- **Final state:** [`validation/final_state.txt`](validation/final_state.txt).
  - No relevant process was left.
  - No file changed in the post-merge, old fresh-disabled or enabled bases, the r2 overlay or
    `/opt/mm`.
  - `/home/user/spiderx_ws` is unchanged (branch, HEAD `1e11464`, worktree, stash). Its
    `.git/config` was rewritten by the session's worker restart, with its normal content
    (noted there).

## Tests, by tool (all offline)
| Tool | Tests | Kind |
|---|---|---|
| `scripts/s4/s4e_terminal_preflight.py` | 27 | pure analysis; standalone imports; teleop consistency; **scripted** pseudo-terminal runs, labelled as such (normal, repeated, idle, cap, Ctrl+C, SIGTERM/SIGHUP, error); refusals |
| `scripts/s4/s4e_observer.py` | 31 | static no-key-path checks; relay; monitor stops; **scripted** sessions with a fake teleop; CLI refusals |
| `scripts/s4/score_s4e.py` | 40 | in-memory bench sessions (real node and teleop logic), shadow and live, and mutations: contradictions FAIL, gaps are UNMEASURED |
| `scripts/s4/s4e_harness_template/` | 17 | stubs only: refusals (mode, installation, gates, ref, time, disk), flows, watchdog, interruption, owned-only cleanup with decoys, restore before pid files |
| restore pattern `local/m55-s4e-h-*` | 12 | collected; the earlier look-alike tests are unchanged and pass |

## Findings from the tests (fixed before validation)
- **Inherited SIGINT.** Background jobs of a non-interactive shell inherit SIGINT *ignored*. The
  template starts the launch and the recorder with SIGINT at its default.
- **Zombies.** Cleanup treated zombies as running. The template's cleanup is now zombie-aware.
- **A spawn race.** The observer's start-of-session pty scan could see the child before
  `setsid`. `spawn` now waits for the exec.
- **The observer's CLI** exited 0 on a refusal (`sys.exit(main())` was missing).

## Not shown
Human keyboard operation, a shadow or live S4e session, hardware, navigation, autonomous or
unrestricted walking, robustness, levels 1–2 under human input, and full S4 acceptance.

Earlier qualifications are unchanged:
- S2 and S3 were accepted retrospectively.
- S4a–S4d used automated pseudo-terminal input, not human operation.
- S4d-M's simulator server was killed by SIGKILL during shutdown.
- S4d-E cancelled before measurable motion and is not a moving-body stop.

## Integrity
- [`SHA256SUMS`](SHA256SUMS) covers every file here.
- [`secret_scan.txt`](secret_scan.txt): the masked scan of the changed files and of this folder.
