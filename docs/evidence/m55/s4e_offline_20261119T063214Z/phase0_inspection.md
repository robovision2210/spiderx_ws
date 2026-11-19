# S4e Phase 0: inspection record (2026-11-19, Cloud)

## Repository state (06:31:33Z)
- **Fetch.** `origin/main` = `e7ffd207dbff6a871c6c522713ced1bddf4df0b7`, unchanged from the last
  verified value.
- **The existing checkout** `/home/user/spiderx_ws`: branch `claude/spiderx-s4d-e-preparation`
  at `1e11464`, worktree clean, no stash. It was not modified.
- **Remote branches.** The merged `claude/spiderx-*` stack branches, `claude/stoic-shannon-ur2mes`,
  three `copilot/*` branches and `main`. No S4e branch.
- **Open PRs:** only robovision2210/spiderx_ws#1 and robovision2210/spiderx_ws#2 (old Copilot
  drafts, unrelated). No S4e PR. The nine-PR stack is merged (robovision2210/spiderx_ws#26 →
  … → robovision2210/spiderx_ws#25, 2026-11-12).
- **Processes:** no Gazebo, ROS, driver, colcon or pytest process.
- **Existing S4e material on `main`** (reading only):
  - S4 plan: S4e is "optional, owner's PC", checks 1.11 (key repeat) and 1.13 (crash path).
  - S4d plan: R8 and OD9 keep human keyboard work separate.
  - Owner decision 4 of the S4 plan (does S4 need a human at a keyboard?) is open.
  - `test_m61_restore_script.py` refuses `local/m55-s4e-<UTC>` as a look-alike (hence D9).
- **Work clone.** A new clone at `/root/spiderx_s4e_20261119T063214Z/spiderx_ws`, branch
  `spiderx/s4e-human-keyboard` from `e7ffd20`, so no other checkout is touched.

## Re-read
- `docs/S4_KEYBOARD_TELEOP_PLAN.md`, `docs/S4D_ACCEPTANCE_PLAN.md` (§2, §10, §15–17) and
  `docs/S4D_E_CONTRACT.md`.
- `docs/M55_KEYBOARD_WALKING.md` §8 (teleop), §10 phase 1 (checks 1.7, 1.9, 1.11, 1.13) and
  §16.1–16.2 (stopping timeline, terminal key semantics).
- `m55_teleop.py` in full: keys, the hold window, the repeat-gap note, the quit sequence, cbreak
  input.
- `m55_locomotion.py`: states, release (`intent_stop`), lease (`lease_expired`), stop latch,
  estop, home, FAULTED refusals, status fields.
- The S2/S3/S4c/S4d-E drivers (the pty and key-repeat chain, the stop handling, the fresh-sample
  helpers), the S4d-E scorer and its bench tests.
- The restore script and its S4d-E tests.
- The S4d-E live harness tools, in its evidence.
- The config `m55_locomotion.yaml` (`ab0dccd4…`).

## VERIFIED versus assumed
| Item | Status | Source |
|---|---|---|
| `main` = `e7ffd20`; its tree = the approved tip's tree; merge order; `87198b9` an ancestor | VERIFIED (Cloud) | post-merge validation report, sha256 `aaef40bf…` (outside the repository) |
| Fresh disabled build; 1993 collected tests, 0 failures or skips | VERIFIED (Cloud) | same |
| Direct tests: 2 / 48 (3 `S2_BAG` skips) / 86 / 252 | VERIFIED (Cloud) | same (the final report had results for all of them; none UNVERIFIED) |
| Installed imports, gates `False`, both M6 `--live` refusals, library `917fc4d7…`, fingerprints `62f8a100…`, config `ab0dccd4…`, links, clean state | VERIFIED (Cloud) | same |
| Teleop hold window 0.8 s, heartbeat 10 Hz, lease 0.5 s, tick 20 Hz; no key-up from a terminal | VERIFIED by reading the code and config | `m55_teleop.py`, `m55_locomotion.yaml` |
| Release to STOPPING 0.8–0.95 s; teleop loss ≤ 0.55 s | Documented and tested in M5.5 §16.1 (in-memory); S4b measured 0.889 s, 0.818 s and 0.466 s with automated input | M5.5 §16.1, S4b evidence |
| Linux `CLOCK_MONOTONIC` is system-wide, so the observer and the node share a clock on one host | Assumed (standard Linux behaviour; S4 drivers relied on it) | S4D_E_CONTRACT §3 |
| The owner's terminal repeat settings and PC RTF | NOT VERIFIED (M5.5 quotes RTF 0.21–0.25 from an earlier session) | — |
| Home-PC validation | NOT DONE (pending) | — |
| Any human keyboard operation | NOT DONE | — |

## Unclear points, resolved as owner decisions rather than guessed
- **The level:** D1, level 0 recommended.
- **Where the person operates:** D2, the owner's PC; this session cannot carry keystrokes.
- **The crash-path close:** D4, excluded.
- **The automatic emergency response:** D8, the estop service, which is not a key.
- **The branch pattern:** D9, `local/m55-s4e-h-*`.

All are listed in `docs/S4E_CONTRACT.md` §10 with recommended defaults.
