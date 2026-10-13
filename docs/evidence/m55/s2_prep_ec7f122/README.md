# S2 preparation at `ec7f122` (Cloud, 2026-10-13): no goal sent, all gates False

The owner approved the S2 acceptance criteria and these preparation steps; the S2 run itself is
not approved. Design: [`S2_FIXED_BASE_CRAWL_DESIGN.md`](../../../S2_FIXED_BASE_CRAWL_DESIGN.md).

## Build and suite (`verify/`)

- Clean build and full `colcon test` (`PYTEST_DISABLE_PLUGIN_AUTOLOAD=1`) at `ec7f122`:
  **1931 tests, 0 errors, 0 failures, 0 skipped**.
  - That is 1922 + 9: 6 restore-script cases and 3 M5.5 gate cases (the enabling-patch check,
    and the enabler-scan teeth in both modes).
- **Gates:** all three are `False`, imported in a fresh environment (`restore_verify_only_after_rehearsal.txt`).
- **`--live`:** exit 3 and HARD-DISABLED, for both `m6_gait_replay.py` and `m6_live_playback`.

## Restore script (`scripts/m61_restore_and_verify_disabled.sh`)

- **(a)** It imports `m55_contract` too: all three gates must be `False`, from this workspace.
- **(c)** Both M6 `--live` tools must exit 3.
- **Leftovers:** a remaining `m55_locomotion_node` or `m55_teleop_keyboard` fails the check.
- **Deletion:** `local/m55-s2-*` enabling branches may be deleted.
- **Tests:** 14 hermetic cases (8 before).
- **Mutants** (`verify/restore_mutants.txt`): 4 of 4 caught.
  - the M55 gate not imported;
  - `m6_live_playback` not checked;
  - the S2 branch pattern removed;
  - the M55 node not treated as a leftover.
- **Real run:** `--verify-only` after the rehearsal ended DISABLED AND VERIFIED.

## Enabling patch (not applied)

- **File:** `docs/patches/m55_enable_s2.UNAPPLIED.patch`, 2 lines in 2 files (the gate and its pin).
  `git apply --check` passes; nothing was applied.
- **SHA-256:** `d1dacac5d9b2bc0b63220c3583526d45f5edacb657394b508710978972b20236`.
- **Expected test delta on the enabling branch: none.**
  - The M5.5 gate tests are now mode-explicit, as for M6.1.
  - Before that, 12 tests assumed the committed `False`.
- **Check without the patch** (`verify/m55_tests_both_gate_modes.txt`): gate and pin set `True`
  in memory by `tools_gate_modes/s2_enabled_sim.py`, no file patched.
  - Result: 240 of the 241 M5.5 tests pass.
  - The one failure is expected: the patch's reverse-apply check needs the patched files.
  - With the committed `False`, all 240 tests that were run then passed.

## Shadow rehearsal on the welded fixed base (S2 step A2; also the S0 re-run)

- **Harness** (`rehearsal_*/tools/`):
  - the frozen M6.1-A tools (`frozen_m61a_tools.sha256`), the M6.1-A read-only preflight;
  - the M5.5 node in SHADOW;
  - the status-driven `s2_driver.py`, which sends teleop keystrokes only;
  - `analyze_rehearsal.py`.
- **Bags** stay in `~/spiderx_evidence/`; attempt 5's `bag_metadata.yaml` is committed.
  `driver.jsonl` is gzipped; run `gunzip -k` before re-running the analysis.

| Attempt | Result | Cause (harness, not node) and fix |
|---|---|---|
| 1 | `arm` refused, `command_owner_conflict` | The bag subscribed to `_action/status`; the node counts those subscribers as action clients. Correct refusal. Fix: never record that topic (design §6) |
| 2 | 57 goals, then the driver stopped | It expected re-arm at boundary 1 after the lease. In shadow nothing moved, so arm resolves to neutral (boundary 0). Fix: mode-aware (58 goals in shadow, 59 in S2) |
| 3 | `joint_states_stale` fault at goal 29 | A 0.8 s CPU stall of the node process (4 cores, Gazebo GUI ≈ 150 % CPU). The recorder saw `/joint_states` every ≤ 0.081 s (`run/stall_analysis.txt`). Fix: load, not thresholds (below) |
| 4 | controllers not active | The known Gazebo startup race (M6.1-A, 1 in 5). Fix: the harness aborts on a failed launch or preflight |
| **5** | **PASS**, driver exit 0 | — |

**Attempt 5** (`rehearsal_20261013T094157Z/run/analysis.json`):

| Check | Result |
|---|---|
| Goals planned (shadow) | **58** = 54 cycle + stop 1 + home 1 + lease 1 + estop 1. S2: 59, adding the home after the lease re-arm. 1 cancel |
| Nothing sent | 0 `joint_trajectory` messages; 0 action feedback; node action clients: none; node publishers: status only; joints moved 1.4e-19 rad |
| Rate rule on the real graph | 0 rejections in 2537 commands |
| Check 1.7 (space) | 1.76 s sim ≤ 2.7 (goal 1.7 + 1.0); 3.03 s wall ≤ 60: PASS |
| Check 1.9 (teleop SIGKILL, lease) | 1.73 s sim ≤ 2.7; 3.03 s wall ≤ 120: PASS; kill → STOPPING 0.48 s wall (1.13: ≤ 0.55) |
| Weld | z 0.125 m throughout, tilt 0 |
| Monitors | at the node: max `/joint_states` age 0.074 s, pose 0.05 s, graph 1.05 s; RTF 0.59 |

## Wall-time bounds that RTF 0.20 could trip during S2

| Bound | Setting | Can it trip a correct S2 run? | Measured (A2, RTF 0.59) | Action |
|---|---|---|---|---|
| Command lease | 0.5 s wall | No: the 10 Hz wall heartbeat is independent of RTF | lease expired only when planned | keep |
| Teleop hold window | 0.8 s wall | No: key repeat | — | keep |
| Rate rule | > 20 in 1 s, > 4 in 0.1 s | No (S0b) | 0 rejections | keep |
| Goal response timeout | 2.0 s wall | No: the action server answers on its executor thread; RTF slows only the simulation | not exercisable in shadow | keep; S2 stop condition |
| Result watchdog | goal + 2 s **sim**; wall backstop at RTF 0.1 | No at RTF ≥ 0.1 (at 0.2 it uses half the backstop) | — | keep |
| Post-result sample | 2.0 s wall | No: one sample is needed; at 100 Hz sim that is 20 Hz wall at RTF 0.2 | not in shadow | keep |
| `/joint_states` stale | 0.5 s wall | Not through RTF (20 Hz wall at 0.2). **Yes through host CPU starvation** (attempt 3) | max 0.074 s at the node | **keep** (safety); host load reduced (GUI nice 19, node nice −5, no `/clock` in the bag); residual: a safe fault |
| Sim stall | 5.0 s wall | No | — | keep |
| Body pose stale | 1.0 s wall | Not through RTF; a 0.68 s bridge stall was seen in attempt 3 | max 0.05 s | keep (safety) |
| Graph snapshot stale | 3.0 s wall | No (CPU only) | max 1.05 s | keep |
| Check 1.1 node up | 150 s wall | No (start-up is CPU-bound) | 86 s | keep |
| Check 1.7 stop → READY | was 5 s wall | **Yes** | 1.76 s sim | changed in S0b: sim, 60 s wall backstop |
| Check 1.9 teleop close → DISARMED | was 6 s wall | **Yes** (a 4.7 s phase ≈ 24 s wall) | 1.73 s sim | **changed**: goals to finish + 1 s sim, 120 s wall backstop |
| Check 1.13 detection | 0.95 / 0.55 s wall | No: wall by design (hold window, lease, tick) | 0.48 s | keep |
| Phase 4: release → READY, close → DISARMED | ≤ 4.7 s / 6 s | Yes, at S4 | — | **changed** to the 1.7/1.9 sim rules |
| M6.1-A preflight sim progress | ≥ 0.1 s sim in 1 s wall | Not at RTF ≥ 0.1 (2× margin at 0.2) | READY | keep |
| S2 driver wall caps | sim duration / 0.1 + 60 s | No at RTF ≥ 0.1 | — | design |
