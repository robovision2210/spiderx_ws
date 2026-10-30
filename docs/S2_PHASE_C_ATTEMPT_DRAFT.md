# S2 Phase C: one attempt on the modified plant (DRAFT, NOT AUTHORIZED)

**Status.** A draft procedure; nothing in it has been executed.
- S2's original verdict, 2026-10-29, stays **FAIL**. S3 stays blocked.
- **Plant evidence so far (Phase B):** loading and configuration only. The gain-response law has not been measured.

## 1. Disabled baseline

**Baseline.** Branch `claude/spiderx-sim-actuator-model`, with `src/` equal to commit **`ca49db3`**. The original S2 baseline (`src/` = `ec7f122`) no longer applies.
- Any later commit qualifies if `git diff --quiet ca49db3 <commit> -- src` holds.
- Record the exact SHA in the run evidence.

**`src/` changes relative to `ec7f122`** (`git diff --stat ec7f122 ca49db3 -- src`, 4 files):

| File | Change |
|---|---|
| `src/spiderx_description/config/gz_ros2_control_plant.yaml` | New: `position_proportional_gain: 1.0` for `/gz_ros2_control` |
| `src/spiderx_description/urdf/spiderx_fortress_control.xacro` | +5/−1: a second `<parameters>` element and a comment. **Shared by every controlled-Fortress launch** |
| `src/spiderx_controller/test/test_sim_actuator_model.py` | New test |
| `src/spiderx_controller/CMakeLists.txt` | +1: test registration |

**Changes outside `src/`:**
- `third_party/gz_ros2_control_overlay/`: pin, patch and build script;
- `scripts/s2/response_law.py` and its tests;
- docs and evidence.

**Unchanged, and checked again before motion:**
- the gait templates and phase library: 54-goal digest `62f8a10075e9…e732`, re-verified 2026-10-30. P7 compares `library.json` between the disabled and enabled builds;
- `m55_locomotion.yaml` and the acceptance criteria P1–P8 (P2 tracking ≤ 0.02 rad);
- the node's tracking abort (0.05 rad) and the controller path and goal tolerances (0.05 rad);
- URDF limits, masses and friction;
- controller rate 100 Hz, physics step 0.001 s, and the weld at 0.125 m.

## 2. Overlay provenance and runtime loading (prerequisite)

**Provenance.** `OVERLAY_PROVENANCE.txt` must show:
- commit `bfc5845aa7f0250cbc384773dba5c9847d609b95`;
- patch `14238400208d0452…7bac`, equal to the committed patch;
- libraries `ef37a04f…` (`libgz_ros2_control-system.so`) and `c3db97f2…` (`libgz_hardware_plugins.so`).

The overlay source must be that commit plus exactly the patch. If any of this differs, **stop**.

**Runtime loading checks.** After the launch and **before arming**, run the Phase B checks (`docs/S2_SIM_ACTUATOR_MODEL_DESIGN.md` §5; evidence `docs/evidence/m55/actuator_phase_b_20261030T053703Z/`):
- the server process belongs to this launch's session;
- its `/proc/<pid>/maps` holds only the overlay libraries;
- their mapped-object hashes equal the provenance, and no stock copy is loaded;
- exactly one `set to: 1` log line, and `ros2 param get` gives 1.0;
- `update_rate` 100 and physics period 0.001 s.

On any failure, stop, run the restore and report BLOCKED. No goal is sent.

## 3. Pre-motion regression (disabled build of the baseline)

Required because the shared controlled-Fortress xacro changed:

1. **Full suite.** `PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 ROS_LOCALHOST_ONLY=1 colcon test`, then `colcon test-result --verbose`. Every result passes, and the count is recorded; the reference is 1929 at `f4f00b2`, plus the 7 new tests.
2. **Discovery checks.** A missing test is a failure, not a pass:
   - `test_sim_actuator_model.xunit.xml` (7 tests) and `test_config_check.xunit.xml` are among the results;
   - `spiderx_scripts`: colcon falls back to unittest and runs 0 tests, so run its 2 tests directly with pytest and record them as "+2 direct";
   - `cd scripts/s2 && python3 -m unittest test_s2_driver_live test_response_law` passes: 8 + 16, plus 3 with `S2_BAG` set;
   - the overlay gtest `test_parameter_loading` passes 9 of 9 (the `build_overlay.sh` log).
3. **Gates.** A fresh import shows all three False, from this workspace, and both M6 `--live` tools exit 3 with HARD-DISABLED.
4. **Phase library.** `library_fingerprint.py` gives `62f8a100…e732` on the disabled build, and again on the enabled build.
5. **Analyzer self-check.** `response_law.py` on the 2026-10-29 bag:
   - expected 0.1 gives SUPPORTED;
   - expected 1.0 gives CONTRADICTED;
   - the bag hash is unchanged.

## 4. Driver and enabling patch

**Driver.** `scripts/s2/s2_driver.py` at `95b00f0` (sha256 `44cd4959…8388`), unchanged. Its in-memory bench `test_s2_driver_live` must pass, 8 of 8. Its online stop conditions are unchanged:
- a status tracking error above 0.02 rad, a fault, a timeout or a driver error leads to `x` (estop: at most one cancel).

**Enabling patch.** `docs/patches/m55_enable_s2.UNAPPLIED.patch`, sha256 `d1dacac5d9b2bc0b63220c3583526d45f5edacb657394b508710978972b20236`, unchanged.
- `git apply --check` passed against the baseline on 2026-10-30. It was checked, not applied.
- It is applied only on a local branch `local/m55-s2-<UTC>` created from the baseline. That branch is never pushed and is deleted by the restore.

## 5. Environment, build and restore source order

- **Runtime shells.** A run-local `env.sh`, in the Phase B form:
  1. `source /opt/mm/activate.sh`;
  2. `source <overlay>/install/local_setup.bash`;
  3. `source /home/user/spiderx_ws/install/setup.bash`;
  4. then the frozen domain 0, `ROS_LOCALHOST_ONLY=1`, `DISPLAY=:77` and middleware settings.

  Never source the frozen `env.sh`, which lacks the overlay.
- **Builds.** The disabled build, the enabled build and the restore rebuild use the toolchain only:
  - `env -i` with `/opt/mm/activate.sh`, then `rm -rf build install && PYTHONPATH=/opt/mm/st59site colcon build --symlink-install`;
  - give the restore script the same command through `--build-cmd`.

  The overlay is never built into the workspace. It is a runtime layer, re-checked by §2 after every launch.
- **Restore.** `scripts/m61_restore_and_verify_disabled.sh` with these options, and it must report DISABLED AND VERIFIED:
  - `--base <baseline branch>`;
  - `--enabling-ref local/m55-s2-<UTC>` and `--delete-enabling-branch`;
  - `--launch-pid-file`, `--xvfb-pid-file`, `--evidence <dir>` and the `--build-cmd` above.

## 6. Sequence (one attempt)

1. §1–§5 pass on the disabled build. The evidence directory is created outside tracked source.
2. Enabling branch, then the patch, then the enabled build. Gates: M5.5 True; M6.1 and M6.0-D False. The library digest is identical.
3. One launch of the **welded** `fortress_m61a_fixed_base.launch.py` through the harness copy. Wait for both controllers, then run §2.
4. The node in dispatch mode, then the driver script as in the S2 design. Limits: **at most 59 goals and at most 1 cancel.**
5. Stop with `stage_stop.sh`, then check for leftovers, then run the restore of §5. This is mandatory, whatever happened.
6. **Offline plant-validity check**, on the closed bag:

   ```bash
   python3 scripts/s2/response_law.py --bag <bag_0.db3> --expected-gain 1.0 --out <evidence>/response_law
   ```

   It must give SUPPORTED (exit 0):
   - |ĝ − 1| < 1e-6 pooled, for every required joint, and for every identified segment;
   - residual ≤ 1e-9 rad.

   Otherwise:
   - CONTRADICTED (exit 1) or INSUFFICIENT_EVIDENCE (exit 2): the attempt is **INCONCLUSIVE, plant not as configured or not measurable**;
   - P1–P8 may be computed for the record, but S2 cannot PASS on that attempt.
7. Only after SUPPORTED: score P1–P8 with the frozen scorer `analyze_s2.py` (sha256 `e9c2efcb…`), thresholds unchanged.
   - The analyzer's tracking section (P2 basis) must agree with the scorer's P2 value.
   - S2 PASS needs every P1–P8.
   - A PASS still claims only that the crawl phase trajectories executed and tracked on a welded body: not walking, balance or contact.
8. The evidence (run directory, logs, bag hash, `response_law/`, P1–P8 summary, checksums) is left uncommitted for review.

## 7. Offline plant check vs online stop conditions

- **Online, can stop the attempt:**
  - the driver's stop conditions (status tracking error > 0.02 rad gives an estop with one cancel, plus faults, timeouts and driver errors);
  - the node's faults (tracking 0.05 rad, watchdogs);
  - the controller's tolerances;
  - the harness watchdog.
- **Offline, cannot stop it:** `response_law.py` and the P1–P8 scorer run after the bag is closed. A CONTRADICTED plant found offline invalidates the attempt's interpretation; it cannot stop the motion retroactively.
- No live monitor is added by this draft. One would need its own design and tests.

## 8. One-attempt authorization (DRAFT, NOT AUTHORIZED)

> **DRAFT, NOT AUTHORIZED. Do not execute.**
>
> I authorize ONE S2 fixed-base crawl attempt on the modified simulated plant (`position_proportional_gain` 1.0 through the pinned gz_ros2_control overlay):
> - **Baseline and rig:** the baseline is `<commit SHA>`, whose `src/` equals `ca49db3`. The rig is the **welded** body only.
> - **Gates:** only the M5.5 gate is enabled, by the unchanged patch `d1dacac5…20236`, on a local branch that is never pushed. The M6.1 and M6.0-D gates stay False.
> - **Before arming:** the §2 runtime loading checks and the §3 pre-motion regression must pass. Otherwise stop: no goal, report BLOCKED.
> - **Limits:** at most **59** phase goals and at most **1** cancel. No retry and no relaunch after any goal has been sent. No free-base launch and **no S3**.
> - **Changes:** no gain, threshold, trajectory, gait or acceptance change.
> - **Mandatory cleanup:** `stage_stop.sh`, the leftover check, and the restore with DISABLED AND VERIFIED, whatever the outcome.
> - **Scoring:** first the offline plant-validity check (`response_law.py`, expected gain 1.0, SUPPORTED); then P1–P8 with the frozen scorer.
> - **Results:** reported for review, not committed or pushed without approval.
>
> Owner: ____________  Date: ____________
