# Simulated actuator model: design note (S2 follow-up)

**Status:** Phase A, implementation and offline tests, on branch `claude/spiderx-sim-actuator-model`.
- Phases B (launch-only verification) and C (S2 retry) each need their own approval and have not been executed.
- Nothing has run in Gazebo, and no goal or cancel has been sent.
- All three dispatch gates stay False. S2 stays **FAIL** (2026-10-29) and S3 stays blocked.
- **Evidence:** `docs/evidence/m55/s2_fixed_base_20261029T100210Z/analysis_offline_20261029/` (commit `9c1a61b`), including `REVIEW_ADDENDUM.md`.

**Decision (owner).** Route (a): an explicitly idealized position servo with gain 1.0, for this simulation-only software-integration milestone.
- It is not a model of a real servo: there is no backlash, deadband, torque–speed curve or compliance.
- 1.0 comes from the sampled response model (§3), not from re-running S2 until it passes.
- The position command interface stays.
- Unchanged: gait timing, trajectory fingerprints, tracking criteria, safety thresholds, joint limits, masses, friction, controller rate and physics timestep.

## 1. Source findings: gz_ros2_control 0.7.15

Reference: `ros-controls/gz_ros2_control` tag `0.7.15` = commit `bfc5845aa7f0250cbc384773dba5c9847d609b95`.

| Finding | Basis |
|---|---|
| The default gain is `0.1`, declared on the plugin's own node (`gz_ros2_control`) | **Code:** `gz_system.cpp` L205–213 |
| `write()` sets v_cmd = −K_p · (x̂ − c) · f, where f = the controller manager's `update_rate` (int) | **Code:** `gz_system.cpp` L684–703 |
| x̂ changes only in `read()` | **Code:** `gz_system.cpp` L565–592 |
| `write()` runs on every physics step (`PreUpdate`) | **Code:** `gz_ros2_control_plugin.cpp` L531–565 |
| `read()` + `update()` run only once `sim_period ≥ 1/f` (`PostUpdate`) | **Code:** plugin L568–588, L517–521 |
| **No supported route sets K_p.** The plugin parses no gain XML element (L275–349). It creates its node (L358–359) *before* installing the `<parameters>` YAML as global arguments (L391–398), so that node's overrides are empty | **Code.** Also an **offline probe** against the installed rclcpp: plugin-order node 0.1; a node created later 1 (`REVIEW_ADDENDUM.md`). The humble head (0.7.21) has the same order |
| The installed binary is this code | **Provenance only.** The recipe builds release tag `0.7.15-1` with no patches; its sources are byte-identical to upstream; installed headers are identical |
| Runtime values: K_p 0.1 (log); f 100 Hz with physics step 1 ms (log warning). Measured: g = 0.1000000000, K = 10.000 s⁻¹, Δx = T·v to 4e-15 | **Runtime data** (S2 bag and `launch.log`) |

**How the parameter reaches the plugin node at startup:** it does not. The YAML configures the controller manager and the controllers, which are created after the arguments are installed. The plugin node receives only `declare_parameter`'s default.

## 2. Controller-sampled recurrence

**Assumptions:**
- **A1** Each physics step realizes v_cmd as the joint velocity: effort is not limiting, and no contact or joint limit blocks the joint.
- **A2** Position integrates as x ← x + h·v.
- **A3** N·h·f = 1 (the update comes every N = 10 steps).
- **A4** Nothing else writes the joint.

**Derivation.** Between updates k and k+1, x̂ = x[k] and c = c[k] stay fixed (code), so v_cmd is constant and

  x[k+1] = x[k] + N·h·K_p·f·(c[k] − x[k]) = x[k] + K_p·(c[k] − x[k]),  v[k+1] = K_p·f·(c[k] − x[k]).

**Error recursion:** e[k+1] = (1 − K_p)·e[k] + (c[k+1] − c[k]).

S2 measured g = K_p = 0.1 to 3e-11, so A1–A3 held on the welded rig. **The measured recurrence is exactly what this sample-and-hold arrangement produces.**

## 3. Gain 0.1 vs candidate gain 1.0 (analytic)

T = 10 ms.

| Property | K_p = 0.1 (current) | K_p = 1.0 (candidate) |
|---|---|---|
| Pole 1 − K_p | 0.9 | 0 (deadbeat) |
| Step response | geometric; 5 % left after 29 updates (0.29 s); τ = 94.9 ms | reaches the command at the next update |
| Constant-speed error | (T/K_p)·ċ = 0.100 s·ċ, after a transient | T·ċ = 0.010 s·ċ, from the first update |
| Overshoot | none | none (K_p > 1 alternates; K_p ≥ 2 is unstable) |
| Margin to instability | ×20 | ×2 |
| Commanded velocity, smooth tracking | tends to ċ | ≈ ċ at each update |
| Commanded velocity after a command step Δ | 10·Δ s⁻¹ | 100·Δ s⁻¹ |
| **Prediction**, goal 1, full 1.7 s: max \|e\| | 0.02106 rad (it reproduces the measured first 1.0 s to 1e-11) | 0.00219 rad |
| **Prediction**, peak commanded \|v\| | 0.211 rad/s | 0.219 rad/s |

The predictions are labelled as such. They come from the recurrence, the rebuilt goal 1 and the measured start state, assume A1–A4 with no contact, and are not Gazebo validation. Only goal 1 was evaluated; the gait library was not regenerated.

**Independent rationale for 1.0.** In this sample-and-hold law, K_p = 1 is the unique gain under which the joint reaches each command exactly at the next sample. That is the nominal meaning of a position command interface at a 10 ms update. It is also the largest gain with no overshoot, and it keeps a 2× margin to instability. The residual T·|ċ| is a consequence of the choice; the 0.02 rad criterion played no part in it.

**Risks:**
- **Sampling.** A one-update lag remains.
  - After a cancel, the hold equals the previous update's state. With K_p = 1 the joint overshoots it by about T·ċ and returns in one update: a velocity reversal of about 2|ċ| within one physics step.
  - With K_p = 0.1 the overshoot is the same size, but the return takes about 0.3 s.
- **Overshoot.** The ideal model has none. A period mismatch would give an effective gain other than 1; for example, an 11-step period gives 1.1, an alternating error that shrinks ×0.1 per update. It becomes unstable only at an effective gain ≥ 2.
- **Commanded velocity.**
  - It stays near the reference speed: ≤ 0.225 rad/s, the library's design bound of 0.9 × 0.25.
  - Response to command steps is 10× larger. Goal starts begin from the measured state (the controller's lead-in), so the main step is the cancel hold.
  - The per-step acceleration is bounded only by the 100 N·m placeholder effort limit.
- **Contact.** Contact or effort limits break A1. A stiffer servo:
  - enforces positions harder;
  - saturates sooner;
  - passes larger impulsive loads to a free body, which changes foot slip, bounce and tipping.

  **Nothing here predicts free-base (S3) behaviour.** The welded S2 rig guarantees foot clearance, so S2 does not exercise contact.

## 4. Implementation (Phase A)

The route-(a) choice was preceded by a configuration stop. In 0.7.15 the gain cannot be set by configuration (§1), which is why the earlier xacro proposal was withdrawn (`REVIEW_ADDENDUM.md` §2).

**Overlay: `third_party/gz_ros2_control_overlay/`** (details in its `README.md`)
- It is a pin plus one patch plus a build script; no upstream source is vendored, and `/opt/mm` is never touched.
- The script:
  - fetches tag `0.7.15`, sparse-checked-out to the package;
  - verifies commit `bfc5845…`;
  - applies `patches/0001-plugin-node-parameter-overrides.patch`;
  - builds with the toolchain's GCC into `~/spiderx_gz_overlay_ws`, outside the repository;
  - runs the gtest;
  - records `OVERLAY_PROVENANCE.txt`.
- The upstream license is kept as `LICENSE.upstream`.
- **The fix.** `Configure()` now creates its node with the `<parameters>` files as node-local `--params-file` arguments (`plugin_node_options()`).
  - Remapping rules, `robot_description` and the controller-manager setup are as before. Remapping already reached the node, because rules are resolved against the global arguments at use; parameter overrides did not, because they are fixed when the node is created.
  - The gain is declared by `declare_position_proportional_gain()`. The default stays 0.1 when unspecified. A value that is not a finite double greater than 0 is reported and `initSim()` fails; it is never clamped or replaced. The gain is read once at start-up, so there is no dynamic change.
  - No XML element was added: none is supported in 0.7.15.

**Configuration**
- New file `src/spiderx_description/config/gz_ros2_control_plant.yaml` holds `gz_ros2_control.ros__parameters.position_proportional_gain: 1.0`. Its scope is the plugin node `/gz_ros2_control` only.
- It is added as a second `<parameters>` element of the plugin in `spiderx_fortress_control.xacro`, so it applies to every Fortress launch with `enable_control:=true`.
- `spiderx_ros2_controllers.yaml` is deliberately untouched: M6 trajectories record its SHA-256 as provenance.
- The legacy Gazebo Classic files and the passive description are unchanged.
- **Plant change.** This changes the simulated plant. Earlier Fortress runs, including the S2 FAIL, used 0.1, and no runtime equivalence with them is assumed. Their evidence and verdicts are untouched.

**Offline results (2026-10-29)**

| Check | Result |
|---|---|
| Clean-workspace overlay build | OK |
| `test_parameter_loading` | 9 of 9 pass |
| Lint, the patch's own files | All pass. The one uncrustify divergence is pre-existing upstream (§ README) |
| Installed libraries | RPATH is the toolchain's lib directory; `dlopen(RTLD_NOW)` OK |
| Resolution order | With the overlay sourced between the toolchain and the workspace, `get_package_prefix`, pluginlib and the launch's plugin directory all resolve to the overlay. Without it, they resolve to stock |
| SpiderX focused tests | 144 pass: new `test_sim_actuator_model.py`, `test_config_check.py`, `test_m61a_fixed_base.py`, `test_m61a_observer.py` |
| Phase-library digest | Recomputed with the committed `tools/library_fingerprint.py`: `62f8a100…e732`, 54 goals, gate False. Identical to the 2026-10-29 record |
| Gates | All three False |

**Not run:** the full historical suite. No shared code path changed:
- the edits are a new configuration file, one xacro element in the control branch only, docs, tests and `third_party/`;
- the M6 provenance inputs and the library inputs are untouched.

**Limitations**
- Only a sourced overlay applies the plant file; stock 0.7.15 silently keeps 0.1. Phase B must prove the loaded library and value.
- The offline tests exercise the production helpers in `Configure()`'s order. They do not exercise `Configure()` itself or the Gazebo write loop: that is Phase B and Phase C.
- Rebuilding needs network once (the GitHub fetch), plus the GCC-15 `-include cstdint` flag for the vendored gtest.
- The plugin node now receives every parameter-file entry addressed to it, wildcards included. SpiderX's files address it only through the plant file.
- Gain 1.0 rests on assumptions A1–A4 (§2). On the welded rig they held at 0.1. **Nothing here predicts free-base (S3) behaviour.**

## 5. Phase B: launch-only verification (NOT executed; separate approval)

**Preconditions:**
- The branch head is built.
- The overlay is built, and `~/spiderx_gz_overlay_ws/OVERLAY_PROVENANCE.txt` exists.
- The SpiderX workspace is built (`colcon build --symlink-install`).
- No leftover processes.

**Environment file for both shells.** It is the frozen `env.sh` with one added line, the overlay, placed before the workspace:

```bash
source /opt/mm/activate.sh >/dev/null 2>&1
source ~/spiderx_gz_overlay_ws/install/local_setup.bash
source /home/user/spiderx_ws/install/setup.bash
export ROS_DOMAIN_ID=0 ROS_LOCALHOST_ONLY=1; unset RMW_IMPLEMENTATION ROS_DISCOVERY_SERVER
export DISPLAY=:77
```

**Checks before launch.** Stop on any failure.

```bash
O=$HOME/spiderx_gz_overlay_ws/install/gz_ros2_control
test "$(ros2 pkg prefix gz_ros2_control)" = "$O"
test -z "${IGN_GAZEBO_SYSTEM_PLUGIN_PATH:-}${GZ_SIM_SYSTEM_PLUGIN_PATH:-}"   # nothing may shadow it
python3 -c "from ament_index_python import get_resources as g; \
  assert g('gz_ros2_control__pluginlib__plugin')['gz_ros2_control'] == '$O'"
python3 -c "from spiderx_controller import m55_contract as a, m61_live_contract as b, \
  m6_live_contract as c; assert not (a.M55_LOCOMOTION_DISPATCH_ENABLED or \
  b.M61_LIVE_DISPATCH_ENABLED or c.LIVE_DISPATCH_ENABLED)"
```

**Launch only:**
- Start with the frozen `stage_start.sh` and wait with `stage_wait_controllers.sh` (fixed-base launch).
- Start no M5.5 node, no driver and no teleop, so dispatch stays disabled and nothing sends a goal.

**Checks while running.** Each one names the condition that means STOP.

1. **Loaded libraries.** These are the proof; the source tree or a successful build is not.

   ```bash
   PID=$(grep -l 'libgz_ros2_control-system\.so' /proc/[0-9]*/maps 2>/dev/null | cut -d/ -f3 | head -1)
   grep -oh '/[^ ]*lib\(gz_ros2_control-system\|gz_hardware_plugins\)\.so' /proc/$PID/maps | sort -u
   sha256sum $(grep -oh '/[^ ]*lib\(gz_ros2_control-system\|gz_hardware_plugins\)\.so' /proc/$PID/maps | sort -u)
   ```

   Expect exactly the two `$O/lib/...` paths, with the hashes in `OVERLAY_PROVENANCE.txt`. **Any `/opt/mm/...` path for either library is an accidental fallback to stock 0.7.15: STOP.** This includes a mixed load, e.g. the system plugin from the overlay with the hardware plugin from stock.

2. **Runtime log.** `grep -c "The position_proportional_gain has been set to: 1$" launch.log` must equal 1. STOP on any of:
   - `... set to: 0.1`: stock, or the file not applied;
   - `... must be ...`: an invalid value;
   - the FATAL `Could not initialize robot simulation interface`.

3. **ROS parameters:**
   - `ros2 param get /gz_ros2_control position_proportional_gain` must give `1.0`;
   - `ros2 param get /controller_manager update_rate` must give `100`;
   - the period warning must still read `(0.01 s) … (0.001 s)`.

4. **Controllers:**
   - `ros2 control list_controllers` shows both controllers active;
   - `ros2 action info /leg_trajectory_controller/follow_joint_trajectory` shows 0 action clients.

**Stop:**
1. Run the frozen `stage_stop.sh`: SIGINT to the launch group, leftover listing, daemon stop.
2. Run `scripts/m61_restore_and_verify_disabled.sh --verify-only`, which must print DISABLED AND VERIFIED.
3. `pgrep -af "ign gazebo|gz sim|ruby.*ign|controller_manager|ros2 launch|Xvfb"` must print nothing.
4. Keep the run's `launch.log`, `/proc` capture and parameter outputs as evidence.

**What Phase B cannot show.** A parameter query does not prove that the write loop uses the value.

## 6. Phase C: S2 retry (separate authorization; requirements only)

**Acceptance is unchanged.** Before scoring, the response law must be measured on the new bag: x[k+1] − x[k] = g·(c[k] − x[k]), with |ĝ − 1.0| < 1e-6 over every joint and update. Otherwise the run stops as *plant not as configured*.

**Analysis tool.** The committed analysis tool's hold detection assumes g = 0.1. It needs a gain argument before Phase C, which is a separate small change.

**No claim in advance.** Nothing above claims that this verification has happened, or that S2 will pass.

## 7. Provenance and rollback

- **Baseline.** `9c1a61b` is the K_p = 0.1 baseline, the 2026-10-29 plant. The S2 evidence and its analysis are unchanged.
- **Reproduce the old plant.** Do not source the overlay; stock ignores the plant file and keeps 0.1. Or check out `9c1a61b`.
- **Roll back this change.** Revert the configuration commit (the plant file and the xacro element) and stop sourcing the overlay. Remove `~/spiderx_gz_overlay_ws` by hand if no longer needed.
- **Every future run must record its plant:** the log line, the `/proc` library paths and hashes, and `ros2 param get`.
