# Phase B: launch-only verification of the actuator overlay (2026-10-30): PASS

**Scope.** One launch of the **welded** fixed-base simulation (`fortress_m61a_fixed_base.launch.py`), with all three dispatch gates False.
- Not started: the M5.5 node, a driver, teleop, any request, goal or cancel, a free-base launch, or an S2 retry.
- **Claims loading and configuration only:** not tracking, not the gain-response law (nothing moved), not contact, balance or walking.
- S2 stays FAIL. Phase C is not authorized.

## Inputs (`provenance/`)

- **Repository:** branch `claude/spiderx-sim-actuator-model`, HEAD `502214d`, clean tree. PR #19 open, draft.
- **Overlay workspace:** `/root/spiderx_gz_overlay_ws`, found by search rather than assumed.
  - Source: upstream `bfc5845` plus exactly the committed patch (sha256 `14238400…7bac`).
  - Both library hashes equal `OVERLAY_PROVENANCE.txt`.
- **SpiderX install:** corresponds to HEAD. It is a symlink install with no dangling links, and 14 of 14 key runtime inputs are identical to their HEAD blobs.

## Environment (`env/`)

- **Run-local copy.** The frozen harness (`../s2_fixed_base_20261029T100210Z/tools`) was copied outside the repository. Its three stage scripts are unchanged; `env.sh` has one added line, `source /root/spiderx_gz_overlay_ws/install/local_setup.bash`, between the toolchain and the workspace (`env_sh_changes.diff`). A deadline watchdog (`watchdog.sh`) guarded against an interrupted session.
- **Pre-launch checks** (`prelaunch_checks.txt`):
  - ament prefix, pluginlib and the launch-time plugin directory are all the overlay;
  - both libraries resolve their dependencies (`ldd`, plus `dlopen` with RTLD_NOW);
  - no plugin-path override;
  - the three gates are False, imported from this workspace.

## Results

| Check | Result |
|---|---|
| Launch | One launch; both controllers active after 40 s (`run/`) |
| Simulator process | PID 1332, `ign gazebo server`, in this launch's session 1144: 1144 → 1242 → 1245 → 1332. The only process mapping the plugin (`process/`) |
| `libgz_ros2_control-system.so` | Overlay path only; inode = overlay file; mapped-object sha256 `ef37a04f…` = provenance; not deleted |
| `libgz_hardware_plugins.so` | Overlay path only; inode = overlay file; mapped-object sha256 `c3db97f2…` = provenance; not deleted |
| Stock counterparts | None mapped, including no mixed load. The server's plugin path holds the overlay `lib` only |
| Gain | 1 log line: `position_proportional_gain has been set to: 1`. `ros2 param get` returns 1.0. No validation or init error (`graph/`) |
| Rate / physics step | Controller manager `update_rate` 100. Runtime warning: control period 0.01 s against physics period 0.001 s |
| Graph | 2 controllers active. 1 action server, `/leg_trajectory_controller/follow_joint_trajectory`, with 0 clients. 0 publishers on `/leg_trajectory_controller/joint_trajectory`. No locomotion, driver or teleop node or process |
| Observer | Read-only preflight: READY, 12 of 12 checks. Body z 0.125 m (range 0); attachment and spawn deviation 0 m; it sent 0 goals and created 0 publishers (`observer/`) |
| Goals / cancels sent by this task | **0 / 0** |
| Cleanup | Frozen `stage_stop.sh`: process group empty after 9 s, but **not graceful**: the Gazebo server was SIGKILLed during shutdown (Notes). Daemon and Xvfb stopped; no leftovers. `m61_restore_and_verify_disabled.sh --base claude/spiderx-sim-actuator-model --verify-only`: **DISABLED AND VERIFIED**, with the gate modules imported from `src/spiderx_controller` (`cleanup/`) |

## Notes

- **Startup waits, not extra launch attempts.** `run/launch.log` lines 57–58 hold two `robot_state_publisher service not available, waiting again...` messages, 0.5 s apart. They are gz_ros2_control waiting inside its URDF fetch during the one launch (`ros2 launch` PID 1144); the plugin then loaded normally. Nothing was relaunched.
- **Shutdown was not graceful.** `run/launch.log` line 295 reads `Escalating to SIGKILL on [Ignition Gazebo Server]`: after the harness's SIGINT, the Gazebo server did not exit within ign's grace period, and ign's launcher killed it. Line 296's `process has finished cleanly [pid 1242]` refers to the `sh -c ruby ... ign gazebo` wrapper, not to the server. The leftover check found no remaining process, but the server's own shutdown was forced.

## Remaining uncertainty

- **The write loop is unproven.** That it applies 1.0 needs motion: the response-law fit (|ĝ − 1| < 1e-6) required in Phase C.
- **One run.** This is a single launch, on the welded rig, without contact.

`SHA256SUMS` covers every file here. Reviewed and committed as evidence only (the response law was not measured).
