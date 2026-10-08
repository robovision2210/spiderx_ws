# M6.1-A clearance report: preserved evidence

`clearance.json` is the offline ground-clearance report of the welded SpiderX (M6.1-A §3),
regenerated twice from one recorded revision with a clean tree. It is committed here because
`log/` is generated output that a clean build used to delete (§14 of the implementation record).

| Item | Value |
|---|---|
| Code revision (clean tree) | `9d24b1ed3a9a8210925b3cff9718561768ad92e4` (`claude/stoic-shannon-ur2mes`) |
| Command | `ros2 run spiderx_controller m61a_clearance --out <dir>` (run twice) |
| Exit codes / wall time | 0 / 272.2 s; 0 / 267.1 s (4-core Cloud VM; another offline job ran concurrently) |
| Output SHA-256, run 1 = run 2 | `10083779a72fa9c3d55f67a97fefb47a2a25e341f87a615d86893be0c82d84c9` (byte-identical) |
| Report schema | `spiderx.m61a.clearance/1` |
| Trajectory analysed | `241760e7dfd5ef12`, content `94a492c43fcc046125d6bc71ee7f1d9a8a5d8466f1a1bdd9958a16044dd58e64` |
| `config/m61a_fixed_base.yaml` | `eb04c61285de99bb265f8bcfa826a2cde62d25e6069c98763a10d04c37156ddb` |
| `config/m61_trot_cycle.yaml` | `b91839fc1000e48ba947bb91a9c8e86071093794f7b175a43c116f6e1ecb4f39` |
| `config/m61_limits.yaml` | `0ff8fc9ee6f1ba70f8e14acda61e3889d3ef3395fdfceaf51f6ea6fc11fe70c7` |
| `m61a_clearance.py` | `d224c298356d0339c30df0360f341f59348070c847972f768087c17896f6c765` |
| `src/spiderx_description` (git tree) | `effc7431ca2d9a3962a9553492171a0ab31521b2` (URDF, collision meshes) |

**Key results (unchanged from M6.1-A §3):**
- neutral 0.05454 m below `base_link`;
- commanded spline 0.05459 sampled, 0.05463 bound;
- lead-in box bound 0.06703;
- tracking box bound 0.06227;
- every reachable configuration (limits + 0.02 rad) 0.10906 m bound (sampled 0.10806);
- recommended mount 0.125 m (required ≥ 0.12406), leaving 15.9 mm passive clearance;
- the 0.075 m candidate fails the reachable set (−34.1 mm);
- `config check: consistent`.

**Determinism.**
- The report contains no timestamps, durations or host data. The two runs are byte-identical.
- The only nondeterministic metadata is the output **directory name** (`<UTC>`), which is not
  part of the file.
- Check with:
  `python3 docs/evidence/compare_reports.py <run1>/clearance.json <run2>/clearance.json`
  (exit 0: identical).

**Environment.**
- Ubuntu 24.04.4 container, x86_64, 4 CPUs, RoboStack ROS 2 Humble;
- Python 3.11.16, numpy 1.26.4, PyYAML 6.0.3;
- xacro 2.0.13, urdfdom 4.0.1, libsdformat12 12.8.0;
- libignition-gazebo6 6.16.0 is installed but **not started**: the analysis is offline;
- the build used setuptools 59.6.0 from a private `--target` directory; the shared
  environment's 84.0.0 is unchanged.

**Limitations.**
- These are the §3 limitations: CAD collision geometry, a flat ground at z = 0, a rigid
  zero-tilt weld, no contact margins.
- The reachable-set bound assumes Gazebo keeps joints within their limits + 0.02 rad.
- Nothing here ran in Gazebo.

**Regenerate.** From a clean checkout of the revision above, after
`colcon build --symlink-install`, run the command above twice and compare the two files.

## Verification of the review corrections at this revision

From a clean worktree of `9d24b1e`: `colcon build --symlink-install` (exit 0), then
`colcon test --packages-select spiderx_controller spiderx_scripts` (exit 0, 6 min 48 s) and
`colcon test-result --verbose` (exit 0): **1465 tests, 0 errors, 0 failures, 0 skipped**.
- That is 1448 before the review, plus 17:
  - `test_m61_gait_replay` +8: 3 new mock scenarios, 3 competing-commander refusals, the late
    client, the wrapper;
  - `test_m61a_fixed_base` +7: 2 assess codes, 5 `command_owner_codes` cases;
  - `test_m61a_observer` +2: a late publisher and a late action client on an isolated domain,
    discovered after 0.060 s and 0.053 s in the targeted run.
- The isolated-domain tests ran (none skipped).
- All three dispatch gates stay `False`.
