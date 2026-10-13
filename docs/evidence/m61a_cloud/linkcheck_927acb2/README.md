# run_06: physics provenance of the Gazebo pose entries for E8 (Cloud, no motion)

**What this run is.**
- One supervised launch of `fortress_m61a_fixed_base.launch.py` at `927acb2`. It is not a
  criterion-6 run.
- **The analysis was frozen first.** `spiderx_controller.m61a_link_check` (SHA-256
  `649b0c4f…7059e31`) was committed and pushed in `927acb2` before the launch. Its thresholds
  are module constants, pinned by a test.
- **Harness.** Start, wait and stop used the frozen stage scripts. The 14 frozen files in
  `tools/` are byte-identical to `../observation_7f4c30f/tools/`, checked against
  `../closeout/frozen_harness.json` before the run.
- **Subscriptions only.** A description reader, then `ros2 bag record` for 15 s of four topics:
  - `/spiderx/sim/world_poses` (all 68 entries);
  - `/joint_states`;
  - `/clock`;
  - `/robot_description`.
- **Nothing else.** Nothing was commanded, no goal was sent and there was no simulator control.
  The gates stayed `False`.

| Path | Content |
|---|---|
| `tools/run_linkcheck.sh` | The procedure: start, wait for the controllers, description, 15 s recording, Ctrl+C-equivalent stop |
| `tools/capture_description.py` | Read-only `/robot_description` reader (transient local) |
| `tools/` (other files) | The frozen phase-1 harness, unchanged |
| `revision.txt` | `HEAD` (`927acb2`), a clean tree apart from this folder, the module's SHA-256, and the imported file |
| `run_06/bag.tar.xz` | The rosbag2 directory (sqlite3, 11,729 messages), compressed losslessly. The archive round trip was checked byte for byte |
| `run_06/analysis/link_check.json`, `.txt` | The frozen analysis, applied once, offline: `--bag run_06/bag --server-log run_06/launch.log` |
| `run_06/robot_description.urdf` | The description, captured separately. Byte-identical to the copy in the bag, which the analysis used |
| `run_06/*` (other files) | Launch log, steps, controller state, shutdown record |

## Results

| Check (frozen in `927acb2`) | Result |
|---|---|
| **E8 body pose.** Composition `spiderx` entry · `dummy_link` entry · T_dummy_body, compared with the description's weld origin; limits 1 mm, 3.33 mrad, abs(dz) 1 mm | **PASS.** Deviation **0** (exact in double precision) in all 678 messages. The model entry stayed the identity throughout |
| **E8 provenance.** The 12 leg-link entries relative to the `dummy_link` entry, compared with URDF FK at the measured joints and with FK(0) | **PASS** on 678 of 678 messages. Residual against FK(q): at most **7.6e-8 m and 7.5e-7 rad** (limits 1e-5). FK(q) and FK(0) differ by up to 1.07e-4 rad. The entries differ from FK(0) by **at least 9.6e-5 rad** on the discriminating links |
| Server log (`launch.log`) | No `Internal error` line and no `does not have a world pose` line |
| **E9** | **NO_MOTION**, as expected: consistent, but E9 is not satisfied without motion |

**Held joints.** About 4.4e-5 to 9.7e-5 rad (gravity sag under the position controllers),
constant to about 1e-19 rad over the recording. These are the same values as in `run_04`. The
simulation advanced 8.7 s during the 15 s recording (RTF about 0.58).

## What this establishes

**Measured in this launch.** The leg-link entries are DART's poses relative to the body at the
measured joint positions. They are not the SDF initial values.

**From the source** (gz-sim 6.16.0 `Physics.cc`; gz-physics 5.3.2 dartsim `SimulationFeatures.cc`):
- **Link poses have one writer.** `Physics.cc` line 2799 is the only place a link pose is
  written, relative to `modelWorldPoses`.
- **That cache has one writer.** Only `UpdateModelPose` (line 2612) fills it, and it writes the
  model pose from DART's canonical body at the same time (line 2625).
- **The other model-pose write does not apply.** Line 2114 writes a static model on a pose
  command. This model is not static, and no command was sent.
- **Consequence.** The `spiderx` entry in this launch is DART's value. It is not the unwritten
  SDF default, which happens to be equal (identity).

**Bounded staleness.** `SimulationFeatures::Write` (lines 82–118) reports a body again whenever
its pose differs from its last **reported** pose by more than 1e-6 in any position axis or
quaternion component. The composed body therefore equals DART's merged-body pose to within that
bound in every message.

**Composition.**
- Physics divides by the same X_ML (the `dummy_link` entry) that the composition multiplies
  back, so X_ML cancels exactly.
- T_dummy_body is the URDF `dummy_joint` origin (identity) inside one rigid DART body.
- So the E8 body pose is a physics measurement, and comparing it with the weld origin compares a
  measurement with the specification. The weld is not assumed.

## What remains unproven

1. **DART is trusted as the simulation's ground truth.** This covers WeldJoint rigidity and the
   integrator. There is no independent sensor; none is needed in simulation, but this is the
   trust boundary.
2. **The canonical re-report path was not exercised after the first write.** The body never
   moved in this fixed-base launch. Its runtime evidence is the free-base M2 launches (same
   model and toolchain): the model entry followed the body from the spawn height to 0.0545 m
   (`docs/M2_TEST_RESULTS.md`). The source supports it as well.
3. **The first write is shown only indirectly.** Its value equals the SDF default, so it is
   shown through the leg entries and the source, not by its value.
4. **Motion is not covered.** Dynamic link/kinematic consistency during motion is E9. E9 says
   nothing independent about global body height or tilt.

## Reproduce

```
tar -xJf run_06/bag.tar.xz -C <dir>
python3 -m spiderx_controller.m61a_link_check --bag <dir>/bag --out <dir>/analysis \
  --server-log run_06/launch.log
```

A re-analysis from the archive gave an identical `link_check.json`, apart from the bag path.

**Caveat.** While the run waited for the controllers, the ros2 CLI daemon fault occurred. The
frozen wait helper restarted the daemon, as designed (`run_06/daemon_events.txt`), as in `run_04`.

`SHA256SUMS` covers every file in this folder.
