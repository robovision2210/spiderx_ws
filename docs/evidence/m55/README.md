# M5.5 free-base gait feasibility report: preserved evidence

`report.json` is the offline feasibility report of the crawl (M5.5 §2 and §5): validated speed
levels, the phase-goal library, the static-margin method, and the support analysis behind "crawl,
not trot". It was regenerated twice from one recorded revision with a clean tree, and is
committed here because the original run's copy in `log/` was deleted by a
`rm -rf build install log` clean build.

| Item | Value |
|---|---|
| Code revision (clean tree) | `49cdeffccbc76ad7a751072e0263e25d8b3d7982` (`claude/spiderx-m55-keyboard-walking`) |
| Command | `ros2 run spiderx_controller m55_gait_feasibility --out <dir>` (run twice) |
| Exit codes / wall time | 0 / 82.2 s; 0 / 83.5 s (4-core Cloud VM; another offline job ran concurrently) |
| Output SHA-256 | run 1 `56bd5592e1e276a2b327594feb5ca4aab66d8c867a7a213d2ca8af978be36bca` (this file); run 2 `320322a3d2c88b7d06208bfda5367d3b83df4d9c6a5a8ed457b2aa470ef48a7c` |
| Deterministic SHA-256 (`levels[].build_s` removed), run 1 = run 2 | `541055933938ff53a8eda9d21a17f9b33abddb8271c19dc2a67c05a11f0f129b` |
| Report schema / result | `spiderx.m55.gait_feasibility/1`, `ok: true`, `dispatch_gate: false` |
| `config/m55_locomotion.yaml` (also recorded in the report) | `5b243247ec6ca9be02ea1900dd9f72e7558c6abd5ce974d473ef2514ee4f2b49` |
| `m55_crawl.py` | `c10d0722d23670550781302fc23f76f6d306ea7d055ceb81dd1d4261e7467702` |
| `m55_feasibility.py` | `2e000af13bf8c5330213c912c0f75d19262168c2fa95b9e610227e4056c17d21` |
| `m55_locomotion.py` | `b963f37a15e39ba371e0a1130e84128222fab585409d471174d9e69a161a5b3f` |
| `gait_metrics.py` (mass model, margin) | `090ccc2243a770723e92f89b2f70bd765494b035ae445f5462d0b4dbaba7acbc` |
| `leg_kinematics.py` (IK) | `c620759f36f3f3ad1f7bb71688760efe9d6a425192f2b8cb6edb6949c9e3038b` |
| `src/spiderx_description` (git tree) | `effc7431ca2d9a3962a9553492171a0ab31521b2` (URDF, inertials) |

**Key results.**

| Stride | Min static margin (sampled) | Max margin step between samples | Max \|q̇\| | Max spline error | Speed incl. lead-in |
|---|---|---|---|---|---|
| 20 mm | 19.82 mm | 0.475 mm | 0.2245 rad/s | 7e-6 rad | 1.064 mm/s |
| 40 mm | 19.61 mm | 0.472 mm | 0.2247 rad/s | 7e-6 rad | 1.896 mm/s |
| 60 mm | 19.39 mm | 0.468 mm | 0.2246 rad/s | 3e-5 rad | 2.344 mm/s |

Other results:
- 54 phase goals.
- Three-foot margins without body shift: FL +3.8, FR +4.0, RL −4.0, RR −3.8 mm.
- Trot: two-foot support, no static margin.

These figures equal those of the original run at `2857d50`, whose file was lost. The report's
`template_labels` state the method:
- the margin is sampled every 20 ms on the dense path and the controller spline, not a
  continuous bound;
- support: the three non-swinging feet for the whole swing;
- point feet with **assumed** contacts;
- COM from the URDF inertials with a level body;
- the joint-speed bound is a development value, not an actuator rating.

**Determinism.**
- Repeated generation differs only in `levels[].build_s`, the wall time each template took to
  build. That is expected nondeterministic metadata.
- The output directory name (`<UTC>`) is not part of the file.
- Check with:
  `python3 docs/evidence/compare_reports.py <run1> <run2> --ignore 'levels[].build_s'`
  (exit 0).

**Environment.** As for [`../m61a/`](../m61a/README.md): RoboStack ROS 2 Humble, Python 3.11.16,
numpy 1.26.4. Gazebo is installed but **not started**: the analysis is offline.

**Limitations.**
- A quasi-static approximation: CAD steel-density masses, point feet, flat ground, assumed
  contacts, no slip or dynamics.
- A sampled minimum.
- Nothing here ran in Gazebo, and nothing walked.
