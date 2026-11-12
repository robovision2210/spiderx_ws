# S4a shadow rehearsal, 2026-11-01 (dispatch disabled)

**Result: PASS (SHADOW ONLY).** This was one free-base SHADOW launch with all three gates `False`. **Zero** trajectory goals or cancels reached the real controller.

This is a rehearsal of the S4a script, **not** live S4a acceptance. The displacements are measured and recorded, but not scored: the live scorer was not run. **Live S4a is not authorized.**

## Inputs (`provenance/`)
- **Branch and HEAD.** `claude/spiderx-s4a-preparation` at `7c1ecbc`, clean tree, no `local/*` branch.
- **r2 overlay** (`input_checks.txt`):
  - gz_ros2_control 0.7.15 (`bfc5845`) plus patches 0001 (`14238400…`) and 0002 (`520e73cc…`);
  - every present source file equals upstream plus both patches;
  - libraries `3a8f2044…` (system) and `06777e14…` (hardware plugins);
  - compiler as in the provenance; the toolchain was installed by `cloud_env_setup.sh` `57cc33f8…`.
- **Installed runtime.** Built by the S3 restore's clean rebuild of `d918799`, using symlink-install.
  - 169 files are symlinked into `src/`, and each equals its HEAD blob.
  - The only `src/` change since that build is a test file, which is not installed.
  - The first, stricter classifier flagged the toolchain's ament environment hooks and bytecode caches. Its output is kept as `input_checks_attempt1_strict_classifier.txt`.
- **Gates.** All three `False` on a fresh import from this workspace. The phase library is 54 goals, `62f8a100…`, and the node's `library.json` is `917fc4d7…` (the S3 disabled library).
- **S4a tools.** `s4a_driver.py` `243974e7…` and `score_s4a.py` `631fe2ec…`. The base drivers `s3_driver.py` and `s2_driver.py` are the ones that ran S3. The focused tests at HEAD: 48 OK, plus 24 passed (`focused_tests_at_head.txt`).

## Harness (`tools/`, run-local; historical copies unchanged)
- **Base.** The S3 live harness, after verifying it against its committed checksums.
- **Changes** (`harness_vs_s3_live.diff`): 7 files.
  - The SHADOW pre-key checks restore the S3 shadow-2 lines exactly (`harness_vs_shadow2.diff`).
  - `scripts/s4/s4a_driver.py --shadow-rehearsal` replaces the S3 driver.
  - Cleanup also stops `s4a_driver`.
  - Restore is `--verify-only` against `7c1ecbc`.
- **Unchanged:** `env.sh` (sources the r2 overlay), the `stage_*` scripts (`fortress_m55_walking.launch.py`, which starts the node itself), and `settle_check.py`.
- **Bag.** One recorder, with no `_action/status` and no `/clock`.
- **Hashes** are in `harness_SHA256SUMS.txt`. The watchdog (3600 s) and the trap were armed before the launch.

## Run (`run/`)
- **Timeline.** Launch at 15:16:30Z. Both controllers active at 15:17:08Z. Node up in SHADOW mode (gate False) at 15:17:33Z.
- **Pre-key checks** (`premotion/`): PASS.
  - The simulator 18415 alone maps the two r2 libraries, with matching hashes, and no stock libraries.
  - Gain 1.0 (log and parameter), 100 Hz, physics step 0.001 s.
  - Action clients: 0. Command publishers: 0. The node has no client.
  - Settling: 10.02 s of sim, z 0.054500 m, range 9.0 µm, tilt 1.5e-7 rad, every status SHADOW, gate False, DISARMED.
- **Driver.** Ran 15:19:18–15:24:24Z and exited 0: "script complete (shadow rehearsal: dispatch disabled; not acceptance)".
- **Planned shadow goals:** 19, all at level 0, in the exact script order: 9→8 … 1→0, then 0→1 … 8→9, then 0→1. Goals 1–18 SUCCEEDED; goal 19 ended "shadow cancel" (status 5). The node logged 18 `phase_done`, 1 `cancel_sent` and 1 `cancel_response`.
- **Real controller:**
  - the bag holds 0 `joint_trajectory` and 0 `_action/feedback` messages;
  - the controller reference changed by 0.0 rad (2888 samples);
  - the JTC log has 0 goal or cancel lines;
  - action clients: 0 before; after, only `/rosbag2_recorder`.
- **Cycle checks.** Each ran at a fresh sample, before the next direction key: reverse (−1, count −1) at sim 52.02, then forward (+1, count 0) at sim 71.29. The measured progress was −3.0e-10 and −3.3e-10 m. The signed bound was not applied (verified SHADOW).
- **Key writes.** All 8 scripted keys (`a s ␣ w ␣ w x q`) returned 1, each within 26–43 µs. For `x`, the write ran 3167.044239–3167.044272 (monotonic), and the node's estop fault and cancel came 1.255 ms after the write start.
- **States.** DISARMED → READY → (STARTING, WALKING, STOPPING, READY) twice → STARTING → FAULTED (estop).
  - The two refusals are `disarm` after the estop (reset required).
  - There were no command rejections and no tracking findings.
- **Margins:**
  - RTF 0.194, with 10 s windows of 0.172–0.214 (minimum 0.1);
  - node-observed age, maximum: joint states 0.128 s (limit 0.5) and body pose 0.086 s (limit 1.0);
  - the closest joint was 0.436 rad from its limit.
- **Hold.** 10.0 s of sim after the estop (from sim 71.6): joint motion 5.9e-12 rad, body motion 2.0e-10 m, z ≥ 0.054495 m, tilt 1.5e-7 rad.
- **Finding.** One startup `[ERROR]` line, "robot_state_publisher service not available, waiting again". gz_ros2_control retried once, 0.25 s before loading the controller manager. It was not seen in S3. The controllers came up normally.

## Stop and verification
- **Exits.** The simulator server exited 0 on SIGINT (strace). The launch logs `m55_locomotion_node` "finished cleanly". The launch group was empty after 4 s.
- **Cleanup.** No leftovers. The watchdog stood down without acting.
- **Restore** `--verify-only` against `7c1ecbc`: **DISABLED AND VERIFIED**. All three imported gates are `False`, and no enabling branch was created (`restore/`, `disabled_state_after.txt`).
- **Bag** (kept outside the repository): `/root/spiderx_evidence/s4a_shadow_20261101T150658Z/bag/bag_0.db3`, sha256 `3f237853…`, 146,292,736 bytes (`run/bag_hashes.txt`).
- **Media.** Two screenshots, both at rest: before the first key, and after the script. They are not proof of motion.
