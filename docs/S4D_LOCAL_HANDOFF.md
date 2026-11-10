# S4d local handoff: inspecting and validating on the home Ubuntu PC

**Documentation only.** This file was committed after the tested executable SHA
`8f99bcc4ab27294265da91a31ce03999423acc7f`; its commit changes no code, test, configuration or
acceptance verdict. Nothing in it has been run on your PC. Every result quoted below is a
**cloud** result unless it says otherwise.

## 0. Where things stand

**Branch and PR stack** (each draft PR targets the branch below it; none is merged into `main`):

| Branch | Head | PR |
|---|---|---|
| `claude/spiderx-s4d-preparation` | this file's commit; parent `8f99bcc` (tested executable) | #24 (draft) → `claude/spiderx-s4d-plan` |
| `claude/spiderx-s4d-plan` | `34396cf` | no PR; on top of `claude/spiderx-s4c-preparation` |
| `claude/spiderx-s4c-preparation` | `40f5f2c` | #23 → s4b |
| `claude/spiderx-s4b-preparation` | `20a7c83` | #22 → s4a |
| `claude/spiderx-s4a-preparation` | `29d5d64` | #21 → s3 |
| `claude/spiderx-s3-preparation` | `579a2e5` | #20 → sim-actuator-model |
| `claude/spiderx-sim-actuator-model` | `a033041` | #19 → `claude/spiderx-m55-keyboard-walking` (`9c1a61b`, no PR; 34 commits ahead of `main`) |

**Cloud verification (RoboStack Humble, Python 3.11, the r2 `gz_ros2_control` overlay):**
- Fresh **disabled** build of `8f99bcc` in new colcon bases: 8 packages; **1983 tests, 0 errors,
  0 failures, 0 skipped** (1933 xunit + 50 CTest). Direct tests: `scripts/s4` 206 OK,
  `scripts/s3` 86 OK, `scripts/s2` 48 OK (3 skipped: they need `S2_BAG`), `src/spiderx_scripts/test` 2 passed.
- Both installations imported all three gates `False`; both M6 `--live` tools refused (exit 3).
- One S4d-M **shadow** rehearsal on 2026-11-10 (dispatch disabled): **PASS (SHADOW ONLY: not S4d
  acceptance)**.
  - M1–M10 and M12–M16 PASS; M11 N/A.
  - The 111 planned shadow goals, 0 cancels, the close while moving (`lease_expired`) and the reconnect probe.
  - The real controller received **0** goals, cancels and feedback.
  - Verify-only restore: DISABLED AND VERIFIED.
  - The body does not move in shadow, so no distance, cycle band or drift is demonstrated.
  - Its evidence is **uncommitted** in the cloud workspace, pending your review, so it is not in the branch you fetch.
- **Not done anywhere:** a live S4d-M attempt, S4d-E, or any enabling. All three gates are `False`
  on every pushed commit.

**Local verification status: none for S4d.** The last local verification you reported was M6.0-D
(offline, mock and isolated-domain only), on `claude/spiderx-m6d-live-playback` at `412eb45`
(`docs/M6D_LIVE_PLAYBACK_RESULTS.md` §12).

**Cloud-only, never copy to your PC:** `/opt/mm` (RoboStack toolchain), `/opt/mm/st59site`,
`/root/spiderx_gz_overlay_ws_r2` (overlay built with conda compilers), and any `/root/...` or
`/home/user/...` path in evidence files. Your PC uses `/opt/ros/humble` and your own clone.

Below, `WS` is your clone (for example `~/spiderx_ws`). Run each block in a **new terminal**.

## A. Read-only inspection (first; changes nothing)

```bash
WS=~/spiderx_ws; cd "$WS"
git status --short --branch          # any line besides the branch line = local changes: stop at B
git stash list; git worktree list; git branch -vv | head -20; git log --oneline -3
git remote -v
ls -d build install log build_* install_* 2>/dev/null; du -sh build install log 2>/dev/null; df -h "$WS"
grep -n -E "source .*(setup|activate)|ROS_" ~/.bashrc       # an old workspace sourced automatically?
printenv | grep -E "^(ROS_DISTRO|AMENT_PREFIX_PATH|COLCON_PREFIX_PATH|PYTHONPATH|ROS_DOMAIN_ID|ROS_LOCALHOST_ONLY|RMW_IMPLEMENTATION)="
pgrep -af "gz sim|ign gazebo|ros2 launch|controller_manager|m55_|teleop|ros2 bag" || echo "no ROS/sim processes"
python3 --version; python3 -m pytest --version
dpkg -l | grep -E "ros-humble-(gz-ros2-control|ros-gz-sim|controller-manager|joint-trajectory-controller) " | awk '{print $2, $3}'
```

Gates as your **current** installation imports them (read-only; a subshell, so this terminal stays clean):

```bash
( source /opt/ros/humble/setup.bash && source "$WS/install/setup.bash" && cd / && python3 - <<'EOF'
import importlib, os
for mod, name in (("m61_live_contract", "M61_LIVE_DISPATCH_ENABLED"),
                  ("m6_live_contract", "LIVE_DISPATCH_ENABLED"),
                  ("m55_contract", "M55_LOCOMOTION_DISPATCH_ENABLED")):
    m = importlib.import_module("spiderx_controller." + mod)
    print(name, getattr(m, name), m.__file__, "->", os.path.realpath(m.__file__))
EOF
)
```

Expected: three lines ending in `False`, each imported from `$WS/install/...` (Python 3.10 on
Ubuntu 22.04, so `lib/python3.10/site-packages`), resolving into `$WS/src/...`. If any value is
`True`, or a path points at another workspace, **stop** and report it; do not fix it in place.

## B. Safe synchronization (only after A, only with a clean worktree)

Stop instead of continuing if A showed local changes, stashes you do not recognise, or a running
simulation. Never `git reset --hard`, `git checkout -- .`, `git clean`, `git stash` or a force
push to make this work.

```bash
cd "$WS"
git fetch origin claude/spiderx-s4d-preparation       # changes no file in the worktree
git log --oneline -3 origin/claude/spiderx-s4d-preparation   # the top commit adds this file; its parent is 8f99bcc
git rev-parse origin/claude/spiderx-s4d-preparation   # compare with the SHA reported to you
```

Then **one** of:
- the branch is new locally: `git switch -c claude/spiderx-s4d-preparation --track origin/claude/spiderx-s4d-preparation`;
- it exists locally: `git switch claude/spiderx-s4d-preparation && git merge --ff-only origin/claude/spiderx-s4d-preparation`
  (if it refuses, the branches diverged: stop and report);
- you prefer not to touch the existing checkout: `git worktree add ../spiderx_ws_s4d origin/claude/spiderx-s4d-preparation`
  (a new directory; it must not exist yet) and use it as `WS` below.

Check: `git status --short` prints nothing, and
`git grep -n -E "^(M61_LIVE_DISPATCH_ENABLED|LIVE_DISPATCH_ENABLED|M55_LOCOMOTION_DISPATCH_ENABLED)\b" -- src/`
prints exactly three lines ending in `= False`
(`m55_contract.py:19`, `m61_live_contract.py:19`, `m6_live_contract.py:18`).

## C. Disabled local validation (new colcon bases; nothing deleted)

The existing `build/`, `install/` and `log/` stay untouched. The new bases are outside the
repository and must not exist yet. `--symlink-install` links them to `$WS/src`, so keep the
worktree at this commit while you use them.

```bash
source /opt/ros/humble/setup.bash                     # nothing else sourced in this terminal
cd "$WS"; B=~/spiderx_s4d_local_$(date -u +%Y%m%dT%H%M%SZ); test ! -e "$B" && echo "new base $B"
export ROS_LOCALHOST_ONLY=1 ROS_DOMAIN_ID=231 PYTHONDONTWRITEBYTECODE=1
colcon --log-base "$B/log" build --build-base "$B/build" --install-base "$B/install" --symlink-install
source "$B/install/setup.bash"
colcon --log-base "$B/log" test --build-base "$B/build" --install-base "$B/install" --test-result-base "$B/build"
colcon --log-base "$B/log" test-result --test-result-base "$B/build" --verbose | tail -1
```

Expected: 8 packages built; the cloud summary was `1983 tests, 0 errors, 0 failures, 0 skipped`.
Record your actual counts; a different total or any skip needs an explanation, not a forced match.

Not collected by `colcon test` (run from `$WS`, with `$B/install` still sourced):

```bash
(cd src/spiderx_scripts && python3 -m pytest -q -p no:cacheprovider test)      # 2 passed
(cd scripts/s2 && python3 -m unittest discover -s . -p 'test_*.py')            # 48, OK (skipped=3: S2_BAG)
(cd scripts/s3 && python3 -m unittest discover -s . -p 'test_*.py')            # 86, OK
(cd scripts/s4 && python3 -m unittest discover -s . -p 'test_*.py')            # 206, OK
```

Import paths and gates from the new installation: rerun the gate block of A with
`source "$B/install/setup.bash"` instead of `$WS/install`. Expected: three `False`, imported from
`$B/install/...`, resolving into `$WS/src/...`. **Only after that**, the two documented refusals:

```bash
ros2 run spiderx_controller m6_gait_replay.py --live --domain-id 231 < /dev/null; echo "exit $?"   # HARD-DISABLED, exit 3
ros2 run spiderx_controller m6_live_playback --live --domain-id 231 < /dev/null; echo "exit $?"    # HARD-DISABLED, exit 3
python3 docs/evidence/m55/s2_fixed_base_20261029T100210Z/tools/library_fingerprint.py
# {"goals": 54, "fingerprints_sha256": "62f8a10075e90571bfc42afb4635eddd235d8681f589f027734fa5ca2c03e732", "m55_gate_imported": false}
```

This validates the disabled code and tests on your toolchain. It does not validate simulation
behaviour, the plant or timing on your PC.

## D. Future runtime work (each needs its own authorization)

- Any local simulation launch, shadow or live. **Plant difference:** the cloud S2–S4 evidence used
  the r2 overlay (`third_party/gz_ros2_control_overlay/`: gain 1.0, shutdown backport).
  - Stock 0.7.15 ignores `gz_ros2_control_plant.yaml` and runs with gain 0.1. Per the overlay README,
    the Humble head (0.7.21, checked 2026-10-29) still creates its node the same way.
  - Check which version your apt packages provide (A), and do not assume the cloud plant.
  - `build_overlay.sh` targets the RoboStack toolchain. An equivalent overlay for `/opt/ros/humble`
    is a separate, reviewed step.
- The S4d-M **live** attempt (`docs/S4D_ACCEPTANCE_PLAN.md` §13–§14), S4d-E (OD10), and S4e human
  keyboard work (OD9).
- Any enabling change. Enabling branches are local only (`local/m55-s4d-m-*`) and never pushed;
  `scripts/m61_restore_and_verify_disabled.sh` restores and verifies afterwards.

## Common environment failures

| Symptom | Likely cause | What to do |
|---|---|---|
| Gates imported from an unexpected path | another workspace sourced (often `~/.bashrc`) | new terminal; source only `/opt/ros/humble` and the intended install |
| Every pytest file fails at start-up with a `launch_testing` plugin error | pytest plugin autoload (the cloud needed this with pytest 9) | rerun `colcon test` with `PYTEST_DISABLE_PLUGIN_AUTOLOAD=1` |
| `ModuleNotFoundError: spiderx_controller` in direct tests | the new installation is not sourced | `source "$B/install/setup.bash"` |
| `rosdep`/apt reports missing packages | dependency not installed | your decision; do not install during inspection |
| A test talks to another ROS graph | default domain, not localhost-only | `ROS_LOCALHOST_ONLY=1`, unused `ROS_DOMAIN_ID` |
| Paths with `python3.11`, `/opt/mm` or `/root` in instructions | cloud-specific | translate to `python3.10`, `/opt/ros/humble`, your `WS`; never copy |

Report back: A's output, the SHA you synchronized to, C's counts and skips, the gate lines and
the two refusals. A live S4d-M attempt still needs a separate review and authorization.
