# SpiderX integration readiness

> **NO MERGE AUTHORIZED.** This report proposes a merge order and the checks it needs. It merges, retargets,
> rebases and deletes nothing.

The owner has **not** performed the home-PC checks, so local validation remains **pending**. The project is not
complete. This report gives no completion percentage and no timeline.

## 1. What was inspected (2026-11-12, fetched from `origin`)
- **`origin/main`:** `f22ac9f763d402cfefa2c72b2042c0f2450f2bf1`, the merge of PR #17 on 2026-10-13.
- **Branches:** every remote branch (29), with heads, ancestry and merge-base against `main`.
- **Pull requests:** all of them, PRs #1–#27, open and closed, with their head and base refs.
- **Reviews and CI.** No reviews were found on the drafts that were sampled.
  - There is no CI: no `.github/` directory on `main` or on the stack, and 0 check runs or statuses on the drafts.
  - "CI state" below therefore means *none configured*.
- **Gates.** All three source gates are `False` at every stack head:
  - `M55_LOCOMOTION_DISPATCH_ENABLED`;
  - `M61_LIVE_DISPATCH_ENABLED`;
  - `LIVE_DISPATCH_ENABLED` (M6.0-D).
  `main` has the M61 and M6.0-D gates, both `False`. The M55 gate does not yet exist there.
- **Enabling branches.** No `local/*` branch exists locally or on `origin`.
- **Checks run for this report**, all read-only:
  - ancestry with `git merge-base --is-ancestor`;
  - textual-conflict detection with `git merge-tree --write-tree`, which changes no ref or worktree;
  - a relative-link check on `README.md`, `STATUS.md` and `docs/*.md` at the stack tip.
  Nothing was built, tested, launched or enabled.

## 2. The integration stack, in dependency order
Each branch's intended parent is the branch above it. Proven by ancestry:
- every head contains its parent, and the stack is 0 commits behind `main`;
- for every step, the merge result's tree is identical to the head's tree.

"Own" counts commits relative to the parent; descendant commits are not counted again.

| # | Milestone | Branch | Head (full SHA) | Intended parent | PR (state) | Verification evidence (cloud only) | Blocker |
|---|---|---|---|---|---|---|---|
| 0 | M1–M6.0-D, M6.1-A, M6.1 | `main` | `f22ac9f763d402cfefa2c72b2042c0f2450f2bf1` | — | merged: #4–#18 (#10 as the spawn-race v2) | per milestone, in `docs/` on `main` | — |
| 1 | M5.5 keyboard walking (SHADOW; gate `False`) | `claude/spiderx-m55-keyboard-walking` | `9c1a61bc7deaba464a85657f1665638b84e0e0e0` | `main` | [#26](https://github.com/robovision2210/spiderx_ws/pull/26) (draft, new 2026-11-12) | Suites 1894/0/0/0 and 1922/0/0/0 (S0b). Shadow session 2026-10-13, nothing sent. S2 attempt 2026-10-29 **FAIL** (1 goal, 1 cancel), with diagnosis | Owner review; home-PC validation |
| 2 | Simulated actuator model: pinned overlay, gain 1.0 | `claude/spiderx-sim-actuator-model` | `a0330417757c86fd0d3690555a29a836e90caccb` | 1 | [#19](https://github.com/robovision2210/spiderx_ws/pull/19) (draft) | Phase B launch-only **PASS**. S2 Phase C 2026-10-30, frozen **FAIL**. r1 retrospective assessment | Owner review; overlay on the home PC (untested) |
| 3 | S3 preparation and run; S2 accepted under r1 | `claude/spiderx-s3-preparation` | `579a2e5dc7d1cf4a3e2a916215714f0851848028` | 2 | [#20](https://github.com/robovision2210/spiderx_ws/pull/20) (draft) | S3 run 2026-11-01, frozen **INCONCLUSIVE** (A3 UNMEASURED), accepted under S3-r1. S2 accepted under r1 (owner, 2026-10-31) | Owner review |
| 4 | S4a, level-0 reverse and return | `claude/spiderx-s4a-preparation` | `29d5d641a13bdc67c07b8925ba2f446fff0044db` | 3 | [#21](https://github.com/robovision2210/spiderx_ws/pull/21) (draft) | Live 2026-11-01: **PASS** A1–A8 | Owner review |
| 5 | S4b, release, reversal, home, lease loss | `claude/spiderx-s4b-preparation` | `20a7c83c729982a97b330973124fa72c3dc6489f` | 4 | [#22](https://github.com/robovision2210/spiderx_ws/pull/22) (draft) | Live 2026-11-02: **PASS** B1–B11 | Owner review |
| 6 | S4c, internal levels 1 and 2 | `claude/spiderx-s4c-preparation` | `40f5f2cceb111241c051a1c63a8e6dea9e240d88` | 5 | [#23](https://github.com/robovision2210/spiderx_ws/pull/23) (draft) | Level 1 (2026-11-02) and level 2 (2026-11-03): **PASS** C1–C11 each | Owner review |
| 7 | S4d plan draft (documentation only) | `claude/spiderx-s4d-plan` | `34396cfaae1841ad20e29b014beda454da408b0a` | 6 | [#27](https://github.com/robovision2210/spiderx_ws/pull/27) (draft, new 2026-11-12) | — | Owner review |
| 8 | S4d-M at internal level 2 | `claude/spiderx-s4d-preparation` | `0257980eab95b90436c848746b017e37c2e8c3a8` | 7 | [#24](https://github.com/robovision2210/spiderx_ws/pull/24) (draft) | Fresh disabled 1983/0/0/0. Shadow PASS (shadow only). Live **PASS** M1–M16; see §5 | Owner review; home-PC validation |
| 9 | S4d-E, the level-2 estop | `claude/spiderx-s4d-e-preparation` | `c770e83a91b0b6691bdbe00a198e4a9db7df9924`, plus this report's commit | 8 | [#25](https://github.com/robovision2210/spiderx_ws/pull/25) (draft) | Fresh disabled 1993/0/0/0 at `87198b9`. Shadow PASS (shadow only). Live **PASS** E1–E12 under the frozen contract; see §5 | Owner review; home-PC validation |

Own commits per layer, in order: 34 (24 M5.5 commits plus 10 merges that brought in M6.1-A and M6.1 work, all already
in `main`), 12, 13, 6, 6, 12, 1, 14 and 9.

From `main` to the stack tip: 2451 files, +133,557/−57. Of those, 2363 files are evidence under `docs/evidence/` and
88 are code, tests, tooling and documentation.

## 3. Classification
- **Already in `main`:**
  - M1, M2, M3, M4 and M4.1 (the controller spawn-race fix, merged as v2 in #10);
  - M4.5 and M5;
  - M6.0 with its graph preflight;
  - the M6.0-D implementation (#15) and its live-enabling design (#16);
  - M6.1-A (#17) and the M6.1 trot replay (#18).
  The M6.1 merges inside the M5.5 branch are already in `main`.
- **Carried by descendants.** Each layer is contained in every layer above it. The S4d-E head carries the whole stack.
- **Awaiting an existing PR:** #19–#25, all drafts.
- **Missing a PR, created now** (base and scope unambiguous; draft; no merge authorized):
  - **[#26](https://github.com/robovision2210/spiderx_ws/pull/26), M5.5 → `main`.** The M5.5 PR had been deferred "until
    M6.1-A is reviewed", and M6.1-A merged as #17.
  - **[#27](https://github.com/robovision2210/spiderx_ws/pull/27), S4d plan → S4c.** It is the base of #24.
- **Missing a PR, owner decision needed** (left unchanged):
  - **`claude/spiderx-engineering-console-bmtwo3`** (`d68c647`): 2 commits on `main`, only `tools/engineering_console/`.
    - Its content describes `claude/spiderx-s4d-preparation` at `d4fe748`, which is no longer that branch's head.
    - It does not conflict with `main` or with the stack tip.
    - **Decide:** whether the tool belongs in this repository; its base (`main`, or after the stack); and whether to
      refresh its content first.
  - **`claude/spiderx-m6d-enable-gate`** (`606d229`): 3 commits on `77fd171`, 30 commits behind `main`.
    - It holds `docs/M6D_FIRST_GOAL_RESULTS.md`, which is absent from `main`.
    - Its history includes a pushed M6.0-D enabling commit and the later re-disable; the gate is `False` at its tip.
    - `main`'s documents already cite it as deliberately unmerged.
    - It does not conflict with `main`.
    - **Decide:** keep it as an external record (the status quo), or integrate only the results document in a separate,
      reviewed change.
- **Stale or superseded; owner decision; no action taken:**
  - `claude/spiderx-controller-spawn-race` (v1): superseded by v2, merged in #10. It conflicts with `main` in 4 files.
  - `claude/spiderx-m6d-live-playback-plan`: an older draft of `docs/M6D_LIVE_PLAYBACK_PLAN.md`. `main` has a later
    version, and the branch conflicts in that file.
  - Open drafts [#1](https://github.com/robovision2210/spiderx_ws/pull/1) and
    [#2](https://github.com/robovision2210/spiderx_ws/pull/2), the Copilot scaffolds of 2026-09-24.
    - They are 147 commits behind `main` and conflict with it in 24 and 27 files.
    - They predate the merged Fortress path (#4) and the architecture alignment (#5).
    - **Decide:** close them or keep them.
- **Blocked by owner decisions or local validation:** every merge, since none is authorized; the home-PC validation
  (§7); and the decisions above.

## 4. Integration risks
### Proven
- **No conflict inside the stack.** It is linear. Every merge in §6 is a merge whose result tree equals the PR head's
  tree, which was checked for all nine steps.
- **Textual conflicts exist only on the stale side branches** of §3.
- **The descriptions of #19–#23 predated evidence that now sits on their own heads.**
  - #19 said Phase C had not run; #20–#23 said nothing had run.
  - On 2026-11-12 each got a dated status note at the top. Their original text is kept below the note.

### Untested (each needs a check; none is a proven defect)
- **Runtime enters `main` for the first time:**
  - **[#26](https://github.com/robovision2210/spiderx_ws/pull/26), M5.5.**
    - A new node, teleop, launch and config.
    - The new gate defaults to `False`, which means SHADOW mode.
    - `package.xml` adds `exec_depend` on `std_srvs` and `geometry_msgs`; rosdep must resolve them on the home PC.
  - **[#19](https://github.com/robovision2210/spiderx_ws/pull/19), the simulated plant.**
    - It changes `spiderx_fortress_control.xacro`, the controlled-Fortress xacro shared by every launch with
      `enable_control:=true`.
    - The xacro gains `gz_ros2_control_plant.yaml`, which sets `position_proportional_gain: 1.0`.
    - **Only the patched overlay applies that gain.** It is pinned in `third_party/gz_ros2_control_overlay/`, and
      `build_overlay.sh` targets the RoboStack toolchain.
    - Stock gz_ros2_control 0.7.15 silently keeps 0.1. A home PC without an equivalent overlay therefore simulates a
      different plant from every cloud run since S2 Phase C.
  - **[#20](https://github.com/robovision2210/spiderx_ws/pull/20).** The node-shutdown backport in `m55_rclpy.py`, and
    overlay patch 0002.
- **Gate defaults.** `False` everywhere.
  - The M5.5 enabling patch exists only as `docs/patches/m55_enable_s2.UNAPPLIED.patch`.
  - Every live run used a local enabling branch that was never pushed and was deleted after verification.
- **Test discovery.**
  - The stack registers new colcon tests: `test_m55_*`, `test_sim_actuator_model`, `test_m55_rclpy_shutdown`, and more
    restore-script tests. 1993 tests were collected at `87198b9`, in the cloud. `main`'s count was not measured here.
  - The `scripts/s2`, `scripts/s3` and `scripts/s4` tests (48, 86 and 252) are **not** collected by colcon and must be
    run directly.
  - There is no CI to run either set.
- **Cloud-only paths and toolchain.**
  - The verification used RoboStack Humble (`/opt/mm`, Python 3.11) and the r2 overlay at
    `/root/spiderx_gz_overlay_ws_r2`.
  - `scripts/env/cloud_env_setup.sh` is cloud-specific.
  - The run-local harness copies under `docs/evidence/*/tools/` hard-code `/root/...` paths. They are evidence, not
    runtime.
  - The home PC (`/opt/ros/humble`, Python 3.10, apt gz_ros2_control of unknown version) has not been tested.
- **Repository size.** The stack adds 2363 evidence files. The `.git` pack is about 12 MB.
  - No rosbag is in Git; the evidence records only their hashes.
  - The raw bags were delivered as conversation attachments. No durable copy has been confirmed.
- **Intermediate states.** Each layer's `README.md` and `STATUS.md` describe the project as of that layer's date.
  Merging only part of the stack leaves `main` with that historical wording until later layers land.
- **Evidence links.** 407 relative links in `README.md`, `STATUS.md` and `docs/*.md` resolve at the stack tip,
  checked 2026-11-12.

## 5. Claims that must survive integration unchanged
- **S2.** The frozen FAIL verdicts stand: the 2026-10-29 attempt, and Phase C on 2026-10-30. S2 was accepted
  retrospectively under criteria revision r1 (owner, 2026-10-31).
- **S3.** The frozen INCONCLUSIVE verdict stands, with A3 UNMEASURED. S3 was accepted retrospectively under S3-r1: six
  level-0 forward cycles, about 0.1201 m.
- **S4a, S4b and S4c.** PASS in cloud simulation, with automated input through the keyboard-teleop path.
- **S4d-M.** A qualified cloud-simulation PASS (M1–M16): one 111-goal attempt with zero cancels, about 0.3 m forward
  and back (+0.30006 m and −0.29985 m).
  - **The simulator server was killed by SIGKILL during shutdown**, after the script. It was not graceful, and the cause
    is not established ([postmortem](evidence/m55/s4d_live_20261111T052817Z_shutdown_postmortem_20261111T101720Z/README.md)).
- **S4d-E.** PASS under the frozen contract (E1–E12).
  - **The one active goal was canceled about 0.04 s of sim after it started, before measurable motion.** The run shows
    the estop, cancel and hold path, not a moving body being stopped.
  - Tracking rests on one feedback sample
    ([evidence](evidence/m55/s4d_e_live_20261111T113831Z/README.md)).
- **Not validated:** human keyboard operation (S4e), hardware and navigation. Home-PC validation is pending.

## 6. Proposed merge order (NO MERGE AUTHORIZED)
Merge bottom-up, one PR at a time, each into the branch it now targets:

1. [#26](https://github.com/robovision2210/spiderx_ws/pull/26) M5.5 → `main`
2. [#19](https://github.com/robovision2210/spiderx_ws/pull/19) simulated actuator model
3. [#20](https://github.com/robovision2210/spiderx_ws/pull/20) S3, and S2 under r1
4. [#21](https://github.com/robovision2210/spiderx_ws/pull/21) S4a
5. [#22](https://github.com/robovision2210/spiderx_ws/pull/22) S4b
6. [#23](https://github.com/robovision2210/spiderx_ws/pull/23) S4c
7. [#27](https://github.com/robovision2210/spiderx_ws/pull/27) S4d plan
8. [#24](https://github.com/robovision2210/spiderx_ws/pull/24) S4d-M
9. [#25](https://github.com/robovision2210/spiderx_ws/pull/25) S4d-E

**Notes on the mechanics:**
- **Retargeting.** After a PR merges into `main`, the next PR's base must be moved to `main` before it merges. That is a
  retarget, which this batch does not perform; it needs the owner, or a later authorized batch.
- **Merge method.** Use **merge commits**, not squash or rebase. The evidence records cite commit SHAs, such as
  `8f99bcc`, `87198b9` and `01ed29e` in restore reports. Squashing would keep those commits out of `main`'s history.
- **One window.** Merge the whole stack in one window, to avoid the historical-status intermediates of §4.
- **Side branches** (§3) are outside this order, pending owner decisions.

## 7. Checks
**Before each merge** (cloud):
1. The PR's head and base equal the SHAs in §2. If either moved, re-review.
2. The merge preview's tree equals the head's tree (`git merge-tree --write-tree`). Otherwise stop.
3. All three gates are `False` in the result, and there is no `local/*` ref.
4. The owner has reviewed the diff. Pay particular attention to #26 (runtime), #19 (plant and overlay) and #20
   (shutdown).
5. A test run, depending on what changed since the last record:
   - The executable inputs have not changed since `87198b9` (documentation and evidence only), so the recorded
     1993-test disabled result applies at the stack tip.
   - Any code change requires a fresh disabled build and full suite in new bases, plus the direct `scripts/` tests.

**Pending local validation** (home PC, per [the handoff](S4D_LOCAL_HANDOFF.md); **not done**):
- the toolchain and apt gz_ros2_control version;
- an equivalent overlay, or an explicit decision to run at gain 0.1;
- a disabled build with the full colcon suite and the direct tests;
- gate imports and both M6 `--live` refusals;
- library identity (`62f8a100…`, `library.json` `917fc4d7…`).
Any simulation or motion there needs its own authorization.

**After integration** (cloud, then home PC):
- a fresh clone of the final `main`;
- a fresh disabled build in new bases, the full colcon suite and the direct tests;
- gates `False` and both M6 `--live` refusals;
- library identity;
- the link check;
- `README.md` and `STATUS.md` read as the stack tip's qualified wording.

## 8. Rollback, without destructive resets
- **Undo a merge** with `git revert -m 1 <merge commit>` on `main`. That adds a new commit.
  - Revert in reverse merge order.
  - Never reset or force-push `main`.
  - The branches and evidence stay intact, so a reverted layer can be re-merged later. Re-merging needs a revert of the
    revert.
- **Record a pre-merge marker.** Before merging, record `main`'s SHA, currently `f22ac9f`, in the merge batch's report.
  Creating a tag is a separate decision.

## 9. The smallest next batch that can still be done in the cloud
- **Without merge authorization:** nothing substantive remains in the cloud.
  - The stack is verified: tree equality, gates, and the 1993-test result at the unchanged executable inputs.
  - What is left is owner decisions (§3), owner review of #19–#27, and the home-PC validation.
- **With an explicit merge authorization for #26 only:**
  1. merge #26 into `main` with a merge commit;
  2. run the post-integration checks of §7 on `main`, in the cloud;
  3. report;
  4. stop before #19.

Each later layer would repeat that pattern, after the retarget in §6.

**NO MERGE AUTHORIZED.**
