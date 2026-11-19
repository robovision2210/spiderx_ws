# S4e: the human step — operator package (NOTHING HERE IS AUTHORIZED TO RUN)

Prepared 2026-11-19, offline in Cloud, for [`S4E_CONTRACT.md`](S4E_CONTRACT.md).
- Nothing was launched.
- **Home-PC validation is still pending.**
- Each step below needs its own owner message, in the order of the contract's §11.

## 1. Can this cloud session pass your real keystrokes? No.
- **Chat messages carry no key timing.** This session runs in a cloud container. Your messages
  reach it whole, after you send them: no key press, key release, hold or auto-repeat timing
  arrives, and no terminal of yours is attached to the container.
- **Relaying them would be scripted input.** Typing chat text into the teleop for you would be
  scripted pseudo-terminal input. That is what S4a–S4d already did, and the contract says it can
  never satisfy a human-input criterion (§8, H1). It is not offered as S4e.
- **The container cannot reach your computer.** A link to your computer would still need you at a
  terminal on that computer. That is the home-PC step below, not this session.

## 2. The smallest honest alternative
Run the human step on your own Ubuntu PC. This is already the S4 plan's S4e row ("owner's PC")
and owner decision D2. The order:
1. the pending home-PC validation (its own authorization);
2. on your PC, the terminal preflight P1 (no ROS, nothing launched, but still home-PC work, so it
   waits for your message like the rest);
3. one human-operated **shadow** rehearsal (§4);
4. after its review, one bounded **live** attempt (§5).

This session can prepare tools and documents, and score the evidence you send back, offline.

## 3. Which terminal, and what to avoid
- **Use** a terminal emulator on your PC's own desktop (GNOME Terminal, Konsole or xterm), typing
  on its physical keyboard. Use the **same window** for the preflight and the session.
- **Do not use:**
  - SSH: the client machine generates the repeats, and network delay adds jitter;
  - tmux or screen: they add another relay;
  - a remote desktop.
  - Do not type anything else in that window (no passwords): its keys are recorded (contract §6).
- **Your repeat settings** are recorded by P1 when it can read them: `xset q`, or
  `gsettings get org.gnome.desktop.peripherals.keyboard delay` and `repeat-interval`.
- **If P1 reports a slow first repeat:** shorten the delay, for example `xset r rate 400 30`
  (D7). Do not change `teleop.hold_s`.

## 4. Shadow rehearsal — exact steps (NOT AUTHORIZED)
All three gates `False`; nothing is enabled and nothing moves. `<install>` is the installation you
validated on the home PC, and `<sha>` is its commit.

**Terminal A (the harness), in a run-local copy outside the repository:**
```bash
mkdir -p ~/s4e_run/session && cp -r ~/spiderx_ws/scripts/s4/s4e_harness_template ~/s4e_run/h
export S4E_MODE=shadow WS=~/spiderx_ws RUNINST=<install> BASELINE=<sha> \
       OLD_INSTALL=<an installation that must not be sourced>
source /opt/ros/humble/setup.bash && source <install>/setup.bash
bash ~/s4e_run/h/watchdog.sh ~/s4e_run/session/run &        # armed first: 3600 s
bash ~/s4e_run/h/guarded.sh ~/s4e_run/session ~/s4e_bags/shadow
```

**Terminal B (yours; the physical keyboard).** Before you `go`, run the preflight:
```bash
source /opt/ros/humble/setup.bash && source <install>/setup.bash
python3 ~/spiderx_ws/scripts/s4/s4e_terminal_preflight.py --out ~/s4e_run/session/preflight.json
```
1. Hold `w` for 10–20 s by a clock and release.
2. Wait 3 s, then tap `w` once.
3. Press Ctrl+D.

Continue only on `VERDICT (...): PASS`.

At **PRE-KEY CHECKPOINT** in terminal A, check the simulation as in the earlier stages (settled,
SHADOW, gate `False`, ownership). Then, in a third terminal:
```bash
echo "pre-key checks passed" > ~/s4e_run/session/run/go.flag
```
To stop instead: `echo "<reason>" > ~/s4e_run/session/run/abort.flag`.

**Terminal B, the session:**
```bash
python3 ~/spiderx_ws/scripts/s4/s4e_observer.py --out ~/s4e_run/session/observer --mode shadow
```
Then the keys of contract §4.2:

| Step | Do this |
|---|---|
| K1 | `a`; wait for `state READY` |
| K2 | hold `w` 10–20 s by a clock, release; wait for READY |
| K3 | hold `w`; after about 3 s press space (keep holding); hold until READY plus about 2 s; release |
| K4 | `h`; wait for READY |
| K5 | hold `w`; after about 2 s press `x` (still holding); release; wait for the `[observer] hold complete` line |
| K6 | `q` |

- The observer exits by itself.
- Then write the attestation:
  ```bash
  cat > ~/s4e_run/session/attestation.txt <<'EOF'
  I, <initials>, operated the S4e session on <date> by pressing keys on a physical keyboard
  myself. No script, macro, key remapper or other program generated any key.
  EOF
  ```

**How to exit early (any time):**
- Moving: `x`. Not moving: `q`.
- In terminal A, Ctrl+C stops the harness; the cleanup and the verify-only restore run on exit.
- Nothing is retried.

## 5. Live attempt — summary (NOT AUTHORIZED; only after the shadow is reviewed)
**Before the launch:**
- On a local branch `local/m55-s4e-h-<UTC>`, never pushed: exactly the two-line M55 patch
  (`m55_contract.py` and `test/m55_gate.py`), as every earlier stage.
- A fresh enabled build in new colcon bases; the branch name in `~/s4e_run/session/enabling_branch.txt`.
- The harness with `S4E_MODE=live`.

**The session:** the same steps as §4, with `--mode live`.

**Restoration (by `guarded.sh` on any exit):**
- `restore.sh` runs the full restore with `--enabling-ref <branch> --delete-enabling-branch`,
  which deletes the branch only after every check passed and only if it was never pushed;
- DISABLED AND VERIFIED; both installations import all three gates `False`.

## 6. What to send back (keep the bag outside the repository)
- `~/s4e_run/session/` as one archive:
  - `preflight.json`, `attestation.txt`;
  - `observer/observer.jsonl`, `observer/observer_summary.json`;
  - `run/` (steps, node evidence, captures, scans, cleanup, restore logs).
- The bag's `metadata.yaml` and its sha256. The bag itself only if asked.
- Your `xset q` output, and anything unexpected you saw on the screen.

The scorer then runs offline in Cloud:
```bash
python3 scripts/s4/score_s4e.py --session <session> --bag <bag_0.db3> \
  --disabled-library docs/evidence/m55/s4c_l1_live_20261102T182303Z/disabled/library.json \
  --restore-report <session>/restore/restore_verify_<UTC>.txt \
  --run-install-gates <json> --out <new dir> [--shadow]
```

## 7. Draft authorization (a): human-operated shadow rehearsal — NOT AUTHORIZED
> AUTHORIZED: ONE human-operated S4e SHADOW rehearsal on my PC under docs/S4E_CONTRACT.md §5.
> - **Baseline:** main `<sha>` with the S4e tools at `<commit>`; home-PC validation PASSED on
>   `<date>` (`<record>`); all three gates `False`; no enabling.
> - **Budgets:** one launch; a session cap of 20 min; a watchdog of 3600 s with at least 35 min
>   left before the launch and before the teleop; the disk rule; at most 30 recorded shadow goals
>   and one modeled cancel.
> - **Abort rules:** contract §4.3. `x` if moving, otherwise `q`; no retry or relaunch.
> - **Restoration:** `restore.sh --verify-only`; both installations import all three gates
>   `False`.
> - **Not authorized:** live, enabling, any real goal, a second launch, hardware.

## 8. Draft authorization (b): one bounded live attempt — NOT AUTHORIZED
> AUTHORIZED: ONE human-operated S4e LIVE attempt on my PC under docs/S4E_CONTRACT.md §4, at
> internal level 0, after the shadow rehearsal `<record>` was reviewed.
> - **Enabling:** from a local `local/m55-s4e-h-<UTC>` enabling branch: exactly the two-line M55
>   patch, never pushed.
> - **Baseline:** `<sha>`; disabled library `917fc4d7…`; config `ab0dccd4…`.
> - **Budgets:** one launch; at most 30 goals; exactly one cancel (my `x`); travel 0.15 m; a
>   session cap of 20 min; a watchdog of 3600 s with at least 35 min left before the launch and
>   before the teleop; the disk rule.
> - **Stops:** contract §3.
> - **Abort rules:** §4.3. No retry, relaunch, reset or re-arm.
> - **Restoration:** `restore.sh` with the recorded branch and `--delete-enabling-branch`;
>   DISABLED AND VERIFIED; both installations import all three gates `False`.
> - **Scoring:** `score_s4e.py`, without `--shadow`.
> - **Not authorized:** a second attempt, another level, reverse, hardware, a merge.

## 9. What a PASS would and would not mean
See contract §8: one person, one keyboard, one session, level 0, in simulation.
- **Not:** hardware, navigation, autonomous or unrestricted walking, robustness, levels 1–2, or
  full S4 acceptance.
- **Never:** "human-operated" without both the H1 evidence and the attestation.
