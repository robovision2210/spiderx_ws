# S4e debugging exercises

Hands-on exercises for the S4e tools ([walkthrough](S4E_DESIGN_WALKTHROUGH.md),
[contract](S4E_CONTRACT.md)).
- **Offline:** none of them launches a simulator, enables anything or needs a robot.
- **Throwaway copies:** do them in a copy of the checkout, and never commit an exercise's change.

Setup for every exercise (a sourced installation, as the direct test runs use):
```bash
source /opt/ros/humble/setup.bash && source <install>/setup.bash
export SPIDERX_REPO=$PWD          # the checkout you are in
cd scripts/s4
```

## A. Terminal preflight (no ROS)
- **A1. See your own repeat.** In a real terminal:
  `python3 s4e_terminal_preflight.py --min-hold-s 3`.
  - Hold `w` for 4 s, wait 3 s, tap `w`, press Ctrl+D.
  - Read the first-repeat delay and the rate.
  - Why does the tool say "this record does not satisfy the S4e preflight"? (Hint: `--min-hold-s`.)
- **A2. Make it stutter.**
  - Run `xset r rate 900 30` (a delay above H). Repeat A1: the verdict must FAIL.
  - Restore your setting afterwards (`xset r rate 400 30`, or your own values from `xset q`).
- **A3. Break and repair the terminal restore.**
  - Comment out the `tcsetattr` line in `main`'s `finally` and run
    `python3 -m unittest -v test_s4e_terminal_preflight.ScriptedPty`.
  - Which tests fail, and why do the tests compare the slave's termios, not the master's?
- **A4. The release-then-tap trap.** In `test_s4e_terminal_preflight.Analysis`, write a test
  where the tap comes 1.0 s after the release.
  - Predict the verdict before running it.
  - Then explain why the operator is told to wait 3 s (`REPEAT_WINDOW_S`).

## B. Observer (no ROS needed for these)
- **B1. Prove it never sends a key.** Add `os.write(self.master, b'w')` anywhere in
  `s4e_observer.py` and run `python3 -m unittest test_s4e_observer.NeverSendsAKey`. Which
  assertion catches it?
- **B2. The quit cut.** In `RelayTests`, send `b'aqw'` as one chunk.
  - What reaches the "teleop" side?
  - What is recorded as `not_relayed`?
  - Which contract criterion needs this (H8)?
- **B3. A new stop condition.**
  - Add "goal duration above 3.0 s of sim" to `Monitor._status`, with a unit test in
    `MonitorTests`.
  - Run all observer tests.
  - Then decide: does the frozen contract allow it? (No: §3 lists the stops. A real change needs
    a contract revision first.)
- **B4. The spawn race.**
  - Remove the close-on-exec pipe wait from `spawn` and run `Sessions` several times.
  - Look for a `pty_holders` record whose slave holder has the observer's session id. Explain it.

## C. Scorer (bench sessions; real node and teleop logic in memory)
- **C1. Read a verdict.** Run `python3 -m unittest -v test_score_s4e.Nominal`. Then, in a Python
  shell:
  ```python
  import test_score_s4e as T
  ev, obs, bag = T.bench_session(True)
  r = T.scored(ev, obs, bag, True)
  print(T.V(r), r['overall'], r['H']['H4'])
  ```
  Find t_last, t_stop and the detection time. Why is it between 0.80 and 0.95 s on the bench?
- **C2. Flip a contradiction into a gap, and back.**
  - Remove every node event after K2's last character (see `Release`). Why is H4 UNMEASURED
    rather than FAIL?
  - Now change only the first transition's `why`. Why is that FAIL?
- **C3. The human-input rule.**
  - Set `obs[0]['input_source'] = 'scripted-pty-test'` and score.
  - Check that H3–H8 FAIL while `underlying_verdict` stays PASS. Why must the scorer keep both?
- **C4. Find the defect class we already hit.** The node logs result codes, not names.
  - Change H7 back to compare with `'CANCELED'` in a scratch copy and watch which tests fail.
  - This is the defect fixed after the first test run (see the evidence index).
- **C5. Extend, the right way.**
  - Draft H13, "the operator's first-repeat delay recorded in the preflight is below 0.6 s",
    as a contract revision.
  - Write `assess_*`, a passing bench test and two mutations (FAIL, UNMEASURED).
  - Do not edit H1–H12.

## D. Harness template (stubs only)
- **D1. The inherited-SIGINT pitfall.**
  - In `s4e_session.sh`, replace `start_owned` for the launch with a plain
    `setsid bash -c "exec $LAUNCH_CMD" &`.
  - Run `python3 -m unittest test_s4e_harness_template.Flows`. Why does cleanup now take about
    20 s and end with SIGKILL? (A background job of a non-interactive shell starts with SIGINT
    ignored.)
- **D2. Zombies.** Remove `alive_in` from `cleanup.sh` (use `pgrep -s` again) and run
  `Safety.test_an_interruption_stops_owned_processes_only_and_restores`. Explain the output.
- **D3. Owned only.** Start `bash -c 'exec -a "ign gazebo server" sleep 300' &` yourself, then run
  `Flows`. Confirm your process survives and is *listed* in `cleanup.log`. Then kill it.
- **D4. Add the missing piece.**
  - The template does not yet record the run installation's gates after the restore; the scorer
    needs that for H11. S4d-E had `run_install_gates.sh`, in
    `docs/evidence/m55/s4d_e_live_20261111T113831Z/tools/`.
  - Port it into the template and call it from `restore.sh`.
  - Add a stub test.
- **D5. Refusal order.** Which refusal wins when both the gates and the disk are wrong? Read
  `s4e_session.sh` and confirm with a test.

## E. Questions to answer in your own words
1. Why can no software check prove that a human typed the keys? What does the S4e evidence rely
   on instead (H1)?
2. Why is the release measured from the last character, and what would be wrong with measuring
   it from the next expected repeat?
3. Why does the observer use the node's `estop` service, and not the `x` key, as its emergency
   response?
4. What would a PASS **not** show? List five things (contract §8).
