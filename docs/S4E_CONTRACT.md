# S4e: one human-operated keyboard session in simulation — frozen contract

**FROZEN 2026-11-19, before any S4e observer, scorer, harness or test was written.** Branch
`spiderx/s4e-human-keyboard`, from `main` at `e7ffd207dbff6a871c6c522713ced1bddf4df0b7` (tree
`e11d4655e72d4395b6be358fab34236bfa59a563`, the approved stack tip's tree).

**Authorized (owner, 2026-11-19), offline only:**
- the S4e tooling: a terminal preflight, an observer for human input, a scorer, their tests, a
  run-local harness template with stubs and the narrow `local/m55-s4e-h-*` restore support;
- one fresh disabled build and full suite;
- documentation, evidence and status commits on this branch, and one draft PR.

**Not authorized:** any simulator launch (shadow or live), enabling patch or branch, controller
goal or cancel, motion, hardware, home-PC work, push to `main`, merge, force push, branch deletion,
dependency installation or settings change. Sections 4–6 describe sessions that are **NOT
AUTHORIZED TO RUN**; each needs its own owner message (§11).

This contract changes no runtime code, gait, plant, gain, phase library, configuration, historical
driver, scorer or tool, and no earlier frozen criterion or verdict. The only change to an existing
file is one narrow restore pattern (§9).

## 0. Baseline: verified and not verified
**VERIFIED in Cloud, 2026-11-19** (the post-merge validation report, kept outside the repository,
sha256 `aaef40bf…`; summarized in this branch's evidence index):
- `main` = `e7ffd20`; its tree equals the approved tip `1e11464`'s tree; merge order #26, #19, #20,
  #21, #22, #23, #27, #24, #25; the validated executable `87198b9` is an ancestor;
- a fresh disabled build in new bases; the collected suite 1993 tests (1943 xunit + 50 CTest),
  0 errors, 0 failures, 0 skips; the direct tests `spiderx_scripts` 2 passed, `scripts/s2` 48 OK
  with the 3 known `S2_BAG` skips, `scripts/s3` 86 OK, `scripts/s4` 252 OK;
- installed imports from the fresh installation, all three gates `False`, both M6 `--live`
  refusals (exit 3, HARD-DISABLED) after the gate check, library 54 goals, fingerprints
  `62f8a100…`, `library.json` `917fc4d7…`, config `ab0dccd4…`; 0 broken relative links; clean
  state.

**NOT VERIFIED:** home-PC validation (pending); any human keyboard operation, anywhere; the owner's
terminal key-repeat settings; the owner's PC real-time factor (M5.5 §16.1 quotes 0.21–0.25 from an
earlier owner session; not re-measured).

**Qualifications carried forward unchanged:**
- S2 and S3 were accepted retrospectively (S3 under criteria revision S3-r1).
- S4a–S4d used **automated pseudo-terminal input** through the real teleop. That is not human
  operation.
- S4d-M: the simulator server was killed by SIGKILL during shutdown (not graceful).
- S4d-E: the cancel came before measurable motion. It is not a moving-body stop.

## 1. Facts from the code (MEASURED by reading, at `e7ffd20`)
- **Characters, not key events.** `m55_teleop_keyboard` reads its terminal in cbreak mode
  (`select`, then `os.read(fd, 64)`). A terminal never reports a key release.
  - A motion key counts as held for `teleop.hold_s` = **0.8 s** (H) after its last character
    (`TeleopState.twist`).
  - The teleop publishes `/cmd_vel` at `heartbeat_hz` = 10 Hz (B = 0.1 s): the held motion at the
    selected level's speed, otherwise zero.
  - A same-key gap above H, within 2.0 s, prints `key repeat gap … > hold window …`.
- **Keys** (`KEYMAP`): `a` arm, `z` disarm, `w`/`W` forward, `s`/`S` reverse, space stop, `x` or
  a lone Esc estop, `r` reset, `h` home, `+`/`=` faster, `-` slower, `j`/`l` rejected, `q` quit.
  Ctrl+C and Ctrl+D also quit; escape sequences are ignored with a message.
- **Teleop quit** (`q`, Ctrl+C/D, EOF, SIGHUP, SIGTERM): a zero heartbeat, then `stop` and
  `disarm`, twice in total, best effort within 1 s.
- **Node** (`m55_locomotion.py`). Events carry the node's `time.monotonic()` (`wall` in
  `events.jsonl`); the session ticks at 20 Hz (T = 0.05 s).
  - Release: a zero command gives `intent_stop`, then STOPPING (the phase in flight completes),
    then READY at that boundary.
  - Lease: no valid command for `lease_s` = 0.5 s gives `lease_expired`, then STOPPING, then
    DISARMED.
  - `stop`: STOPPING (`stop`); the stop latch holds until a zero command.
  - `estop`: FAULTED (`estop`) and one cancel for an unresolved goal, both in the request handler.
  - `home` at READY: at boundary 0 it is accepted, "already at a cycle boundary", with no goal.
    Otherwise it sends the fewest phases (at most 4) to boundary 0, then READY `homed`.
  - FAULTED refuses `disarm` and `arm` (`faulted_reset_required`). It accepts `stop` with "not
    moving (FAULTED); held until a zero command".
  - The status `mode` is `shadow` or `dispatch`. A dispatching transport is refused while the gate
    is `False`.
- **Timing** (M5.5 §16.1, tested): release to STOPPING 0.8–0.95 s wall; teleop loss ≤ 0.55 s.
- **Level 0:** phases 1.2–2.8 s of sim including the 0.2 s lead-in; a cycle is 18.8 s of sim;
  20 mm stride; 1.06 mm/s.
- **Clock.** Linux `CLOCK_MONOTONIC` is system-wide, so an observer's `time.monotonic()` and the
  node's `wall` compare on one host (as for the S4 drivers, S4D_E_CONTRACT §3). Different hosts
  do not compare: the observer and the node must run on the same host.

## 2. Scope
**One bounded, human-operated keyboard session in simulation, on the owner's PC.**
- One launch, at internal level 0 (20 mm stride, teleop label `1/3`), forward only.
- About one to two cycles of travel in total.
- Release, controlled stop, home, emergency stop and close, all by a person at a physical
  keyboard.

**Why level 0** (recommended, D1). The evidence for operator input on contact is at level 0: S4b
(release, reversal, home, lease loss) and S3/S4a (estop with a goal in flight). S4c (levels 1 and
2) and S4d (level 2) show the same tracking (maximum 0.00225 rad) and sub-millimetre drift as level
0, so a faster level adds no evidenced plant risk. But level 0 bounds what human timing can add:
- the shortest phases (≤ 2.8 s of sim), so a late release commits the least motion;
- the smallest stride (20 mm per cycle) and the slowest speed (1.06 mm/s);
- no `+`/`-` key is needed, so fewer keys can go wrong.

S4e then adds exactly one unknown: the person.

**Not in scope:** levels 1–2 or reverse (`s`) under human input, turning, the crash-path close
while walking (D4), reset or re-arm, hardware and navigation.

## 3. Limits and budgets (every session)
**Live stops.** The observer acts on these (§4.3). They are the S3/S4 values, unchanged, except
where noted.
- Travel > 0.15 m from the arm pose (the S3 cap).
- Tilt > 0.10 rad, or z < 0.045 m, each in 2 consecutive fresh samples.
- Cumulative drift relative to the arm pose, in its body frame: |lateral| > 50 mm or |heading
  change| > 0.15 rad. These are S4d's cumulative live stops (OD4), adopted because human timing
  makes per-cycle bands inapplicable; there is no per-cycle band stop.
- A joint within 0.02 rad of its URDF limit.
- Tracking > 0.05 rad (also the node's own abort); > 0.02 rad is a finding.
- Any `/cmd_vel` rejection (`command_rejections`); any FAULTED before the person's `x`.
- More than 30 goals; any goal at a level other than 0; any `level_change`.
- In shadow, any status that is not SHADOW with the gate `False`. In live, any status that is not
  DISPATCH with the gate `True`.
- No status within 60 s of the teleop's start, or none for 5 s of wall after the first one.
- The session cap: 20 min of wall from the teleop's start.

**Harness budgets** (as S4d OD8 and OD16):
- A watchdog of 3600 s, armed before the launch. Before the launch, and again before the teleop
  starts, at least 20 + 15 min must remain.
- Disk, before the launch and before recording: free ≥ max(5 GiB, 2E + 0.1 GB), where
  E = 2 × 30.8 MB/min × the watchdog time left (a planning assumption, not a measured rate).

**Session budget:** at most 30 goals; exactly one cancel (the person's `x`). An emergency response
always overrides the budget and ends the session.

## 4. Operator procedure (live; NOT AUTHORIZED TO RUN)
### 4.1 Before the launch
**P1. Terminal preflight** (no ROS; nothing launched). Run it in the same terminal window that will
run the teleop:

```
python3 scripts/s4/s4e_terminal_preflight.py --out <session>/preflight.json
```

- Hold `w` for 10–20 s by a clock, then release.
- Tap `w` once.
- Press Ctrl+D to finish.

Go on only if the tool reports human mode (an interactive terminal), a first repeat earlier than
0.8 s after the press, and every gap within the hold below 0.8 s. Otherwise do not launch; the
remedy is a shorter repeat delay (M5.5 §11, D7).

**P2. Launch and pre-key checkpoint.** The harness launches the simulator and the node once, then
waits at the PRE-KEY CHECKPOINT. The operator checks settle, mode, gates and ownership, then gives
`go` (or `abort`).

### 4.2 The session (in the preflight's terminal, through the observer)
```
python3 scripts/s4/s4e_observer.py --out <session>/observer --mode live    # shadow: --mode shadow
```

The observer starts the real teleop; the operator types into it as usual.
- It refuses to start (nothing is started) unless its standard input and output are an interactive
  terminal in the foreground, its output directory is new, and the gates match the mode: live,
  exactly M55 `True`, M61 `False` and M6.0-D `False`; shadow, all three `False`. Both a fresh
  interpreter and the observer process must show those gates.

| Step | Keys | Expected |
|---|---|---|
| K1 | `a` once | READY at boundary 0 |
| K2 | hold `w` for **10–20 s** by a clock, touching nothing else; release | walking without a stop or a `key repeat gap` line; after the release STOPPING (`intent_stop`), then READY |
| K3 | hold `w`; after about 3 s (WALKING), press space once while still holding `w`; keep holding until READY, then about 2 s more; release | STOPPING (`stop`), READY; no goal while `w` is still held (the latch) |
| K4 | `h` once, at READY | READY `homed` at boundary 0, or "already at a cycle boundary" with no goal |
| K5 | hold `w`; after about 2 s (WALKING), press `x` once while holding; release `w` right after | FAULTED (`estop`), one cancel, then a hold; the observer prints when 10 s of sim have passed |
| K6 | `q` once, after that line | stop ×2 accepted and disarm ×2 refused (`faulted_reset_required`); the teleop exits 0 |

### 4.3 Improvisation, aborts and who decides
- **May improvise:** the timing within each step's stated range; waiting longer at READY or
  FAULTED. The person may also press space, `x` (or Esc) or `q` at any moment to stop. Any
  unplanned safety key ends the attempt; the remaining steps are not done.
- **May not:** any other key (`+ = - s S j l r z`, arrows, function keys, other letters); a second
  `a`; `h` outside K4; `a` after `x`; holding `w` over 20 s in K2; repeating a step; restarting the
  teleop or the observer; a second launch.
  - After any deviation, the operator ends the session: `x` if moving, otherwise `q`.
- **Operator abort.** Anything unexpected ends the session: a message not understood, an unplanned
  FAULTED or refusal, motion not commanded, a frozen display, an unintended key. The operator
  decides in the moment; there is no retry.
- **Automatic abort** (the observer, §3; no discretion): it stops relaying input, calls the node's
  `estop` service once, then ends the teleop (SIGTERM, then SIGKILL after 15 s).
- **Harness watchdog:** at its deadline, owned-process cleanup and the disabled restore.
- **Owner:** decides go/no-go before the launch, and afterwards whether any further attempt is
  authorized. Nobody continues an aborted session or relaunches under the same authorization.

## 5. Shadow rehearsal (NOT AUTHORIZED TO RUN)
The same procedure with all three gates `False`. Nothing is enabled; the node records its goals in
its ShadowTransport, and nothing moves.
- `--mode shadow` refuses unless all three gates are `False`, both in a fresh interpreter and in
  the observer process. Every status must read SHADOW with the gate `False`; otherwise it stops.
- Scoring with `--shadow`:
  - tracking and the moving-body qualifier are N/A;
  - travel, tilt and drift are measured on the standing body;
  - H10 requires zero controller goals, cancels and feedback.
- Best label: **"PASS (SHADOW ONLY: not S4e acceptance)"**.
- The human-timing criteria (H3, H4, H5) are measured for real in shadow. They show the person's
  keyboard and the node's detection, not motion.

## 6. Evidence and recording rules
- **Observer record** (`observer/observer.jsonl`, `observer/observer_summary.json`):
  - every chunk read from the operator's terminal, with `t_read`, `t_written`, its bytes (hex)
    and its actions as the teleop's own `KeyDecoder` decodes them;
  - every teleop output chunk, with its time; every status received;
  - stop decisions; the teleop's pid, exit status and exit time;
  - `input_source: human-tty` and the terminal checks;
  - the processes holding the teleop's pseudo-terminal at start and at end.
- **Preflight record** (`preflight.json`): `input_source: human-tty`, each run of repeats, the
  delay and intervals, the largest gap and the verdict. The desktop's repeat settings
  (`xset q`, `gsettings`) when available, otherwise "not available".
- **Operator attestation** (`attestation.txt`), written by the operator after the session:
  > I, \<initials\>, operated the S4e session on \<date\> by pressing keys on a physical keyboard
  > myself. No script, macro, key remapper or other program generated any key.
- **Harness:**
  - a process scan at start and end (no S2–S4d driver);
  - the node evidence (`events.jsonl`, `final_status.json`, `library.json`);
  - one bag outside the repository, with its hash (the S4d-E topic list);
  - captures before and after; the restore report.
- **Privacy:** only the keys typed into the observed teleop terminal during the session, and into
  the preflight, are recorded. Type nothing else (no passwords) in that window.
- **No screen or camera recording is required.** A screen recording only if the owner makes one;
  it is never proof of motion.

## 7. Clocks and key-timing rules
- **Observer clock:** the observer's `time.monotonic()`, on the node's host.
  - `t_read`: when the observer's read of the chunk returned.
  - `t_written`: after its write to the teleop's terminal returned. The teleop reads after
    `t_written`; the relay latency is recorded.
- **Node clock:** `wall` in `events.jsonl`. **A status receipt time is never a transition time.**
- **"No more bytes arrived" is never a key-up event.**
  - The last byte of a hold is the last character the terminal delivered. The physical release
    came after it, before the next repeat would have arrived.
  - All release timing is measured from that last byte.
- **Holds.** A hold is a maximal run of forward (`w`/`W`) characters whose consecutive gaps are
  ≤ 2.0 s (the teleop's own repeat window). A space (K3) or an estop (K5) inside a hold is "inside"
  when the previous forward character came less than H = 0.8 s before it.
- **Sim time** comes from the statuses' `monitors.sim_time_s`, interpolated on the observer clock,
  and from the bag's joint-state stamps.

## 8. Criteria H1–H12 (`scripts/s4/score_s4e.py`, frozen with this contract)
Every check is PASS, FAIL, UNMEASURED or N/A.
- Missing evidence is UNMEASURED, never PASS. A contradiction is FAIL.
- N/A only where this contract says so (shadow).
- **Overall:** FAIL if any check fails; otherwise INCONCLUSIVE if any is UNMEASURED; otherwise
  PASS. A shadow result reads "… (SHADOW ONLY: not S4e acceptance)".

**The human-input rule.** H3–H8 are human-input criteria. They can be PASS only when H1 is PASS.
- If H1 is FAIL, each of H3–H8 is FAIL: "scripted-input evidence can never satisfy a human-input
  criterion".
- If H1 is UNMEASURED, each of H3–H8 is UNMEASURED.

**H1 Human-input provenance.**
- PASS when all of these hold:
  - the observer record says `input_source: human-tty`, with stdin and stdout terminals and the
    foreground check true;
  - the preflight record says `input_source: human-tty`;
  - the attestation is present and contains its sentence;
  - the process scans at start and end show no S2–S4d driver;
  - the teleop's pseudo-terminal was held only by the observer and the teleop.
- FAIL on any scripted marker (`scripted-pty-test`, `test-fixture`), any S2–S4d driver record in
  the session, or another holder of the pseudo-terminal.
- UNMEASURED when any of those records is missing.

**H2 Plan, budget and mode.**
- The decoded keys, with each hold collapsed, are exactly: `a`; a hold (K2); a hold with exactly
  one space inside it and forward characters after it (K3); `h`; a hold with exactly one estop
  inside it (K5); `q`.
  - No other byte reached the teleop, including ignored escape sequences.
  - K2 lasted 10.0–20.0 s from its first to its last character.
- The node's requests are exactly: arm, stop, home, estop, then the close (stop ×2 and disarm ×2,
  compared as counts).
- The only refusals are the close's two `faulted_reset_required` disarm refusals.
- At most 30 goals, every one at level 0; no `level_change`.
- Exactly one cancel, the estop's. No fault but that estop. No command rejection.
- Every status is in the session's mode: live, DISPATCH with the gate `True`; shadow, SHADOW with
  the gate `False`.
- The observer's result is "session complete".

**H3 Key repeat (check 1.11).**
- The preflight passed (§4.1) and recorded the repeat settings, or "not available" with the
  measured values.
- The K2 hold lasted ≥ 10.0 s; every gap in it was < 0.8 s.
- The teleop printed no `key repeat gap` line from K2's first character to 0.1 s after its last.
- The node left STARTING/WALKING for no state from K2's first goal until K2's last character.
- FAIL on a gap ≥ 0.8 s, a `key repeat gap` line, or a stop before the last character.
- UNMEASURED if the hold was shorter than 10 s or a record is missing.

**H4 Release detection (checks 1.13 and 1.7).** t_last is the `t_read` of K2's last forward
character.
- The node's first transition after t_last is STOPPING (`intent_stop`) at t_stop, with
  **0.80 s ≤ t_stop − t_last ≤ 1.00 s**. That is the tested 0.80–0.95 s plus 0.05 s for the relay
  and the host. Outside that range: FAIL.
- READY follows, measured from t_last, within the sim durations of the goals in flight at any time
  from t_last to t_stop (one goal, or two if a phase boundary fell in that window), plus 1.0 s of
  sim. The wall backstop is 120 s.
- No goal is sent from t_stop until K3's first character.
- UNMEASURED if an event or the sim mapping is missing.

**H5 Controlled stop and latch (K3; check 1.7).**
- The space is inside the K3 hold.
- The node's `stop` request is accepted at or after the space's `t_read`. STOPPING (`stop`) comes
  within 0.050 s of the request, on the node clock.
- READY follows within the in-flight goal's duration + 1.0 s of sim (wall backstop 120 s).
- **Latch:** forward characters continued for ≥ 1.0 s after READY, and no goal was sent from
  STOPPING until `h`.
- FAIL if a goal was in flight at the request and no STOPPING (`stop`) came within 0.050 s, or on a
  goal sent before `h`.
- UNMEASURED if no goal was in flight at the request (the stop was not exercised while walking),
  if the hold ended before READY + 1.0 s (latch not exercised), or if an event is missing.

**H6 Home (K4).**
- `h` at READY.
- If the boundary is not 0: one `homing` event; at most 4 goals; READY `homed` at boundary 0
  within 12.0 s of sim of the request (wall backstop 120 s).
- If it is 0: accepted, "already at a cycle boundary", with no goal.
- Either way, no goal is sent after READY until K5's first character.

**H7 Emergency stop by the person (K5).**
- The estop is inside the K5 hold, while a goal was active: accepted before the node's `estop`
  request, and without a result before it.
- Exactly one `estop` fault, FAULTED (`estop`) within 0.050 s of the request, and exactly one
  cancel for that goal in the same handling, on the node clock.
- Cancel response 0. The result is CANCELED. SUCCEEDED despite the cancel is UNMEASURED; any other
  status FAILs; no result is UNMEASURED. Shadow: the modeled "shadow cancel".
- **Hold:** over ≥ 10 s of sim from the result, every status FAULTED with no goal; joint motion
  ≤ 0.01 rad, tilt ≤ 0.10 rad and z ≥ 0.045 m (from the bag). A coverage gap is UNMEASURED.
- No goal after the fault.
- **Qualifier, recorded but not scored:** the joint motion from the goal's acceptance to the
  cancel. At least 0.005 rad reads "moving-body stop"; less reads "before measurable motion, as
  S4d-E". Shadow: N/A.

**H8 Close by the person (K6; check 1.9, clean path).**
- `q` after the hold.
- After it, the requests are exactly stop ×2 (accepted, with the FAULTED message) and disarm ×2
  (refused, `faulted_reset_required`). No transition follows.
- The teleop exits 0 within 15 s of wall of `q`'s `t_read`.
- The observer relays no byte after `q`.

**H9 Safety and margins** (from the arm to the end; bag and statuses).
- The observer finished without a stop condition.
- Travel ≤ 0.15 m from the arm pose; tilt ≤ 0.10 rad; z ≥ 0.045 m.
- Cumulative |lateral| ≤ 30 mm and |heading change| ≤ 0.10 rad, relative to the arm pose.
- Minimum joint margin ≥ 0.02 rad.
- Live: tracking ≤ 0.05 rad for every goal with feedback; > 0.02 rad is a finding. Shadow: N/A.
- Node-observed ages within the monitor limits (joint states 0.5 s, pose 1.0 s, graph 3.0 s).
- A coverage gap is UNMEASURED.

**H10 Ownership and controller traffic.**
- Every status: 0 foreign action clients and 0 command-topic publishers.
- The captures before and after show 0 command publishers. `joint_trajectory` is absent from the
  bag, with recorder support.
- Live: the controller log's received and accepted goals equal the node's goals; exactly one cancel
  request.
- Shadow: 0 received, 0 accepted, 0 cancel requests; no feedback in the bag.

**H11 Restore, library and run installation** (as S4d-E E12):
- the restore report reads DISABLED AND VERIFIED;
- `library.json` equals the disabled baseline (`917fc4d7…`);
- the run installation imports all three gates `False`.

**H12 Evidence completeness and consistency.**
- Present: the observer record with its start and end, the node's `events.jsonl`,
  `final_status.json` and `library.json`, the bag hash, the preflight, the attestation, the process
  scans and the captures. A missing item is UNMEASURED.
- Consistency: the relayed byte total equals the sum of the recorded chunks, and every `t_written`
  is ≥ its `t_read`. A mismatch FAILs.

**Recorded only:** the RTF, the operator's repeat delay and rate, and the relay-latency
statistics.

**What a live PASS would show.** In simulation, at internal level 0, one person at a physical
keyboard on the owner's PC operated the real teleop:
- a held `w` of at least 10 s, with the terminal's own auto-repeat, walked without a stutter;
- the release stopped the walk within 0.80–1.00 s of the last key character;
- space, `h`, `x` and `q` behaved as specified;
- every limit held.

**What it would NOT show:**
- hardware operation, navigation, autonomous or unrestricted walking;
- robustness to other operators, keyboards, terminals, SSH, real-time factors or durations;
- levels 1–2, reverse or turning under human input;
- the crash-path close (teleop kill) under human operation (D4);
- dynamic stability; full S4 acceptance (the S4 plan's owner decision 4 stays open);
- the identity of the operator (the attestation is a declaration).

## 9. Restore support
`scripts/m61_restore_and_verify_disabled.sh` may delete a verified, never-pushed
`local/m55-s4e-h-*` branch, under the same rules as every stage.
- The frozen S4d tests already refuse `local/m55-s4e-<UTC>` as a look-alike. They stay unchanged,
  so S4e uses the narrower `local/m55-s4e-h-*` (D9).
- Look-alikes stay refused.
- The shadow rehearsal does not use it, because nothing is enabled.

## 10. Owner decisions (S4e, D1–D12; recommended defaults)
| # | Decision | Recommended default |
|---|---|---|
| D1 | Level | Internal level 0 (§2) |
| D2 | Where the person operates | The owner's PC, after the pending home-PC validation passes. This cloud session cannot carry real keystrokes (`docs/S4E_OPERATOR_PACKAGE.md`) |
| D3 | Procedure | K1–K6 as in §4.2: estop, then close; no reset, re-arm or reverse |
| D4 | Crash-path close while walking (check 1.13, second half) | Excluded. S4b measured it with an automated SIGKILL; a human-closed terminal while walking would be a separate, separately authorized attempt |
| D5 | Release-detection band | 0.80–1.00 s on the observer clock (§8 H4) |
| D6 | K2 hold | 10–20 s of wall |
| D7 | Repeat-delay remedy | If the preflight fails, shorten the desktop's repeat delay (for example `xset r rate 400 30`). Never raise `teleop.hold_s` for S4e |
| D8 | Automatic emergency response | The observer stops relaying, calls the node's `estop` service once, then ends the teleop |
| D9 | Enabling-branch pattern | `local/m55-s4e-h-<UTC>`, never pushed (§9) |
| D10 | Human-input evidence | The observer's human-tty record, the preflight, the operator's attestation and the process scans (H1) |
| D11 | Budgets | ≤ 30 goals, one cancel, travel 0.15 m, session cap 20 min, watchdog 3600 s, the disk rule (§3) |
| D12 | Operator | The owner, as the only operator, for both the shadow and the live session |

## 11. Authorization boundaries
Each needs its own owner message, in this order, and none continues automatically:
1. the home-PC validation (pending);
2. one human-operated **shadow** rehearsal on the owner's PC (§5);
3. after its review, one bounded **live** attempt (§4), from a local `local/m55-s4e-h-*` enabling
   branch that is never pushed.

The drafts of both authorizations are in `docs/S4E_OPERATOR_PACKAGE.md`, each marked NOT
AUTHORIZED.

## 12. Tools frozen with this contract (written after it, on this branch)
- `scripts/s4/s4e_terminal_preflight.py`: the standalone terminal diagnostic (no ROS).
- `scripts/s4/s4e_observer.py`: the human-input observer. It never writes a byte of its own to the
  teleop: it relays, unchanged and in order, only bytes it has just read from the operator's
  terminal, until it closes input. It has no key list, no repeat generator and no option to
  supply keys. It sends no `/cmd_vel` and no motion request. Its single command is the node's
  `estop` service, on a stop condition.
- `scripts/s4/score_s4e.py`: H1–H12. After its first test it changes only to fix a defect, never
  to change a criterion.
- `scripts/s4/s4e_harness_template/`: the run-local harness template; its tests use stubs only.
