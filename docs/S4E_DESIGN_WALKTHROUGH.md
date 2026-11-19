# S4e design walkthrough: a line-by-line guide

For the owner, to explain, debug and extend the S4e tools. The rules are in
[`S4E_CONTRACT.md`](S4E_CONTRACT.md); this explains how the code follows them. Line numbers are at
the version first validated on 2026-11-19; if they drift, search for the function name.

Exercises to try: [`S4E_DEBUGGING_EXERCISES.md`](S4E_DEBUGGING_EXERCISES.md).

## 0. The one idea everything rests on
A terminal delivers **characters**, never key presses or releases.
- **A held key** is a first character, then, after the desktop's *repeat delay*, more characters
  at its *repeat rate*.
- **A release** is only "no more characters". You see it after the fact.
- **The teleop** therefore treats a motion key as held until H = 0.8 s after its last character
  (`m55_teleop.py` `TeleopState.twist`).

Every S4e tool keeps that distinction explicit:
- the preflight prints "NOT a key-up event";
- the observer records arrival times, not presses;
- the scorer measures release timing from the **last character**.

## 1. The data path
```
physical keyboard -> desktop (repeat) -> terminal emulator -> pty A (operator's terminal)
   -> s4e_observer.py: read (t_read), record, write (t_written)        <- the ONLY write
   -> pty B -> m55_teleop_keyboard (cbreak; KeyDecoder; TeleopState; /cmd_vel 10 Hz; services)
   -> m55_locomotion node (events.jsonl on the node clock)
s4e_observer.py also reads: /spiderx/locomotion/status, /joint_states (stop conditions)
```

Everything runs on one host. That makes the observer's `time.monotonic()` and the node's `wall`
the same clock (Linux `CLOCK_MONOTONIC`), so `t_stop − t_last` subtracts two times on one clock.

## 2. `scripts/s4/s4e_terminal_preflight.py` (standalone; no ROS)
| Lines | What and why |
|---|---|
| 1–40 | The docstring is the user manual. It states the verdict rules, the scripted/human split and the exit codes |
| 51–58 | Constants copied from the teleop (H, the 2.0 s repeat window). `test_s4e_terminal_preflight.TeleopConsistency` fails if they ever differ from `m55_teleop` and the config. The tool stays standalone |
| 79–92 `key_name` | Readable names: `space`, `Esc`, `Ctrl+D`, and hex for anything else |
| 94–105 `runs` | Groups characters exactly as the teleop does: the same byte, nothing else between, each within 2.0 s of the previous one. A different byte, or a longer gap, starts a new run |
| 107–121 `summarize` | `gaps[0]` is the first-repeat delay; `gaps[1:]` are the repeat intervals; the median gives the rate. `gaps_at_or_above_hold_s` are the stutters |
| 123–133 `slow_first_repeat_suspects` | One subtle case. A first repeat later than 2.0 s splits one hold into "single character" plus "run". So a single character followed within 0.8–5 s by a run of the same key is suspected. A *release then tap* (run, then single) is not, because of the order |
| 135–161 `verdict` | FAIL on any motion-key stutter or suspect; else UNMEASURED without a ≥ 10 s forward hold (missing evidence is never PASS); else PASS |
| 179–205 `environment` | Read-only `xset q` and `gsettings`, each with a 2 s timeout; "not available" otherwise. SSH is noted, because the client then generates the repeats |
| 208–229 `tty_checks`, `refusal` | The human check needs stdin and stdout to be terminals, in the foreground process group. The refusal happens **before** the terminal is touched |
| 231–237 `cbreak` | No echo, no line buffering. ISIG stays on, so Ctrl+C still raises SIGINT |
| 239–299 `Preflight` | The live display. `chunk` prints each arrival and closes the previous run when another byte comes. `tick` notices idle (2 s) and closes the run. Every run end prints the not-a-key-up note |
| 301–325 `loop` | `select` with a 0.1 s timeout, so idle and the duration cap are noticed. Ctrl+D (0x04) ends it. The test-only error hook works only in scripted mode |
| 327–411 `main` | The order matters: refuse (`--out` exists, no terminal) → save termios → signal handlers → cbreak → loop. The `finally` always restores, then compares (`terminal_restored`). The record is written with mode `'x'`, so it is never overwritten |

**Scripted versus human.** `--scripted-pty-test` changes the label on every line and the
`input_source`. There is no other difference, so the tests exercise the real code path while
never producing a "human" record.

## 3. `scripts/s4/s4e_observer.py` (the human-input observer)
### 3.1 The relay (lines 138–223): the part that must never send a key
| Lines | What and why |
|---|---|
| 150–160 `pump` | One `select` over the operator's terminal and the teleop's pty master |
| 162–195 `_relay_operator` | `os.read(self.fd_in, 1024)`, then `t_read`, then, unless input is closed, `os.write(self.master, view)` on **the same bytes**, then `t_written`. `quit_cut` (99–107) splits a chunk after the first quit so that nothing after `q` reaches the teleop (H8). Bytes after the input closed are recorded as `not_relayed`, never written |
| 200–211 `_teleop_output` | The teleop's output goes to the operator's screen and into the record, with its time (H3 reads the `key repeat gap` lines from here) |
| 219–223 `close_input` | One-way: once closed, input never reopens |

The tests prove the "never sends a key" property three ways (`test_s4e_observer.NeverSendsAKey`):
- the AST shows exactly one `os.write` to `self.master`, writing the bytes from `os.read(self.fd_in)`;
- there is no publisher, action client, `cmd_vel` or `TIOCSTI`;
- the CLI has only `--out`, `--mode` and `--domain-id`.

### 3.2 The monitor (lines 226–366): contract §3, without ROS
- `on_status` and `on_joint_state` are called from the ROS thread; `check(now)` runs on the main
  loop. The lock covers only the hand-over.
- `_status` (289–317) applies the per-status stops: mode/gate, rejections, FAULTED **before the
  person's estop**, level, goal budget, goal level, tracking.
- `_pose` (319–359) applies the fresh-sample stops, with S3's `fresh_pose` and `Debounce`
  unchanged:
  - it takes the arm reference at the first fresh non-DISARMED sample;
  - then tilt and z (2 fresh samples), travel, and cumulative lateral and heading from the arm
    pose (`body_delta`);
  - it also detects the K5 hold: FAULTED, with no goal, for 10 s of sim after the person's `x`.
    It prints one line, so the operator knows when `q` may follow.
- `person_estop` is fed by the relay: the decoded `x` makes the later FAULTED planned.

### 3.3 ROS (lines 369–425)
One subscription each to the status and the joint states, and **one** service client, `estop`.
The executor runs in a daemon thread, as the S2 driver's does.

### 3.4 Processes (lines 428–515)
- `spawn` makes the teleop the session leader of its own pty, so Ctrl+C and SIGTERM reach it as
  in a real terminal.
  - The parent waits on a close-on-exec pipe until the child has exec'd.
  - Without that, the start-of-session pty scan could see the child before `setsid` (a race the
    tests found).
- `pty_holders` finds who holds the pty:
  - the master: an fd to `/dev/ptmx` whose `fdinfo` `tty-index` matches;
  - the slave: an fd linking to `/dev/pts/N`.
  - H1 requires only the observer on the master and only the teleop's session on the slave.
- `driver_scan` lists running S2–S4d drivers by script name.

### 3.5 The session (lines 518–682)
- `run` keeps a fixed order:
  1. record start;
  2. raw mode;
  3. spawn;
  4. scans;
  5. ROS;
  6. the loop.
- **Any exit but success runs `emergency`:**
  1. close input;
  2. call the `estop` service **once**;
  3. `end_teleop`: SIGTERM to the teleop's session (its clean quit), SIGKILL after 15 s, pumping
     its output meanwhile.
- **Then always:** the end scans, the terminal restore (even if the terminal has hung up), the
  summary, the ROS close.
- `loop` (561–579) ends **successfully only** when the teleop exits 0 after a relayed quit.
  - Exiting otherwise, or not exiting within 15 s of the quit, is a stop condition.
  - So is the operator's terminal closing.
- `main` refuses before touching anything: an existing `--out`, no terminal, gates not as the
  mode requires (fresh interpreter **and** this process), no rclpy or limits. Then
  `os._exit(rc)` skips the rclpy teardown that can abort at exit (as S3).

## 4. `scripts/s4/score_s4e.py` (H1–H12)
### 4.1 Extraction (lines 120–250)
- `observer_context` turns the observer records into keys, tokens, statuses, teleop output, pty
  holders and scans.
- `node_context` reuses the frozen S4b helpers (`goals_from_events`, `transitions_from_events`).
- `macro` (169–186) turns tokens into steps:
  - forward characters within 2.0 s form one hold;
  - a stop or estop less than 0.8 s after the hold's last forward character is **inside** it;
  - everything else is its own step.
- `plan_steps` (219–231) accepts only `arm, hold, hold(stop inside, w after), home,
  hold(estop inside), quit`.
- `sim_mapper` interpolates sim time between status receipts and refuses gaps over 2 s.
  Bounds in sim time therefore never use a guess.

### 4.2 The criteria
| Function | Criterion | The rule worth remembering |
|---|---|---|
| `assess_provenance` 258 | H1 | A scripted marker FAILs before anything else. A missing record is UNMEASURED |
| `assess_plan` 301 | H2 | Exact keys, requests, refusals, budget, mode and the observer's result |
| `assess_repeat` 348 | H3 | A K2 shorter than 10 s is UNMEASURED (the check was not done), not FAIL |
| `assess_release` 383 | H4 | t_stop is the **first** transition after the last character; it must be STOPPING `intent_stop`, 0.80–1.00 s later |
| `assess_stop` 418 | H5 | A space while no goal was in flight is UNMEASURED (not exercised); while walking, no STOPPING is FAIL |
| `assess_home` 459 | H6 | Two cases: "already at a cycle boundary", or homing (≤ 4 goals, READY `homed` at 0, ≤ 12 s of sim) |
| `assess_estop` 522 | H7 | Result codes are action_msgs GoalStatus (5 CANCELED, 4 SUCCEEDED). The hold uses the bag with a coverage check. The moving-body qualifier is recorded, never scored |
| `assess_close` 579 | H8 | `q` is the first relayed quit, found without the full plan, so a byte after `q` FAILs even when the plan does not parse |
| `assess_safety` 602 | H9 | Bag poses from the arm on, with coverage; tracking is live only |
| `assess_traffic` 648 | H10 | Shadow requires zero controller traffic and no feedback |
| `assess_evidence` 676 | H12 | Missing files are UNMEASURED; inconsistent byte counts or times FAIL |
| `human_rule` 701 | H3–H8 | Applied last. H1 FAIL forces FAIL; H1 UNMEASURED forces UNMEASURED; `underlying_verdict` keeps what was measured |

**Defects fixed after the first test** (logged in the evidence; no criterion changed):
1. H7 compared the result with a string instead of the GoalStatus code.
2. H8 needed the whole plan to parse before finding `q`.

### 4.3 Loading (lines 759–879)
`load_session` reads the session layout the harness writes. `load_bag` follows S4d-E's loader:
- poses from `/spiderx/sim/world_poses`;
- joint states and feedback from the S2 bag reader;
- the clock alignment through the statuses the observer also received.

The loader is exercised only on real evidence. The tests feed `score()` directly.

## 5. `scripts/s4/s4e_harness_template/` (run-local, stubs-only tested)
See its [README](../scripts/s4/s4e_harness_template/README.md). The control flow:
- **`guarded.sh`** runs `s4e_session.sh` as a child and arms an EXIT trap: cleanup, then restore.
  An interruption before any pid file exists still restores.
- **`s4e_session.sh`:**
  1. refusals (`checks.sh`);
  2. `start_owned` launch;
  3. checkpoint;
  4. time and disk again;
  5. capture, scan, recorder;
  6. wait for the operator's observer summary;
  7. scan, capture, stop.
- **`cleanup.sh`** acts only through `launch.pid` (session) and `bag.pid` (group). Look-alikes
  are listed, never killed. Zombies are not "running".
- **Two findings from the stub tests are now in the template:**
  - `start_owned` resets SIGINT and SIGQUIT, because background jobs inherit them ignored;
  - the cleanup is zombie-aware.

## 6. Restore support
`scripts/m61_restore_and_verify_disabled.sh` accepts `local/m55-s4e-h-*`, in four places: the
header comment, the allowed patterns, the refusal message and the branch listing.
- The frozen tests that refuse `local/m55-s4e-<UTC>` are untouched and still pass.
- The new tests are at the end of `test_m61_restore_script.py`.

## 7. How to extend safely
1. **A new criterion:** write it into a new contract revision **first**, then a pure `assess_*`
   function, then a bench test that passes, then mutations that FAIL and UNMEASURE.
   Never edit H1–H12 in place.
2. **A new stop:** add it to `Monitor` with a unit test in `MonitorTests`, and keep the stop path
   unchanged: input closed, one estop, teleop ended.
3. **Never add a write to `self.master`.** `NeverSendsAKey` will fail, and should.
