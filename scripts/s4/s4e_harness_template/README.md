# S4e run-local harness template

**NOT AUTHORIZED TO RUN.** A shadow or live S4e session needs the owner's separate message
(`docs/S4E_CONTRACT.md` §11). This directory is a template: it is tested only with stubs
(`scripts/s4/test_s4e_harness_template.py`). Nothing here has launched anything.

Derived from the tested S4d-E live harness: its tools are kept with that run's evidence, in
`docs/evidence/m55/s4d_e_live_20261111T113831Z/tools/`.

## Use (when authorized)
1. Copy this directory to a run-local directory **outside the repository**.
2. Set the parameters in `env.sh`, or export them: mode, workspace, run installation, baseline,
   and, for live, the recorded `local/m55-s4e-h-*` branch in `<session>/enabling_branch.txt`.
3. Arm the watchdog first, in the background: `bash watchdog.sh <session>/run &`.
4. Run the session: `bash guarded.sh <session> <bag dir>`.
5. At the PRE-KEY CHECKPOINT the operator checks the simulation, then writes
   `<session>/run/go.flag` (or `abort.flag`).
6. The operator starts the observer **in their own terminal**: the harness never starts it.

## Files
| File | Role |
|---|---|
| `env.sh` | Parameters. Every external action is a command here (launch, recorder, captures, gates, disk, restore), so the tests can stub it |
| `checks.sh` | Refusals before the launch: mode, installation, gates, ref, watchdog time, disk |
| `s4e_session.sh` | One session: checks, one launch, PRE-KEY CHECKPOINT, captures, driver scans, the recorder, the wait for the operator's observer, stop |
| `guarded.sh` | Arms cleanup and restore on every exit, including an interruption before any pid file exists |
| `watchdog.sh` | Deadline safety net: owned cleanup, then restore |
| `cleanup.sh` | Stops only the processes this run recorded (launch session, recorder group). Look-alikes are listed, not killed; zombies are not running |
| `restore.sh` | The disabled restore: `--verify-only` in shadow, the recorded branch in live. Idempotent |
| `scan.sh` | The S2–S4d drivers running now, as JSON (H1) |

## Changes from the S4d-E harness, and why
- **The launch and the recorder start with SIGINT and SIGQUIT at their defaults.** They are
  background jobs of a non-interactive shell, which otherwise inherit SIGINT *ignored*. A program
  cannot trap a signal ignored on entry, so the cleanup's SIGINT would never reach it.
  - The stub tests found this: every stop fell back to SIGKILL after 20 s.
  - `ros2 launch` installs its own handlers, which is probably why S4d-E was not affected. The
    template no longer depends on that.
- **Cleanup ignores zombies.** A stopped process whose parent was interrupted first stays a
  zombie until it is reaped. It is no longer waited for or signalled as if it were running.
- **The session layout is the scorer's:**
  - `<session>/preflight.json`, `attestation.txt`, `observer/`, `run/`;
  - `run/process_scan_{start,end}.json`;
  - captures, node evidence and the recorder log under `run/`.
- **No driver.** The person types into the observer, in their own terminal.
