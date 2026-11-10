Harness tests before the launch (2026-11-10; stub sandboxes in the session scratchpad; no simulator, recorder or driver).
The tested watchdog.sh, cleanup.sh and s4d_shadow_run.sh are byte-identical to tools/ (cmp), except the run-script copy's
three stubbed lines (run_script_test_copy.diff: captures, recorder and driver replaced so nothing real could start).
W1 watchdog, 6 s deadline: fired after 10 s (5 s polling), touched stopped.flag, ran cleanup.sh then restore_cmd.sh (stubs).
W2 watchdog, stopped.flag at 2 s: exited after 5 s without acting.
W3 watchdog, no argument: deadline = start + 7200 s; stood down.
C1 INVALID setup (kept for the record): started from an interactive shell with job control, so setsid forked and the
   recorded pids were short-lived wrappers; cleanup.sh correctly killed nothing it could not attribute and listed the
   dummies as still present. The dummies were then stopped by pid.
C2 owned-process cleanup (non-interactive, as the harness): driver (pid file), teleop pty session (pty.fork, as
   s2_driver.spawn; pid from driver.jsonl), recorder group, launch session with a SIGINT-ignoring member in its own
   process group, Xvfb: all stopped (SIGINT, then SIGKILL fallbacks where the dummies ignored SIGINT); an unowned decoy
   with "m55_teleop_keyboard" in its command line was left untouched and reported.
S1 no watchdog -> REFUSED before the launch (OD8). S2 watchdog 6000 s -> REFUSED (OD8: 5998 s < 6300 s).
S3 watchdog 12000 s -> OD8 PASS, OD16 FAIL on the bag filesystem (need 24.7 GB > 22.4 GB) -> REFUSED.
S4 watchdog 7200 s -> OD8 and OD16 PASS (need 14.88 GB), stub launch, then with the deadline file shortened to
   simulate time used: the pre-key OD8 check FAILED -> ABORT, no key pressed (the 300 s delay before "controllers" was
   a stub artifact: the stub's dummy Xvfb held the tee pipe; the real stage_start.sh redirects both background processes).
RI run_install_gates.sh dry run on the fresh installation: PASS; score_s4d's M15 check reads the JSON as True.
