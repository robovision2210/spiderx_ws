S4d-M LIVE harness stub tests, 2026-11-11, before the watchdog and enabling (sandbox in the session scratchpad; no simulator,
recorder, driver or ROS graph). The live run script was tested as a copy with only these substitutions
(run_script_test_copy_*.diff): the launch stages, the recorder and the captures are stubs; for the "fake" cases also the run
installation (a fake installation holding only the three gate modules), the scratch git repository with a
local/m55-s4d-m-TEST branch differing from its base by exactly the two-line pattern, and the stub driver at its path.
cleanup.sh, watchdog.sh and s4d_live_guarded.sh are the live copies (byte-identical).
Refusals before the launch (nothing launched, no key): L1 the run installation missing (real paths); L2 gates not exactly
True/False/False; L3 no recorded enabling branch; L4 another branch checked out; L4w a diff wider than the two-line patch;
L5 no watchdog; L6 a 6000 s watchdog (OD8); L7 the disk rule (OD16, 12000 s). After a stub launch: L8 OD8 before the first key
(aborted, no key); L9 OD16 before recording (aborted: no recording, no key).
Watchdog: W1 fires (cleanup.sh, then restore_cmd.sh, both stubs); W2 stands down on stopped.flag without acting; W3 7200 s default.
Interruption (SIGTERM to the guarded wrapper's process group): I1 during the pre-key wait (before the driver) and I2 while the
stub driver ran: the trap ran cleanup.sh (twice: TERM, then EXIT; it is idempotent); the stub launch session, Xvfb and the
recorder group were stopped; in I2 the driver exited on SIGTERM.
C2 owned-process cleanup: the driver, its teleop pty session, the recorder group, the launch session (a SIGINT-ignoring member
included) and Xvfb stopped through their recorded pids/groups/sessions; the unowned decoy survived.
