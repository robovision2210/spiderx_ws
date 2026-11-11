S4d-E shadow harness stub tests, 2026-11-11, before the watchdog and the one launch (sandbox in the session scratchpad;
no simulator, recorder, driver or ROS graph). The E run script was tested as a copy with only these substitutions
(run_script_test_copy.diff): the run installation path (a fake installation holding only the three gate modules, all
False; or M55 True for S2; or a nonexistent path for S1), the workspace (a scratch repo whose scripts/s4/s4d_e_driver.py
is a stub that logs and exits 4 on SIGINT/SIGTERM), the recorder (a stub process) and the captures. cleanup.sh,
watchdog.sh and s4d_e_shadow_guarded.sh are the run copies (byte-identical); the launch stages and restore are stubs.
Refusals before the launch (nothing launched, no key): S1 run installation not in the environment; S2 gates not all
False; S3 no watchdog; S4 a 2000 s watchdog (< the E cap 1800 + 900 s, OD8); S5 the disk rule (OD16, 12000 s).
After a stub launch: S6 OD8 before the first key (aborted, no key, no driver); S7 OD16 before recording (aborted: no
recording, no key). Watchdog: W1 fires (cleanup, then restore, both stubs); W2 stands down on stopped.flag; W3 7200 s.
Interruption (SIGTERM to the guarded wrapper's group): I1 during the pre-key wait, I2 while the stub E driver ran: the
trap ran cleanup.sh (TERM, then EXIT; idempotent); the launch session, Xvfb and the recorder group were stopped; in I2
the driver exited on SIGTERM. C2 owned cleanup with the E driver's command line: the recorded driver, teleop pty
session, recorder group, launch session (a SIGINT-ignoring member included) and Xvfb stopped; an UNOWNED decoy with the
same driver command line survived; colcon test processes running at the time were listed, not touched. (The member
counts printed right after cleanup include unreaped, defunct processes; none remained after the test shell exited.)
