# S4e run-local harness template: parameters (docs/S4E_CONTRACT.md sections 3, 5, 6 and 11).
# NOT AUTHORIZED TO RUN. Copy this directory to a run-local directory OUTSIDE the repository and
# set these for one session; every external action goes through a command below, so that the
# template's tests can replace each one with a stub. Sourced by every script here.
: "${S4E_MODE:=shadow}"                          # shadow | live (live needs its own authorization)
: "${WS:=$HOME/spiderx_ws}"                      # the checkout the run installation was built from
: "${RUNINST:=}"                                 # the run installation (colcon install base)
: "${OLD_INSTALL:=$WS/install}"                  # an installation that must NOT be in the environment
: "${BASELINE:=}"                                # the disabled baseline commit
: "${ENABLING_PATTERN:=local/m55-s4e-h-*}"       # live only (contract section 9, D9)
: "${WATCHDOG_S:=3600}"                          # D11: armed before the launch
: "${SESSION_CAP_S:=1200}"; : "${AFTER_S:=900}"  # D11: 20 min + 15 min must remain
: "${NODE_UP_WAIT_S:=400}"; : "${PREMOTION_WAIT_S:=1500}"; : "${SESSION_WAIT_S:=1800}"
: "${DISK_MIN_B:=5368709120}"                    # 5 GiB (S4d OD16)
: "${RATE_B_S:=1026667}"                         # 2 x 30.8 MB per wall minute: a PLANNING ASSUMPTION
: "${RESTORE_RESERVE_B:=100000000}"
: "${GATES_CMD:=cd / && python3 -c 'from spiderx_controller import m55_contract as a, m61_live_contract as b, m6_live_contract as c; print(a.M55_LOCOMOTION_DISPATCH_ENABLED, b.M61_LIVE_DISPATCH_ENABLED, c.LIVE_DISPATCH_ENABLED, a.__file__)'}"
: "${DISK_FREE_CMD:=python3 -c 'import os, sys; s = os.statvfs(sys.argv[1]); print(s.f_bavail * s.f_frsize)'}"
: "${LAUNCH_CMD:=ros2 launch spiderx_bringup fortress_m55_walking.launch.py evidence_dir=$RUN/node}"
: "${BAG_CMD:=ros2 bag record -s sqlite3 --include-hidden-topics -o $BAG /spiderx/locomotion/status /joint_states /spiderx/sim/world_poses /cmd_vel /leg_trajectory_controller/joint_trajectory /leg_trajectory_controller/controller_state /leg_trajectory_controller/follow_joint_trajectory/_action/feedback}"
: "${CAPTURE_CMD:=timeout 20 ros2 topic info -v /leg_trajectory_controller/joint_trajectory}"
: "${RESTORE_CMD:=bash $WS/scripts/m61_restore_and_verify_disabled.sh}"
: "${DRIVER_PATTERN:=s2_driver.py|s3_driver.py|s4a_driver.py|s4b_driver.py|s4c_driver.py|s4d_driver.py|s4d_e_driver.py}"
