#!/usr/bin/env bash
RUN=$1; D=$RUN/premotion; mkdir -p $D
echo "PASS: one process maps the plugin and it belongs to this launch: 999 (STUB)" > $D/candidates.txt
printf 'libgz_ros2_control-system.so: STUB -> PASS\nLIBRARY VERDICT: PASS\n' > $D/library_checks.txt
echo "node action clients: 1, of which the trajectory action: 1; publishers towards joint_trajectory: 0; launch-log DISPATCH lines: 1 (STUB)" > $D/ownership.txt
if [ "$2" = PASS ]; then echo "PREMOTION VERDICT: PASS" | tee $D/verdict.txt; else echo "PREMOTION VERDICT: FAIL: node_shadow_log action_clients (STUB: wrong mode)" | tee $D/verdict.txt; fi
