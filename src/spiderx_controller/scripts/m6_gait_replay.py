#!/usr/bin/env python3
"""M6.1 protected trot gait replay: --dry-run and --mock only; --live is HARD-DISABLED in this
build (m61_live_contract.M61_LIVE_DISPATCH_ENABLED = False; see
spiderx_controller/m6_gait_replay.py).

    ros2 run spiderx_controller m6_gait_replay.py --dry-run
    echo SEND-ONE-TROT-CYCLE | ros2 run spiderx_controller m6_gait_replay.py --mock --scenario success
"""
import sys

from spiderx_controller.m6_gait_replay import main

if __name__ == '__main__':
    sys.exit(main())
