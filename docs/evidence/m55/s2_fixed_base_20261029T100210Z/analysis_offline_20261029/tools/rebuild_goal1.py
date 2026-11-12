#!/usr/bin/env python3
"""Rebuild S2 goal 1 (level 0, forward, b0 -> b1) offline from the repository's crawl code and
check it against the fingerprint the node recorded when it dispatched the goal.

    rebuild_goal1.py <expected fingerprint> <out.json>

Pure Python (spiderx_controller from the sourced workspace). No ROS graph, no motion. Designing
and validating the 20 mm template takes about 20 s.
"""
import json
import sys
import time

from spiderx_controller import m55_crawl as cr
from spiderx_controller import m55_locomotion as loc

want, out = sys.argv[1], sys.argv[2]
cfg = loc.load_config()
t0 = time.monotonic()
template = cr.build_template(cr.load_designer(), cfg.crawl_params(cfg.gait.stride_levels_m[0]))
library = loc.PhaseLibrary([template], cfg.dispatch.phase_lead_s)
goal = library.goal(0, 0, 1)
match = goal.fingerprint == want
print(f'built in {time.monotonic() - t0:.1f} s; fingerprint {goal.fingerprint}; match {match}')
d = goal.to_dict()
d['config_sha256'] = cfg.sha256
d['fingerprint_matches_dispatched'] = match
with open(out, 'w') as f:
    json.dump(d, f, indent=1)
sys.exit(0 if match else 1)
