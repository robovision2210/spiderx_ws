#!/usr/bin/env python3
"""library.json exactly as m55_rclpy writes it when the node starts (its config, library.describe()
and non-claims), from load_config() + build_library(). It is the disabled-build basis of the frozen
scorer's P7 without an extra launch. Usage: library_json.py <out library.json>"""
import json
import sys

from spiderx_controller import m55_contract as c55
from spiderx_controller import m55_locomotion as loc

cfg = loc.load_config()
library = loc.build_library(cfg)
with open(sys.argv[1], 'w') as f:
    json.dump({'config': cfg.to_dict(), 'library': library.describe(),
               'non_claims': list(c55.NON_CLAIMS)}, f, indent=1, sort_keys=True)
print(json.dumps({'m55_gate_imported': c55.M55_LOCOMOTION_DISPATCH_ENABLED, 'out': sys.argv[1]}))
