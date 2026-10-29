#!/usr/bin/env python3
"""Digest of the M5.5 phase library exactly as the node builds it (load_config + build_library):
sorted goal fingerprints -> SHA-256. Prints JSON."""
import hashlib, json
from spiderx_controller import m55_contract as c55
from spiderx_controller import m55_locomotion as loc
lib = loc.build_library(loc.load_config())
fps = sorted(lib.fingerprints)
print(json.dumps({'goals': len(fps), 'fingerprints_sha256': hashlib.sha256('\n'.join(fps).encode()).hexdigest(),
                  'm55_gate_imported': c55.M55_LOCOMOTION_DISPATCH_ENABLED}))
