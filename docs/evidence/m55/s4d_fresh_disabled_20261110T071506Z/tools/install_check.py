#!/usr/bin/env python3
"""S4c follow-up: the Python side of one installation check, run from / in a fresh shell where
ONLY this installation is sourced (after the toolchain and the r2 overlay). Records the module
paths (as imported, and resolved), the three gates, the phase library (the S2 fingerprint method,
unchanged) and library.json (the node's own format) against the disabled baseline, and the config
file the node loads. Exit 0 only if all three gates are False and every module comes from this
installation. Usage: install_check.py <install prefix> <baseline library.json> <out dir>"""
import hashlib
import importlib
import json
import os
import sys

prefix, base_lib, out = os.path.abspath(sys.argv[1]), sys.argv[2], sys.argv[3]
WS = '/home/user/spiderx_ws'


def sha(path):
    with open(path, 'rb') as f:
        return hashlib.sha256(f.read()).hexdigest()


res, ok = {'install_prefix': prefix, 'modules': {}, 'gates': {}}, True
for mod in ('m55_contract', 'm61_live_contract', 'm6_live_contract', 'm55_locomotion', 'm55_teleop',
            'm6_action_client'):
    m = importlib.import_module('spiderx_controller.' + mod)
    inside = os.path.abspath(m.__file__).startswith(prefix + os.sep)
    res['modules'][mod] = {'imported_as': m.__file__, 'resolves_to': os.path.realpath(m.__file__),
                           'imported_from_this_installation': inside}
    ok &= inside
for mod, name in (('m55_contract', 'M55_LOCOMOTION_DISPATCH_ENABLED'),
                  ('m61_live_contract', 'M61_LIVE_DISPATCH_ENABLED'),
                  ('m6_live_contract', 'LIVE_DISPATCH_ENABLED')):
    v = getattr(importlib.import_module('spiderx_controller.' + mod), name)
    res['gates'][name] = v
    ok &= v is False
from spiderx_controller import m55_contract as c55  # noqa: E402
from spiderx_controller import m55_locomotion as loc  # noqa: E402
cfg_path = loc.config_path()
cfg = loc.load_config()
lib = loc.build_library(cfg)
fps = sorted(lib.fingerprints)
res['config'] = {'path': cfg_path, 'resolves_to': os.path.realpath(cfg_path),
                 'sha256': sha(cfg_path),
                 'src_sha256': sha(os.path.join(WS, 'src/spiderx_controller/config',
                                                loc.CONFIG_FILE))}
res['config']['equals_src'] = res['config']['sha256'] == res['config']['src_sha256']
res['library'] = {'goals': len(fps),
                  'fingerprints_sha256': hashlib.sha256('\n'.join(fps).encode()).hexdigest()}
lj = os.path.join(out, 'library.json')
with open(lj, 'w') as f:            # exactly as the S4 library_json.py tool writes it
    json.dump({'config': cfg.to_dict(), 'library': lib.describe(),
               'non_claims': list(c55.NON_CLAIMS)}, f, indent=1, sort_keys=True)
res['library']['library_json_sha256'] = sha(lj)
res['library']['baseline_library_json_sha256'] = sha(base_lib)
res['library']['library_json_equals_baseline'] = sha(lj) == sha(base_lib)
res['all_gates_false_and_from_this_installation'] = bool(ok)
with open(os.path.join(out, 'imports_gates_library.json'), 'w') as f:
    json.dump(res, f, indent=1, sort_keys=True)
print(json.dumps(res, indent=1, sort_keys=True))
sys.exit(0 if ok else 1)
