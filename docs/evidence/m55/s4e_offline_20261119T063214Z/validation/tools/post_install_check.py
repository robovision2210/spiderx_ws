#!/usr/bin/env python3
"""Post-merge: read-only cross-check of one installation record (imports_gates_library.json, refusal_*.txt)
against the clone and the expected identities. Usage: post_install_check.py <record dir> <install prefix> <clone>"""
import json, os, sys
O, P, WS = sys.argv[1], os.path.abspath(sys.argv[2]), sys.argv[3]
FP = '62f8a10075e90571bfc42afb4635eddd235d8681f589f027734fa5ca2c03e732'
LJ = '917fc4d796efe2344d0edcf038a1f7186d053622df250d9b389690a87d485d8e'
d = json.load(open(os.path.join(O, 'imports_gates_library.json')))
ok, out = True, []
def chk(cond, msg):
    global ok
    ok &= bool(cond); out.append(('PASS ' if cond else 'FAIL ') + msg)
src = os.path.join(WS, 'src', 'spiderx_controller') + os.sep
mods = d['modules']
chk(all(v['imported_as'].startswith(P + os.sep) for v in mods.values()), f'{len(mods)} modules imported from the fresh installation {P}')
chk(all(v['resolves_to'].startswith(src) for v in mods.values()), f'every module resolves (symlink install) into the clone {src}')
chk(set(d['gates']) == {'M55_LOCOMOTION_DISPATCH_ENABLED', 'M61_LIVE_DISPATCH_ENABLED', 'LIVE_DISPATCH_ENABLED'}
    and all(v is False for v in d['gates'].values()), 'installed gates: ' + ', '.join(f'{k}={v}' for k, v in sorted(d['gates'].items())))
lib = d['library']
chk(lib['goals'] == 54, f'phase library goals {lib["goals"]} (expected 54)')
chk(lib['fingerprints_sha256'] == FP, f'fingerprints {lib["fingerprints_sha256"][:8]}... (expected {FP[:8]}...)')
chk(lib['library_json_sha256'] == LJ and lib['library_json_equals_baseline'], f'library.json {lib["library_json_sha256"][:8]}... = disabled baseline {LJ[:8]}...')
c = d['config']
chk(c['path'].startswith(P + os.sep) and c['resolves_to'].startswith(WS + '/src/') and c['equals_src'],
    f'config {os.path.basename(c["path"])} loaded from the fresh installation, resolves into the clone, sha {c["sha256"][:8]}... = src')
for tool in ('m6_gait_replay.py', 'm6_live_playback'):
    p = os.path.join(O, f'refusal_{tool}.txt')
    txt = open(p).read() if os.path.exists(p) else ''
    line = next((l for l in txt.splitlines() if 'HARD-DISABLED' in l), '(no HARD-DISABLED line)')
    chk('HARD-DISABLED' in txt, f'{tool} --live refusal text: {line.strip()[:140]}')
print('\n'.join(out)); print('POST-INSTALL VERDICT:', 'PASS' if ok else 'FAIL'); sys.exit(0 if ok else 1)
