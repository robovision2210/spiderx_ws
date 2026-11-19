#!/usr/bin/env python3
"""Post-merge: read-only test-discovery record for one fresh colcon run. For each package build dir with a
CTestTestfile.cmake: the registered targets (parsed from the file, no ctest run), each target's test source as
written in its command, and the matching xunit result file(s) (path, testcases, failures+errors, newer than the
run start). Also compares the controller's test/test_*.py files with the registered targets (uncollected files are
listed, not failed), and refuses any old SpiderX checkout/installation path in the CTest files.
Usage: discovery_check.py <bases dir> <clone> <run start epoch s>"""
import glob, os, re, sys
import xml.etree.ElementTree as ET
F, WS, t0 = sys.argv[1], sys.argv[2], float(sys.argv[3])
OLD = re.compile(r'/home/user/spiderx_ws|/root/spiderx_fresh_disabled_|/root/spiderx_live_enabled_|/root/spiderx_e_live_enabled_')
ok = True
for ctf in sorted(glob.glob(os.path.join(F, 'build', '*', 'CTestTestfile.cmake'))):
    bdir, txt = os.path.dirname(ctf), open(ctf).read()
    pkg = os.path.basename(bdir)
    tests = re.findall(r'add_test\(\s*(?:\[=\[)?([^\]\s"]+)(?:\]=\])?\s+(.*?)\)\s*\n', txt, re.S)
    if OLD.search(txt):
        print(f'{pkg}: FAIL an old SpiderX path appears in CTestTestfile.cmake'); ok = False
    if not tests:
        print(f'{pkg}: 0 registered targets'); continue
    print(f'{pkg}: {len(tests)} registered targets; CTestTestfile.cmake {ctf}')
    names = set()
    for name, cmd in tests:
        names.add(name)
        srcs = sorted(set(re.findall(r'(/[^\s"\]]+/test/[^\s"\]]+\.py)', cmd)))
        src_ok = bool(srcs) and all(s.startswith(WS + '/src/') and os.path.exists(s) for s in srcs)
        xs = sorted(glob.glob(os.path.join(bdir, 'test_results', pkg, f'{name}.*.xml')))
        cases = fails = 0; fresh = bool(xs)
        for x in xs:
            fresh &= os.path.getmtime(x) >= t0
            r = ET.parse(x).getroot()
            for s in ([r] if r.tag == 'testsuite' else list(r.iter('testsuite'))):
                cases += int(s.get('tests', 0)); fails += int(s.get('failures', 0)) + int(s.get('errors', 0))
        good = src_ok and fresh and fails == 0
        ok &= good
        print(f'  {name}: source {",".join(os.path.relpath(s, WS) for s in srcs) or "(none found)"} (in clone: {src_ok}); '
              f'result {",".join(os.path.relpath(x, F) for x in xs) or "(none)"}; testcases {cases}; failures+errors {fails}; fresh {fresh}'
              f'{"" if good else "  <-- FAIL"}')
    tdir = os.path.join(WS, 'src', pkg, 'test')
    if os.path.isdir(tdir):
        files = sorted(os.path.splitext(f)[0] for f in os.listdir(tdir) if re.match(r'test_.*\.py$', f))
        unreg = [f for f in files if f not in names]
        print(f'{pkg}: test/test_*.py files {len(files)}; registered {len(names)}; files not registered: {unreg or "none"}')
print('DISCOVERY VERDICT:', 'PASS' if ok else 'FAIL')
