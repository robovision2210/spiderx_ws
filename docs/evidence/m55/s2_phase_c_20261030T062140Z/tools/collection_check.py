#!/usr/bin/env python3
"""Fresh-collection check of one colcon test run (Phase C step 2). For every ament_cmake package:
the registered CTest targets (ctest -N) against this run's CTest Test.xml entries and the
per-target xunit files, all newer than the run start. pytest cases (xunit testcases) and CTest
wrapper entries are counted separately; their sum must equal colcon test-result's total.
Usage: collection_check.py <workspace> <run start epoch s> <colcon_test_result.txt>"""
import glob
import os
import re
import subprocess
import sys
import xml.etree.ElementTree as ET

ws, t0, result_txt = sys.argv[1], float(sys.argv[2]), sys.argv[3]
ok = True
pytest_cases = ctest_entries = 0
print(f'workspace {ws}; run start {t0}')
for ctf in sorted(glob.glob(os.path.join(ws, 'build', '*', 'CTestTestfile.cmake'))):
    bdir = os.path.dirname(ctf)
    pkg = os.path.basename(bdir)
    out = subprocess.run(['ctest', '-N'], cwd=bdir, capture_output=True, text=True).stdout
    registered = re.findall(r'Test\s+#\d+:\s+(\S+)', out)
    tags = sorted(glob.glob(os.path.join(bdir, 'Testing', '*', 'Test.xml')), key=os.path.getmtime)
    if not registered:
        print(f'{pkg}: 0 registered CTest targets')
        continue
    if not tags or os.path.getmtime(tags[-1]) < t0:
        print(f'{pkg}: FAIL no fresh CTest Test.xml'); ok = False
        continue
    root = ET.parse(tags[-1]).getroot()
    status = {t.findtext('Name'): t.get('Status') for t in root.iter('Test') if t.get('Status')}
    ctest_entries += len(status)
    missing, bad, stale, cases, fails = [], [], [], 0, 0
    for name in registered:
        if status.get(name) != 'passed':
            bad.append(f'{name}={status.get(name)}')
        xs = glob.glob(os.path.join(bdir, 'test_results', pkg, f'{name}.*.xml'))
        if not xs:
            missing.append(name)
            continue
        for x in xs:
            if os.path.getmtime(x) < t0:
                stale.append(os.path.basename(x))
            r = ET.parse(x).getroot()
            suites = [r] if r.tag == 'testsuite' else list(r.iter('testsuite'))
            for s in suites:
                cases += int(s.get('tests', 0))
                fails += int(s.get('failures', 0)) + int(s.get('errors', 0))
    pytest_cases += cases
    good = not (missing or bad or stale or fails)
    ok &= good
    print(f'{pkg}: {len(registered)} registered CTest targets, {len(status)} CTest entries, '
          f'{cases} xunit testcases, failures+errors {fails}; missing result {missing or "none"}; '
          f'not passed {bad or "none"}; stale {stale or "none"} -> {"PASS" if good else "FAIL"}')
for px in sorted(glob.glob(os.path.join(ws, 'build', '*', 'pytest.xml'))):   # ament_python packages
    r = ET.parse(px).getroot()
    n = sum(int(s.get('tests', 0)) for s in ([r] if r.tag == 'testsuite' else r.iter('testsuite')))
    fresh = os.path.getmtime(px) >= t0
    pytest_cases += n if fresh else 0
    print(f'{os.path.basename(os.path.dirname(px))}: pytest.xml {n} testcases, fresh {fresh}')
total = re.findall(r'Summary:\s+(\d+) tests, (\d+) errors, (\d+) failures, (\d+) skipped', open(result_txt).read())
print(f'xunit testcases {pytest_cases} + CTest entries {ctest_entries} = {pytest_cases + ctest_entries}; '
      f'colcon test-result: {total[-1] if total else "no summary"}')
if not total or int(total[-1][0]) != pytest_cases + ctest_entries or any(int(v) for v in total[-1][1:]):
    ok = False
print('COLLECTION VERDICT:', 'PASS' if ok else 'FAIL')
