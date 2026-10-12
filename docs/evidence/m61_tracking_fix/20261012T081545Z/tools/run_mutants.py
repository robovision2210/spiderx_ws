#!/usr/bin/env python3
"""Deliberate mutants of the M6.1 tracking-reference fix: each must make the tests FAIL.

Each mutant edits one or two source lines in the working tree, runs the tracking tests and the mock
scenario table, and restores the file byte for byte (SHA-256 checked). Nothing is committed.
Usage (from src/spiderx_controller, environment sourced): run_mutants.py <out.txt>
"""
import hashlib
import subprocess
import sys

TRK = 'spiderx_controller/m61_tracking.py'
GR = 'spiderx_controller/m6_gait_replay.py'
MUTANTS = [   # (name, file, [(old, new), ...]); every old string must occur exactly once
    ('M1 zero time_from_start counts as informative', TRK,
     [('math.isfinite(tfs) and tfs > TFS_MIN_S', 'math.isfinite(tfs) and tfs >= 0.0')]),
    ('M2 negative time_from_start counts as informative (abs)', TRK,
     [('math.isfinite(tfs) and tfs > TFS_MIN_S',
       'math.isfinite(tfs) and abs(tfs) > TFS_MIN_S')]),
    ('M3 offset limit inclusive (>=)', TRK,
     [('abs(est - accept_sim) > offset_limit_s', 'abs(est - accept_sim) >= offset_limit_s')]),
    ('M4 informative estimate ignored (always acceptance)', TRK,
     [('t0, anchor = est, FEEDBACK_ESTIMATE', 't0, anchor = accept_sim, ACCEPTANCE')]),
    ('M5 fallback never recorded', TRK,
     [("'fallback_time_from_start_zero': bool(fb) and not good,",
       "'fallback_time_from_start_zero': False,")]),
    ('M6 feedback timing never captured', GR,
     [('self._feedback_timing.append(feedback_timing(ev[1] if len(ev) > 1 else None))',
       'pass')]),
    ('M7 M6.1 overrides removed: the pre-fix M6.0-D rule', GR,
     [('    def _ref_time(self, stamp):\n        t0, rec = self._anchor()',
       '    def _ref_time_unused(self, stamp):\n        t0, rec = self._anchor()'),
      ('    def _start_estimate(self):\n', '    def _start_estimate_unused(self):\n')]),
    ('M8 start estimate reverted to the unfiltered M6.0-D median', GR,
     [("        return self._anchor()[1]['feedback_estimate_s']",
       '        return lp.LiveSession._start_estimate(self)')]),
    ('M9 abort threshold inclusive (>=)', GR,
     [("return 'tracking_error' if worst > self.limits.client_tracking_abort_rad else None",
       "return 'tracking_error' if worst >= self.limits.client_tracking_abort_rad else None")]),
    ('M10 fallback event never logged', GR,
     [("        if rec['fallback_time_from_start_zero'] and not self._fallback_logged:",
       '        if False:')]),
    ('M11 the fallback skips the in-flight check', GR,
     [('            self._fallback_inflight += 1\n',
       '            self._fallback_inflight += 1\n            return None\n')]),
]
TESTS = ['test/test_m61_tracking.py', 'test/test_m61_gait_replay.py', '-k',
         'tracking or mock_scenario']


def sha(path):
    with open(path, 'rb') as f:
        return hashlib.sha256(f.read()).hexdigest()


def pytest():
    r = subprocess.run([sys.executable, '-m', 'pytest', '-q', '-rf', '-p', 'no:cacheprovider',
                        *TESTS], capture_output=True, text=True)
    lines = r.stdout.splitlines()
    tail = [ln.strip('= ') for ln in lines if ' passed' in ln or ' failed' in ln][-1:]
    failed = sorted({ln.split(' - ')[0].replace('FAILED ', '') for ln in lines
                     if ln.startswith('FAILED')})
    return r.returncode, (tail[0] if tail else '(no summary)'), failed


out = []
rc, tail, _ = pytest()
out.append(f'baseline (unmutated): rc={rc}  {tail}')
assert rc == 0, tail
caught = 0
for name, path, edits in MUTANTS:
    with open(path) as f:
        src = f.read()
    before = sha(path)
    mutated = src
    for old, new in edits:
        assert mutated.count(old) == 1, (name, old, mutated.count(old))
        mutated = mutated.replace(old, new, 1)
    try:
        with open(path, 'w') as f:
            f.write(mutated)
        rc, tail, failed = pytest()
    finally:
        with open(path, 'w') as f:
            f.write(src)
    assert sha(path) == before, f'{path} not restored'
    ok = rc != 0
    caught += ok
    out.append(f'{name}: {"CAUGHT" if ok else "SURVIVED"}  rc={rc}  {tail}')
    out += [f'    {t}' for t in failed[:8]] + ([f'    ... {len(failed) - 8} more']
                                               if len(failed) > 8 else [])
rc, tail, _ = pytest()
out.append(f'after restore: rc={rc}  {tail}')
out.append(f'mutants caught: {caught}/{len(MUTANTS)}')
text = '\n'.join(out) + '\n'
print(text)
with open(sys.argv[1], 'w') as f:
    f.write(text)
sys.exit(0 if caught == len(MUTANTS) and rc == 0 else 1)
