"""S0b mutants of the windowed receive-rate rule. Each mutant edits the source in place, runs the
focused M5.5 tests, and restores the file. A mutant is caught if pytest fails."""
import pathlib
import subprocess
import sys

PKG = pathlib.Path('/home/user/spiderx_ws/src/spiderx_controller')
CMD = PKG / 'spiderx_controller/m55_command.py'
LOC = PKG / 'spiderx_controller/m55_locomotion.py'
TESTS = ['test/test_m55_rate_rule.py', 'test/test_m55_command.py',
         'test/test_m55_locomotion.py', 'test/test_m55_review.py']
M = [
    ('M1_burst_cap_ge', CMD, [('return burst > self.max_burst', 'return burst >= self.max_burst')]),
    ('M2_sustained_cap_ge', CMD, [('if len(self._recv) > self.max_in_window + 1e-9:',
                                   'if len(self._recv) >= self.max_in_window - 1e-9:')]),
    ('M3_rate_window_edge_inclusive', CMD,
     [('self._recv[0] <= wall - self.rate_window_s + 1e-9', 'self._recv[0] < wall - self.rate_window_s - 1e-9')]),
    ('M4_burst_window_edge_inclusive', CMD,
     [('t > wall - self.burst_window_s + 1e-9', 't >= wall - self.burst_window_s - 1e-9')]),
    ('M5_rejected_not_counted', CMD,
     [('        self._recv.append(wall)\n', ''),
      ('        self.last_valid_wall = wall\n', '        self._recv.append(wall)\n        self.last_valid_wall = wall\n')]),
    ('M6_burst_check_removed', CMD, [('return burst > self.max_burst', 'return False')]),
    ('M7_sustained_check_removed', CMD, [('if len(self._recv) > self.max_in_window + 1e-9:', 'if False:')]),
    ('M8_no_stop_on_rate_reject', CMD,
     [('        if self._rate_exceeded(wall):\n            self.intent = STOP\n',
       '        if self._rate_exceeded(wall):\n')]),
    ('M9_max_burst_lower_bound_removed', LOC, [(' or c.max_burst < 2:', ':')]),
    ('M10_session_ignores_max_burst', LOC,
     [('cfg.command.burst_window_s, cfg.command.max_burst)', 'cfg.command.burst_window_s, 1000)')]),
    ('M11_old_pair_rule_restored', CMD,
     [('        burst = sum(1 for t in self._recv if t > wall - self.burst_window_s + 1e-9)\n'
       '        return burst > self.max_burst',
       '        return len(self._recv) > 1 and self._recv[-1] - self._recv[-2] < 0.02')]),
    ('Z_unmutated_control', CMD, []),
]


def main():
    results = []
    for name, path, edits in M:
        orig = path.read_text()
        try:
            src = orig
            for a, b in edits:
                assert src.count(a) == 1, (name, a)
                src = src.replace(a, b)
            path.write_text(src)
            p = subprocess.run([sys.executable, '-m', 'pytest', '-q', '-p', 'no:cacheprovider', '-x']
                               + TESTS, cwd=PKG, capture_output=True, text=True, timeout=600)
            tail = [ln for ln in p.stdout.splitlines() if ln.startswith('FAILED') or 'passed' in ln
                    or 'failed' in ln][-2:]
            caught = p.returncode != 0
            results.append((name, caught, tail))
            print(f'{name}: {"CAUGHT" if caught else "SURVIVED"} rc={p.returncode} | ' + ' | '.join(tail),
                  flush=True)
        finally:
            path.write_text(orig)
    mutants = [r for r in results if not r[0].startswith('Z_')]
    control = [r for r in results if r[0].startswith('Z_')][0]
    print(f'caught {sum(r[1] for r in mutants)}/{len(mutants)}; control passes: {not control[1]}')
    return 0 if all(r[1] for r in mutants) and not control[1] else 1


if __name__ == '__main__':
    sys.exit(main())
