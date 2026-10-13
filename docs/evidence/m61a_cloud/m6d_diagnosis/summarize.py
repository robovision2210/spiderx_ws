"""Tabulate diagnostic runs: python3 summarize.py PREFIX [PREFIX ...] (diagnosis only)."""
import glob
import json
import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))


def row(d):
    j = json.load(open(os.path.join(d, 'diag.json')))
    log = open(os.path.join(d, 'pytest.log')).read()
    s = j.get('stack') or {}
    rd = j.get('readiness') or []
    outcome = [o['outcome'] for o in j.get('outcomes', [])]
    fail = ''
    for o in j.get('outcomes', []):
        if o['outcome'] == 'failed' and o.get('longrepr_tail'):
            for line in o['longrepr_tail'].splitlines():
                if line.startswith('E ') and 'AssertionError' in line:
                    fail = line[2:].strip()[:90]
                    break
    return {
        'run': os.path.basename(d), 'domain': j.get('domain'),
        'result': 'pass' if outcome and all(x == 'passed' for x in outcome) else 'FAIL',
        'failure': fail,
        'r1': (rd[0]['ready'], rd[0]['failure_codes']) if len(rd) > 0 else None,
        'r2': (rd[1]['ready'], rd[1]['failure_codes']) if len(rd) > 1 else None,
        'r_samples': [r['samples']['count'] if r.get('samples') else None for r in rd],
        'r_violations': [r['samples']['order_violations'] if r.get('samples') else None
                         for r in rd],
        'tick_overlaps': s.get('tick_overlaps'), 'pub_violations': s.get('publish_order_violations'),
        'never_retrieved': 'never retrieved' in log,
        'peers_before': [len(g['result'].get('nodes', [])) for g in j.get('graph_snapshots', [])],
    }


def main():
    rows = []
    for prefix in sys.argv[1:]:
        for d in sorted(glob.glob(os.path.join(HERE, 'runs', prefix + '_*'))):
            if os.path.exists(os.path.join(d, 'diag.json')):
                rows.append(row(d))
    for r in rows:
        print(json.dumps(r))
    n = len(rows)
    fails = [r for r in rows if r['result'] == 'FAIL']
    print(f'# runs {n}, failed {len(fails)}, never_retrieved {sum(r["never_retrieved"] for r in rows)}'
          f', runs with tick overlaps {sum(1 for r in rows if r["tick_overlaps"])}'
          f', runs with publish-order violations {sum(1 for r in rows if r["pub_violations"])}')


if __name__ == '__main__':
    main()
