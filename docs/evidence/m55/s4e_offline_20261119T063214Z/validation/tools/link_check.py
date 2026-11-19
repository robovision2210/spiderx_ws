#!/usr/bin/env python3
"""Post-merge: relative Markdown links in every tracked *.md of a checkout (read-only). A link is checked when its
target is not a URL or an in-page anchor; its path (anchor stripped) must exist relative to the linking file.
Usage: link_check.py <checkout>   Prints totals per scope (top-level and docs/*.md; evidence; other) and every broken link."""
import os, re, subprocess, sys
ws = sys.argv[1]
files = subprocess.run(['git', '-C', ws, 'ls-files', '*.md'], capture_output=True, text=True).stdout.split()
LINK = re.compile(r'\[[^\]]*\]\(([^)\s]+)\)')
tot, bad = {}, []
for f in files:
    scope = 'evidence' if f.startswith('docs/evidence/') else ('top+docs' if '/' not in f or (f.startswith('docs/') and f.count('/') == 1) else 'other')
    txt = open(os.path.join(ws, f), encoding='utf-8', errors='replace').read()
    for m in LINK.finditer(txt):
        t = m.group(1)
        if re.match(r'^([a-z]+:|#)', t):
            continue
        tot[scope] = tot.get(scope, 0) + 1
        p = os.path.normpath(os.path.join(ws, os.path.dirname(f), t.split('#')[0]))
        if not os.path.exists(p):
            bad.append((scope, f, t))
print(f'markdown files {len(files)}; relative links checked: ' + ', '.join(f'{k} {v}' for k, v in sorted(tot.items())))
for k in sorted(tot):
    print(f'{k}: broken {sum(1 for b in bad if b[0] == k)}')
for b in bad:
    print('BROKEN', b)
print('LINK VERDICT:', 'PASS' if not [b for b in bad if b[0] == 'top+docs'] else 'FAIL (top-level/docs)', '| all scopes broken:', len(bad))
