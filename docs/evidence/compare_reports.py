#!/usr/bin/env python3
"""Compare two regenerated JSON reports, ignoring declared nondeterministic fields.

    python3 docs/evidence/compare_reports.py A.json B.json [--ignore levels[].build_s ...]

Prints the SHA-256 of each file, of each file's canonical deterministic part (sorted keys,
ignored fields removed), and every differing path. Exit 0 = the deterministic parts are equal,
1 = they differ, 2 = usage error. Standard library only.

A path is dotted keys; `[]` means "every element of this list", e.g. `levels[].build_s`.
"""
import argparse
import hashlib
import json
import sys


def _strip(node, parts):
    if not parts:
        return
    head, rest = parts[0], parts[1:]
    if head.endswith('[]'):
        lst = node.get(head[:-2]) if isinstance(node, dict) else None
        for item in lst or []:
            if rest:
                _strip(item, rest)
        return
    if not isinstance(node, dict) or head not in node:
        return
    if rest:
        _strip(node[head], rest)
    else:
        del node[head]


def deterministic(doc, ignore):
    doc = json.loads(json.dumps(doc))
    for path in ignore:
        _strip(doc, path.split('.'))
    return doc


def canonical(doc):
    return json.dumps(doc, sort_keys=True, separators=(',', ':'))


def diff(a, b, path=''):
    if type(a) is not type(b):
        return [path or '<root>']
    if isinstance(a, dict):
        out = []
        for k in sorted(set(a) | set(b)):
            p = f'{path}.{k}' if path else k
            if k not in a or k not in b:
                out.append(p)
            else:
                out += diff(a[k], b[k], p)
        return out
    if isinstance(a, list):
        if len(a) != len(b):
            return [f'{path} (length {len(a)} != {len(b)})']
        out = []
        for i, (x, y) in enumerate(zip(a, b)):
            out += diff(x, y, f'{path}[{i}]')
        return out
    return [] if a == b else [path]


def main(argv=None):
    p = argparse.ArgumentParser(description=__doc__.split('\n')[0])
    p.add_argument('a')
    p.add_argument('b')
    p.add_argument('--ignore', nargs='*', default=[])
    args = p.parse_args(argv)
    docs = []
    for path in (args.a, args.b):
        with open(path, 'rb') as f:
            raw = f.read()
        doc = deterministic(json.loads(raw), args.ignore)
        print(f'{path}\n  file sha256          {hashlib.sha256(raw).hexdigest()}\n'
              f'  deterministic sha256 {hashlib.sha256(canonical(doc).encode()).hexdigest()}')
        docs.append(doc)
    d = diff(*docs)
    print(f'ignored: {args.ignore or "nothing"}')
    if d:
        print(f'DIFFERENT in {len(d)} deterministic path(s):')
        for x in d[:50]:
            print(f'  {x}')
        return 1
    print('deterministic parts IDENTICAL')
    return 0


if __name__ == '__main__':
    sys.exit(main())
