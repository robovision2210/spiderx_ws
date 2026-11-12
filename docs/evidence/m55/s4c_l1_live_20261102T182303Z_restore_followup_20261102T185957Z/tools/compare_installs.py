#!/usr/bin/env python3
"""S4c follow-up (read-only, informational): the fresh installation A against the normal
installation B, file by file (bytecode caches excluded). Same relative paths; symlinks resolve to
the same target; regular files have the same content once each installation's own install and
build base paths are replaced by placeholders. Nothing is modified.
Usage: compare_installs.py <A install> <A build> <B install> <B build> <out json>"""
import hashlib
import json
import os
import sys


def walk(root):
    out = {}
    for d, dirs, files in os.walk(root):
        names = files + [x for x in dirs if os.path.islink(os.path.join(d, x))]
        dirs[:] = [x for x in dirs if x != '__pycache__' and not os.path.islink(os.path.join(d, x))]
        for f in names:
            if f.endswith('.pyc'):
                continue
            p = os.path.join(d, f)
            out[os.path.relpath(p, root)] = p
    return out


def placeholder(text, inst, build):
    return text.replace(build, '<BUILD>').replace(inst, '<INSTALL>')


def describe(path, inst, build):
    if os.path.islink(path):
        return 'link', placeholder(os.path.realpath(path), inst, build)
    with open(path, 'rb') as f:
        data = f.read().decode('latin-1')
    return 'file', hashlib.sha256(placeholder(data, inst, build).encode('latin-1')).hexdigest()


Ai, Ab, Bi, Bb, outp = sys.argv[1:6]
A, B = walk(Ai), walk(Bi)
same, differ = 0, []
for rel in sorted(set(A) & set(B)):
    a, b = describe(A[rel], Ai, Ab), describe(B[rel], Bi, Bb)
    if a == b:
        same += 1
    else:
        differ.append({'path': rel, 'A': a, 'B': b})
res = {'A_install': Ai, 'B_install': Bi, 'files_A': len(A), 'files_B': len(B), 'same': same,
       'differ': differ, 'only_in_A': sorted(set(A) - set(B)), 'only_in_B': sorted(set(B) - set(A))}
with open(outp, 'w') as f:
    json.dump(res, f, indent=1, sort_keys=True)
print(json.dumps({k: (len(v) if isinstance(v, list) else v) for k, v in res.items()}))
for d in differ[:20]:
    print('differs:', d['path'])
for p in res['only_in_A'][:10]:
    print('only in A:', p)
for p in res['only_in_B'][:10]:
    print('only in B:', p)
