#!/usr/bin/env python3
"""Assemble the SpiderX Engineering Console into single HTML files. Standard library only.

    python3 tools/engineering_console/build.py

Outputs: dist/spiderx_console.html (stand-alone, open it in a browser) and
dist/artifact.html (same page without the document wrapper, for publishing as an artifact).
It also checks that every repository path cited by the console exists at this commit.
"""
import json
import os
import re
import sys

import extract

HERE = os.path.dirname(os.path.abspath(__file__))
SRC = os.path.join(HERE, 'src')


def read(name):
    with open(os.path.join(SRC, name), encoding='utf-8') as f:
        return f.read()


def check_references(content_js, verbose):
    consts = {'D': 'docs/', 'SRC': 'src/', 'CTL': 'src/spiderx_controller/', 'DSC': 'src/spiderx_description/',
              'RUN3': 'docs/evidence/m61_cloud_one_cycle/20261012T103615Z/', 'E55': 'docs/evidence/m55/'}
    refs = set()
    for m in re.finditer(r"\b(D|SRC|CTL|DSC|RUN3|E55) \+ '([^']+)'", content_js):
        refs.add(consts[m.group(1)] + m.group(2))
    for m in re.finditer(r"'((?:README|STATUS)\.md(?::\d+)?|(?:media|docs|src|tools)/[^']+)'", content_js):
        refs.add(m.group(1))
    bad = []
    for ref in sorted(refs):
        path, _, line = ref.partition(':')
        line = line if line.isdigit() else ''
        path = path.split(' ')[0].split('<')[0]
        full = os.path.join(os.path.dirname(os.path.dirname(HERE)) if path.startswith('tools/') else extract.REPO, path)
        if not os.path.exists(full):
            bad.append(ref)
            continue
        if line and os.path.isfile(full):
            with open(full, encoding='utf-8', errors='replace') as f:
                lines = f.read().splitlines()
            if int(line) > len(lines):
                bad.append(ref + ' (line beyond end of file)')
            elif verbose:
                print(f'{ref:78s} | {lines[int(line) - 1].strip()[:90]}')
    return refs, bad


def main():
    data = extract.extract()
    content = read('05_content.js') + read('06_content2.js') + read('30_ui.js') + read('35_ui2.js')
    refs, bad = check_references(content, '--refs' in sys.argv)
    if bad:
        sys.exit('Cited paths missing from the repository:\n  ' + '\n  '.join(bad))
    payload = json.dumps(data, separators=(',', ':'))
    fonts = ('<link rel="stylesheet" href="https://fonts.googleapis.com/css2?family=Chakra+Petch:wght@500;600'
             '&family=IBM+Plex+Mono:wght@400;500;600&family=IBM+Plex+Sans:wght@400;500;600&display=swap">')
    page = ('<title>SpiderX Command Center</title>\n<meta name="claude:viewport-fit-sides" content="cover">\n'
            + fonts + '\n<style>\n' + read('styles.css') + '</style>\n' + read('body.html')
            + '\n<script>\nconst DATA = ' + payload + ';\n</script>\n<script>\n'
            + '\n'.join(read(n) for n in ('05_content.js', '06_content2.js', '10_robot.js', '20_gl.js', '30_ui.js', '35_ui2.js', '40_boot.js')) + '\n</script>\n')
    os.makedirs(os.path.join(HERE, 'dist'), exist_ok=True)
    with open(os.path.join(HERE, 'dist', 'artifact.html'), 'w', encoding='utf-8') as f:
        f.write(page)
    with open(os.path.join(HERE, 'dist', 'spiderx_console.html'), 'w', encoding='utf-8') as f:
        f.write('<!doctype html>\n<html lang="en">\n<head>\n<meta charset="utf-8">\n'
                '<meta name="viewport" content="width=device-width, initial-scale=1, viewport-fit=cover">\n'
                '</head>\n<body>\n' + page + '</body>\n</html>\n')
    print(f'built: {len(page) / 1e6:.2f} MB, {len(refs)} cited paths checked, snapshot {data["snapshot"]["short"]}')


if __name__ == '__main__':
    main()
