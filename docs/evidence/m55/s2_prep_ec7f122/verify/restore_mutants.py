"""Mutants of the restore-script extension; each must make test_m61_restore_script.py fail."""
import pathlib, subprocess, sys
WS = pathlib.Path('/home/user/spiderx_ws')
P = WS / 'scripts/m61_restore_and_verify_disabled.sh'
M = [
    ('R1_m55_gate_not_imported',
     '                  ("spiderx_controller.m55_contract", "M55_LOCOMOTION_DISPATCH_ENABLED")):',
     '                  ):'),
    ('R2_live_playback_not_checked', 'for TOOL in m6_gait_replay.py m6_live_playback; do',
     'for TOOL in m6_gait_replay.py; do'),
    ('R3_s2_branch_pattern_removed', ' && [[ "$ENABLING" != local/m55-s2-* ]]', ''),
    ('R4_m55_node_not_a_leftover', '|m55_locomotion_node|m55_teleop_keyboard)', ')'),
    ('Z_control', None, None),
]
bad = 0
for name, a, b in M:
    orig = P.read_text()
    try:
        if a:
            assert orig.count(a) == 1, name
            P.write_text(orig.replace(a, b))
        r = subprocess.run([sys.executable, '-m', 'pytest', '-q', '-p', 'no:cacheprovider',
                            'test/test_m61_restore_script.py'], cwd=WS / 'src/spiderx_controller',
                           capture_output=True, text=True, timeout=900)
        last = r.stdout.strip().splitlines()[-1]
        caught = r.returncode != 0
        print(f'{name}: {"CAUGHT" if caught else "SURVIVED"} | {last}', flush=True)
        bad += (caught if name.startswith('Z') else not caught)
    finally:
        P.write_text(orig)
print('all caught, control passes' if bad == 0 else f'PROBLEM: {bad}')
sys.exit(1 if bad else 0)
