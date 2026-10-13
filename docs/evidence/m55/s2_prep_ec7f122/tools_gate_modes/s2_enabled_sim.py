"""pytest plugin: simulate the S2 enabling patch IN MEMORY (no file is changed or applied).

Sets m55_contract.M55_LOCOMOTION_DISPATCH_ENABLED and the pin m55_gate.EXPECTED_... to True
after collection, and makes test_m55_locomotion's source reader see the patched gate line, so
source scans see what an enabled checkout would contain. Use: -p s2_enabled_sim."""
import sys

GATE_OFF = 'M55_LOCOMOTION_DISPATCH_ENABLED = False'
GATE_ON = 'M55_LOCOMOTION_DISPATCH_ENABLED = True'


def pytest_collection_finish(session):
    from spiderx_controller import m55_contract as c55
    c55.M55_LOCOMOTION_DISPATCH_ENABLED = True
    gate = sys.modules.get('m55_gate')
    if gate is not None:
        gate.EXPECTED_M55_LOCOMOTION_DISPATCH_ENABLED = True
    for name, mod in list(sys.modules.items()):
        if name.startswith('test_m55') and hasattr(mod, '_package_sources'):
            orig = mod._package_sources

            def patched(orig=orig):
                for n, src in orig():
                    if n == 'm55_contract.py':
                        assert src.count('\n' + GATE_OFF + '\n') == 1
                        src = src.replace('\n' + GATE_OFF + '\n', '\n' + GATE_ON + '\n')
                    yield n, src
            mod._package_sources = patched
    print(f'\n[s2_enabled_sim] gate {c55.M55_LOCOMOTION_DISPATCH_ENABLED}, pin '
          f'{getattr(gate, "EXPECTED_M55_LOCOMOTION_DISPATCH_ENABLED", None)}')
