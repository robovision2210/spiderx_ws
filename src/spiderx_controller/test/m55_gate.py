"""The committed M5.5 continuous-locomotion dispatch gate expectation (separate from
test/m6d_gate.py and test/m61_gate.py).

Enabling M5.5 dispatch is a separate, owner-approved change after the local acceptance phases
(docs/M55_KEYBOARD_WALKING.md): it flips m55_contract.M55_LOCOMOTION_DISPATCH_ENABLED and this
expectation together, and nothing else. On main both stay False.
"""
EXPECTED_M55_LOCOMOTION_DISPATCH_ENABLED = False
