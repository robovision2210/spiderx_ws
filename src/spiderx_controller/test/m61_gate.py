"""The committed M6.1 live-dispatch gate expectation (separate from test/m6d_gate.py).

Enabling M6.1 live dispatch is a separate, owner-approved change on a branch that is never merged:
it flips m61_live_contract.M61_LIVE_DISPATCH_ENABLED and this expectation together, and nothing
else. On main both stay False.
"""
EXPECTED_M61_LIVE_DISPATCH_ENABLED = False
