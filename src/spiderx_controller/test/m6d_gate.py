"""The ONE test-side expectation of the M6.0-D live gate (tests only).

m6_live_contract.LIVE_DISPATCH_ENABLED must equal this value. The owner-approved enabling commit
changes exactly two lines: that constant and this one (docs/M6D_LIVE_ENABLING_DESIGN.md §4).
Tests of the disabled and enabled paths set the gate explicitly, so they stay valid either way.
"""

EXPECTED_LIVE_DISPATCH_ENABLED = False
