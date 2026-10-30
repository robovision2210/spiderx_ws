# Response-law analyzer: regression on the 2026-10-29 S2 bag

**What this is.** Tool validation, not a new run: `scripts/s2/response_law.py` applied, read-only, to the original S2 bag.
- Bag: `~/spiderx_evidence/s2_run_20261029T100210Z/bag/bag_0.db3`, outside the repository.
- Its SHA-256 `53d093da…d816` is identical before and after reading: see `inputs` in each `response_law.json`.
- The original analysis (`../s2_fixed_base_20261029T100210Z/analysis_offline_20261029/`) and its tool are unchanged.

| Run | Exit | Plant verdict | Pooled ĝ | Residual max |
|---|---|---|---|---|
| `expected_0.1/` | 0 | **SUPPORTED** | 0.0999999999718, from 393 recorded-command updates (4716 joint-updates) | 1.9e-12 rad |
| `expected_1.0/` | 1 | **CONTRADICTED** | the same fit; it differs from 1.0 | — |

Details of the expected-0.1 run:
- **Per joint:** all 12 joints are identified, each within 2.0e-10 of 0.1.
- **Per segment:** tracking 0.0999999999712; the hold after the cancel 0.1000000003; the idle period before the goal is UNMEASURED (unexcited).
- **Evidence:** 367 controller_state updates, 18 feedback-only updates, 8 updates recorded by both (identical commands). 37 controller_state samples at rest were excluded as ambiguous. Controller_state header stamps trail their update by 0–8 ms.

**Tracking.** Reported separately, on the P2 basis (action feedback); identical in both runs. It reproduces the original findings:
- FAIL: max |error| 0.020972637967721877 rad, `rr_thigh_joint`, at 74.07 s;
- first crossing at 74.00 s (`rf_thigh_joint`);
- 3 of 26 samples above 0.02 rad;
- the node status maps goal 1, with its cancel sent.

**What it shows.** The analyzer recovers the recorded plant (g = 0.1) without assuming it, and rejects a wrong expected gain. It is an offline check of a completed bag; it does not stop runs. **S2 stays FAIL.**

`SHA256SUMS` covers the files here.
