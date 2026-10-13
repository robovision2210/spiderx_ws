# Criterion 6, version 2: frozen definition, tool and self-test

These files are frozen before the single validation observation (`run_04`). They change nothing
about runs 1–3: there, version 1 (individual ranges within ±8 mm) is the criterion of record and
it failed. The rationale and derivation are in
[`docs/M61A_CRITERION6_V2_PROPOSAL.md`](../../../M61A_CRITERION6_V2_PROPOSAL.md).

| File | Content |
|---|---|
| `crit6_v2_spec.json` | The frozen definition: objective (δ = G8 3 mm / 0.01 rad), sensor model, metric, uncertainty, decision targets (1 % / 5 %), thresholds (τ 1.946 mm / 6.103 mrad, σ_max 0.641 mm / 2.369 mrad), prerequisites, beam exclusions, fit settings, counterfactual offsets |
| `crit6_v2.py` | Evaluates one capture: version 1 (reported unchanged, plus the minimax over planar poses), version 2 (verdict) and the counterfactual. It refuses to run if the ray caster's SHA-256 differs from the spec |
| `selftest_crit6_v2.py` | Synthetic captures only: no discrepancy gives PASS; a body displaced 3 mm while `pose/info` reports the weld gives FAIL; 30 scans give INCONCLUSIVE |

The ray caster is `../observation_7f4c30f/tools/scan_check.py`, SHA-256 `b7758cab…9eef`,
unchanged since the observation.

Usage, from this directory (needs numpy, no ROS):

```bash
python3 selftest_crit6_v2.py ../../../../src/spiderx_description/worlds/spiderx_fortress.sdf \
    ../observation_7f4c30f/run_01/captures/capture.json    # template only: ranges replaced
python3 crit6_v2.py crit6_v2_spec.json <run>/captures/capture.json \
    ../../../../src/spiderx_description/worlds/spiderx_fortress.sdf --out <run>/criterion6_v2.json
```

`SHA256SUMS` covers the three files above; run `sha256sum -c SHA256SUMS` in this folder.
