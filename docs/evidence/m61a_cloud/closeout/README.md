# Closeout of the Cloud M6.1-A phase-1 observation

This folder is derived from [`../observation_7f4c30f/`](../observation_7f4c30f/README.md), which
is not modified (its `SHA256SUMS` still verifies). Nothing here required a new simulator run. The
write-up is in [`docs/M61A_CLOUD_VERIFICATION.md`](../../../M61A_CLOUD_VERIFICATION.md) §8.

| File | What it is |
|---|---|
| `closeout_tables.py` | The read-only generator of the files below. Run it from this folder; it needs numpy and `../observation_7f4c30f/tools/scan_check.py` |
| `requirements_by_run.json` / `.md` | Every M6.1-A §10 criterion plus the supplementary checks, per run: PASS / FAIL / UNMEASURED, with value, threshold, scope and the evidence path of each cell |
| `scan_masks.json` | Per run, for the `/scan` check: <ul><li>the excluded seam and edge-grazing beam indices, each with its angle, surface, residual and justification;</li><li>the raw and the seam-filtered results;</li><li>the individual-range error statistics by incidence band.</li></ul> |
| `frozen_harness.json` | **Frozen before any further observation:** <ul><li>the SHA-256 of every harness tool;</li><li>the environment and the procedure;</li><li>both scan masks as explicit index lists;</li><li>the fit settings;</li><li>the gated (6a, 6b) and reported (6c, 6-raw) definitions of every criterion.</li></ul> |

`SHA256SUMS` covers every file.
