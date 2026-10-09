# Preserved evidence

Reports quoted by the documents, committed so that no build or cleanup can remove them. Each
folder has a README with the revision, configuration hashes, command, environment, output
hashes, a repeated-generation comparison and the limitations.

| Folder | Report | Quoted in |
|---|---|---|
| [`m61a/`](m61a/README.md) | `m61a_clearance` ground clearance of the welded model | `docs/M61A_FIXED_BASE_IMPLEMENTATION.md` §3 |
| [`m61a_cloud/`](m61a_cloud/) | Cloud M6.1-A verification: the 2026-10-09 incident bundle (M6.0-D isolated test failure), its diagnosis, the verification of `7f4c30f` and the three Cloud phase-1 observation runs | `docs/M61A_CLOUD_VERIFICATION.md` |

`compare_reports.py` compares two regenerated JSON reports, ignoring declared nondeterministic
fields. Clean builds are `rm -rf build install` and keep `log/`; copy local evidence to
`~/spiderx_evidence/` before anything else touches it.
