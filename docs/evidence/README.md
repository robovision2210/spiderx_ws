# Preserved evidence

Reports quoted by the documents, committed so that no build or cleanup can remove them. Each
folder has a README with the revision, configuration hashes, command, environment, output
hashes, a repeated-generation comparison and the limitations.

| Folder | Report | Quoted in |
|---|---|---|
| [`m61a/`](m61a/README.md) | `m61a_clearance` ground clearance of the welded model | `docs/M61A_FIXED_BASE_IMPLEMENTATION.md` §3 |

`compare_reports.py` compares two regenerated JSON reports, ignoring declared nondeterministic
fields. Clean builds are `rm -rf build install` and keep `log/`; copy local evidence to
`~/spiderx_evidence/` before anything else touches it.
