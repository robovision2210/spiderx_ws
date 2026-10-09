# Incident bundle: M6.0-D isolated success test failed before M6.1-A observation (Cloud)

This is the evidence from the first Cloud attempt at M6.1-A phase-1 verification on
`claude/stoic-shannon-ur2mes` @ `41fc10f`, preserved verbatim. One required test failed, so the
attempt stopped before any simulator launch. `REPORT.md` is the report written at the time.

| Item | Value |
|---|---|
| Environment | Cloud VM (Ubuntu 24.04 container, 4 CPUs, RoboStack ROS 2 Humble, Python 3.11). Not the owner PC. |
| Build | `colcon build`: 8 packages, exit 0 (`build_test/build.log`) |
| Tests | 1465 tests, 0 errors, **2 failures** (one pytest case plus its CTest wrapper), 0 skipped (`build_test/test.log`) |
| Failing test | `test_m6d_isolated_success.py::test_gate_enabled_main_succeeds_end_to_end_with_one_goal`: `('REFUSED', 'readiness_not_ready', [])` (`build_test/failure/`) |
| Static checks | All passed; `--live` exited 3 (`build_test/static/summary.tsv`) |
| Simulator | Not started; `run_01..03/NOT_PERFORMED.txt` |

## Integrity

- **`SHA256SUMS` covers all 80 files.** To check them, run `sha256sum -c SHA256SUMS` in this folder.
- **The original archive is kept outside the repository.** It is `spiderx_evidence/archive/m61a_phase1_20261009T075335Z.tar.gz`, SHA-256 `1f6c9ae4aa9fe5e6492046ac02f09f838d8685c13d8cc03bb7ad1d367d99f85b`. Its extracted copy verified against `SHA256SUMS` before this copy was made.

## Sanitization

- **The files are byte-identical to the originals**, so the manifest still verifies.
- **No credentials or personal data were found.** Before committing, the bundle was scanned for:
  - proxy, token, secret, password and authorization strings;
  - GitHub and Anthropic key prefixes;
  - e-mail addresses;
  - IP addresses;
  - long base64 strings.

  The only matches were test names containing "session" and a padding string in a test fixture.
- **What the files do contain:**
  - container paths (`/home/user/spiderx_ws`, `/root/envs/humble`);
  - the generic VM hostname `vm`;
  - in `failure/colcon_test_log_spiderx_controller/command.log`, the three environment variables colcon prints (`CMAKE_PREFIX_PATH`, `CONDA_PROMPT_MODIFIER`, `PYTHONPATH`).

## Gap

The failing run's `live_outcome.json` held the readiness reports. pytest wrote it under
`/tmp/pytest-of-root/pytest-599/`, and its keep-three temporary-directory rotation deleted it before
it could be copied. The diagnosis of the failure is in
[`docs/M61A_CLOUD_VERIFICATION.md`](../../../M61A_CLOUD_VERIFICATION.md).

`tools/` holds the observation scripts prepared at the time. They are evidence tooling, not package
source.
