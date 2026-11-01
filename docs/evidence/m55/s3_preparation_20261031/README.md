# S3 preparation, offline (2026-10-31)

Nothing here ran in a simulator; no goal, cancel or launch. S3 motion is not authorized.

| File | Content |
|---|---|
| `level0_joint_margins.json` | `scripts/s3/level0_joint_margins.py`: planned distance to the URDF limits for the 9 level-0 forward goals (waypoints and cubic interpolation); minimum 0.3541 rad (`rf_foot_joint`) |
| `focused_validation.txt` | One focused run: `scripts/s2` 48 tests (3 bag tests skipped), `scripts/s3` 52, restore 18; flake8; gates False on a fresh import; phase-library digest `62f8a100…` (54 goals) |
| `overlay_r2_provenance.txt` | `~/spiderx_gz_overlay_ws_r2/OVERLAY_PROVENANCE.txt` (0.7.15 + patches 0001 and 0002), plus its gtest counts (9 + 7, no failures) |

The r2 overlay's shutdown fix is verified offline only; its Gazebo runtime check is pending
(`docs/S3_FREE_BASE_CRAWL_DESIGN.md` §8).
