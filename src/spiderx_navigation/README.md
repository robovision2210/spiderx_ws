# spiderx_navigation

Nav2 for SpiderX, one parameter file per server (`config/`), `behavior_tree/`, and
`launch/navigation.launch.py`.

**Configured but blocked**:

- Nothing consumes `/cmd_vel` yet (no gait).
- There is no odometry.
- `base_link` faces +y, so Nav2 needs a REP-103 base frame first.

The footprint uses the measured mesh extents in `base_link`. Every velocity is a
`SIMULATION_PLACEHOLDER`. Not started by any other launch file.
