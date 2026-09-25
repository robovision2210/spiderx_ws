# SpiderX maps

No map is committed yet. A map must come from SLAM (`ros2 launch spiderx_mapping slam.launch.py`)
while the robot actually moves, which requires locomotion and odometry (see `STATUS.md`).

Save each map into its own folder:

```bash
ros2 run nav2_map_server map_saver_cli -f ~/spiderx_ws/src/spiderx_mapping/maps/<map_name>/map
```
