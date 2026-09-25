# spiderx_scripts

Diagnostic nodes that work against today's simulation, and against hardware when the sensor exists.

| Command | Output |
|---|---|
| `ros2 run spiderx_scripts read_lidar` | Closest `/scan` return in the front / left / rear / right sectors (±20°) |
| `ros2 run spiderx_scripts read_joint_states` | 12 joint angles grouped by leg, in degrees (grouping from `spiderx_controller`) |
