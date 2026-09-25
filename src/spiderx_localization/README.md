# spiderx_localization

| File | Purpose |
|---|---|
| `config/ekf.yaml`, `launch/local_localization.launch.py` | robot_localization EKF. Future inputs: `/spiderx/leg_odometry`, `/imu/data`. Output: `odom → dummy_link` |
| `config/amcl.yaml`, `launch/global_localization.launch.py map:=<map.yaml>` | Map server + AMCL (`OmniMotionModel`, an assumption to confirm with the gait) |

**Configured but blocked** until odometry exists. There is no default map. Not started by any other
launch file.
