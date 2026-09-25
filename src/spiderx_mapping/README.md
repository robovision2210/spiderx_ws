# spiderx_mapping

SLAM Toolbox for SpiderX: `config/slam_toolbox.yaml`, `launch/slam.launch.py`, `maps/`.

**Configured but blocked.** It uses the real `/scan` and `base_link`, but SLAM needs
`odom → dummy_link` from real odometry, which SpiderX does not have yet. Not started by any other
launch file.
