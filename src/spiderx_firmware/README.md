# spiderx_firmware

Real-hardware layer. **Never included by a simulation launch.**

| File | Status |
|---|---|
| `launch/lidar.launch.py` | RPLidar A1 driver → `/scan` in `lidar_link` (configured; needs the real sensor) |
| `config/actuator_interface_template.yaml` | **TEMPLATE**: 12-joint servo map, all values placeholders |
| `config/imu_template.yaml` | **TEMPLATE**: IMU driver, topic and mount; no `imu_link` in the URDF yet |

This package contains no microcontroller firmware or servo driver yet. See
`docs/SPIDERX_HARDWARE_INTERFACE.md`.
