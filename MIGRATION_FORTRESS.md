# SpiderX Gazebo Classic → Gazebo Fortress Migration

## What changed
- New launch path: `ros2 launch spiderx_bringup fortress.launch.py`
- New Fortress world: `src/spiderx_description/worlds/spiderx_fortress.sdf`
- New Fortress launch files:
  - `src/spiderx_description/launch/fortress.launch.py`
  - `src/spiderx_bringup/launch/fortress.launch.py`
- Classic-only integrations were not removed and remain available in the legacy launch files.

## Plugin and simulator changes
- **Removed from Fortress flow**: `gazebo_ros`, `spawn_entity.py`, `gzserver/gzclient`, `GAZEBO_MODEL_PATH`, `libgazebo_ros_ray_sensor.so`, and `gazebo_ros2_control`.
- **Used in Fortress flow**: `ros_gz_sim`, `ros_gz_bridge`, `GZ_SIM_RESOURCE_PATH`, and `ros_gz_sim create`.

## Robot model generation workflow
The Fortress launch generates a runtime SDF from the existing xacro/URDF, then spawns it:

```bash
xacro src/spiderx_description/urdf/spiderx.urdf.xacro \
  enable_classic_gazebo:=false enable_fortress_lidar:=true \
  | gz sdf -p > /tmp/spiderx_fortress_model.sdf
```

This keeps the existing URDF/xacro + meshes as the source of truth while avoiding Gazebo Classic plugins in Fortress.

## Lidar migration and bridge
- The Fortress path uses a Gazebo Sim lidar sensor definition (no Classic ROS plugin).
- Bridged topic mapping:
  - Gazebo Sim: `/scan` (`gz.msgs.LaserScan`)
  - ROS 2: `/scan` (`sensor_msgs/msg/LaserScan`)
- Bridge node:
  - `ros_gz_bridge parameter_bridge /scan@sensor_msgs/msg/LaserScan@gz.msgs.LaserScan`
- Frame remains `lidar_link`.

## Known limitations
- The existing `spiderx.gazebo` file still contains Classic `gazebo_ros2_control` integration and is only for Gazebo Classic flows.
- Fortress launch intentionally spawns SpiderX as a **passive model** (no validated `gz_ros2_control` setup in this migration).
- Legged walking control and Nav2 execution are deferred until a verified gait controller + validated `gz_ros2_control` integration are added.
