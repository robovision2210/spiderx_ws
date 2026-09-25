# SpiderX: Simulation vs Hardware

SpiderX keeps one robot description and one set of topic and frame names for both worlds.
Only the source of the data changes.

| Item | Simulation (Gazebo Fortress) | Hardware |
|---|---|---|
| Entry point | `ros2 launch spiderx_bringup fortress.launch.py` | `ros2 launch spiderx_bringup real_robot.launch.py` |
| Robot description | `spiderx.urdf.xacro sim_backend:=fortress` | `spiderx.urdf.xacro sim_backend:=none` |
| Time | `/clock` from Gazebo, `use_sim_time: true` | Wall clock, `use_sim_time: false` |
| `/scan` in `lidar_link` | Gazebo `gpu_lidar` → `ros_gz_bridge` ✅ | RPLidar A1 driver (`spiderx_firmware`) 🟡 |
| `/joint_states` | Gazebo `JointStatePublisher` ✅ | Future actuator interface ⚪ |
| Joint commands | None (passive model) | None (no actuator interface) |
| Serial devices | **Never opened** (checked by `validate_fortress.sh`) | `/dev/ttyUSB0` (lidar) |
| Mass / inertia | CAD values (Fusion default steel) | Real robot. Weigh the parts |
| Collision | Full CAD meshes | Real contact |
| Legacy | Gazebo Classic tags (`sim_backend:=classic`), reference only | none |

## Rules

1. **No hardware in simulation.** Simulation launch files never include `spiderx_firmware` or any serial driver.
2. **Same names in both worlds.** Topic and frame names are identical, so SLAM, localization and navigation configs work unchanged on both.
3. **Nothing fake.** No node may publish `/odom` or `/joint_states` values that are not measured in either world.
4. **Placeholders are labelled.** Anything estimated for simulation carries `SIMULATION_PLACEHOLDER`, `SIMULATION_ESTIMATE`, `PLACEHOLDER` or `TEMPLATE` in the file.
