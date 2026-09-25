# SpiderX: Gazebo Classic → Gazebo Fortress migration

Target: Ubuntu 22.04, ROS 2 Humble, Gazebo Fortress (Ignition Gazebo 6), `ros_gz_sim`, `ros_gz_bridge`.

Recommended command:

```bash
ros2 launch spiderx_bringup fortress.launch.py
```

## 1. What maps to what

| Gazebo Classic | Gazebo Fortress (this repo) |
|---|---|
| `gazebo_ros/gazebo.launch.py` (gzserver + gzclient) | `ros_gz_sim/gz_sim.launch.py`, which runs `ign gazebo` (`-s --headless-rendering` when `headless:=true`) |
| `spawn_entity.py -topic robot_description` | `ros_gz_sim create -topic robot_description`: the xacro stays the single source of truth, with no generated SDF file |
| `GAZEBO_MODEL_PATH` | `IGN_GAZEBO_RESOURCE_PATH` (Fortress) and `GZ_SIM_RESOURCE_PATH`, both **appended** by the launch file |
| `libgazebo_ros_ray_sensor.so` (`type="ray"`) | Fortress `gpu_lidar` sensor plus `ros_gz_bridge` (`gz.msgs.LaserScan` → `sensor_msgs/msg/LaserScan`) |
| `<frame_name>` in the ROS plugin | `<ignition_frame_id>lidar_link</ignition_frame_id>` on the sensor |
| `/clock` from gazebo_ros | `/clock` bridged from Gazebo (`gz.msgs.Clock` → `rosgraph_msgs/msg/Clock`) |
| `joint_state_publisher` or `gazebo_ros2_control` joint states | Fortress `JointStatePublisher` system, bridged to `/joint_states` |
| `gazebo_ros2_control` + `gazebo_ros2_control/GazeboSystem` | **Not migrated.** The Fortress equivalent is `gz_ros2_control`; see §6 |
| `<material>Gazebo/Silver</material>` | Dropped: Fortress uses the URDF `<material>` colours |
| `<mu1>/<mu2>/<self_collide>` | Kept with identical values, generated in `spiderx_fortress.gazebo.xacro` |
| World `model://ground_plane` + `model://sun` | Inline ground plane and sun in `worlds/spiderx_fortress.sdf`, so no Fuel download is needed |

## 2. How the description selects a simulator

`urdf/spiderx.urdf.xacro` takes `sim_backend:=classic|fortress|none`:

| Value | Includes | Used by |
|---|---|---|
| `classic` (default) | `spiderx.trans`, `spiderx.gazebo` (legacy, unchanged) | `display.launch.py`, any legacy Classic launch |
| `fortress` | `spiderx_fortress.gazebo.xacro` (gpu_lidar, JointStatePublisher, contact parameters) | `spiderx_description/launch/fortress.launch.py` |
| `none` | no simulator tags | `spiderx_bringup/launch/real_robot.launch.py` |

For every value the CAD links, joints, inertias and meshes are identical. The default
output was verified to be semantically identical to the pre-migration URDF, apart from the
new `lidar_link` and `lidar_joint`.

## 3. Files

| File | Role |
|---|---|
| `src/spiderx_bringup/launch/fortress.launch.py` | Entry point: includes the simulation, plus optional RViz |
| `src/spiderx_description/launch/fortress.launch.py` | Gazebo, robot_state_publisher, spawn and bridge |
| `src/spiderx_description/worlds/spiderx_fortress.sdf` | Fortress world: Physics, UserCommands, SceneBroadcaster and Sensors (ogre2) systems; ground, sun, walls, obstacles, GUI camera |
| `src/spiderx_description/urdf/spiderx_fortress.gazebo.xacro` | Fortress-only sensor, system and contact tags |
| `src/spiderx_description/config/fortress_bridge.yaml` | `ros_gz_bridge` configuration (all GZ → ROS) |
| `src/spiderx_bringup/launch/real_robot.launch.py` → `src/spiderx_firmware/launch/lidar.launch.py` | Real RPLidar A1 driver, deliberately outside the simulation path (originally `hardware_lidar.launch.py`) |
| `scripts/validate_fortress.sh` | Static validation |
| Legacy, unchanged: `urdf/spiderx.gazebo`, `urdf/spiderx.trans` | Gazebo Classic tags |
| `src/spiderx_description/config/controllers.yaml` | Legacy Classic controller config. It was referenced but missing, which broke `colcon build` |

## 4. Corrections relative to the earlier Fortress scaffolding (PR #2)

| Problem in PR #2 | Fix here |
|---|---|
| Generated the SDF with `gz sdf -p`. `gz` does not exist on Fortress (the CLI is `ign`), so the step failed with exit 127 and the robot never spawned (reproduced on Fortress 6.16) | Spawns straight from `/robot_description` with `ros_gz_sim create -topic robot_description` |
| Spawned even when SDF generation failed; hard-coded `/tmp` path | No intermediate file |
| Bidirectional `@` scan bridge; no `/clock` bridge | One-way `GZ_TO_ROS` bridges for `/scan`, `/clock` and `/joint_states`, in YAML |
| `joint_state_publisher` published fabricated zero joint states | Real joint states from Gazebo |
| Lidar floated 7.4 cm above the top plate, off centre, pointing right; scan frame not overridden | Mounted on the measured top-plate centre, yawed to face forward, `ignition_frame_id=lidar_link` |
| Garden-style `gz-sim-*` plugin names (only accepted through aliasing in later 6.x releases) | Canonical `ignition-gazebo-*-system` names |
| `GZ_SIM_RESOURCE_PATH` overwritten; `IGN_GAZEBO_RESOURCE_PATH` not set | Both appended |
| The Classic bringup started the hardware RPLidar on `/dev/ttyUSB0` in simulation | The hardware driver lives only in the real-robot path (`real_robot.launch.py`) |

## 5. Validation record

### Done in the cloud

Environment: Ignition Gazebo **6.16.0**, sdformat 12.8.0, ros_gz 0.244.20 and ROS 2 Humble,
installed from RoboStack/conda-forge on a VM with no GPU. Rendering used Xvfb and Mesa
llvmpipe.

- `colcon build --symlink-install`: both packages build.
- `scripts/validate_fortress.sh`: all checks pass.
  - xacro and `check_urdf` for all three backends.
  - URDF→SDF with sdformat 12 gives 13 links and 12 joints, a `gpu_lidar` at `0.051 -0.045 0.141` with yaw π/2, and no Classic plugin.
  - The world SDF validates, all 30 meshes resolve, and every launch file compiles and parses its arguments.
  - No Classic tokens appear in the Fortress files.
- `ros2 launch spiderx_bringup fortress.launch.py` with the GUI, run end to end under Xvfb:
  - The model spawns, and `/clock`, `/scan`, `/joint_states` and TF are all present.
  - `/scan`: `frame_id=lidar_link`, 360 samples at exactly 1.000°, range 0.15–12 m.
  - **Measured wall and pillar distances match the world geometry within ±8 mm** at the expected bearings.
  - `/joint_states`: 12 joints. TF resolves `base_link → rr_foot_1`.
  - Passive physics is stable: the root settles at z ≈ 0.012 m, with roll and pitch below 2e-4 rad.
  - Real-time factor was about 0.3–0.45, limited by software rendering on the VM.
- `rviz:=true`: RViz loads the config (Global Status OK) and shows the robot model, TF and scan.
- `hardware_lidar.launch.py` (now `real_robot.launch.py`): robot_state_publisher starts; the driver fails only because no serial device exists.

### Needs local verification (GPU machine)

- Visual quality and real-time factor with a real GPU.
- `headless:=true`. It needs ogre-next with EGL support: the OSRF packages have it, but the cloud's conda build does not, so it is **untested**.
- The apt-installed versions of `ros_gz`, and the `rosdep` resolution.
- The hardware lidar with a physical RPLidar A1.

## 6. Future work (required before Nav2 can move the robot)

1. **Actuation.**
   - Install `ros-humble-gz-ros2-control`.
   - Add an `<ros2_control>` block using the hardware plugin `gz_ros2_control/GazeboSimSystem` under `sim_backend:=fortress`, and the system plugin `gz_ros2_control-system`.
   - Humble 0.7.x also keeps `ign_ros2_control/IgnitionSystem` and `ign_ros2_control-system` as compatibility aliases.
   - Reuse `config/controllers.yaml`.
   - Replace the effort/velocity placeholders (100 N·m, 100 rad/s) with the servo specifications, and add joint damping.
2. **Gait.** Stand-up posture, leg IK using the per-joint sign conventions in `SPIDERX_URDF_AUDIT.md` §5, a gait generator, and a `/cmd_vel` → gait interface.
3. **State estimation.** Leg odometry and/or IMU → `/odom` and `odom → base_link`. Add a REP-103 `base_footprint`, because `base_link` faces +y.
4. **Navigation.** Then add a simulation mapping/localization/Nav2 launch, with the footprint expressed in the real `base_link` coordinates.

No `/odom` or `/cmd_vel` behaviour is simulated today, and none is faked.

## 7. Troubleshooting

| Symptom | Cause / fix |
|---|---|
| `ign: command not found` | Fortress is not installed: `sudo apt install ros-humble-ros-gz-sim` pulls it in |
| Gazebo crashes at start with an ogre2 or OpenGL error (VMs, old GPUs) | Try `export LIBGL_ALWAYS_SOFTWARE=1`, which is slow but works. `headless:=true` needs EGL |
| Robot visible but `/scan` missing | The Sensors system needs rendering. Check the Gazebo console for ogre2 errors, and check that `ign topic -l` lists `/spiderx/scan` |
| `Found additional publishers on /clock` or `Moved backwards in time` warnings | A previous Gazebo instance is still running. Close it (`pkill -f "ign gazebo"`) and relaunch |
| Two simulations interfere across machines | Set distinct `ROS_DOMAIN_ID` and `IGN_PARTITION` values |
| `gz` runs Gazebo Classic tools | Expected when Classic is also installed. Fortress uses `ign` |
