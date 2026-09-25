# SpiderX – ROS 2 Quadruped (Humble · Gazebo Fortress)

SpiderX is a 12-DOF quadruped robot: four legs, each with hip abduction, hip flexion and
knee. This workspace provides:

- its CAD-exported robot description (URDF/xacro and STL meshes);
- a **Gazebo Fortress (Ignition Gazebo 6)** simulation with a simulated 2D lidar bridged to ROS 2 `/scan`;
- a real-robot lidar bring-up.

> **Status: passive simulation.** SpiderX spawns, settles under gravity and publishes
> `/scan`, `/joint_states`, `/clock` and TF. No gait, IK or joint controller exists yet, so
> the simulated robot does **not** stand actively, walk or navigate. See
> [Limitations and roadmap](#limitations-and-roadmap).

![SpiderX in Gazebo Fortress (cloud validation capture, software rendering)](media/cloud_validation/gazebo_fortress_gui.png)

## Architecture

```
                      spiderx_description/urdf/spiderx.urdf.xacro   (single source of truth)
                                   │  xacro sim_backend:=fortress
                                   ▼
              ┌──────────── robot_state_publisher ─────────────┐
              │  /robot_description          /tf, /tf_static   │
              └──────┬─────────────────────────────▲───────────┘
                     │ ros_gz_sim create            │ /joint_states
                     ▼ (-topic robot_description)   │
  ┌──────────────────────────────────────┐    ┌─────┴───────────────────────────────┐
  │ Gazebo Fortress (ign gazebo, via     │    │ ros_gz_bridge (parameter_bridge)    │
  │ ros_gz_sim gz_sim.launch.py)         │    │ config/fortress_bridge.yaml         │
  │  world: spiderx_fortress.sdf         │    │  /clock          gz → ROS           │
  │  • Physics (DART), SceneBroadcaster, │───▶│  /spiderx/scan → /scan              │
  │    UserCommands, Sensors (ogre2)     │ gz │  /spiderx/joint_states → /joint_states│
  │  model: spiderx                      │    └─────────────────────────────────────┘
  │  • gpu_lidar on lidar_link           │
  │  • JointStatePublisher system        │    optional: RViz (rviz:=true)
  └──────────────────────────────────────┘
```

| Package | Contents |
|---|---|
| `spiderx_description` | URDF/xacro, meshes, Fortress world, `fortress.launch.py`, bridge config, legacy Gazebo Classic config |
| `spiderx_bringup` | `fortress.launch.py` (recommended entry point), `hardware_lidar.launch.py`, RViz config |

## Prerequisites

- Ubuntu 22.04
- [ROS 2 Humble](https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debs.html), desktop install
- Gazebo Fortress, which `ros-humble-ros-gz-*` installs automatically
- A GPU with a working OpenGL driver is recommended, because the lidar is rendered with ogre2

## Install

```bash
sudo apt update && sudo apt install ros-humble-ros-gz-sim ros-humble-ros-gz-bridge ros-humble-ros-gz-interfaces
```

Then resolve the remaining dependencies declared in `package.xml`:

- Required: `xacro` and `robot_state_publisher`.
- Optional: `rviz2`, `rplidar_ros`, and the legacy Classic packages.

```bash
cd ~/spiderx_ws
sudo rosdep init 2>/dev/null; rosdep update
rosdep install --from-paths src --ignore-src -r -y
```

`gazebo_ros2_control` belongs to the legacy Classic path and pulls in Gazebo Classic. If you only need Fortress, add `--skip-keys gazebo_ros2_control`.

## Build

```bash
source /opt/ros/humble/setup.bash
cd ~/spiderx_ws && colcon build --symlink-install && source install/setup.bash
```

## Launch

```bash
ros2 launch spiderx_bringup fortress.launch.py
```

| Argument | Default | Meaning |
|---|---|---|
| `rviz` | `false` | Also open RViz (robot model, TF, `/scan`) |
| `headless` | `false` | Server only, no GUI. The lidar is rendered through EGL (`--headless-rendering`) |
| `world` | `spiderx_fortress.sdf` | Absolute path to another Fortress world |
| `spawn_x`, `spawn_y`, `spawn_z`, `spawn_yaw` | `0`, `0`, `0.075`, `0` | Spawn pose of the root link. The feet sit 0.0545 m below it, so keep `spawn_z` above that |
| `gz_verbosity` | `2` | Gazebo console verbosity |

Close the Gazebo window, or press Ctrl-C, to stop everything.

## Verify

Run these in a second terminal after `source ~/spiderx_ws/install/setup.bash`:

```bash
ros2 topic list                     # expect /clock /scan /joint_states /tf /tf_static /robot_description
ros2 topic echo /scan --once        # frame_id: lidar_link, 360 ranges, range_min 0.15, range_max 12.0
ros2 topic echo /clock --once       # simulation time advances
ros2 topic hz /scan                 # ~10 Hz at real-time factor 1
ros2 run tf2_ros tf2_echo base_link lidar_link   # translation 0.051 -0.045 0.141
ros2 run tf2_ros tf2_echo base_link rr_foot_1    # leg TF, driven by Gazebo /joint_states
./scripts/validate_fortress.sh      # static checks: xacro, URDF, SDF, launch files, no Classic tokens
```

### Simulation interface

| ROS 2 topic | Type | Source |
|---|---|---|
| `/scan` | `sensor_msgs/msg/LaserScan` | Gazebo `gpu_lidar` on `lidar_link`: 360 samples at 1° steps, 0.15–12 m, 10 Hz, σ = 1 cm noise |
| `/joint_states` | `sensor_msgs/msg/JointState` | Gazebo `JointStatePublisher`, 12 revolute joints |
| `/clock` | `rosgraph_msgs/msg/Clock` | Gazebo simulation time; every node runs with `use_sim_time` |
| `/tf`, `/tf_static` | `tf2_msgs/msg/TFMessage` | `robot_state_publisher` |

TF root: `dummy_link` → `base_link` → legs, `top_1`, `lidar_link`. There is no `odom` or
`map` frame; see the limitations below.

## Other launch files

| Command | Purpose |
|---|---|
| `ros2 launch spiderx_description display.launch.py` | View the model in RViz with joint sliders, without simulation |
| `ros2 launch spiderx_bringup hardware_lidar.launch.py serial_port:=/dev/ttyUSB0` | **Real robot only.** Starts the RPLidar A1 driver in `lidar_link`, plus `robot_state_publisher`. Never started by the simulation launch |

## Gazebo Classic (legacy)

Gazebo Classic support is kept for reference only. The Classic tags in
`urdf/spiderx.gazebo` and `urdf/spiderx.trans`, together with `config/controllers.yaml`, are
still produced by the default `xacro spiderx.urdf.xacro` (`sim_backend:=classic`). They are
**not** used by the Fortress path, and this workspace ships no Classic launch file. See
[MIGRATION_FORTRESS.md](MIGRATION_FORTRESS.md).

## Limitations and roadmap

- **Passive model.** No joint is actuated. After spawning, the legs fold under gravity until the feet reach their joint limits (about ±0.44 rad). The root link settles at z ≈ 0.012 m, about 4 cm lower than the CAD standing pose (0.0545 m). This was observed on Fortress 6.16.
- **No locomotion.** Nothing converts `/cmd_vel` into leg motion, and no `/odom` or `odom → base_link` source exists. Nav2, AMCL and SLAM are therefore **not** runnable in simulation, and no such launch is provided.
- **Frames.** `base_link` follows the CAD export: +y is the robot's front, which is not REP-103. `lidar_link` is yawed so that its +x points forward.
- **Mass properties.** They come from CAD but use the default steel material, giving 7.3 kg in total. See [docs/URDF_INERTIA_AUDIT.md](docs/URDF_INERTIA_AUDIT.md).

Work required before the quadruped can move in simulation:

1. Replace the effort/velocity placeholders with servo specifications, and add joint damping.
2. Integrate `gz_ros2_control` (`ros-humble-gz-ros2-control`, hardware plugin `gz_ros2_control/GazeboSimSystem`) and validate it with the existing controller configuration.
3. Add a gait generator with leg IK and a `cmd_vel` → gait interface.
4. Add odometry (leg odometry or IMU fusion) and a REP-103 `base_footprint` frame.
5. Tune the Nav2 configuration against the real footprint.

## Documentation

- [docs/SPIDERX_URDF_AUDIT.md](docs/SPIDERX_URDF_AUDIT.md): model audit covering the kinematic tree, link and joint tables, and the repair plan
- [docs/URDF_INERTIA_AUDIT.md](docs/URDF_INERTIA_AUDIT.md): inertial values and their provenance
- [MIGRATION_FORTRESS.md](MIGRATION_FORTRESS.md): the Classic → Fortress mapping, validation record and troubleshooting
- [media/README.md](media/README.md): the showcase screenshots and GIF to capture

## License

Apache-2.0 (see the `package.xml` files).
