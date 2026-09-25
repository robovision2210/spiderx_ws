# spiderx_controller

SpiderX quadruped joint configuration and **M1 joint position control** (Gazebo Fortress +
gz_ros2_control). Joint-position control only: no standing, balance or gait (see `../../STATUS.md`).

| File | Content |
|---|---|
| `config/spiderx_legs.yaml` | 4 legs × (hip, thigh, foot): joint names, parent/child links, axes, URDF limits, soft limits (−/+0.05 rad), motion constraints (placeholder / unknown) |
| `config/spiderx_poses.yaml` | Named poses. `cad_neutral` (all 0 rad) is the CAD standing geometry. It is geometric only and has never been held by a controller |
| `config/spiderx_ros2_controllers.yaml` | `joint_state_broadcaster` + `leg_trajectory_controller` (12 joints, position). Loaded by `fortress_control.launch.py` |
| `launch/controller.launch.py` | Spawns the two controllers in order (event handler, no sleeps) |
| `scripts/test_one_joint.py`, `scripts/test_neutral_pose.py` | Safe M1 test tools: refuse unknown joints and out-of-limit targets; one slow trajectory; report errors |
| `spiderx_controller/joint_safety.py` | Shared safety rules (unit-tested) |
| `spiderx_controller/config_check.py`, `scripts/validate_controller_config` | Checks every file above against the URDF |

```bash
ros2 run spiderx_controller validate_controller_config
```

```bash
ros2 launch spiderx_bringup fortress_control.launch.py
ros2 run spiderx_controller test_one_joint.py --joint lf_hip --target 0.2 --return-to-initial
ros2 run spiderx_controller test_neutral_pose.py
```

It is not diff-drive: a quadruped needs joint control (M1, done) → stand → IK → gait → `/cmd_vel` bridge.
See `docs/SPIDERX_DEVELOPMENT_ROADMAP.md`.
