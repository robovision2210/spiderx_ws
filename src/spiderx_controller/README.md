# spiderx_controller

Controller **foundation** for the SpiderX quadruped. It commands no motion yet (see `../../STATUS.md`).

| File | Content |
|---|---|
| `config/spiderx_legs.yaml` | 4 legs × (hip, thigh, foot): joint names, parent/child links, axes, URDF limits, soft limits (−/+0.05 rad), motion constraints (placeholder / unknown) |
| `config/spiderx_poses.yaml` | Named poses. `cad_neutral` (all 0 rad) is the CAD standing geometry. It is geometric only and has never been held by a controller |
| `config/spiderx_ros2_controllers.yaml` | **Scaffold**: `joint_state_broadcaster` + `leg_trajectory_controller` (12 joints, position). Not loaded by any launch until milestone M1 |
| `spiderx_controller/config_check.py`, `scripts/validate_controller_config` | Checks every file above against the URDF |

```bash
ros2 run spiderx_controller validate_controller_config
```

It is not diff-drive: a quadruped needs joint control → stand → IK → gait → `/cmd_vel` bridge.
See `docs/SPIDERX_DEVELOPMENT_ROADMAP.md`.
