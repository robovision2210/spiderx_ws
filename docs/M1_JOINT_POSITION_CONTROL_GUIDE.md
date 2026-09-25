# M1 Guide – Joint Position Control in Gazebo Fortress

> **What M1 is:** every leg joint of SpiderX can be commanded to an angle in simulation.
> **What M1 is not:** standing, balancing or walking. Those are M2–M4
> ([roadmap](SPIDERX_DEVELOPMENT_ROADMAP.md)).

## What gz_ros2_control is

[ros2_control](https://control.ros.org/humble/) is the standard ROS 2 framework for driving joints.
It has three parts:

- **controllers**, which decide what each joint should do;
- the **controller manager**, which runs the controllers;
- **hardware interfaces**, which talk to the actual motors.

`gz_ros2_control` is a hardware interface whose "motors" are the joints of a Gazebo simulation.
It loads as a Gazebo plugin, starts `/controller_manager` inside Gazebo, and exposes each SpiderX
joint with:

- a `position` **command** interface: the angle we ask for;
- `position` and `velocity` **state** interfaces: what the joint actually does.

SpiderX uses two controllers:

| Controller | Job |
|---|---|
| `joint_state_broadcaster` | Publishes `/joint_states` (12 joints) |
| `leg_trajectory_controller` (JointTrajectoryController) | Moves the 12 joints smoothly to requested angles |

## Position control vs standing vs walking

| Level | Meaning | Status |
|---|---|---|
| Joint position control (M1) | "Move this joint to 0.2 rad": each joint tracks its own target | ✅ this milestone |
| Standing (M2) | Keep the body at a height and level, reacting to disturbances | ⚪ not implemented |
| Walking (M4–M5) | Coordinated foot trajectories from a gait plus IK, driven by `/cmd_vel` | ⚪ not implemented |

In control mode, SpiderX rests on its feet because every joint is held at its commanded angle
by stiff placeholder motors (100 N·m). That is **not** standing control.

## Install (Ubuntu 22.04 + ROS 2 Humble)

```bash
sudo apt update
sudo apt install ros-humble-gz-ros2-control ros-humble-ros2-control ros-humble-ros2-controllers \
  ros-humble-controller-manager ros-humble-joint-state-broadcaster \
  ros-humble-joint-trajectory-controller ros-humble-control-msgs ros-humble-ros2controlcli
```

`rosdep install --from-paths src --ignore-src -r -y --skip-keys gazebo_ros2_control` installs the
same packages, because they are declared in the `package.xml` files.

## Build

```bash
cd ~/spiderx_ws
git fetch origin && git checkout claude/spiderx-m1-joint-control && git pull
source /opt/ros/humble/setup.bash
rm -rf build install log && colcon build --symlink-install
source install/setup.bash
```

Expected: `Summary: 8 packages finished`.

## Launch (Terminal 1)

```bash
ros2 launch spiderx_bringup fortress_control.launch.py
```

| Argument | Default | Meaning |
|---|---|---|
| `enable_control` | `true` | `false` runs the passive simulation instead |
| `rviz` | `false` | Also open RViz |
| `headless` | `false` | Server only, with EGL rendering (not yet tested) |

The passive, verified command `ros2 launch spiderx_bringup fortress.launch.py` is unchanged.

Wait until Terminal 1 shows both of these messages; allow up to about a minute:

```
[spawner_joint_state_broadcaster]: Configured and activated joint_state_broadcaster
[spawner_leg_trajectory_controller]: Configured and activated leg_trajectory_controller
```

## Check the controllers (Terminal 2)

```bash
source /opt/ros/humble/setup.bash && source ~/spiderx_ws/install/setup.bash
ros2 control list_controllers
```

Expected: both controllers `active`.

## Single-joint test

```bash
ros2 run spiderx_controller test_one_joint.py --joint lf_hip --target 0.2 --return-to-initial
```

The tool:

1. prints the controller states;
2. refuses unknown joints and targets outside the URDF limits minus a 0.05 rad margin;
3. moves the joint over 3 s, holding the other 11 joints;
4. reports the position before and after, the error, and PASS/FAIL (threshold 0.05 rad);
5. then moves the joint back.

Options: `--target`, `--duration`, `--margin`, `--tolerance`, `--settle`.

Try the safety refusal, which sends nothing:

```bash
ros2 run spiderx_controller test_one_joint.py --joint lf_foot_joint --target 0.6
```

## Neutral-pose test (all 12 joints)

```bash
ros2 run spiderx_controller test_one_joint.py --joint rr_hip --target 0.2      # move something first
ros2 run spiderx_controller test_neutral_pose.py
```

This sends all 12 joints to `cad_neutral` (0 rad) in one 6 s trajectory and prints a per-joint
error table. It prints "Joint-position test only – this is NOT a standing controller."

## Automated validation

```bash
./scripts/validate_m1_control.sh             # static
./scripts/validate_m1_control.sh --runtime   # launches, moves joints, shuts down (~2–5 min)
```

## Stopping safely

- Press **Ctrl+C** in Terminal 1, or close the Gazebo window.
- A stack trace from `GazeboSimROS2ControlPlugin::~…` may appear during shutdown. This is known upstream behaviour at exit.
- If anything keeps running: `pkill -f "ign gazebo"`.

## If something fails, send

1. The output of `ros2 control list_controllers`
2. The full output of the failing test command
3. `ros2 launch spiderx_bringup fortress_control.launch.py 2>&1 | tee ~/m1_launch.log`, then share `~/m1_launch.log`. The most useful lines are those containing `gz_ros2_control`, `spawner`, `ERROR` or `Failed`
4. A screenshot of the Gazebo window

## Acceptance checklist

- [ ] `colcon build --symlink-install`: 8 packages finished
- [ ] `fortress_control.launch.py`: SpiderX appears in Fortress
- [ ] `ros2 control list_controllers`: both controllers `active`
- [ ] `ros2 topic info /joint_states`: `Publisher count: 1`
- [ ] `test_one_joint.py --joint lf_hip --target 0.2 --return-to-initial`: PASS
- [ ] The unsafe command (`lf_foot_joint --target 0.6`) is REFUSED
- [ ] `test_neutral_pose.py` after displacing a joint: PASS, with at least 1 joint moved
- [ ] `./scripts/validate_m1_control.sh --runtime`: All M1 checks passed
- [ ] `./scripts/validate_fortress.sh --runtime`: the passive path still passes
- [ ] Screenshot recorded (`media/README.md`)
