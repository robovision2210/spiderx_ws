# Media

## Local validation captures (included)

These were captured on the owner's Ubuntu 22.04 PC during the first local test, cropped to the
Gazebo window.

| File | What it shows |
|---|---|
| `local_validation/gazebo_fortress_ubuntu.png` | `ros2 launch spiderx_bringup fortress.launch.py`: SpiderX spawned from `<urdf-string>`, and the full entity tree |
| `local_validation/gazebo_lidar_rays_collisions.png` | Close-up with Gazebo's collision view (magenta) and the lidar rays (green) |

## Cloud validation captures (included)

These images were captured during automated validation, on a cloud VM with **no GPU**. They
used Xvfb and Mesa software rendering, which is why the real-time factor was about 0.3.
They are genuine, but they are not showcase quality.

| File | What it shows |
|---|---|
| `cloud_validation/gazebo_fortress_gui.png` | Output of `ros2 launch spiderx_bringup fortress.launch.py`: the Fortress GUI with SpiderX spawned in the test world. The lidar cylinder sits on the top plate and the legs are passively folded. |
| `cloud_validation/m1_joints_held_cad_neutral.png` | M1: after `test_neutral_pose.py`, all 12 joints are held at `cad_neutral` by gz_ros2_control position control. **Not** a standing controller |
| `cloud_validation/m2_cad_neutral_posture_hold.png` | M2: SpiderX CAD neutral posture hold in Gazebo Fortress — simulation only; not balance or walking. Captured after `run_posture_hold_test.py` reported "Simulation posture hold verified." (Gazebo window only) |
| `cloud_validation/rviz_scan_tf.png` | Output of `rviz:=true`: the RobotModel driven by Gazebo `/joint_states`, TF, and `/scan` outlining the walls, boxes and pillar, with Global Status OK. |

## S2 Phase C fixed-base attempt (cloud, 2026-10-30): FAIL

This capture was taken during the one authorized S2 Phase C attempt. It shows the Gazebo GUI on Xvfb, with Mesa software rendering and no GPU. It was taken with the pre-installed ImageMagick `import`, then cropped to the Gazebo window; the strip removed was empty. There was no other edit.

| File | What it shows |
|---|---|
| `s2_phase_c/20261030T062140Z/s2_phase_c_before_first_goal.png` | `fortress_m61a_fixed_base.launch.py` with the gain-1.0 actuator overlay. SpiderX is welded 0.125 m above the floor, after preflight READY and the pre-motion checks, before the first goal |

*Simulation-only fixed-base crawl trajectory validation. The body is welded above the ground; this does not demonstrate walking, ground contact or balance.*

- **Purpose:** to show the fixed-base rig used for the S2 crawl-trajectory check: the controller executes the phase trajectories on a body that cannot touch the ground.
- **Attempt verdict: FAIL.** The run did not stop early: the script completed, with 59 goals and 1 scripted cancel. The offline plant check matched gain 1.0. The frozen scorer rated:
  - **P4 FAIL:** the `joint_trajectory` topic is absent from the bag, which the scorer does not count as zero messages;
  - **P2 UNMEASURED:** the scripted estop goal was cancelled before any feedback arrived;
  - **P1, P3 and P5–P8:** PASS.
- **Evidence:** `docs/evidence/m55/s2_phase_c_20261030T062140Z/`. This is the run evidence; it is under review and not yet committed.
- **Limitations:** the image was taken before any motion, and a static screenshot does not prove that a trajectory or cycle completed. A second capture, after the script, differs by 12 pixels at this distance, so it is not published. The rendering is software-only, not showcase quality.

## Showcase media to capture locally (placeholder)

Capture these on an Ubuntu 22.04 machine with a GPU, then add them here and reference them
from the top-level `README.md`.

| # | File name | How to capture |
|---|---|---|
| 1 | `spiderx_fortress_overview.png` | `ros2 launch spiderx_bringup fortress.launch.py`. After about 5 s, once the robot has settled, frame the whole walled world with the robot in the centre. Use Gazebo's screenshot button (camera icon). |
| 2 | `spiderx_closeup.png` | Same launch. Close-up at roughly 45° showing the CAD legs, the top plate and the lidar. |
| 3 | `spiderx_rviz_scan.png` | `ros2 launch spiderx_bringup fortress.launch.py rviz:=true`. RViz with RobotModel, TF and LaserScan visible, and the scan outlining the walls and obstacles. |
| 4 | `spiderx_lidar_rays.gif` (5–10 s) | In Gazebo, open ⋮ → *Visualize Lidar*, choose topic `/spiderx/scan`, and record the rays around the robot, for example with Peek or SimpleScreenRecorder. |
| 5 | `spiderx_terminal_checks.png` | A terminal showing `ros2 topic list`, `ros2 topic hz /scan` and `ros2 run tf2_ros tf2_echo base_link lidar_link`. |
| 6 | `spiderx_tf_tree.png` | Run `ros2 run tf2_tools view_frames` while the simulation runs, then convert the resulting PDF (`frames_*.pdf`) to PNG. |
| 7 | *(future)* `spiderx_standing.gif`, `spiderx_walking.gif` | **Only after** the standing controller and gait exist and pass their roadmap tests. |

Keep individual files under about 2 MB. Scale the GIF to 800 px wide or less.

Do **not** caption any media as walking or navigating. The simulated robot is passive; see
the Limitations section in the top-level README.
