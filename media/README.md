# Media

## Cloud validation captures (included)

Both images were captured during automated validation, on a cloud VM with **no GPU**. They
used Xvfb and Mesa software rendering, which is why the real-time factor was about 0.3.
They are genuine, but they are not showcase quality.

| File | What it shows |
|---|---|
| `cloud_validation/gazebo_fortress_gui.png` | Output of `ros2 launch spiderx_bringup fortress.launch.py`: the Fortress GUI with SpiderX spawned in the test world. The lidar cylinder sits on the top plate and the legs are passively folded. |
| `cloud_validation/rviz_scan_tf.png` | Output of `rviz:=true`: the RobotModel driven by Gazebo `/joint_states`, TF, and `/scan` outlining the walls, boxes and pillar, with Global Status OK. |

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

Keep individual files under about 2 MB. Scale the GIF to 800 px wide or less.

Do **not** caption any media as walking or navigating. The simulated robot is passive; see
the Limitations section in the top-level README.
