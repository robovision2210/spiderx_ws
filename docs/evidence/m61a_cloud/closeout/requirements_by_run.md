| ID | Criterion | Scope | run_01 | run_02 | run_03 |
|---|---|---|---|---|---|
| 1 | preflight READY | per run | **PASS**: READY | **PASS**: READY | **PASS**: READY |
| 1b | 120 s observation READY at its end | per run | **PASS**: READY | **PASS**: READY | **PASS**: READY |
| 2a | body z mean within 1 mm of 0.125 m | per run | **PASS**: 0.125 | **PASS**: 0.125 | **PASS**: 0.125 |
| 2a | body z range <= 1 mm | per run | **PASS**: 0.0 | **PASS**: 0.0 | **PASS**: 0.0 |
| 2b | attachment translation <= 1 mm | per run | **PASS**: 0.0 | **PASS**: 0.0 | **PASS**: 0.0 |
| 2b | attachment rotation <= 0.0033 rad | per run | **PASS**: 0.0 | **PASS**: 0.0 | **PASS**: 0.0 |
| 2c | spawn deviation <= 1 mm | per run | **PASS**: 0.0 | **PASS**: 0.0 | **PASS**: 0.0 |
| 2d | link deviation <= 0.1 mm | per run | **PASS**: 0.0 | **PASS**: 0.0 | **PASS**: 0.0 |
| 2e | tilt <= 0.0033 rad | per run | **PASS**: 0.0 | **PASS**: 0.0 | **PASS**: 0.0 |
| 3 | pose receipt max gap < 1.0 s (p99.9 recorded) | per run | **PASS**: {"max_s": 0.0763, "p999_s": 0.0531, "samples": 6846} | **PASS**: {"max_s": 0.0677, "p999_s": 0.0516, "samples": 6844} | **PASS**: {"max_s": 0.0613, "p999_s": 0.0535, "samples": 6861} |
| 3 | joint-state sim gap max <= 0.25 s | per run | **PASS**: 0.05 | **PASS**: 0.02 | **PASS**: 0.01 |
| 4 | no unexpected pose sample codes | per run | **PASS**: {} | **PASS**: {} | **PASS**: {} |
| 4 | header.frame_id values recorded (not gated) | per run | **PASS**: {"": 13692} | **PASS**: {"": 13688} | **PASS**: {"": 13722} |
| 5 | visual: legs clear of the ground (GUI screenshot) | per campaign (one screenshot) | **UNMEASURED**: inconclusive: no foot-level view; the ~16 mm gap is not resolvable in the views taken | **PASS**: feet visibly above the floor (qualitative reviewer judgement) | **PASS**: feet visibly above the floor (qualitative reviewer judgement) |
| 6a | /scan pose agreement: fitted lidar pose within 8 mm, seam beams masked | per run (frozen definition) | **PASS**: {"dx_mm": 0.384, "dy_mm": 0.078, "dyaw_mrad": 0.023, "robust_1sigma_mm": [0.299, 0.309]} | **PASS**: {"dx_mm": 0.283, "dy_mm": 0.029, "dyaw_mrad": 0.11, "robust_1sigma_mm": [0.249, 0.257]} | **PASS**: {"dx_mm": 0.165, "dy_mm": 0.015, "dyaw_mrad": 0.043, "robust_1sigma_mm": [0.287, 0.297]} |
| 6b | /scan per-surface median residual within +-8 mm, seam beams masked | per run (frozen definition) | **PASS**: worst 2.42 mm | **PASS**: worst 1.60 mm | **PASS**: worst 1.93 mm |
| 6c | /scan every individual range within +-8 mm (strict per-beam reading of M0) | per run (reported) | **FAIL**: 85.8 % within; p95 11.32 mm, max 17.39 mm | **FAIL**: 88.4 % within; p95 11.31 mm, max 18.02 mm | **FAIL**: 84.4 % within; p95 11.74 mm, max 18.89 mm |
| 6-raw | /scan pose agreement without the seam mask (raw, all beams) | per run (reported) | **FAIL**: {"dx_mm": 0.4, "dy_mm": 0.16, "1sigma_mm": [15.5, 15.9], "residual_abs_max_mm": 2679.1} | **FAIL**: {"dx_mm": 0.311, "dy_mm": 0.105, "1sigma_mm": [15.5, 15.9], "residual_abs_max_mm": 2680.5} | **FAIL**: {"dx_mm": 0.187, "dy_mm": 0.078, "1sigma_mm": [15.5, 15.9], "residual_abs_max_mm": 2680.6} |
| 7 | command ownership: no other commander visible | per run | **PASS**: {"ros2_action_info_clients": 0, "ros2_topic_info_publishers": 0, "observer_start": [0, 0], | **PASS**: {"ros2_action_info_clients": 0, "ros2_topic_info_publishers": 0, "observer_start": [0, 0], | **PASS**: {"ros2_action_info_clients": 0, "ros2_topic_info_publishers": 0, "observer_start": [0, 0], |
| 8 | clean shutdown: launch group empty, nothing left | per run | **PASS**: process group empty after 3 s | **PASS**: process group empty after 3 s | **PASS**: process group empty after 4 s |
| M | mount from the running model: weld world->dummy_link (0, 0, 0.125, rpy 0) | per run | **PASS**: {"robot_description_weld": {"type": "fixed", "parent": "world", "child": "dummy_link", "xy | **PASS**: {"robot_description_weld": {"type": "fixed", "parent": "world", "child": "dummy_link", "xy | **PASS**: {"robot_description_weld": {"type": "fixed", "parent": "world", "child": "dummy_link", "xy |
| Z | zero motion commands and goals | per run | **PASS**: {"launch_log_goal_lines": 0, "command_topic_messages": 0, "goal_statuses": 0, "observer_go | **PASS**: {"launch_log_goal_lines": 0, "command_topic_messages": 0, "goal_statuses": 0, "observer_go | **PASS**: {"launch_log_goal_lines": 0, "command_topic_messages": 0, "goal_statuses": 0, "observer_go |
| H1 | supplementary: controller reference constant over the capture | per run (supplementary) | **UNMEASURED**: {"(not subscribed: helper looked up the type by discovery and found none)": {"messages": 0 | **PASS**: {"/leg_trajectory_controller/controller_state": {"messages": 207, "max_ref_change_rad": 0. | **PASS**: {"/leg_trajectory_controller/controller_state": {"messages": 290, "max_ref_change_rad": 0. |
| H2 | supplementary: joint states unchanged over the capture | per run (supplementary) | **PASS**: 4.811147140404426e-19 | **PASS**: 1.7618285302889447e-19 | **PASS**: 6.776263578034403e-20 |

Evidence paths per cell are in requirements_by_run.json (relative to docs/evidence/m61a_cloud/).
