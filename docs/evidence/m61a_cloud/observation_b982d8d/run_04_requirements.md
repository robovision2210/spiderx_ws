| ID | Criterion | Result | Value |
|---|---|---|---|
| 1 | preflight READY | **PASS** | `[[]]` |
| 1b | 120 s observation READY at its end | **PASS** | `[]` |
| 2a | body z mean within 1 mm of 0.125 m | **PASS** | `0.125` |
| 2a | body z range <= 1 mm | **PASS** | `0.0` |
| 2b | attachment max translation <= 1 mm | **PASS** | `0.0` |
| 2b | attachment max rotation <= 0.0033 rad | **PASS** | `0.0` |
| 2c | spawn deviation <= 1 mm | **PASS** | `0.0` |
| 2d | link deviation <= 0.1 mm | **PASS** | `0.0` |
| 2e | tilt <= 0.0033 rad | **PASS** | `0.0` |
| 3 | pose receipt max gap < 1.0 s (wall) | **PASS** | `{"count": 6814, "mean_s": 0.017904423192573, "p99_s": 0.03758817800007819, "p999_s": 0.05780369000012797, "max_s": 0.3403347359999316}` |
| 3 | joint-state sim gap max <= 0.25 s | **PASS** | `0.019999999999999574` |
| 4 | no unexpected pose sample codes | **PASS** | `{}` |
| 7 | command ownership: no other commander visible | **PASS** | `{"ros2_action_info_clients": 0, "ros2_topic_info_publishers": 0, "observer_start": {"command_publishers": 0, "action_clients": 0}, "observer_end": {"command_pub…` |
| M | running /robot_description weld = world->dummy_link (0, 0, 0.125, rpy 0) | **PASS** | `{"type": "fixed", "parent": "world", "child": "dummy_link", "xyz": "0.0 0.0 0.125", "rpy": "0 0 0.0"}` |
| Z2 | capture: 0 messages on the command topic and no action goal status | **PASS** | `{"command_messages": 0, "status_messages": 0, "goal_statuses": []}` |
| Z3 | controller reference constant over the capture (holding, no new command) | **PASS** | `{"/leg_trajectory_controller/controller_state": {"messages": 241, "max_ref_change_rad": 0.0, "max_abs_error_rad": 1.3435378555678133e-11}, "/leg_trajectory_cont…` |
| Z | no trajectory goal or topic command reached the controller (launch log) | **PASS** | `0` |
| 6b (supplementary) | /scan: per-surface median residual within +-8 mm (supplementary) | **PASS** | `{"box_a/link": -0.00027915429836500305, "box_b/link": 0.0018083042840737473, "pillar/link": 0.0019687246183845986, "walls/wall_east": -0.0005716112201665791, "w…` |
| 6a (supplementary) | /scan: fitted lidar planar pose within 8 mm of the weld-implied pose (8 cube-map seam beams excluded) | **PASS** | `{"fit_minus_expected_m_m_rad": [0.0004883092869906874, 5.194826094979954e-05, 0.00011655825343992454], "one_sigma_m_m_rad": [0.0002657986403651435, 0.0002748504…` |
| 8 | clean shutdown: launch group empty, nothing left | **PASS** | `"none"` |
| 6 (v1, of record) | every individual range within +-8 mm of the welded pose (single scans and 40-scan means; seam and grazing beams excluded) | **FAIL** | `{"single_scan_within": 0.5307803468208092, "mean_within": 0.8554913294797688, "mean_max_abs_m": 0.018809217120341337, "minimax_any_planar_pose_m": 0.01767402171…` |
| 6 (v2 proposal) | fitted body planar pose vs pose/info (frozen spec f44c27c) | **PASS** | `{"dxy_m": 0.00048522159143015754, "dyaw_rad": 0.00011658756472502878, "sigma_used_m_m_rad": [0.0005783291516546807, 0.00035604524967438613, 0.000259605503819285…` |
| 6 (v2 counterfactual) | measured error pattern on a displaced body: offsets >= G8 FAIL, every offset recovered within 3 sigma | **PASS** | `[[[0.003, 0.0, 0.0], "FAIL", true], [[0.0, 0.003, 0.0], "FAIL", true], [[-0.0021213203, 0.0021213203, 0.0], "FAIL", true], [[0.0, 0.0, 0.01], "FAIL", true], [[0…` |
| R (new readiness) | preflight: /clock advanced and sim time advanced while the body pose arrived | **PASS** | `{"code": null, "detail": {"min_sim_advance_s": 0.1, "samples": 55, "sim_advance_s": 0.6029999999999998, "window_s": 1.0}, "ok": true}` |
