cmake_minimum_required(VERSION 3.8)
project(spiderx_controller)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)

ament_python_install_package(${PROJECT_NAME})

install(
  DIRECTORY config launch
  DESTINATION share/${PROJECT_NAME}
)

install(PROGRAMS
  scripts/validate_controller_config
  scripts/test_one_joint.py
  scripts/test_neutral_pose.py
  scripts/run_posture_hold_test.py
  scripts/validate_leg_kinematics
  scripts/m4_pose_validation
  scripts/m4_5_gait_analysis
  scripts/m5_offline_evaluation
  scripts/m6_offline_preflight
  scripts/m6_live_preflight
  scripts/m6_live_playback
  scripts/m6_gait_replay.py
  scripts/m61a_clearance
  scripts/m61a_observe_fixed_base
  scripts/m55_locomotion_node
  scripts/m55_teleop_keyboard
  scripts/m55_gait_feasibility
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(test_config_check test/test_config_check.py)
  ament_add_pytest_test(test_joint_safety test/test_joint_safety.py)
  ament_add_pytest_test(test_posture_config test/test_posture_config.py)
  ament_add_pytest_test(test_posture_metrics test/test_posture_metrics.py)
  ament_add_pytest_test(test_leg_kinematics test/test_leg_kinematics.py)
  ament_add_pytest_test(test_kinematics_targets test/test_kinematics_targets.py)
  ament_add_pytest_test(test_all_leg_kinematics test/test_all_leg_kinematics.py)
  ament_add_pytest_test(test_m4_pose_targets test/test_m4_pose_targets.py)
  ament_add_pytest_test(test_m4_pose_validation test/test_m4_pose_validation.py)
  ament_add_pytest_test(test_controller_launch test/test_controller_launch.py)
  ament_add_pytest_test(test_validator_state_checks test/test_validator_state_checks.py)
  ament_add_pytest_test(test_gait_config test/test_gait_config.py)
  ament_add_pytest_test(test_gait_trajectory test/test_gait_trajectory.py)
  ament_add_pytest_test(test_gait_kinematics test/test_gait_kinematics.py)
  ament_add_pytest_test(test_gait_metrics test/test_gait_metrics.py)
  ament_add_pytest_test(test_gait_report test/test_gait_report.py)
  ament_add_pytest_test(test_gait_regression test/test_gait_regression.py)
  ament_add_pytest_test(test_eval_study test/test_eval_study.py)
  ament_add_pytest_test(test_eval_runner test/test_eval_runner.py)
  ament_add_pytest_test(test_eval_records test/test_eval_records.py)
  ament_add_pytest_test(test_eval_tables test/test_eval_tables.py)
  ament_add_pytest_test(test_eval_regression test/test_eval_regression.py)
  ament_add_pytest_test(test_m6_trajectory test/test_m6_trajectory.py)
  ament_add_pytest_test(test_m6_offline_preflight test/test_m6_offline_preflight.py)
  ament_add_pytest_test(test_m6_action_client test/test_m6_action_client.py)
  ament_add_pytest_test(test_m6_live_preflight test/test_m6_live_preflight.py)
  ament_add_pytest_test(test_m6d_contract test/test_m6d_contract.py)
  ament_add_pytest_test(test_m6d_session test/test_m6d_session.py)
  ament_add_pytest_test(test_m6d_adapter_isolated test/test_m6d_adapter_isolated.py)
  ament_add_pytest_test(test_m6d_cli_readiness test/test_m6d_cli_readiness.py)
  ament_add_pytest_test(test_m6d_live_enabling test/test_m6d_live_enabling.py)
  ament_add_pytest_test(test_m6d_evidence_safety test/test_m6d_evidence_safety.py)
  ament_add_pytest_test(test_m6d_freshness_confirmation test/test_m6d_freshness_confirmation.py)
  ament_add_pytest_test(test_m6d_isolated_success test/test_m6d_isolated_success.py)
  ament_add_pytest_test(test_m61_gait_replay test/test_m61_gait_replay.py)
  ament_add_pytest_test(test_m61a_fixed_base test/test_m61a_fixed_base.py TIMEOUT 600)
  ament_add_pytest_test(test_m61a_clearance test/test_m61a_clearance.py TIMEOUT 600)
  ament_add_pytest_test(test_m61a_observer test/test_m61a_observer.py TIMEOUT 300)
  ament_add_pytest_test(test_m55_command test/test_m55_command.py)
  ament_add_pytest_test(test_m55_locomotion test/test_m55_locomotion.py TIMEOUT 300)
  ament_add_pytest_test(test_m55_gait test/test_m55_gait.py TIMEOUT 900)
  ament_add_pytest_test(test_m55_ros test/test_m55_ros.py TIMEOUT 300)
endif()

ament_package()
