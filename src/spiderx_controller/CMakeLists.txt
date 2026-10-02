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
endif()

ament_package()
