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
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(test_config_check test/test_config_check.py)
  ament_add_pytest_test(test_joint_safety test/test_joint_safety.py)
endif()

ament_package()
