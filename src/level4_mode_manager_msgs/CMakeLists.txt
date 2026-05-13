cmake_minimum_required(VERSION 3.8)
project(level4_mode_manager_msgs)

find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/Level4Availability.msg"
  "msg/Level4DrivingStatus.msg"
  "srv/EnableLevel4Driving.srv"
  DEPENDENCIES
    builtin_interfaces
    tier4_external_api_msgs
  ADD_LINTER_TESTS
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_auto_package()
