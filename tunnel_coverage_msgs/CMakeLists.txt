cmake_minimum_required(VERSION 3.16)
project(tunnel_coverage_msgs)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "action/ExecuteCoverage.action"
  "msg/CoverageStatus.msg"
  "msg/CoverageSegment.msg"
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
