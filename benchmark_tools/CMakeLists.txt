cmake_minimum_required(VERSION 3.16)
project(benchmark_tools)

find_package(ament_cmake REQUIRED)

install(DIRECTORY scenarios bags
  DESTINATION share/${PROJECT_NAME}
)

install(PROGRAMS scripts/record_stage0_metrics.py
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
