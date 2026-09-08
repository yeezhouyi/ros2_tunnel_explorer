cmake_minimum_required(VERSION 3.16)
project(tunnel_coverage_executor LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# ── Dependencies ──────────────────────────────────────────────────────────

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tunnel_map_core REQUIRED)
find_package(tunnel_coverage_planner REQUIRED)
find_package(tunnel_coverage_msgs REQUIRED)

# ── Static library (pure logic: task core + checkpoint store) ─────────────

add_library(coverage_executor_core STATIC
  src/checkpoint_store.cpp
  src/coverage_task_core.cpp
)

target_include_directories(coverage_executor_core
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

target_link_libraries(coverage_executor_core
  PUBLIC
    tunnel_map_core::tunnel_map_core
    tunnel_coverage_planner::coverage_planning
)

# ── Executable ────────────────────────────────────────────────────────────

add_executable(tunnel_coverage_executor
  src/coverage_executor_node.cpp
  src/main.cpp
)

target_link_libraries(tunnel_coverage_executor coverage_executor_core)

target_include_directories(tunnel_coverage_executor
  PRIVATE
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
)

ament_target_dependencies(tunnel_coverage_executor
  rclcpp
  rclcpp_action
  nav_msgs
  nav2_msgs
  geometry_msgs
  tf2_ros
  tunnel_map_core
  tunnel_coverage_planner
  tunnel_coverage_msgs
)

# ── Install targets ──────────────────────────────────────────────────────

install(TARGETS tunnel_coverage_executor
  DESTINATION lib/${PROJECT_NAME}
)

install(TARGETS coverage_executor_core
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
)

install(DIRECTORY include/
  DESTINATION include
)

install(DIRECTORY config launch
  DESTINATION share/${PROJECT_NAME}
)

# ── Testing ───────────────────────────────────────────────────────────────

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()

  find_package(ament_cmake_gtest REQUIRED)

  ament_add_gtest(test_checkpoint_store
    test/test_checkpoint_store.cpp
  )
  target_link_libraries(test_checkpoint_store coverage_executor_core)
  target_include_directories(test_checkpoint_store PRIVATE
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  )

  ament_add_gtest(test_task_core
    test/test_task_core.cpp
  )
  target_link_libraries(test_task_core coverage_executor_core)
  target_include_directories(test_task_core PRIVATE
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  )
endif()

ament_package()
