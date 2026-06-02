cmake_minimum_required(VERSION 3.16)
project(tunnel_frontier_explorer LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# ── Dependencies ──────────────────────────────────────────────────────────

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)

# ── Static library (pure algorithms, no ROS dependencies) ─────────────────

add_library(frontier_detection STATIC
  src/frontier_detector.cpp
  src/frontier_blacklist.cpp
  src/frontier_goal_selector.cpp
  src/frontier_scorer.cpp
  src/frontier_visit_history.cpp
)

target_include_directories(frontier_detection
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

# ── Executable ────────────────────────────────────────────────────────────

add_executable(tunnel_frontier_explorer
  src/frontier_explorer_node.cpp
  src/main.cpp
)

target_link_libraries(tunnel_frontier_explorer frontier_detection)

target_include_directories(tunnel_frontier_explorer
  PRIVATE
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
)

ament_target_dependencies(tunnel_frontier_explorer
  rclcpp
  rclcpp_action
  nav_msgs
  nav2_msgs
  visualization_msgs
  geometry_msgs
  tf2_ros
)

# ── Install targets ──────────────────────────────────────────────────────

install(TARGETS tunnel_frontier_explorer
  DESTINATION lib/${PROJECT_NAME}
)

install(TARGETS frontier_detection
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

  # ── FrontierDetector tests ────────────────────────────────────────────
  ament_add_gtest(test_frontier_detector
    test/test_frontier_detector.cpp
  )
  target_link_libraries(test_frontier_detector frontier_detection)
  target_include_directories(test_frontier_detector PRIVATE
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  )

  # ── FrontierBlacklist tests ───────────────────────────────────────────
  ament_add_gtest(test_frontier_blacklist
    test/test_frontier_blacklist.cpp
  )
  target_link_libraries(test_frontier_blacklist frontier_detection)
  target_include_directories(test_frontier_blacklist PRIVATE
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  )

  # ── FrontierGoalSelector tests ───────────────────────────────────────
  ament_add_gtest(test_frontier_goal_selector
    test/test_frontier_goal_selector.cpp
  )
  target_link_libraries(test_frontier_goal_selector frontier_detection)
  target_include_directories(test_frontier_goal_selector PRIVATE
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  )

  # ── FrontierScorer tests ─────────────────────────────────────────────
  ament_add_gtest(test_frontier_scorer
    test/test_frontier_scorer.cpp
  )
  target_link_libraries(test_frontier_scorer frontier_detection)
  target_include_directories(test_frontier_scorer PRIVATE
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  )

  # ── FrontierVisitHistory tests ───────────────────────────────────────
  ament_add_gtest(test_frontier_visit_history
    test/test_frontier_visit_history.cpp
  )
  target_link_libraries(test_frontier_visit_history frontier_detection)
  target_include_directories(test_frontier_visit_history PRIVATE
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  )
endif()

ament_package()
