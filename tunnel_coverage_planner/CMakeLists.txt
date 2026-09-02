cmake_minimum_required(VERSION 3.16)
project(tunnel_coverage_planner LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# ── Dependencies ──────────────────────────────────────────────────────────
# Pure-C++ package: depends only on tunnel_map_core.  No ROS headers are
# linked into the algorithm library (R18 of the cleaning-track contract).

find_package(ament_cmake REQUIRED)
find_package(tunnel_map_core REQUIRED)

# ── Static library (pure algorithms, no ROS dependencies) ─────────────────

add_library(coverage_planning STATIC
  src/cleanable_map_builder.cpp
  src/coverage_tracker.cpp
  src/scanline_planner.cpp
)

target_include_directories(coverage_planning
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

target_link_libraries(coverage_planning tunnel_map_core)

# ── Install targets ──────────────────────────────────────────────────────

install(TARGETS coverage_planning
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
)

install(DIRECTORY include/
  DESTINATION include
)

# ── Export for downstream packages (executor) ────────────────────────────

ament_export_include_directories(include)
ament_export_dependencies(tunnel_map_core)
ament_export_targets(coverage_planning HAS_LIBRARY_TARGET)

# ── Testing ───────────────────────────────────────────────────────────────

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()

  find_package(ament_cmake_gtest REQUIRED)

  ament_add_gtest(test_robot_cleaning_geometry
    test/test_robot_cleaning_geometry.cpp
  )
  target_link_libraries(test_robot_cleaning_geometry coverage_planning)
  target_include_directories(test_robot_cleaning_geometry PRIVATE
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  )

  ament_add_gtest(test_cleanable_map_builder
    test/test_cleanable_map_builder.cpp
  )
  target_link_libraries(test_cleanable_map_builder coverage_planning)
  target_include_directories(test_cleanable_map_builder PRIVATE
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  )

  ament_add_gtest(test_coverage_tracker
    test/test_coverage_tracker.cpp
  )
  target_link_libraries(test_coverage_tracker coverage_planning)
  target_include_directories(test_coverage_tracker PRIVATE
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  )

  ament_add_gtest(test_scanline_planner
    test/test_scanline_planner.cpp
  )
  target_link_libraries(test_scanline_planner coverage_planning)
  target_include_directories(test_scanline_planner PRIVATE
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  )
endif()

ament_package()
