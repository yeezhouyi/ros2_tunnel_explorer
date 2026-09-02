cmake_minimum_required(VERSION 3.16)
project(tunnel_map_core LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# ── Dependencies ──────────────────────────────────────────────────────────
# tunnel_map_core is a pure-C++ library: it deliberately has no ROS
# dependencies so the geometry core can be unit-tested and reused by both the
# frontier explorer and the coverage planner without pulling rclcpp.

find_package(ament_cmake REQUIRED)

# ── Static library (pure algorithms, no ROS dependencies) ─────────────────

add_library(tunnel_map_core STATIC
  src/grid_geometry.cpp
  src/map_digest.cpp
)

target_include_directories(tunnel_map_core
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

# ── Install targets ──────────────────────────────────────────────────────

install(TARGETS tunnel_map_core
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
)

install(DIRECTORY include/
  DESTINATION include
)

# ── Testing ───────────────────────────────────────────────────────────────

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()

  find_package(ament_cmake_gtest REQUIRED)

  ament_add_gtest(test_grid_geometry
    test/test_grid_geometry.cpp
  )
  target_link_libraries(test_grid_geometry tunnel_map_core)
  target_include_directories(test_grid_geometry PRIVATE
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  )

  ament_add_gtest(test_map_digest
    test/test_map_digest.cpp
  )
  target_link_libraries(test_map_digest tunnel_map_core)
  target_include_directories(test_map_digest PRIVATE
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  )
endif()

ament_package()
