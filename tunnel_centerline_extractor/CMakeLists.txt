cmake_minimum_required(VERSION 3.8)
project(tunnel_centerline_extractor)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(std_msgs REQUIRED)

set(INCLUDE_DIR include)

add_library(centerline_algorithms STATIC
  src/distance_field.cpp
  src/skeleton_extractor.cpp
  src/centerline_graph.cpp
)
target_include_directories(centerline_algorithms PUBLIC ${INCLUDE_DIR})

add_executable(tunnel_centerline_extractor
  src/main.cpp
  src/tunnel_centerline_node.cpp
)
target_link_libraries(tunnel_centerline_extractor
  centerline_algorithms
  ${rclcpp_LIBRARIES}
  ${nav_msgs_LIBRARIES}
  ${visualization_msgs_LIBRARIES}
)
target_include_directories(tunnel_centerline_extractor PRIVATE ${INCLUDE_DIR})
ament_target_dependencies(tunnel_centerline_extractor rclcpp nav_msgs visualization_msgs std_msgs)

install(TARGETS tunnel_centerline_extractor centerline_algorithms
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY ${INCLUDE_DIR}/ DESTINATION include/${PROJECT_NAME})
install(DIRECTORY launch config DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)

  ament_add_gtest(test_distance_field test/test_distance_field.cpp)
  target_link_libraries(test_distance_field centerline_algorithms)
  target_include_directories(test_distance_field PRIVATE ${INCLUDE_DIR})

  ament_add_gtest(test_skeleton_extractor test/test_skeleton_extractor.cpp)
  target_link_libraries(test_skeleton_extractor centerline_algorithms)
  target_include_directories(test_skeleton_extractor PRIVATE ${INCLUDE_DIR})

  ament_add_gtest(test_centerline_graph test/test_centerline_graph.cpp)
  target_link_libraries(test_centerline_graph centerline_algorithms)
  target_include_directories(test_centerline_graph PRIVATE ${INCLUDE_DIR})
endif()

ament_package()
