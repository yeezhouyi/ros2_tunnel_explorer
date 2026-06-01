cmake_minimum_required(VERSION 3.16)
project(benchmark_tools)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(Python3 REQUIRED COMPONENTS Interpreter)

install(DIRECTORY scenarios bags config
  DESTINATION share/${PROJECT_NAME}
)

install(PROGRAMS scripts/record_stage0_metrics.py
          scripts/run_navigation_smoke_test.py
          scripts/analyze_stage2b_outlier.py
  DESTINATION lib/${PROJECT_NAME}
)

# Install Python module for importable stats functions
ament_python_install_package(${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()

  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(test_stats tests/test_stats.py
    PYTHON_EXECUTABLE ${Python3_EXECUTABLE}
    APPEND_ENV PYTHONPATH=${CMAKE_CURRENT_SOURCE_DIR}
    TIMEOUT 60
  )
  ament_add_pytest_test(test_path_util tests/test_path_util.py
    PYTHON_EXECUTABLE ${Python3_EXECUTABLE}
    APPEND_ENV PYTHONPATH=${CMAKE_CURRENT_SOURCE_DIR}
    TIMEOUT 60
  )
endif()

ament_package()
