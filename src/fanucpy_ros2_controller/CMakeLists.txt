cmake_minimum_required(VERSION 3.8)
project(fanucpy_ros2_controller)

find_package(ament_cmake REQUIRED)

install(
  DIRECTORY fanuc_driver karel
  DESTINATION share/${PROJECT_NAME}
)

install(
  FILES README.md UPSTREAM.md
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  set(ENV{PYTEST_DISABLE_PLUGIN_AUTOLOAD} "1")
  find_package(Python3 REQUIRED COMPONENTS Interpreter)
  find_package(ament_cmake_pytest REQUIRED)
  find_package(ament_cmake_copyright REQUIRED)
  find_package(ament_cmake_lint_cmake REQUIRED)
  find_package(ament_cmake_xmllint REQUIRED)

  ament_add_pytest_test(
    test_controller_bundle
    test/test_controller_bundle.py
    PYTHON_EXECUTABLE ${Python3_EXECUTABLE}
    ENV PYTEST_DISABLE_PLUGIN_AUTOLOAD=1
  )
  ament_copyright()
  ament_lint_cmake()
  ament_xmllint()
endif()

ament_package()
