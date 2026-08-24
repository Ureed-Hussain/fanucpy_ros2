cmake_minimum_required(VERSION 3.8)
project(fanuc_m10ia_moveit_config)

find_package(ament_cmake REQUIRED)

install(
  DIRECTORY config launch
  DESTINATION share/${PROJECT_NAME}
  PATTERN "__pycache__" EXCLUDE
)

install(
  FILES .setup_assistant README.md
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  # Keep unrelated user-installed pytest plugins out of this offline test.
  set(ENV{PYTEST_DISABLE_PLUGIN_AUTOLOAD} "1")
  find_package(ament_cmake_pytest REQUIRED)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()

  ament_add_pytest_test(
    test_moveit_config
    test/test_moveit_config.py
    WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
    ENV PYTEST_DISABLE_PLUGIN_AUTOLOAD=1
  )
endif()

ament_package()
