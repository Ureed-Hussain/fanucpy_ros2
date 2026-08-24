cmake_minimum_required(VERSION 3.8)
project(fanucpy_ros2_controller)

find_package(ament_cmake REQUIRED)

install(
  DIRECTORY karel
  DESTINATION share/${PROJECT_NAME}
)

install(
  FILES README.md
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_cmake_copyright REQUIRED)
  find_package(ament_cmake_lint_cmake REQUIRED)
  find_package(ament_cmake_xmllint REQUIRED)

  ament_copyright()
  ament_lint_cmake()
  ament_xmllint()
endif()

ament_package()
