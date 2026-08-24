cmake_minimum_required(VERSION 3.8)
project(fanucpy_ros2_interfaces)

find_package(ament_cmake REQUIRED)
find_package(action_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "action/JogCartesian.action"
  "action/RunProgram.action"
  "msg/CartesianState.msg"
  "msg/DriverStatus.msg"
  "srv/GetDigitalOutput.srv"
  "srv/GetNumericRegister.srv"
  "srv/GetPower.srv"
  "srv/SetDigitalOutput.srv"
  "srv/SetNumericRegister.srv"
  DEPENDENCIES action_msgs std_msgs
)

ament_export_dependencies(rosidl_default_runtime)
ament_package()
