cmake_minimum_required(VERSION 3.8)
project(orthrus_interfaces)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake_auto REQUIRED)
find_package(rosidl_default_generators REQUIRED)
ament_auto_find_build_dependencies()

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/CtrlCmd.msg"
  "msg/ImuReceive.msg"
  "msg/MotorCmd.msg"
  "msg/MotorReceive.msg"
  "msg/ReceiveSensor.msg"

  DEPENDENCIES
    sensor_msgs
    std_msgs
    geometry_msgs
)

ament_package()
