cmake_minimum_required(VERSION 3.8)
project(orthrus_control)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

SET(CMAKE_BUILD_TYPE "Debug")
SET(CMAKE_CXX_FLAGS_DEBUG "$ENV{CXXFLAGS} -O0 -Wall -g -ggdb")

# find dependencies
set(THIS_PACKAGE_INCLUDE_DEPENDS
  hardware_interface
  pluginlib
  rclcpp
  rclcpp_lifecycle
  realtime_tools
  std_msgs
  soem_ros2
  tf2_msgs
  tf2
)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(Eigen3 REQUIRED)  

include_directories(${EIGEN3_INCLUDE_DIRS})

foreach(Dependency IN ITEMS ${THIS_PACKAGE_INCLUDE_DEPENDS})
  find_package(${Dependency} REQUIRED)
endforeach()

include_directories(
include/orthrus_control
)

file(GLOB_RECURSE MAIN_SOURCES "src/assembly/*.cpp" "src/ethercat/*.cpp" "src/calibration/*.cpp")

## COMPILE
add_library(
  orthrus_control
  SHARED
  src/OrthrusInterface.cpp
  ${MAIN_SOURCES}
)
target_compile_features(orthrus_control PUBLIC cxx_std_17)

target_include_directories(orthrus_control PUBLIC
$<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>
$<INSTALL_INTERFACE:include/orthrus_control>
)

ament_target_dependencies(
  orthrus_control PUBLIC
  ${THIS_PACKAGE_INCLUDE_DEPENDS}
)

# Causes the visibility macros to use dllexport rather than dllimport,
# which is appropriate when building the dll but not consuming it.
target_compile_definitions(${PROJECT_NAME} PRIVATE "ORTHRUS_CONTROL_BUILDING_DLL")

# Export hardware plugins
pluginlib_export_plugin_description_file(hardware_interface orthrus_control.xml)

# INSTALL
install(
  DIRECTORY include/
  DESTINATION include/orthrus_control
)

install(
  DIRECTORY  description/ros2_control description/urdf  description/launch description/rviz
  DESTINATION share/orthrus_control
)

install(
  DIRECTORY bringup/launch bringup/config
  DESTINATION share/orthrus_control
)

install(TARGETS orthrus_control
  EXPORT export_orthrus_control
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_export_targets(export_orthrus_control HAS_LIBRARY_TARGET)
ament_export_dependencies(${THIS_PACKAGE_INCLUDE_DEPENDS})
ament_package()
