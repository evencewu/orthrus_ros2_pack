cmake_minimum_required(VERSION 3.8)
project(orthrus_controller)


set(CMAKE_VERBOSE_MAKEFILE ON)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

SET(CMAKE_BUILD_TYPE "Debug")
SET(CMAKE_CXX_FLAGS_DEBUG "$ENV{CXXFLAGS} -O0 -Wall -g -ggdb")

# find dependencies
set(THIS_PACKAGE_INCLUDE_DEPENDS
  controller_interface
  generate_parameter_library
  geometry_msgs
  hardware_interface
  pluginlib
  rclcpp
  rclcpp_lifecycle
  rcpputils
  realtime_tools
  nav_msgs
  tf2
  tf2_msgs
  pinocchio
  xbox_interfaces
  visualization_msgs
)

# find dependencies
find_package(ament_cmake REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)
find_package(backward_ros REQUIRED)
find_package(controller_manager REQUIRED)

foreach(Dependency IN ITEMS ${THIS_PACKAGE_INCLUDE_DEPENDS})
  find_package(${Dependency} REQUIRED)
endforeach()

find_package (Eigen3 3.3 REQUIRED)

include_directories(
  include/orthrus_controller
)

# generate parameter listener hpp file
generate_parameter_library(orthrus_controller_parameters 
  config/hardware_interface/orthrus_controller_parameter.yaml
)

add_executable(test1 test/test1.cpp)

target_include_directories(test1 PUBLIC
  $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/diff_test_controller>
)

target_link_libraries(test1 PUBLIC Eigen3::Eigen)

ament_target_dependencies(test1 PUBLIC ${THIS_PACKAGE_INCLUDE_DEPENDS})

# generate lib
add_library(orthrus_controller SHARED
  src/OrthrusController.cpp
  src/visualization/CalibrationVisualization.cpp
  src/visualization/OrthrusVisualization.cpp
  src/interfaces/PinocchioInterfaces.cpp
  src/interfaces/JoyInterface.cpp
  src/algorithm/LeggedOdom.cpp
  src/algorithm/LeggedTouch.cpp
  src/algorithm/LeggedMpc.cpp
  src/safecode/SafeCode.cpp
)

  

target_compile_features(orthrus_controller PUBLIC cxx_std_17)

target_include_directories(orthrus_controller PUBLIC
  $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/diff_test_controller>
)

target_link_libraries(orthrus_controller PUBLIC orthrus_controller_parameters)
ament_target_dependencies(orthrus_controller PUBLIC ${THIS_PACKAGE_INCLUDE_DEPENDS})
# Causes the visibility macros to use dllexport rather than dllimport,
# which is appropriate when building the dll but not consuming it.
target_compile_definitions(orthrus_controller PRIVATE "ORTHRUS_CONTROLLER_BUILDING_DLL")
pluginlib_export_plugin_description_file(controller_interface orthrus_plugin.xml)

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

install(
  DIRECTORY include/
  DESTINATION include/orthrus_controller
)

install(TARGETS test1
  DESTINATION lib/${PROJECT_NAME}
)

install(TARGETS orthrus_controller orthrus_controller_parameters
  EXPORT export_orthrus_controller
  RUNTIME DESTINATION bin
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
)

# 为了使下游文件可以访问
ament_export_targets(export_orthrus_controller HAS_LIBRARY_TARGET)
ament_export_dependencies(${THIS_PACKAGE_INCLUDE_DEPENDS})

ament_package()