cmake_minimum_required(VERSION 3.5)
project(orthrus_control)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if (CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    add_compile_options(-Wall -Wextra -Wpedantic -Wnon-virtual-dtor -Woverloaded-virtual)
endif ()

# find dependencies
find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

ament_auto_add_executable(orthrus_control
  src/OrthrusGazeboInterface.cpp
)

target_compile_definitions(orthrus_control PRIVATE "ORTHRUS_CONTROL_BUILDING_DLL")
pluginlib_export_plugin_description_file(hardware_interface orthrus_control.xml)



#############
## Testing ##
#############

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()


#############
## Install ##
#############

ament_auto_package(
  INSTALL_TO_SHARE
  launch
  config
)