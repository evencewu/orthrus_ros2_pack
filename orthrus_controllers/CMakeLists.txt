cmake_minimum_required(VERSION 3.5)
project(orthrus_controllers)

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
find_package(Eigen3 REQUIRED)
find_package(pinocchio REQUIRED)
find_package(Boost REQUIRED COMPONENTS log)
ament_auto_find_build_dependencies()

file(GLOB_RECURSE MAIN_SOURCES "src/estimate/*.cpp" "src/interface/*.cpp" "src/hardware/*.cpp" "src/nmpc/*.cpp" "src/visualization/*.cpp")

ament_auto_add_executable(orthrus_controllers
  src/OrthrusControl.cpp
  ${MAIN_SOURCES}
)

target_link_libraries(orthrus_controllers Eigen3::Eigen)
#target_link_libraries(orthrus_ctrl Eigen3::Eigen Boost::log) 

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
  config
)