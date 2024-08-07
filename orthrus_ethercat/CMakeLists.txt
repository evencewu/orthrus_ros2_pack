cmake_minimum_required(VERSION 3.5)
project(orthrus_ethercat)

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
ament_auto_find_build_dependencies()

include_directories(${EIGEN3_INCLUDE_DIRS})

file(GLOB_RECURSE MAIN_SOURCES "src/assembly/*.cpp" "src/ethercat/*.cpp")

ament_auto_add_library(${PROJECT_NAME} SHARED 
  ${MAIN_SOURCES}
)

ament_auto_add_executable(orthrus_ethercat_test
  src/test/ethercat_test.cpp
  ${MAIN_SOURCES}
)

#ament_auto_add_executable(orthrus_calibrate_mag
#  src/orthrus_calibrate_mag.cpp
#  ${SOURCES}
#)

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

install(
  DIRECTORY include/
  DESTINATION include
)

ament_auto_package(
  INSTALL_TO_SHARE
)


