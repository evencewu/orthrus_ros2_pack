cmake_minimum_required(VERSION 3.5)
project(orthrus_ethercat)

# 找到依赖包
set(THIS_PACKAGE_INCLUDE_DEPENDS
  soem_ros2
  ethercat_interfaces
  rclcpp
)

find_package(ament_cmake REQUIRED)

foreach(Dependency IN ITEMS ${THIS_PACKAGE_INCLUDE_DEPENDS})
  find_package(${Dependency} REQUIRED)
endforeach()

# 包含头文件目录
include_directories(include)

file(GLOB_RECURSE MAIN_SOURCES "src/*.cpp")

# 编译源文件生成库
add_library(${PROJECT_NAME} SHARED ${MAIN_SOURCES})

ament_target_dependencies(${PROJECT_NAME} PUBLIC ${THIS_PACKAGE_INCLUDE_DEPENDS})

# 编译测试用例
#add_executable(test_DM4310 test/test_demo_DM4310.cpp ${MAIN_SOURCES})
#add_executable(test_can test/test_demo_can.cpp ${MAIN_SOURCES})
#
#ament_target_dependencies(test_DM4310
#  ${THIS_PACKAGE_INCLUDE_DEPENDS}
#)
#
#ament_target_dependencies(test_can
#  ${THIS_PACKAGE_INCLUDE_DEPENDS}
#)

#target_link_libraries(test_DM4310 PUBLIC ${PROJECT_NAME})

#install(TARGETS 
#  test_DM4310
#  DESTINATION lib/${PROJECT_NAME}
#)
#
#install(TARGETS 
#  test_can
#  DESTINATION lib/${PROJECT_NAME}
#)

# 安装头文件
install(
  DIRECTORY include/
  DESTINATION include/
)

# 安装库
install(
  TARGETS ${PROJECT_NAME}
  EXPORT export_${PROJECT_NAME}
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include
)

ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME})

ament_package()


