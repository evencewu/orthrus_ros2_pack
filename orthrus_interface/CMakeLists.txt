cmake_minimum_required(VERSION 3.5)
project(orthrus_interface)

# 找到依赖包
set(THIS_PACKAGE_INCLUDE_DEPENDS
  rclcpp
  rclcpp_lifecycle
  pinocchio
  rsf_robot_interface
  rsf_pinocchio_interface
)

find_package(ament_cmake REQUIRED)
find_package (Eigen3 3.3 REQUIRED)

foreach(Dependency IN ITEMS ${THIS_PACKAGE_INCLUDE_DEPENDS})
  find_package(${Dependency} REQUIRED)
endforeach()

# 包含头文件目录
include_directories(include)

# 编译源文件生成库
add_library(${PROJECT_NAME} SHARED src/OrthrusInterface.cpp)

target_link_libraries(${PROJECT_NAME} PUBLIC Eigen3::Eigen)

ament_target_dependencies(${PROJECT_NAME} PUBLIC ${THIS_PACKAGE_INCLUDE_DEPENDS})

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