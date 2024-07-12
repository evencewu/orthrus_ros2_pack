cmake_minimum_required(VERSION 3.5)
project(orthrus_description)

# 找到依赖包
set(THIS_PACKAGE_INCLUDE_DEPENDS
  rclcpp
)

find_package(ament_cmake REQUIRED)

foreach(Dependency IN ITEMS ${THIS_PACKAGE_INCLUDE_DEPENDS})
  find_package(${Dependency} REQUIRED)
endforeach()

# 安装头文件
install(
  DIRECTORY models/
  DESTINATION models/
)

ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME})

ament_package()