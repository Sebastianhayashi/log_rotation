cmake_minimum_required(VERSION 3.5)
project(log_rotation_test)

# 寻找依赖项
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(spdlog REQUIRED)

add_executable(log_rotation_node src/log_rotation_node.cpp)

ament_target_dependencies(log_rotation_node rclcpp spdlog)

install(TARGETS
  log_rotation_node
  DESTINATION lib/${PROJECT_NAME})

ament_package()
