cmake_minimum_required(VERSION 3.5)
project(ros2_log_rotation)

# 设置 C++ 标准
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# 查找依赖
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(spdlog REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(ament_index_cpp REQUIRED)

# 添加 include 目录
include_directories(
  include
)

# 添加可执行文件
add_executable(log_rotation_node
  src/main.cpp
  src/logger_manager.cpp
  src/compression_util.cpp
  src/config.cpp
)

# 链接依赖库
ament_target_dependencies(log_rotation_node
  rclcpp
  spdlog
  yaml-cpp
  ament_index_cpp
)

# 安装可执行文件
install(TARGETS log_rotation_node
  DESTINATION lib/${PROJECT_NAME})

# 安装配置文件
install(DIRECTORY config
  DESTINATION share/${PROJECT_NAME}/
)

# 安装 launch 文件
install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}/
)

# 如果有脚本或 GUI
install(PROGRAMS
  scripts/log_rotation_gui.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
