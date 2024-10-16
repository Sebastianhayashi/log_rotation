cmake_minimum_required(VERSION 3.5)
project(my_ros2_logging_project)

# 查找依赖
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rcl_logging_spdlog REQUIRED)
find_package(rcutils REQUIRED)
find_package(fmt REQUIRED)  # 显式查找 fmt 库

# 添加可执行文件
add_executable(log_test_node src/log_test_node.cpp)

# 链接依赖库
ament_target_dependencies(log_test_node
  rclcpp
  rcl_logging_spdlog
  rcutils
  fmt  # 添加 fmt 库的依赖
)

# 手动链接 pthread 库
target_link_libraries(log_test_node
  pthread  # 显式链接 pthread
)

# 安装可执行文件
install(TARGETS log_test_node
  DESTINATION lib/${PROJECT_NAME})

# 安装配置文件
install(DIRECTORY config
  DESTINATION share/${PROJECT_NAME}/)

# 安装launch文件
install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}/)

ament_package()
