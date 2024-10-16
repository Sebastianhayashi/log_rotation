cmake_minimum_required(VERSION 3.5)
project(my_ros2_logging_project)

# 查找ROS2包依赖
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rcutils REQUIRED)

# 添加可执行文件
add_executable(log_test_node src/log_test_node.cpp)

# 链接依赖库
ament_target_dependencies(log_test_node rclcpp)

# 安装节点可执行文件
install(TARGETS
  log_test_node
  DESTINATION lib/${PROJECT_NAME})

# 安装配置文件
install(DIRECTORY config
  DESTINATION share/${PROJECT_NAME}/)

  # 安装 launch 文件
install(DIRECTORY launch
DESTINATION share/${PROJECT_NAME}/
)

ament_package()
