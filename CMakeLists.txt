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
  fmt  # 添加 fmt 库依赖
)

# 手动链接 pthread 库和 fmt 库
target_link_libraries(log_test_node
  fmt  # 手动添加 fmt 库
  pthread  # 显式添加 pthread 库
)

# 添加链接器标志以解决 DSO 错误
set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} -Wl,--copy-dt-needed-entries")

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
