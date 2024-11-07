cmake_minimum_required(VERSION 3.5)
project(ros2_log_rotation)

# 设置 C++ 标准
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# 查找依赖
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rcl_logging_spdlog REQUIRED)
find_package(rcutils REQUIRED)
find_package(fmt REQUIRED)  # 显式查找 fmt 库

# 添加可执行文件
add_executable(log_rotation_node src/main.cpp)

# 链接依赖库
ament_target_dependencies(log_rotation_node
  rclcpp
  rcl_logging_spdlog
  rcutils
  fmt  # 添加 fmt 库依赖
)

# 手动链接 pthread 库（仅在 Linux 平台）
if(CMAKE_SYSTEM_NAME MATCHES "Linux")
  target_link_libraries(log_rotation_node pthread)
endif()

# 添加 fmt 库
target_link_libraries(log_rotation_node fmt)

# 添加链接器标志以解决 DSO 错误
set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} -Wl,--copy-dt-needed-entries")

# 安装可执行文件
install(TARGETS log_rotation_node
  DESTINATION lib/${PROJECT_NAME})

# 安装配置文件
install(DIRECTORY config
  DESTINATION share/${PROJECT_NAME}/)

# 安装 launch 文件
install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}/)

ament_package()
