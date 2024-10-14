cmake_minimum_required(VERSION 3.5)
project(log_rotation_test)

# 寻找依赖项
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(spdlog REQUIRED)

# 确保使用虚拟环境中的 Python 和 NumPy 路径
find_package(Python3 REQUIRED COMPONENTS Interpreter Development)

# 获取 NumPy 的 include 路径
execute_process(
  COMMAND ${Python3_EXECUTABLE} -c "import numpy; print(numpy.get_include())"
  OUTPUT_VARIABLE NUMPY_INCLUDE_DIR
  OUTPUT_STRIP_TRAILING_WHITESPACE
)

# 添加 NumPy 的 include 目录
include_directories(${NUMPY_INCLUDE_DIR})

# 添加可执行文件
add_executable(log_rotation_node src/log_rotation_node.cpp)

# 链接依赖项
ament_target_dependencies(log_rotation_node rclcpp spdlog)

# 安装可执行文件
install(TARGETS
  log_rotation_node
  DESTINATION lib/${PROJECT_NAME})

# 调用 ament_package
ament_package()
