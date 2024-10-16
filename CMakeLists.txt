cmake_minimum_required(VERSION 3.5)
project(log_rotation_test)

# 寻找 ROS2 相关依赖项
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(spdlog REQUIRED)
find_package(fmt REQUIRED)  # 确保找到 fmt 库

# 确保使用虚拟环境中的 Python 和 NumPy 路径
find_package(Python3 REQUIRED COMPONENTS Interpreter Development)

# 获取 NumPy 的 include 路径并存入变量
execute_process(
  COMMAND ${Python3_EXECUTABLE} -c "import numpy; print(numpy.get_include())"
  OUTPUT_VARIABLE NUMPY_INCLUDE_DIR
  OUTPUT_STRIP_TRAILING_WHITESPACE
)

# 打印 NumPy 的 include 路径以进行验证
message(STATUS "Using NumPy include path: ${NUMPY_INCLUDE_DIR}")

# 添加 NumPy 的 include 目录
include_directories(${NUMPY_INCLUDE_DIR})

# 添加可执行文件 log_rotation_node
add_executable(log_rotation_node src/log_rotation_node.cpp)

# 链接依赖项
ament_target_dependencies(log_rotation_node rclcpp spdlog fmt)  # 确保链接 fmt 库

# 如果 fmt::fmt 的方式无法解决问题，您可以手动链接库路径：
# target_link_libraries(log_rotation_node /usr/lib/x86_64-linux-gnu/libfmt.so)

# 安装可执行文件到指定目录
install(TARGETS
  log_rotation_node
  DESTINATION lib/${PROJECT_NAME})

# 调用 ament_package 生成 package 信息
ament_package()

# 设置 C++ 标准
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
