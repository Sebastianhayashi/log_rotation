cmake_minimum_required(VERSION 3.5)
project(log_rotation_tests)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_gtest REQUIRED)
find_package(rclcpp REQUIRED)

# 添加测试文件
ament_add_gtest(test_logger test_logger.cpp)
target_link_libraries(test_logger
  rclcpp
  spdlog
  log_rotation  # 需要替换为您项目中的目标名
)

ament_add_gtest(test_compression test_compression.cpp)
target_link_libraries(test_compression
  rclcpp
  log_rotation  # 需要替换为您项目中的目标名
)

ament_package()
