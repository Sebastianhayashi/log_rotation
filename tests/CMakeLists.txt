cmake_minimum_required(VERSION 3.5)

# 添加测试可执行文件
add_executable(test_logger
  test_logger.cpp
  ../src/logger_manager.cpp
  ../src/compression_util.cpp
  ../src/config.cpp
)

ament_target_dependencies(test_logger
  rclcpp
  spdlog
  yaml-cpp
  GTest
)

# 链接 GTest 库
target_link_libraries(test_logger
  GTest::GTest
  GTest::Main
)

# 添加测试
ament_add_gtest(test_logger_exe test_logger.cpp)
ament_target_dependencies(test_logger_exe rclcpp)

add_executable(test_compression
  test_compression.cpp
  ../src/compression_util.cpp
)

ament_target_dependencies(test_compression
  GTest
)

target_link_libraries(test_compression
  GTest::GTest
  GTest::Main
)

ament_add_gtest(test_compression_exe test_compression.cpp)
