#include <gtest/gtest.h>
#include "logger_manager.hpp"
#include <rclcpp/rclcpp.hpp>

TEST(LoggerManagerTest, InitializationTest)
{
    auto node = std::make_shared<rclcpp::Node>("test_node");
    Config config;
    config.log_file_path = "/tmp/test_log.log";
    config.max_file_size = 1024; // 1KB
    config.max_files = 3;

    LoggerManager logger_manager(node, config);
    EXPECT_NO_THROW(logger_manager.initialize());
}

TEST(LoggerManagerTest, LogInfoTest)
{
    auto node = std::make_shared<rclcpp::Node>("test_node");
    Config config;
    config.log_file_path = "/tmp/test_log.log";
    config.max_file_size = 1024; // 1KB
    config.max_files = 3;

    LoggerManager logger_manager(node, config);
    logger_manager.initialize();
    EXPECT_NO_THROW(logger_manager.logInfo("This is an info message."));
}
