#include <gtest/gtest.h>
#include "ros2_log_rotation/logger_manager.hpp"
#include "ros2_log_rotation/config.hpp"
#include <rclcpp/rclcpp.hpp>

TEST(LoggerManagerTest, Initialization)
{
    auto node = rclcpp::Node::make_shared("test_node");
    Config config;
    config.loadDefaults();
    LoggerManager logger_manager(node, config);
    EXPECT_NO_THROW(logger_manager.initialize());
}

TEST(LoggerManagerTest, LogInfo)
{
    auto node = rclcpp::Node::make_shared("test_node");
    Config config;
    config.loadDefaults();
    LoggerManager logger_manager(node, config);
    logger_manager.initialize();
    EXPECT_NO_THROW(logger_manager.logInfo("Test info message"));
}

int main(int argc, char **argv)
{
    testing::InitGoogleTest(&argc, argv);
    rclcpp::init(argc, argv);
    auto result = RUN_ALL_TESTS();
    rclcpp::shutdown();
    return result;
}
