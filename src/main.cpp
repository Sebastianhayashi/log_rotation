#include <rclcpp/rclcpp.hpp>
#include "ros2_log_rotation/logger_manager.hpp"
#include "ros2_log_rotation/config.hpp"

int main(int argc, char **argv)
{
    // 初始化 ROS2
    rclcpp::init(argc, argv);

    // 创建一个节点
    auto node = std::make_shared<rclcpp::Node>("log_rotation_node");

    // 创建并加载配置
    Config config;
    if (!config.loadFromFile("config/log_config.yaml"))
    {
        config.loadDefaults();
    }

    // 初始化并启动日志管理器
    auto logger_manager = std::make_shared<LoggerManager>(node, config);
    logger_manager->initialize();

    // 使用自定义的日志管理器记录日志
    logger_manager->logInfo("Log rotation node started.");

    // 模拟日志输出
    rclcpp::Rate rate(1);
    int count = 0;
    while (rclcpp::ok())
    {
        logger_manager->logInfo("Logging message number: " + std::to_string(count++));
        rclcpp::spin_some(node);
        rate.sleep();
    }

    // 关闭 ROS2
    rclcpp::shutdown();
    return 0;
}
