#include <rclcpp/rclcpp.hpp>
#include "logger_manager.hpp"

int main(int argc, char **argv)
{
    // 初始化 ROS2
    rclcpp::init(argc, argv);

    // 创建一个节点
    auto node = std::make_shared<rclcpp::Node>("log_rotation_node");

    // 初始化并启动日志管理器
    auto logger_manager = std::make_shared<LoggerManager>(node);
    logger_manager->initialize();  // 假设此方法用于初始化日志管理器

    // 打印节点启动信息
    RCLCPP_INFO(node->get_logger(), "Log rotation node started.");

    // 进入主循环
    rclcpp::spin(node);

    // 关闭 ROS2
    rclcpp::shutdown();
    return 0;
}
