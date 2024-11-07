#include <rclcpp/rclcpp.hpp>
#include "logger_manager.hpp"
#include "config.hpp"

int main(int argc, char **argv)
{
    // 初始化 ROS2
    rclcpp::init(argc, argv);

    // 创建一个节点
    auto node = std::make_shared<rclcpp::Node>("log_rotation_node");

    // 声明和获取参数
    node->declare_parameter<std::string>("log_file_path", "/tmp/ros2_logs/log_rotation.log");
    node->declare_parameter<size_t>("max_size", 1048576); // 默认 1MB
    node->declare_parameter<size_t>("max_files", 5); // 默认最多 5 个文件

    // 创建并加载配置对象
    Config config;
    node->get_parameter("log_file_path", config.log_file_path);
    node->get_parameter("max_size", config.max_file_size);
    node->get_parameter("max_files", config.max_files);

    // 初始化并启动日志管理器
    auto logger_manager = std::make_shared<LoggerManager>(node, config);
    logger_manager->initialize();

    // 使用自定义的日志管理器记录日志
    logger_manager->logInfo("Log rotation node started.");

    // 进入主循环
    rclcpp::spin(node);

    // 关闭 ROS2
    rclcpp::shutdown();
    return 0;
}
