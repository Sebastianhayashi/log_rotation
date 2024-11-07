#ifndef LOGGER_MANAGER_HPP
#define LOGGER_MANAGER_HPP

#include <rclcpp/rclcpp.hpp>
#include <memory>
#include <string>
#include <spdlog/logger.h>

class LoggerManager
{
public:
    // 构造函数
    explicit LoggerManager(const rclcpp::Node::SharedPtr &node);

    // 初始化日志管理器
    void initialize();

    // 日志记录方法
    void logInfo(const std::string &message);
    void logWarning(const std::string &message);
    void logError(const std::string &message);

private:
    rclcpp::Node::SharedPtr node_;  // ROS2 节点指针
    std::shared_ptr<spdlog::logger> rotating_logger_;  // spdlog 轮转日志记录器
};

#endif // LOGGER_MANAGER_HPP
