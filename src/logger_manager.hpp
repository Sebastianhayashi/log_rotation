#ifndef LOGGER_MANAGER_HPP
#define LOGGER_MANAGER_HPP

#include <rclcpp/rclcpp.hpp>
#include <memory>
#include <string>
#include <spdlog/logger.h>
#include "ros2_log_rotation/config.hpp"
#include "ros2_log_rotation/compression_util.hpp"

class LoggerManager
{
public:
    // 构造函数
    LoggerManager(const rclcpp::Node::SharedPtr &node, const Config &config);

    // 初始化日志管理器
    void initialize();

    // 日志记录方法
    void logInfo(const std::string &message);
    void logWarning(const std::string &message);
    void logError(const std::string &message);

private:
    rclcpp::Node::SharedPtr node_;  // ROS2 节点指针
    std::shared_ptr<spdlog::logger> rotating_logger_;  // spdlog 轮转日志记录器
    Config config_;  // 配置对象
    std::shared_ptr<CompressionUtil> compression_util_;  // 压缩工具
};

#endif // LOGGER_MANAGER_HPP
