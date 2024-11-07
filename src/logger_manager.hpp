#ifndef LOGGER_MANAGER_HPP
#define LOGGER_MANAGER_HPP

#include <rclcpp/rclcpp.hpp>
#include <memory>
#include <string>
#include <spdlog/logger.h>
#include "config.hpp"  // 包含配置头文件

class LoggerManager
{
public:
    // 构造函数，接收节点指针和配置对象
    LoggerManager(const rclcpp::Node::SharedPtr &node, const Config &config);

    // 初始化日志管理器
    void initialize();

    // 日志记录方法
    void logInfo(const std::string &message);
    void logWarning(const std::string &message);
    void logError(const std::string &message);

private:
    rclcpp::Node::SharedPtr node_;  // ROS2 节点指针
    Config config_;  // 配置对象
    std::shared_ptr<spdlog::logger> rotating_logger_;  // spdlog 轮转日志记录器
    bool config_loaded_ = false;  // 标记配置是否已加载
};

#endif // LOGGER_MANAGER_HPP
