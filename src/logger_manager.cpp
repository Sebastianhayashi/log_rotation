#include "logger_manager.hpp"
#include <spdlog/spdlog.h>
#include <spdlog/sinks/rotating_file_sink.h>
#include <filesystem>

LoggerManager::LoggerManager(const rclcpp::Node::SharedPtr &node, const Config &config)
    : node_(node), config_(config)
{
}

void LoggerManager::initialize()
{
    // 读取用户配置或使用默认值
    std::string log_file_path = config_.log_file_path.empty() ? "/tmp/ros2_logs/log_rotation.log" : config_.log_file_path;
    size_t max_file_size = config_.max_file_size > 0 ? config_.max_file_size : 1048576;  // 默认最大文件大小 1MB
    size_t max_files = config_.max_files > 0 ? config_.max_files : 5;  // 默认最多保留 5 个日志文件

    // 创建日志目录
    std::filesystem::create_directories(std::filesystem::path(log_file_path).parent_path());

    // 创建日志轮转器
    rotating_logger_ = spdlog::rotating_logger_mt("log_rotation", log_file_path, max_file_size, max_files);
    rotating_logger_->set_level(spdlog::level::info);  // 设置日志级别为 info
    rotating_logger_->info("Logger initialized for log rotation.");
}

void LoggerManager::logInfo(const std::string &message)
{
    if (rotating_logger_)
    {
        rotating_logger_->info(message);
    }
}

void LoggerManager::logWarning(const std::string &message)
{
    if (rotating_logger_)
    {
        rotating_logger_->warn(message);
    }
}

void LoggerManager::logError(const std::string &message)
{
    if (rotating_logger_)
    {
        rotating_logger_->error(message);
    }
}
