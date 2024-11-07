#include "ros2_log_rotation/logger_manager.hpp"
#include <spdlog/sinks/rotating_file_sink.h>
#include <filesystem>
#include "ros2_log_rotation/compression_util.hpp"

LoggerManager::LoggerManager(const rclcpp::Node::SharedPtr &node, const Config &config)
    : node_(node), config_(config)
{
}

void LoggerManager::initialize()
{
    // 创建日志目录
    std::filesystem::create_directories(std::filesystem::path(config_.log_file_path).parent_path());

    // 创建日志轮转器
    rotating_logger_ = spdlog::rotating_logger_mt("log_rotation", config_.log_file_path, config_.max_file_size, config_.max_files);
    rotating_logger_->set_level(spdlog::level::info);  // 设置日志级别为 info

    // 日志轮转后进行压缩
    rotating_logger_->flush_on(spdlog::level::info);
    rotating_logger_->set_pattern("[%Y-%m-%d %H:%M:%S.%e] [%l] %v");

    // 初始化压缩工具
    compression_util_ = std::make_shared<CompressionUtil>();

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
