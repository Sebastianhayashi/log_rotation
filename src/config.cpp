// src/config.cpp
#include "ros2_log_rotation/config.hpp"

Config::Config()
{
    // 默认构造函数实现
    loadDefaults();
}

void Config::loadDefaults()
{
    log_file_path = "/tmp/ros2_logs/log_rotation.log";  // 默认日志路径
    max_file_size = 1048576;  // 1MB
    max_files = 5;  // 最大文件数
}

bool Config::loadFromFile(const std::string &file_path)
{
    // 简单的文件加载模拟，可以替换为实际的文件解析逻辑
    // 暂时返回 false，表示未成功加载配置文件
    return false;
}
