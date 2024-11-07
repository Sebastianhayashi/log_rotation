#include "config.hpp"
#include <fstream>
#include <yaml-cpp/yaml.h>  // 确保您已安装并链接 yaml-cpp 库

Config::Config()
{
    // 初始化默认值
    loadDefaults();
}

void Config::loadDefaults()
{
    log_file_path = "/tmp/ros2_logs/log_rotation.log";
    max_file_size = 1048576;  // 1MB
    max_files = 5;
}

bool Config::loadFromFile(const std::string &file_path)
{
    try
    {
        YAML::Node config = YAML::LoadFile(file_path);
        log_file_path = config["log_file_path"].as<std::string>();
        max_file_size = config["max_size"].as<size_t>();
        max_files = config["max_files"].as<size_t>();
        return true;
    }
    catch (const std::exception &e)
    {
        // 处理错误，例如文件不存在或格式错误
        std::cerr << "Failed to load config file: " << e.what() << std::endl;
        return false;
    }
}
