#include "ros2_log_rotation/config.hpp"
#include <yaml-cpp/yaml.h>
#include <iostream>

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
        std::cerr << "Failed to load config file: " << e.what() << std::endl;
        return false;
    }
}
