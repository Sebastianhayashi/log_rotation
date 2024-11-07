#ifndef CONFIG_HPP
#define CONFIG_HPP

#include <string>

class Config
{
public:
    // 默认构造函数
    Config();

    // 配置参数
    std::string log_file_path;     // 日志文件路径
    size_t max_file_size;          // 最大日志文件大小（字节）
    size_t max_files;              // 最大保留日志文件数量

    // 加载默认配置
    void loadDefaults();

    // 从文件或其他来源加载配置
    bool loadFromFile(const std::string &file_path);
};

#endif // CONFIG_HPP
