#include <spdlog/spdlog.h>
#include <spdlog/sinks/rotating_file_sink.h>
#include <iostream>
#include <cstdlib>
#include <string>

int main() {
    // 从环境变量中获取日志文件路径、最大文件大小和最大文件数量
    const char* log_file_path_env = std::getenv("LOG_FILE_PATH");
    std::string log_file_path = log_file_path_env ? log_file_path_env : "spdlog_test.log";

    const char* max_size_env = std::getenv("MAX_LOG_SIZE");
    int max_size = max_size_env ? std::stoi(max_size_env) : 1048576;  // 默认 1 MB

    const char* max_files_env = std::getenv("MAX_LOG_FILES");
    int max_files = max_files_env ? std::stoi(max_files_env) : 3;  // 默认保留 3 个文件

    try {
        // 创建基于文件大小的日志轮转
        auto rotating_sink = std::make_shared<spdlog::sinks::rotating_file_sink_mt>(log_file_path, max_size, max_files);
        auto logger = std::make_shared<spdlog::logger>("test_logger", rotating_sink);
        logger->set_level(spdlog::level::info);

        // 输出测试信息到日志文件
        for (int i = 0; i < 100; ++i) {
            logger->info("Testing spdlog message number: {}", i);
            std::cout << "Logged message " << i << " to " << log_file_path << std::endl;
        }

        std::cout << "Logging test completed. Check the log file at: " << log_file_path << std::endl;
    } catch (const spdlog::spdlog_ex &ex) {
        std::cerr << "Log initialization failed: " << ex.what() << std::endl;
        return 1;
    }

    return 0;
}
