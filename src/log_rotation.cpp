#include <rclcpp/rclcpp.hpp>
#include <spdlog/spdlog.h>
#include <spdlog/sinks/rotating_file_sink.h>
#include <cstdlib>
#include <memory>
#include <string>
#include <chrono>
#include <thread>

class LogTestNode : public rclcpp::Node
{
public:
    LogTestNode()
        : Node("log_test_node")
    {
        // 从环境变量中获取日志文件路径、最大文件大小和最大文件数量
        const char* log_file_path_env = std::getenv("LOG_FILE_PATH");
        std::string log_file_path = log_file_path_env ? log_file_path_env : "/home/user/.ros/log/default_log.log";

        const char* max_size_env = std::getenv("MAX_LOG_SIZE");
        int max_size = max_size_env ? std::stoi(max_size_env) : 1048576;  // 默认 1 MB

        const char* max_files_env = std::getenv("MAX_LOG_FILES");
        int max_files = max_files_env ? std::stoi(max_files_env) : 5;  // 默认保留 5 个文件

        // 输出读取到的环境变量值
        RCLCPP_INFO(this->get_logger(), "Log File Path: %s", log_file_path.c_str());
        RCLCPP_INFO(this->get_logger(), "Max Log Size: %d", max_size);
        RCLCPP_INFO(this->get_logger(), "Max Log Files: %d", max_files);

        // 创建基于大小的日志轮转器
        auto rotating_sink = std::make_shared<spdlog::sinks::rotating_file_sink_mt>(
            log_file_path, max_size, max_files);
        rotating_logger_ = std::make_shared<spdlog::logger>("rotating_logger", rotating_sink);
        rotating_logger_->set_level(spdlog::level::info);

        // 启动一个模拟日志生成的线程
        log_thread_ = std::thread([this]() { this->generate_logs(); });
    }

    ~LogTestNode()
    {
        if (log_thread_.joinable()) {
            log_thread_.join();
        }
    }

private:
    void generate_logs()
    {
        // 模拟大量日志输出，确保达到轮转条件
        for (int i = 0; i < 100000; ++i) {
            rotating_logger_->info("INFO log message number: {}", i);
            rotating_logger_->warn("WARNING log message number: {}", i);
            rotating_logger_->error("ERROR log message number: {}", i);
            std::this_thread::sleep_for(std::chrono::milliseconds(1));  // 延时累积文件大小
        }

        RCLCPP_INFO(this->get_logger(), "Finished generating logs.");
    }

    std::shared_ptr<spdlog::logger> rotating_logger_;
    std::thread log_thread_;
};

int main(int argc, char **argv)
{
    // 初始化 ROS2
    rclcpp::init(argc, argv);

    // 创建节点
    auto node = std::make_shared<LogTestNode>();

    // 运行节点
    rclcpp::spin(node);

    // 关闭 ROS2
    rclcpp::shutdown();
    return 0;
}
