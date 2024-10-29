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
    LogTestNode(const std::string & log_file_path, rclcpp::Logger::Level log_level)
        : Node("log_test_node")
    {
        // 设置日志级别
        this->get_logger().set_level(log_level);

        // 创建基于大小的日志轮转器
        auto rotating_sink = std::make_shared<spdlog::sinks::rotating_file_sink_mt>(
            log_file_path, 10485, 5);  //  文件大小，最多保留 5 个备份文件
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

    // 获取日志路径和级别参数
    std::string log_file_path;
    rclcpp::Logger::Level log_level = rclcpp::Logger::Level::Info;

    if (argc > 1) {
        log_file_path = argv[1];
    } else {
        const char * home_dir = std::getenv("HOME");
        if (!home_dir) {
            RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Failed to get home directory");
            rclcpp::shutdown();
            return 1;
        }
        log_file_path = std::string(home_dir) + "/.ros/log/log_rotation_node.log";
    }

    if (argc > 2) {
        std::string level_str = argv[2];
        if (level_str == "debug") {
            log_level = rclcpp::Logger::Level::Debug;
        } else if (level_str == "warn") {
            log_level = rclcpp::Logger::Level::Warn;
        } else if (level_str == "error") {
            log_level = rclcpp::Logger::Level::Error;
        }
    }

    // 创建节点并传入日志路径和级别
    auto node = std::make_shared<LogTestNode>(log_file_path, log_level);

    // 运行节点
    rclcpp::spin(node);

    // 关闭 ROS2
    rclcpp::shutdown();
    return 0;
}
