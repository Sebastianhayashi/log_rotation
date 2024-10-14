#include <rclcpp/rclcpp.hpp>
#include <spdlog/spdlog.h>
#include <spdlog/sinks/rotating_file_sink.h>

class LogRotationNode : public rclcpp::Node {
public:
    LogRotationNode() : Node("log_rotation_node") {
        // 初始化日志轮转：文件大小限制为5MB，保留3个文件
        auto max_size = 5 * 1024 * 1024; // 5 MB
        auto max_files = 3;

        try {
            // 创建旋转文件日志 sink
            auto rotating_logger = spdlog::rotating_logger_mt(
                "log_rotation", "logs/rotation_log.txt", max_size, max_files);

            rotating_logger->set_pattern("[%Y-%m-%d %H:%M:%S.%e] [%l] %v");
            rotating_logger->info("Log Rotation Node initialized.");

            RCLCPP_INFO(this->get_logger(), "ROS 2 log initialized.");

            // 模拟日志输出
            for (int i = 0; i < 1000; ++i) {
                rotating_logger->info("Logging message {}", i);
                RCLCPP_INFO(this->get_logger(), "ROS 2 log message {}", i);
            }
        } catch (const spdlog::spdlog_ex &ex) {
            RCLCPP_ERROR(this->get_logger(), "Log initialization failed: {}", ex.what());
        }
    }
};

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<LogRotationNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
