#include <rclcpp/rclcpp.hpp>
#include <spdlog/spdlog.h>
#include <spdlog/sinks/rotating_file_sink.h>

class LogRotationNode : public rclcpp::Node {
public:
    LogRotationNode(const std::string &node_name)
        : Node(node_name) {
        // 创建日志文件（最大大小为 5MB，最多保留3个备份）
        auto logger = spdlog::rotating_logger_mt(
            node_name, "/tmp/" + node_name + ".log", 5 * 1024 * 1024, 3);
        spdlog::set_default_logger(logger);
        spdlog::info("节点 {} 已启动。", node_name);

        // 设置一个定时器，模拟日志输出
        timer_ = this->create_wall_timer(
            std::chrono::seconds(1),
            [this]() { spdlog::info("日志输出测试: {}", this->now().seconds()); });
    }

private:
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<LogRotationNode>("log_rotation_node");
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
