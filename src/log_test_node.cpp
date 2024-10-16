#include <rclcpp/rclcpp.hpp>
#include <spdlog/spdlog.h>
#include <spdlog/sinks/rotating_file_sink.h>  // 基于大小的日志轮转
#include <spdlog/sinks/daily_file_sink.h>     // 基于时间的日志轮转
#include <memory>

class LogTestNode : public rclcpp::Node
{
public:
    LogTestNode() : Node("log_test_node")
    {
        // 创建基于大小的日志轮转器
        auto rotating_sink = std::make_shared<spdlog::sinks::rotating_file_sink_mt>(
            "/home/user/.ros/log/my_node.log", 10485760, 5);  // 10MB, 保留5个文件
        spdlog::logger rotating_logger("rotating_logger", rotating_sink);

        // 创建基于时间的日志轮转器
        auto daily_sink = std::make_shared<spdlog::sinks::daily_file_sink_mt>(
            "/home/user/.ros/log/my_node_daily.log", 0, 0);  // 每天创建新的日志文件
        spdlog::logger daily_logger("daily_logger", daily_sink);

        // 输出不同级别的日志
        rotating_logger.info("This is an INFO log message from log_test_node.");
        rotating_logger.warn("This is a WARNING log message from log_test_node.");
        rotating_logger.error("This is an ERROR log message from log_test_node.");

        daily_logger.info("Daily log: INFO message");
        daily_logger.warn("Daily log: WARNING message");
        daily_logger.error("Daily log: ERROR message");
    }
};

int main(int argc, char **argv)
{
    // 初始化ROS2
    rclcpp::init(argc, argv);

    // 创建节点
    auto node = std::make_shared<LogTestNode>();

    // 运行节点
    rclcpp::spin(node);

    // 关闭ROS2
    rclcpp::shutdown();
    return 0;
}
