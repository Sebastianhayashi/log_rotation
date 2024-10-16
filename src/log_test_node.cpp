#include <rclcpp/rclcpp.hpp>
#include <spdlog/spdlog.h>
#include <spdlog/sinks/rotating_file_sink.h>  // 基于大小的日志轮转
#include <rcutils/filesystem.h>  // 用于获取用户目录
#include <memory>

class LogTestNode : public rclcpp::Node
{
public:
    LogTestNode() : Node("log_test_node")
    {
        // 获取当前用户的 home 目录
        const char * home_dir = rcutils_get_home_dir();
        if (!home_dir) {
            RCLCPP_ERROR(this->get_logger(), "Failed to get home directory");
            return;
        }

        // 构建日志文件路径 ~/.ros/log/my_node.log
        std::string log_file_path = std::string(home_dir) + "/.ros/log/my_node.log";

        // 创建基于大小的日志轮转器
        auto rotating_sink = std::make_shared<spdlog::sinks::rotating_file_sink_mt>(
            log_file_path, 10485760, 5);  // 10MB, 保留5个文件
        spdlog::logger rotating_logger("rotating_logger", rotating_sink);

        // 输出不同级别的日志
        rotating_logger.info("This is an INFO log message from log_test_node.");
        rotating_logger.warn("This is a WARNING log message from log_test_node.");
        rotating_logger.error("This is an ERROR log message from log_test_node.");
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
