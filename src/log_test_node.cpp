#include <rclcpp/rclcpp.hpp>
#include <spdlog/spdlog.h>
#include <spdlog/sinks/rotating_file_sink.h>  // 基于大小的日志轮转
#include <cstdlib>  // std::getenv
#include <memory>

class LogTestNode : public rclcpp::Node
{
public:
    LogTestNode() : Node("log_test_node")
    {
        // 获取当前用户的 home 目录
        const char * home_dir = std::getenv("HOME");
        if (!home_dir) {
            RCLCPP_ERROR(this->get_logger(), "Failed to get home directory");
            return;
        }

        // 构建日志文件路径 ~/.ros/log/my_node.log
        std::string log_file_path = std::string(home_dir) + "/.ros/log/my_node.log";

        // 创建基于大小的日志轮转器
        auto rotating_sink = std::make_shared<spdlog::sinks::rotating_file_sink_mt>(
            log_file_path, 262144, 5);  // 256KB, 保留5个文件
        spdlog::logger rotating_logger("rotating_logger", rotating_sink);

        // 模拟大量日志输出
        for (int i = 0; i < 10000000; ++i) { // 增加循环次数
            rotating_logger.info("INFO log message number: {}", i);
            rotating_logger.warn("WARNING log message number: {}", i);
            rotating_logger.error("ERROR log message number: {}", i);
        }
        
        RCLCPP_INFO(this->get_logger(), "Finished generating logs.");
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
