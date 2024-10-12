#include <iostream>
#include <string>
#include <vector>
#include <filesystem>
#include "rclcpp/rclcpp.hpp"
#include "rcl_logging_spdlog/logging_interface.h"

// 初始化日志系统
void initialize_logging() {
    // 初始化日志系统，使用默认的分配器
    int ret = rcl_logging_external_initialize(nullptr, rcl_get_default_allocator());
    if (ret != RCL_RET_OK) {
        std::cerr << "Failed to initialize logging system." << std::endl;
        std::exit(1);  // 修复语法错误：替换错误的逗号为分号
    }
}

// 记录日志
void log_info(const std::string& message) {
    // 使用 rclcpp 的日志宏输出信息级别日志
    RCLCPP_INFO(rclcpp::get_logger("custom_logger"), "%s", message.c_str());
}

// 主程序入口
int main(int argc, char *argv[]) {
    // 初始化 ROS2
    rclcpp::init(argc, argv);

    // 初始化日志系统
    initialize_logging();

    // 输出日志消息：系统启动
    log_info("程序启动并初始化了日志系统");

    // 示例：记录节点名称
    std::string node_name = "example_node";
    log_info("为节点 " + node_name + " 创建独立的日志文件");

    // 关闭 ROS2 系统
    rclcpp::shutdown();
    return 0;
}
