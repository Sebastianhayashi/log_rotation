#include <iostream>
#include <string>
#include <vector>
#include <filesystem>
#include <rclcpp/rclcpp.hpp>
#include <rcl_logging_spdlog/logging_interface.h>

// 初始化日志系统
void initialize_logging() {
    int ret = rcl_logging_external_initialize(nullptr, rcl_get_default_allocator());
    if (ret != RCL_RET_OK) {
        std::cerr << "Failed to initialize logging system." << std::endl;
        exit(1);
    }
}

// 记录日志
void log_info(const std::string& message) {
    RCLCPP_INFO(rclcpp::get_logger("custom_logger"), message.c_str());
}

// 主程序
int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);

    initialize_logging();

    // 使用rcl_logging_spdlog记录日志
    log_info("程序启动并初始化了日志系统");

    // 示例: 输出节点名称
    std::string node_name = "example_node";
    log_info("为节点 " + node_name + " 创建独立的日志文件");

    rclcpp::shutdown();
    return 0;
}
