#include <rclcpp/rclcpp.hpp>

class LogTestNode : public rclcpp::Node
{
public:
    LogTestNode() : Node("log_test_node")
    {
        RCLCPP_INFO(this->get_logger(), "Node started.");
        RCLCPP_INFO(this->get_logger(), "This is an INFO log.");
        RCLCPP_WARN(this->get_logger(), "This is a WARN log.");
        RCLCPP_ERROR(this->get_logger(), "This is an ERROR log.");
    }
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<LogTestNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
