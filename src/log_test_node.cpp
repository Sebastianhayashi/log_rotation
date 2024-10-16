#include "rclcpp/rclcpp.hpp"

class LoggingTestNode : public rclcpp::Node
{
public:
  LoggingTestNode() : Node("logging_test_node")
  {
    timer_ = this->create_wall_timer(
      std::chrono::seconds(1),
      std::bind(&LoggingTestNode::timer_callback, this));
  }

private:
  void timer_callback()
  {
    RCLCPP_INFO(this->get_logger(), "Info message from %s", this->get_name());
    RCLCPP_WARN(this->get_logger(), "Warning message from %s", this->get_name());
    RCLCPP_ERROR(this->get_logger(), "Error message from %s", this->get_name());
  }

  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);

  auto node = std::make_shared<LoggingTestNode>();
  rclcpp::spin(node);

  rclcpp::shutdown();
  return 0;
}
