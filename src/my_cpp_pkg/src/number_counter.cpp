#include "rclcpp/rclcpp.hpp"
#include "example_interfaces/msg/int64.hpp"

class NumberCounterNode : public rclcpp::Node
{
public:
NumberCounterNode():Node("number_counter")
  {

    this->subscriber_ = 
      this->create_subscription<example_interfaces::msg::Int64>(
        "number", 
        10,
        std::bind(&NumberCounterNode::callbackNumber, this, std::placeholders::_1));

    RCLCPP_INFO(this->get_logger(), "This is number counter");
  }
private:
  void callbackNumber(const example_interfaces::msg::Int64::SharedPtr msg)
  {
    RCLCPP_INFO(this->get_logger(), "receive: %ld", msg->data);
  }

  rclcpp::Subscription<example_interfaces::msg::Int64>::SharedPtr subscriber_;
};


int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<NumberCounterNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}