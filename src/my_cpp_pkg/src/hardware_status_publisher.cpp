#include "rclcpp/rclcpp.hpp"
#include "my_robot_interfaces/msg/hardware_status.hpp"

class HardwareStatusPublisherNode : public rclcpp::Node
{
public:
  HardwareStatusPublisherNode():Node("hw_sts_pub")
  {
    this->puber_ = this->create_publisher<my_robot_interfaces::msg::HardwareStatus>("hw_sts", 10);
    this->timer_ = this->create_wall_timer(std::chrono::seconds(1), 
                                      std::bind(&HardwareStatusPublisherNode::timerCallback, this));
    RCLCPP_INFO(this->get_logger(), "Hardware Status Publisher has been started");
  }
private:      
  void timerCallback()
  {
    auto msg = my_robot_interfaces::msg::HardwareStatus();
    msg.temperature = 88.88;
    msg.are_motors_ready = true;
    msg.debug_message = "Nothing Special";
    this->puber_->publish(msg);
  }
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<my_robot_interfaces::msg::HardwareStatus>::SharedPtr puber_;

};


int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<HardwareStatusPublisherNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}